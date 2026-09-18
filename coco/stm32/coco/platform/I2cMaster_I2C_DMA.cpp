#include "I2cMaster_I2C_DMA.hpp"
//#include <coco/debug.hpp>
#include <algorithm>


/*
    I2C NACK conditions:
    1. No receiver is present on the bus with the transmitted address so there is no device to respond with an acknowledge.
    2. The receiver is unable to receive or transmit because it is performing some real-time function and is not ready to start communication with the master.
    3. During the transfer, the receiver gets data or commands that it does not understand.
    4. During the transfer, the receiver cannot receive any more data bytes.
    5. A master-receiver must signal the end of the transfer to the slave transmitter.
*/
namespace coco {

// I2cMaster_I2C_DMA

I2cMaster_I2C_DMA::I2cMaster_I2C_DMA(Loop_Queue &loop, gpio::Config sclPin, gpio::Config sdaPin,
    const i2c::Info &i2cInfo, const dma::DualInfo<> &dmaInfo, uint32_t timing)
    : loop_(loop)
{
    // configure I2C pins
    gpio::enableAlternate(sclPin);
    gpio::enableAlternate(sdaPin);

    auto &r = registers_;

    // configure I2C
    auto i2c = r.i2c = i2cInfo.enableClock()
        .enable(timing, I2C_CR1_PE // enable I2C
            | I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN // DMA mode
            | I2C_CR1_TCIE // interrupt on transfer complete
            | I2C_CR1_STOPIE); // interrupt on STOP

    // setup IRQ for I2C (interrupt gets enabled in first call to BufferBase::start())
    i2cIrq_ = i2cInfo.irq;
    nvic::setPriority(i2cIrq_, nvic::Priority::MEDIUM);

    // configure DMA channels
    auto [rxChannel, txChannel] = dmaInfo.enableClock<RxChannel::MODE, TxChannel::MODE>();
    r.dma.rx = rxChannel
        .configure()
        .setSourceAddress(&i2c->RXDR);
    r.dma.tx = txChannel
        .configure()
        .setDestinationAddress(&i2c->TXDR);

    // map DMA to I2C
    i2cInfo.map(dmaInfo);
}

I2cMaster_I2C_DMA::~I2cMaster_I2C_DMA() {
}

void I2cMaster_I2C_DMA::recover() {
    nvic::disable(i2cIrq_);
    if (recoverCount_ == 0 && transfers_.empty())
        startRecover();
    ++recoverCount_;
    nvic::enable(i2cIrq_);
}

void I2cMaster_I2C_DMA::I2C_IRQHandler() {
    auto &r = registers_;

    auto i2c = r.i2c;
    if ((i2c->ISR & I2C_ISR_TCR) != 0) {
        // transfer complete reload: interrupt flag gets cleared by writing NBYTES
        int count = transferCount_;
        uint32_t cr2 = i2c->CR2 & (I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN_Msk | I2C_CR2_AUTOEND); // keep slave address, read/write flag and AUTOEND
        if (count > 255) {
            // reload after 255 bytes
            cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
            transferCount_ = count - 255;
        } else {
            // automatically generate STOP
            cr2 |= count << I2C_CR2_NBYTES_Pos;
        }
        i2c->CR2 = cr2;
    } else {
        // disable DMA
        r.dma.rx.disable();
        r.dma.tx.disable();

        if ((i2c->ISR & I2C_ISR_TC) != 0) {
            // transfer complete with AUTOEND = 0 (interrupt flag gets cleared on START or STOP in CR2)

            // todo: handle partial transfers (BufferBase::Op::PARTIAL flag set)

            transfers_.visitFirst(
                [this](auto &buffer) {
                    // try to start the next transfer
                    int steps = buffer.channel_.transferNext(buffer, buffer.steps_);
                    buffer.steps_ = steps;
                });
        }
        if ((i2c->ISR & I2C_ISR_STOPF) != 0) {
            // stopped: clear interrupt flag at peripheral
            i2c->ICR = I2C_ICR_STOPCF;

            // end of transfer
            if (recovering_) {
                recovering_ = false;
                --recoverCount_;
            } else {
                auto b = transfers_.pop();
                if (b != nullptr) {
                    auto &buffer = *b;
                    bool nack = (i2c->ISR & I2C_ISR_NACKF) != 0;
                    if (nack) {
                        // error: NACK
                        buffer.setError(std::errc::no_such_device_or_address);
                    } else {
                        // success
                        buffer.setSuccess();
                    }
                    i2c->ICR = I2C_ICR_NACKCF;

                    // pass buffer to event loop so that the application can be notified
                    loop_.push(buffer);
                }
            }

            if (recoverCount_ > 0) {
                startRecover();
            } else {
                auto next = transfers_.frontOrNull();
                if (next != nullptr)
                    next->start();
            }
        }
    }
}

void I2cMaster_I2C_DMA::startRecover() {
    auto &r = registers_;

    recovering_ = true;
    int address = (0xfe << I2C_CR2_SADD_Pos) | I2C_CR2_RD_WRN; // address and read flag
    r.i2c->CR2 = I2C_CR2_START // generate start on bus
        | I2C_CR2_AUTOEND // automatically generate STOP
        | address;
}


// I2cMaster_I2C_DMA::BufferBase

I2cMaster_I2C_DMA::BufferBase::BufferBase(uint8_t *headerAndData, int headerCapacity, int capacity, Channel &channel)
    : coco::Buffer(headerAndData, headerCapacity, capacity, BufferBase::State::READY)
    , channel_(channel)
{
    channel.buffers_.add(*this);
}

I2cMaster_I2C_DMA::BufferBase::~BufferBase() {
}

bool I2cMaster_I2C_DMA::BufferBase::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    auto &device = channel_.device_;

    {
        nvic::Guard guard(device.i2cIrq_);

        // add to list of pending transfers and start immediately if list was empty
        if (device.transfers_.push(*this)) {
            if (device.recoverCount_ == 0)
                start();
        }
    }

    // set state
    setBusy();

    return true;
}

bool I2cMaster_I2C_DMA::BufferBase::cancel() {
    if (state_ != State::BUSY)
        return false;
    auto &device = channel_.device_;

    // remove from pending transfers if not yet started, otherwise complete normally
    if (device.transfers_.guardedRemoveExceptFirst(nvic::Guard(device.i2cIrq_), *this)) {
        // cancel succeeded: set buffer ready again
        // resume application code, therefore interrupt is enabled at this point
        setError(std::errc::operation_canceled);
        setReady();
    }

    return true;
}

void I2cMaster_I2C_DMA::BufferBase::onCompletion() {
    setReady();
}


// I2cMaster_I2C_DMA::Channel

I2cMaster_I2C_DMA::Channel::Channel(I2cMaster_I2C_DMA &device, int address, Flags flags)
    : BufferDevice(State::READY)
    , device_(device)
    , address_(address)
    , flags_(flags)
{
}

I2cMaster_I2C_DMA::Channel::~Channel() {
}

int I2cMaster_I2C_DMA::Channel::getBufferCount() {
    return buffers_.count();
}

I2cMaster_I2C_DMA::BufferBase &I2cMaster_I2C_DMA::Channel::getBuffer(int index) {
    return buffers_.get(index);
}

int I2cMaster_I2C_DMA::Channel::transferFirst(BufferBase &buffer) {
    // get header
    auto header = buffer.header_;
    int size = buffer.headerCapacity_;

    // check for variable header size
    if ((flags_ & Flags::VARIABLE_HEADER_SIZE) != 0) {
        size = header[0];
        ++header;
    }

    if (size == 0) {
        // no header, start transfer of buffer data
        start(buffer.op(), buffer.data(), buffer.size(), I2C_CR2_AUTOEND);

        // one more step to do (wait for end)
        return 1;
    } else {
        // start transfer of header
        //debug::out << "start header size " << dec(size) << '\n';
        start(BufferBase::Op::WRITE, header, size, 0);

        // two more steps to do (transfer data, wait for end)
        return 2;
    }

    // -> I2Cx_IRQHandler()
}

int I2cMaster_I2C_DMA::Channel::transferNext(BufferBase &buffer, int steps) {
    if (steps == 1) {
        // no more steps to do
        return 0;
    }

    // start transfer of buffer data
    //debug::out << "start data size " << dec(buffer.size()) << '\n';
    start(buffer.op(), buffer.data(), buffer.size(), I2C_CR2_AUTOEND);

    // one more step to do (wait for end)
    return 1;
}

void I2cMaster_I2C_DMA::Channel::start(BufferBase::Op op, volatile void *data, int size, uint32_t cr2) {
    auto &device = device_;
    auto &r = registers();
    int address = address_ << (I2C_CR2_SADD_Pos + 1); // slave address

    cr2 |= I2C_CR2_START // generate start on bus
        | address; // slave address
    if (size > 255) {
        // reload after 255 bytes
        cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
        device.transferCount_ = size - 255;
    } else {
        // automatically generate STOP
        cr2 |= size << I2C_CR2_NBYTES_Pos;
    }

    if (op != BufferBase::Op::WRITE) {
        // read
        cr2 |= I2C_CR2_RD_WRN;
        r.i2c->CR2 = cr2;

        // configure and enable DMA
        r.dma.rx
            .setDestinationAddress(data)
            .setCount(size)
            .enable();
    } else {
        // write
        // I2C_CR2_RD_WRN = 0
        r.i2c->CR2 = cr2;

        // configure and enable DMA
        r.dma.tx
            .setSourceAddress(data)
            .setCount(size)
            .enable();
    }

    // -> I2Cx_IRQHandler()
}


// I2cMaster_I2C_DMA::RegistersChannel

I2cMaster_I2C_DMA::RegistersChannel::~RegistersChannel() {
}

int I2cMaster_I2C_DMA::RegistersChannel::transferFirst(BufferBase &buffer) {
    // build header: address (max 4 bytes, big endian)
    uint32_t address = buffer.header<uint32_t>();
    int size = addressBytes_;
    for (int i = addressBytes_ - 1; i >= 0; --i) {
        header_[1 + i] = address;
        address >>= 8;
    }

    // start transfer of header
    start(BufferBase::Op::WRITE, header_, size, 0);

    // two more steps to do (transfer data, wait for end)
    return 2;

    // -> I2Cx_IRQHandler()
}


} // namespace coco
