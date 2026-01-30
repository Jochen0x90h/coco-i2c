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
    // enable clocks
    i2cInfo.rcc.enableClock();
    //dmaInfo.rcc.enableClock();

    // configure I2C pins
    gpio::enableAlternate(sclPin);
    gpio::enableAlternate(sdaPin);

    // initialize I2C
    auto i2c = i2c_ = i2cInfo.i2c;
    i2c->TIMINGR = timing;
    i2c->CR1 = I2C_CR1_PE // enable I2C
        | I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN // DMA mode
        | I2C_CR1_TCIE // interrupt on transfer complete
        | I2C_CR1_STOPIE; // interrupt on STOP
    i2cIrq_ = i2cInfo.irq;
    nvic::setPriority(i2cIrq_, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // configure DMA channels
    auto [rxChannel, txChannel] = dmaInfo.enableClock<RxChannel::MODE, TxChannel::MODE>();
    rxChannel_ = rxChannel
        .configure()
        .setSourceAddress(&i2c->RXDR);
    txChannel_ = txChannel
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
    auto i2c = i2c_;
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
    } else if ((i2c->ISR & I2C_ISR_TC) != 0) {
        // transfer complete (RELOAD = 0, AUTOEND = 0): read after writing header
        auto &buffer = transfers_.front();

        int count = buffer.size_;// - buffer.p.headerSize;
        if (count == 0) {
            i2c->CR2 = I2C_CR2_STOP; // generate stop on bus
            // -> I2Cx_IRQHandler()
        } else {
            volatile void *data = buffer.data_;// + buffer.p.headerSize;
            int address = i2c->CR2 & I2C_CR2_SADD_Msk;

            uint32_t cr2 = I2C_CR2_START // generate start on bus
                | I2C_CR2_RD_WRN // write
                | address; // slave address
            if (count > 255) {
                // reload after 255 bytes
                cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
                transferCount_ = count - 255;
            } else {
                // automatically generate STOP
                cr2 |= count << I2C_CR2_NBYTES_Pos;
            }
            cr2 |= I2C_CR2_AUTOEND;
            i2c->CR2 = cr2;

            // configure and enable DMA
            rxChannel_
                .setDestinationAddress(data)
                .setCount(count)
                .enable();
            // -> I2Cx_IRQHandler()
        }
    }

    if ((i2c->ISR & I2C_ISR_STOPF) != 0) {
        // stopped: clear interrupt flag at peripheral
        i2c->ICR = I2C_ICR_STOPCF;

        // end of transfer

        // disable DMA
        rxChannel_.disable();
        txChannel_.disable();

        if (recovering_) {
            recovering_ = false;
            --recoverCount_;
        } else {
            transfers_.pop(
                [this, i2c](BufferBase &buffer) {
                    // set result
                    bool nack = (i2c->ISR & I2C_ISR_NACKF) != 0;
                    if (nack) {
                        // set size
                        if ((i2c->CR2 & I2C_CR2_AUTOEND) == 0) {
                            // stopped when still transferring the header of a read operation: clear
                            buffer.clear();
                        } else {
                            // stopped during read or write
                            bool write = (buffer.op_ & BufferBase::Op::WRITE) != 0;
                            buffer.size_ -= write ? txChannel_.count() : rxChannel_.count();
                        }
                        buffer.result_ = BufferBase::Result::NO_REPLY;
                    } else {
                        buffer.result_ = BufferBase::Result::SUCCESS;
                    }
                    i2c->ICR = I2C_ICR_NACKCF;

                    // pass buffer to event loop so that the application can be notified
                    loop_.push(buffer);
                    return true;
                }
            );
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

void I2cMaster_I2C_DMA::startRecover() {
    recovering_ = true;
    int address = (0xfe << I2C_CR2_SADD_Pos) | I2C_CR2_RD_WRN; // address and read flag
    i2c_->CR2 = I2C_CR2_START // generate start on bus
        | I2C_CR2_AUTOEND // automatically generate STOP
        | address;
}


// I2cMaster_I2C_DMA::BufferBase

I2cMaster_I2C_DMA::BufferBase::BufferBase(uint8_t *headerAndData, int headerCapacity, int capacity, Channel &channel)
    : coco::Buffer(headerAndData, headerCapacity, headerCapacity, capacity, BufferBase::State::READY), channel_(channel)
{
    channel.buffers_.add(*this);
}

I2cMaster_I2C_DMA::BufferBase::~BufferBase() {
}

bool I2cMaster_I2C_DMA::BufferBase::start(Op op) {
    if (st.state != State::READY || (op & Op::READ_WRITE) == 0 || size_ == 0) {
        // starting a buffer when the state is BUSY is a bug
        assert(st.state != State::BUSY);
        return false;
    }

    op_ = op;
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
    if (st.state != State::BUSY)
        return false;
    auto &device = channel_.device_;

    // remove from pending transfers if not yet started, otherwise complete normally
    if (device.transfers_.remove(nvic::Guard(device.i2cIrq_), *this, false))
        setReady(0);

    return true;
}

void I2cMaster_I2C_DMA::BufferBase::start() {
    auto &device = channel_.device_;

    volatile void *data = header_;
    int headerSize = headerCapacity_;
    int address = channel_.address_ << (I2C_CR2_SADD_Pos + 1); // slave address
    bool write = (op_ & Op::WRITE) != 0;
    if (!write && headerSize == 0) {
        // read
        //writeHeader = false;
        int count = size_;
        uint32_t cr2 = I2C_CR2_START // generate start on bus
            | I2C_CR2_RD_WRN // read
            | address; // slave address
        if (count > 255) {
            // reload after 255 bytes
            cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
            device.transferCount_ = count - 255;
        } else {
            // automatically generate STOP
            cr2 |= count << I2C_CR2_NBYTES_Pos;
        }
        cr2 |= I2C_CR2_AUTOEND;
        device.i2c_->CR2 = cr2;

        // configure and enable DMA
        device.rxChannel_
            .setDestinationAddress(data)
            .setCount(count)
            .enable();
        // -> I2Cx_IRQHandler()
    } else {
        // write data or header of read operation
        //writeHeader_ = false;
        int count = headerSize + (write ? size_ : 0);
        uint32_t cr2 = I2C_CR2_START // generate start on bus
            | 0 // write (RD_WRN = 0)
            | address; // slave address
        if (count > 255) {
            // reload after 255 bytes
            cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
            device.transferCount_ = count - 255;
        } else {
            // write: automatically generate STOP, read: restart on TC
            cr2 |= (count << I2C_CR2_NBYTES_Pos);
        }
        if (write)
            cr2 |= I2C_CR2_AUTOEND;
        device.i2c_->CR2 = cr2;

        // configure and enable DMA
        device.txChannel_
            .setSourceAddress(data)
            .setCount(count)
            .enable();
        // -> I2Cx_IRQHandler()
    }
}

void I2cMaster_I2C_DMA::BufferBase::handle() {
    setReady();
}


// I2cMaster_I2C_DMA::Channel

I2cMaster_I2C_DMA::Channel::Channel(I2cMaster_I2C_DMA &device, int address)
    : BufferDevice(State::READY)
    , device_(device), address_(address)
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

} // namespace coco
