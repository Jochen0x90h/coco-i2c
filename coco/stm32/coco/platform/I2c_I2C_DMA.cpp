#include "I2c_I2C_DMA.hpp"
//#include <coco/convert.hpp>
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

// I2c_I2C_DMA

I2c_I2C_DMA::I2c_I2C_DMA(Loop_Queue &loop, const i2c::Info &i2cInfo, gpio::Config sclPin, gpio::Config sdaPin,
    const dma::DualInfo<> &dmaInfo, uint32_t timing, int slaveAddressFilter)
    : loop_(loop)
{
    // configure I2C pins
    gpio::enableAlternate(sclPin);
    gpio::enableAlternate(sdaPin);

    auto &r = registers_;

    // configure I2C
    auto i2c = r.i2c = i2cInfo.enableClock()
        .enable(slaveAddressFilter != 0 ? i2c::Config::TARGET_BYTE_CONTROL : i2c::Config::DEFAULT, timing,
            i2c::Interrupt::TRANSFER_COMPLETE
            | i2c::Interrupt::STOP_DETECTED
            | (slaveAddressFilter != 0 ? i2c::Interrupt::ADDRESS_MATCH : i2c::Interrupt::NONE),
            i2c::DmaRequest::RX_TX);

    // enable own address 1 if filter is set
    if (slaveAddressFilter != 0) {
        i2c->OAR1 = (slaveAddressFilter << 1) | I2C_OAR1_OA1EN;
        //debug::out << hex(this, 4) << " set address " << hex(slaveAddressFilter) << '\n';
    }

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

    // setup IRQ for I2C
    i2cIrq_ = i2cInfo.irq;
    nvic::setPriority(i2cIrq_, nvic::Priority::MEDIUM);
    nvic::clear(i2cIrq_);
    nvic::enable(i2cIrq_);
}

I2c_I2C_DMA::~I2c_I2C_DMA() {
}

void I2c_I2C_DMA::recover() {
    nvic::disable(i2cIrq_);
    if (recoverCount_ == 0 && transfers_.empty())
        startRecover();
    ++recoverCount_;
    nvic::enable(i2cIrq_);
}

void I2c_I2C_DMA::I2C_IRQHandler() {
    auto &r = registers_;

    auto i2c = r.i2c;
    auto status = i2c.status();
    if ((status & i2c::Status::ADDRESS_MATCH) != 0) {
        // target mode: address matched

        // disable interrupt (stretch clock) until a new data transfer is started (TargetBuffer::start())
        i2c.disable(i2c::Interrupt::ADDRESS_MATCH);

        // get address
        int address = getAddress(status);

        // get read flag (controller read means we have to write)
        //bool read = (status & i2c::Status::DIRECTION_MASK) == i2c::Status::DIRECTION_WRITE;

        //debug::out << hex(this) << " AM " << hex(address) << ' ' << dec(read) << '\n';

        // release current transfer (e.g. write, restart, read)
        if (targetTransfer_ != nullptr) {
            // disable DMA
            r.dma.rx.disable();
            r.dma.tx.disable();

            auto &buffer = *targetTransfer_;
            int transferred = buffer.size_ - (buffer.op_ == BufferBase::Op::READ ? r.dma.rx.count() : r.dma.tx.count());
            buffer.setSuccess(transferred);

            // pass buffer to event loop so that the application can be notified
            loop_.push(buffer);
            targetTransfer_ = nullptr;
        }

        // search channel with address
        for (auto &channel : targetChannels_) {
            //debug::out << "channel " << hex(channel.address_) << ' ' << dec(channel.requestInProgress_) << '\n';
            if (channel.address_ == address) {
                channel.active_ = true;

                // send request to the application
                if (!channel.requestInProgress_) {
                    channel.requestInProgress_ = true;
                    loop_.push(channel);
                }
            }
        }
    } else if ((status & i2c::Status::TRANSFER_COMPLETE_RELOAD) != 0) {
        // transfer complete reload: interrupt flag gets cleared by writing NBYTES
        int count = transferCount_;
        //debug::out << hex(this) << " reaload " << dec(count) << '\n';

        if (count > 0) {
            // reload
            // modify CR2, keep slave address, read/write flag and AUTOEND (to automatically generate STOP)
            uint32_t cr2 = i2c->CR2 & (I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN_Msk | I2C_CR2_AUTOEND);
            if (count > 255) {
                // reload after 255 bytes
                cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
                transferCount_ = count - 255;
            } else {
                cr2 |= reload_ | (count << I2C_CR2_NBYTES_Pos);
                transferCount_ = 0;
            }
            i2c->CR2 = cr2;
        } else {
            // disable DMA
            r.dma.rx.disable();
            r.dma.tx.disable();

            if (targetTransfer_ != 0)  {
                // target mode

                // disable transfer complete (stetch clock) until a new data transfer is started (TargetBuffer::start())
                i2c.disable(i2c::Interrupt::TRANSFER_COMPLETE);

                auto &buffer = *targetTransfer_;
                buffer.setSuccess();

                // pass buffer to event loop so that the application can be notified
                loop_.push(buffer);
                targetTransfer_ = nullptr;
            } else {
                // controller mode: data after header
                transfers_.visitFirst(
                    [this](auto &buffer) {
                        // start the next transfer
                        int steps = buffer.channel_.transferNext(buffer, buffer.steps_);
                        buffer.steps_ = steps;
                    });
            }
        }
    } else {
        // disable DMA
        r.dma.rx.disable();
        r.dma.tx.disable();

        //if ((i2c->ISR & I2C_ISR_TC) != 0) {
        if ((status & i2c::Status::TRANSFER_COMPLETE) != 0) {
            // transfer complete (controller mode), but append next transfer, e.g. data after header (AUTOEND = 0)
            // interrupt flag gets cleared by setting START or STOP flag in CR2
            //debug::out << hex(this) << " transfer complete\n";

            // todo: handle partial transfers (BufferBase::Op::PARTIAL flag set)

            transfers_.visitFirst(
                [this](auto &buffer) {
                    // the next transfer
                    int steps = buffer.channel_.transferNext(buffer, buffer.steps_);
                    buffer.steps_ = steps;
                });
        }
        //if ((i2c->ISR & I2C_ISR_STOPF) != 0) {
        if ((status & i2c::Status::STOP_DETECTED) != 0) {
            // stopped: buffer is finished

            //debug::out << hex(this) << " dma rx " << dec(r.dma.rx.count()) << '\n';
            //debug::out << hex(this) << " dma tx " << dec(r.dma.tx.count()) << '\n';

            // get NACK flag
            bool nack = (status & i2c::Status::NOT_ACKNOWLEDGE) != 0;

            //debug::out << hex(this, 4) << " stopped " << dec(nack) << '\n';
            nack = false;

            // clear interrupt flag at peripheral
            i2c.clear(i2c::Status::STOP_DETECTED | i2c::Status::NOT_ACKNOWLEDGE);

            if (targetTransfer_ != nullptr) {
                // target mode
                auto &buffer = *targetTransfer_;
                if (nack) {
                    // error: NACK
                    buffer.setError(std::errc::no_such_device_or_address);
                } else {
                    // success
                    int transferred = buffer.size_ - (buffer.op_ == BufferBase::Op::READ ? r.dma.rx.count() : r.dma.tx.count());
                    if (transferred > 0)
                        --transferred;
                    buffer.setSuccess(transferred);
                }

                // clear active flag
                buffer.channel_.active_ = false;

                // pass buffer to event loop so that the application can be notified
                loop_.push(buffer);
                targetTransfer_ = nullptr;
            } else {
                // end of transfer
                if (recovering_) {
                    recovering_ = false;
                    --recoverCount_;
                } else {
                    auto b = transfers_.pop();
                    if (b != nullptr) {
                        auto &buffer = *b;
                        if (nack) {
                            // error: NACK
                            buffer.setError(std::errc::no_such_device_or_address);
                        } else {
                            // success
                            buffer.setSuccess();
                        }

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
}

void I2c_I2C_DMA::startRecover() {
    auto &r = registers_;

    recovering_ = true;
    int address = (0xfe << I2C_CR2_SADD_Pos) | I2C_CR2_RD_WRN; // address and read flag
    r.i2c->CR2 = I2C_CR2_START // generate start on bus
        | I2C_CR2_AUTOEND // automatically generate STOP
        | address;
}


// I2c_I2C_DMA::BufferBase

I2c_I2C_DMA::BufferBase::BufferBase(uint8_t *headerAndData, int headerCapacity, int capacity, Channel &channel)
    : coco::Buffer(headerAndData, headerCapacity, capacity, BufferBase::State::READY)
    , channel_(channel)
{
    channel.buffers_.add(*this);
}

I2c_I2C_DMA::BufferBase::~BufferBase() {
}

bool I2c_I2C_DMA::BufferBase::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    auto &channel = channel_;
    auto &device = channel.device_;

    {
        nvic::Guard guard(device.i2cIrq_);

        // add to list of pending transfers and start immediately if list was empty
        if (device.transfers_.push(*this)) {
            if (device.recoverCount_ == 0)
                steps_ = channel.transferFirst(*this);
        }
    }

    // set state
    setBusy();

    return true;
}

bool I2c_I2C_DMA::BufferBase::cancel() {
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

void I2c_I2C_DMA::BufferBase::onCompletion() {
    setReady();
}


// I2c_I2C_DMA::Channel

I2c_I2C_DMA::Channel::Channel(I2c_I2C_DMA &device, int address, Flags flags)
    : BufferDevice(State::READY)
    , device_(device)
    , address_(address)
    , flags_(flags)
{
}

I2c_I2C_DMA::Channel::~Channel() {
}

int I2c_I2C_DMA::Channel::getBufferCount() {
    return buffers_.count();
}

I2c_I2C_DMA::BufferBase &I2c_I2C_DMA::Channel::getBuffer(int index) {
    return buffers_.get(index);
}

int I2c_I2C_DMA::Channel::transferFirst(BufferBase &buffer) {
    // get header
    auto header = buffer.header_;
    int size = buffer.headerCapacity_;

    // check for variable header size
    if ((flags_ & Flags::VARIABLE_HEADER_SIZE) != 0) {
        size = header[0];
        ++header;
    }

    // slave address
    int address = address_ << (I2C_CR2_SADD_Pos + 1);

    // CR2 register
    uint32_t cr2 = I2C_CR2_START // generate start on bus
        | address; // slave address

    if (size == 0) {
        // no header: start transfer of buffer data, automatically generate STOP (AUTOEND = 1)
        start(buffer.op(), buffer.data(), buffer.size(), cr2 | I2C_CR2_AUTOEND);

        // one more step to do (wait for end)
        return 1;
    } else {
        // with header

        // continue with data after header without RESTART
        if ((buffer.op() & BufferBase::Op::WRITE) != 0)
            cr2 |= I2C_CR2_RELOAD;

        // start transfer of header
        //debug::out << "start header size " << dec(size) << '\n';
        start(BufferBase::Op::WRITE, header, size, cr2);

        // two more steps to do (transfer data, wait for end)
        return 2;
    }

    // -> I2Cx_IRQHandler()
}

int I2c_I2C_DMA::Channel::transferNext(BufferBase &buffer, int steps) {
    if (steps == 1) {
        // no more steps to do
        return 0;
    }

    //debug::out << hex(&this->device_) << " transferNext\n";

    // slave address
    int address = address_ << (I2C_CR2_SADD_Pos + 1);

    // CR2 register
    uint32_t cr2 = I2C_CR2_START // generate start on bus
        | address; // slave address

    // start transfer of buffer data
    //debug::out << "start data size " << dec(buffer.size()) << '\n';
    start(buffer.op(), buffer.data(), buffer.size(), cr2 | I2C_CR2_AUTOEND);

    // one more step to do (wait for end)
    return 1;
}

void I2c_I2C_DMA::Channel::start(BufferBase::Op op, volatile void *data, int size, uint32_t cr2) {
    auto &device = device_;
    auto &r = registers();
    //int address = address_ << (I2C_CR2_SADD_Pos + 1); // slave address

    // store reload flag for use in reload interrupt handler
    device.reload_ = cr2 & I2C_CR2_RELOAD;

    if (size > 255) {
        // reload after 255 bytes
        cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
        device.transferCount_ = size - 255;
    } else {
        cr2 |= size << I2C_CR2_NBYTES_Pos;
        device.transferCount_ = 0;
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
        //debug::out << hex(&device) << " dma rx " << dec(r.dma.rx.count()) << '\n';
    } else {
        // write
        r.i2c->CR2 = cr2;

        // configure and enable DMA
        r.dma.tx
            .setSourceAddress(data)
            .setCount(size)
            .enable();
        //debug::out << hex(&device) << " dma tx " << dec(r.dma.tx.count()) << '\n';
    }

    // -> I2Cx_IRQHandler()
}


// I2c_I2C_DMA::RegistersChannel

I2c_I2C_DMA::RegistersChannel::~RegistersChannel() {
}

int I2c_I2C_DMA::RegistersChannel::transferFirst(BufferBase &buffer) {
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


// target mode
// -----------


// I2c_I2C_DMA::TargetBufferBase

I2c_I2C_DMA::TargetBufferBase::TargetBufferBase(uint8_t *data, int capacity, TargetChannel &channel)
    : coco::Buffer(data, capacity, TargetBufferBase::State::READY)
    , channel_(channel)
{
    channel.buffers_.add(*this);
}

I2c_I2C_DMA::TargetBufferBase::~TargetBufferBase() {
}

bool I2c_I2C_DMA::TargetBufferBase::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }
    auto &channel = channel_;
    auto &device = channel.device_;

    if (!channel.active_) {
        // channel is not active
        setError(std::errc::device_or_resource_busy);
        return false;
    }

    steps_ = int(op_ & Op::READ_WRITE);

    //debug::out << hex(&device) << " start " << dec(size()) << '\n';

    device.targetTransfer_ = this;
    channel.start(op_, data(), size());

    auto i2c = device.registers_.i2c;
    i2c.clear(i2c::Status::ADDRESS_MATCH);

    // set state
    setBusy();

    // re-enable interrupts
    i2c.enable(i2c::Interrupt::ADDRESS_MATCH | i2c::Interrupt::TRANSFER_COMPLETE);

    return true;
}

bool I2c_I2C_DMA::TargetBufferBase::cancel() {
    if (state_ != State::BUSY)
        return false;
/*
    auto &channel = channel_;
    auto &device = channel_.device_;

    bool canceled = false;
    {
        nvic::Guard guard(device.i2cIrq_);
        if ((op_ & Op::READ) != 0) {
            // read
            canceled = channel.readTransfers_.remove(*this);
        } else {
            // write
            canceled = channel.writeTransfers_.remove(*this);
        }
    }

    // remove from pending transfers if not yet started, otherwise complete normally
    if (canceled) {
        // cancel succeeded: set buffer ready again
        // resume application code, therefore interrupt is enabled at this point
        setError(std::errc::operation_canceled);
        setReady();
    }
*/
    return true;
}

void I2c_I2C_DMA::TargetBufferBase::onCompletion() {
    //debug::out << hex(&channel_.device_) << " TargetBufferBase::onCompletion\n";
    setReady();
}


// I2c_I2C_DMA::TargetChannel

I2c_I2C_DMA::TargetChannel::TargetChannel(I2c_I2C_DMA &device, int address)
    : BufferDevice(State::READY)
    , device_(device)
    , address_(address)
{
    //debug::out << hex(uintptr_t(&device)) << " add TargetChannel\n";
    device.targetChannels_.add(*this);
}

I2c_I2C_DMA::TargetChannel::~TargetChannel() {
}

int I2c_I2C_DMA::TargetChannel::getBufferCount() {
    return buffers_.count();
}

I2c_I2C_DMA::TargetBufferBase &I2c_I2C_DMA::TargetChannel::getBuffer(int index) {
    return buffers_.get(index);
}

Buffer::Op I2c_I2C_DMA::TargetChannel::requestedOp() {
    return (registers().i2c.status() & i2c::Status::DIRECTION_MASK) == i2c::Status::DIRECTION_WRITE ? coco::Buffer::Op::READ : coco::Buffer::Op::WRITE;
}

void I2c_I2C_DMA::TargetChannel::start(BufferBase::Op op, volatile void *data, int size) {
    auto &device = device_;
    auto &r = registers();

    uint32_t cr2 = 0;

    // always reload in reload interrupt handler
    device.reload_ = I2C_CR2_RELOAD;

    if (size > 255) {
        // reload after 255 bytes
        cr2 |= I2C_CR2_RELOAD | (255 << I2C_CR2_NBYTES_Pos);
        device.transferCount_ = size - 255;
    } else {
        cr2 |= I2C_CR2_RELOAD | (size << I2C_CR2_NBYTES_Pos);
        device.transferCount_ = 0;
    }

    if (op != BufferBase::Op::WRITE) {
        // read
        //r.i2c->CR2 = cr2;

        // configure and enable DMA
        r.dma.rx
            .setDestinationAddress(data)
            .setCount(size)
            .enable();
        //debug::out << hex(&device) << " dma RX " << dec(r.dma.rx.count()) << '\n';
    } else {
        // write
        //r.i2c->CR2 = cr2;

        // configure and enable DMA
        r.dma.tx
            .setSourceAddress(data)
            .setCount(size)
            .enable();
        r.dma.rx.setCount(0);
        //debug::out << hex(&device) << " dma TX " << dec(r.dma.tx.count()) << '\n';
    }

    r.i2c->CR2 = cr2;

    // -> I2Cx_IRQHandler()
}

void I2c_I2C_DMA::TargetChannel::onCompletion() {
    //debug::out << hex(&device_) << " TargetChannel::onCompletion\n";

    requestInProgress_ = false;
    notify(Events::REQUEST);
}

} // namespace coco
