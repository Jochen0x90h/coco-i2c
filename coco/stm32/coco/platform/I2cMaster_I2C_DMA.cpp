#include "I2cMaster_I2C_DMA.hpp"
#include <coco/debug.hpp>
#include <algorithm>


namespace coco {

// I2cMaster_I2C_DMA

I2cMaster_I2C_DMA::I2cMaster_I2C_DMA(Loop_Queue &loop, gpio::Config sclPin, gpio::Config sdaPin,
    const i2c::Info &i2cInfo, const dma::Info2 &dmaInfo, uint32_t timing)
    : loop(loop)
{
    // enable clocks (note two cycles wait time until peripherals can be accessed, see STM32G4 reference manual section 7.2.17)
    i2cInfo.rcc.enableClock();
    dmaInfo.rcc.enableClock();

    // configure I2C pins
    gpio::configureAlternate(sclPin);
    gpio::configureAlternate(sdaPin);

    // initialize I2C
    auto i2c = this->i2c = i2cInfo.i2c;
    i2c->TIMINGR = timing;
    i2c->CR1 = I2C_CR1_PE // enable I2C
        | I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN // DMA mode
        | I2C_CR1_TCIE // interrupt on transfer complete
        | I2C_CR1_STOPIE; // interrupt on STOP
    this->i2cIrq = i2cInfo.irq;
    nvic::setPriority(this->i2cIrq, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // initialize RX DMA channel
    this->rxChannel = dmaInfo.channel1();
    this->rxChannel.setPeripheralAddress(&i2c->RXDR);

    // initialize TX DMA channel
    this->txChannel = dmaInfo.channel2();
    this->txChannel.setPeripheralAddress(&i2c->TXDR);

    // map DMA to I2C
    i2cInfo.map(dmaInfo);
}

I2cMaster_I2C_DMA::~I2cMaster_I2C_DMA() {
}

void I2cMaster_I2C_DMA::recover() {
    nvic::disable(this->i2cIrq);
    if (this->recoverCount == 0 && this->transfers.empty())
        startRecover();
    ++this->recoverCount;
    nvic::enable(this->i2cIrq);
}

void I2cMaster_I2C_DMA::I2C_IRQHandler() {
    debug::setRed();
        // check if read DMA has completed
    if ((this->i2c->ISR & I2C_ISR_TCR) != 0) {
        // interrupt flag gets cleared by writing NBYTES
        int count = this->transferCount;
        int address = this->i2c->CR2 & I2C_CR2_SADD_Msk;
        if (count > 255) {
            this->i2c->CR2 = I2C_CR2_RELOAD // reload next section of up to 255 bytes
                | (255 << I2C_CR2_NBYTES_Pos) // number of bytes to write
                | address; // slave address
            this->transferCount = count - 255;
        } else {
            this->i2c->CR2 = I2C_CR2_AUTOEND // automatically generate STOP
                | (count << I2C_CR2_NBYTES_Pos) // number of bytes to write
                | address; // slave address
        }
    }
    if ((this->i2c->ISR & I2C_ISR_STOPF) != 0) {
        // clear interrupt flag at peripheral
        this->i2c->ICR = I2C_ICR_STOPCF;

        // end of transfer

        // disable DMA
        this->rxChannel.disable();//->CCR = 0;
        this->txChannel.disable();//->CCR = 0;

        if (this->recovering) {
            this->recovering = false;
            --this->recoverCount;
        } else {
            this->transfers.pop(
                [this](BufferBase &buffer) {
                    this->loop.push(buffer);
                    return true;
                }
            );
        }

        if (this->recoverCount > 0) {
            startRecover();
        } else {
            auto next = this->transfers.frontOrNull();
            if (next != nullptr)
                next->start();
        }
    }
}

void I2cMaster_I2C_DMA::startRecover() {
    this->recovering = true;
    int address = (0x7f << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN; // address and read flag (1)
    this->i2c->CR2 = I2C_CR2_START // generate start on bus
        | I2C_CR2_AUTOEND // automatically generate STOP
        | address;
}


// BufferBase

I2cMaster_I2C_DMA::BufferBase::BufferBase(uint8_t *data, int capacity, Channel &channel)
    : coco::Buffer(data, capacity, BufferBase::State::READY), channel(channel)
{
    channel.buffers.add(*this);
}

I2cMaster_I2C_DMA::BufferBase::~BufferBase() {
}

bool I2cMaster_I2C_DMA::BufferBase::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    this->op = op;
    auto &device = this->channel.device;

    {
        nvic::Guard guard(device.i2cIrq);

        // add to list of pending transfers and start immediately if list was empty
        if (device.transfers.push(*this)) {
            if (device.recoverCount == 0)
                start();
        }
    }

    // set state
    setBusy();

    return true;
}

bool I2cMaster_I2C_DMA::BufferBase::cancel() {
    if (this->st.state != State::BUSY)
        return false;
    auto &device = this->channel.device;

    // remove from pending transfers if not yet started, otherwise complete normally
    if (device.transfers.remove(nvic::Guard(device.i2cIrq), *this, false))
        setReady(0);

    return true;
}

void I2cMaster_I2C_DMA::BufferBase::start() {
    //this->inProgress = true;
    auto &device = this->channel.device;

    //debug::set(0);

    //int writeSize = 0;
    //int readSize = 0;
    if ((this->op & Op::WRITE) == 0) {
        // read
        //int commandCount = std::min(int(this->op & Op::COMMAND_MASK) >> COMMAND_SHIFT, this->p.size);
        //writeSize = commandCount;
        //readSize = this->xferred - commandCount;


    } else {
        // write
        //debug::setRed();
        auto data = this->p.data;
        int count = this->p.size;

        int address = this->channel.address << (I2C_CR2_SADD_Pos + 1); // address and write flag (0)
        if (count > 255) {
            device.i2c->CR2 = I2C_CR2_START // generate start on bus
                | I2C_CR2_RELOAD // reload next section of up to 255 bytes
                | (255 << I2C_CR2_NBYTES_Pos) // number of bytes to write
                | address; // slave address
            device.transferCount = count - 255;
        } else {
            device.i2c->CR2 = I2C_CR2_START // generate start on bus
                | I2C_CR2_AUTOEND // automatically generate STOP
                | (count << I2C_CR2_NBYTES_Pos) // number of bytes to write
                | address; // slave address
        }

        // configure and enable DMA
        device.txChannel.setMemoryAddress(data);
        device.txChannel.setCount(count);
        device.txChannel.enable(dma::Channel::Config::TX);

        // -> I2Cx_IRQHandler()
    }
}

void I2cMaster_I2C_DMA::BufferBase::handle() {
    setReady();
}


// Channel

I2cMaster_I2C_DMA::Channel::Channel(I2cMaster_I2C_DMA &device, int address)
    : BufferDevice(State::READY)
    , device(device), address(address)
{
}

I2cMaster_I2C_DMA::Channel::~Channel() {
}

int I2cMaster_I2C_DMA::Channel::getBufferCount() {
    return this->buffers.count();
}

I2cMaster_I2C_DMA::BufferBase &I2cMaster_I2C_DMA::Channel::getBuffer(int index) {
    return this->buffers.get(index);
}

} // namespace coco
