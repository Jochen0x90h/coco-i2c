#pragma once

#include <coco/I2cMaster.hpp>
#include <coco/align.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/dma.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/i2c.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/// @param Implementation of I2C hardware interface for stm32f0 and stm32g4 with multiple virtual channels.
///
/// Reference manual:
///   f0: https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf (Code Examples: Section A.14)
///   g4: https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
/// Data sheet:
///   f042: https://www.st.com/resource/en/datasheet/stm32f042f6.pdf
///   f051: https://www.st.com/resource/en/datasheet/dm00039193.pdf
///   g431: https://www.st.com/resource/en/datasheet/stm32g431rb.pdf
/// Resources:
///   I2C
///   DMA
class I2cMaster_I2C_DMA : public I2cMaster {
public:
    /// @param Constructor
    /// @param loop event loop
    /// @param sclPin clock pin and alternate function (SCL, see data sheet), configure as open drain and maybe pull-up
    /// @param sdaPin data pin and alternate function (SDA, see data sheet), configure as open drain and maybe pull-up
    /// @param i2cInfo info of I2C instance to use
    /// @param dmaInfo info of DMA channels to use
    /// @param timing timing configuration for register I2C_TIMINGR, use STM32CubeMX tool to calculate it
    I2cMaster_I2C_DMA(Loop_Queue &loop, gpio::Config sclPin, gpio::Config sdaPin,
        const i2c::Info &i2cInfo, const dma::DualInfo<> &dmaInfo, uint32_t timing);
    ~I2cMaster_I2C_DMA() override;

    // I2cMaster methods
    void recover() override;


    class Channel;

    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::CompletionHandler {
        friend class I2cMaster_I2C_DMA;
    public:
        /// @brief Constructor
        /// @param headerAndData Header and data of the buffer
        /// @param headerCapacity Capacity of the buffer
        /// @param capacity Capacity of the buffer
        /// @param channel channel to attach to
        BufferBase(uint8_t *headerAndData, int headerCapacity, int capacity, Channel &channel);
        ~BufferBase() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        void transfer();
        void onCompletion() override;

        Channel &channel_;
        //Op op_;
    };

    /// @brief Virtual channel to a slave device using a dedicated address.
    ///
    class Channel : public BufferDevice {
        friend class BufferBase;
    public:
        /// @brief Constructor.
        /// @param device the I2C master to operate on
        /// @param address 7 bit I2C address of the slave, 0x00 - 0x7f
        Channel(I2cMaster_I2C_DMA &device, int address);
        ~Channel() override;

        // BufferDevice methods
        int getBufferCount() override;
        BufferBase &getBuffer(int index) override;

    protected:
        // list of buffers
        IntrusiveList<BufferBase> buffers_;

        I2cMaster_I2C_DMA &device_;
        int address_;
    };

    /// @brief Buffer for transferring data to/from a I2C slave.
    /// @tparam H capacity of header
    /// @tparam B capacity of buffer
    template <int H, int B>
    class Buffer : public BufferBase {
    public:
        Buffer(Channel &channel) : BufferBase(buffer, H, B, channel) {}

    protected:
        alignas(4) uint8_t buffer[H + B];
    };

    /// @brief handle I2C interrupt, needs to be called from I2C interrupt handler.
    /// See startup_stm32XXX.c, e.g. I2Cx_IRQHandler() or I2Cx_EV_IRQHandler() where x=1,2... (I2C instance index)
    void I2C_IRQHandler();
protected:
    void startRecover();

    Loop_Queue &loop_;

    // i2c
    I2C_TypeDef *i2c_;
    int i2cIrq_;

    // dma
    using RxChannel = dma::Channel<dma::Mode::RX8>;
    RxChannel rxChannel_;
    using TxChannel = dma::Channel<dma::Mode::TX8>;
    TxChannel txChannel_;

    int recoverCount_ = 0;
    bool recovering_ = false;

    // list of active transfers
    InterruptQueue<BufferBase> transfers_;
    int transferCount_;
};

} // namespace coco
