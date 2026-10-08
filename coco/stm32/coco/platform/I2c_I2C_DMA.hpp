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
/// Supports controller and target mode.
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
class I2c_I2C_DMA : public I2cMaster  {
public:
    /// @param Constructor
    /// @param loop event loop
    /// @param i2cInfo info of I2C instance to use
    /// @param sclPin clock pin and alternate function (SCL, see data sheet), configure as open drain and maybe pull-up
    /// @param sdaPin data pin and alternate function (SDA, see data sheet), configure as open drain and maybe pull-up
    /// @param dmaInfo info of DMA channels to use
    /// @param timing timing configuration for register I2C_TIMINGR, use STM32CubeMX tool to calculate it
    I2c_I2C_DMA(Loop_Queue &loop, const i2c::Info &i2cInfo, gpio::Config sclPin, gpio::Config sdaPin,
        const dma::DualInfo<> &dmaInfo, uint32_t timing, int slaveAddressFilter = 0);
    ~I2c_I2C_DMA() override;

    // I2cMaster methods
    void recover() override;


    class Channel;

    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::CompletionHandler {
        friend class I2c_I2C_DMA;
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
        // CompletionHandler methods
        void onCompletion() override;

        Channel &channel_;
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

    // internal helpers
    using RxChannel = dma::Channel<dma::Mode::RX8>;
    using TxChannel = dma::Channel<dma::Mode::TX8>;
    struct Registers {
        // i2c
        i2c::Instance i2c;

        // dma
        struct {
            RxChannel rx;
            TxChannel tx;
        } dma;
    };

    /// @brief Channel flags.
    ///
    enum class Flags : uint8_t {
        NONE = 0,
        SUPPORT_ERASE = 1 << 2,

        // the first byte of the header is the header size, otherwise the header size is fixed
        VARIABLE_HEADER_SIZE = 1 << 3
    };

    /// @brief Virtual channel to a slave device using a dedicated I2C address.
    /// Default implementation transfers header and data as-is.
    /// The header is either fixed or variable size depending on the flag Flags::VARIABLE_HEADER_SIZE
    class Channel : public BufferDevice {
        friend class I2c_I2C_DMA;
        friend class BufferBase;
    public:
        /// @brief Constructor.
        /// @param device the I2C master to operate on
        /// @param address 7 bit I2C address of the slave, 0x00 - 0x7f
        /// @param flags Flags, use Flags::VARIABLE_HEADER_SIZE for variable header size
        Channel(I2c_I2C_DMA &device, int address, Flags flags = Flags::NONE);
        ~Channel() override;

        // BufferDevice methods
        int getBufferCount() override;
        BufferBase &getBuffer(int index) override;

    protected:
        Registers &registers() {return device_.registers_;}

        // start first transfer and return outstanding steps
        virtual int transferFirst(BufferBase &buffer);

        // start next transfer or return zero if no more steps to do
        virtual int transferNext(BufferBase &buffer, int steps);

        // start transfer of data (header or buffer)
        void start(BufferBase::Op op, volatile void *data, int size, uint32_t cr2);

        I2c_I2C_DMA &device_;

        // target (slave) address
        uint16_t address_;

        // flags
        Flags flags_;

        // list of buffers
        IntrusiveList<BufferBase> buffers_;
    };

    /// @brief Virtual channel for accessing I2C registers using an address.
    /// The buffer header size must be 4 and contains an uint32_t for the address (1-4 bytes are transferred).
    /// Transfers a header consisting of an address of 1 to 4 bytes (big endian), followed by data to write or read.
    class RegistersChannel : public Channel {
    public:
        /// @brief Constructor.
        /// @param device The I2C device to operate on
        /// @param addressBytes Number of address bytes (1 to 4)
        RegistersChannel(I2c_I2C_DMA &device, int address, int addressBytes)
            : Channel(device, address)
            , addressBytes_(addressBytes)
        {
        }
        ~RegistersChannel() override;

    protected:
        // Channel methods
        int transferFirst(BufferBase &buffer) override;

        // number of address bytes (1 to 4)
        uint8_t addressBytes_;

        // address (max 4 bytes)
        uint8_t header_[4] = {};
    };


    // target mode
    // -----------

    class TargetChannel;

    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class TargetBufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::CompletionHandler {
        friend class I2c_I2C_DMA;
    public:
        /// @brief Constructor
        /// @param data Header and data of the buffer
        /// @param capacity Capacity of the buffer
        /// @param channel channel to attach to
        TargetBufferBase(uint8_t *data, int capacity, TargetChannel &channel);
        ~TargetBufferBase() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        // CompletionHandler methods
        void onCompletion() override;

        TargetChannel &channel_;
    };

    /// @brief Buffer for transferring data to/from a I2C slave.
    /// @tparam B capacity of buffer
    template <int B>
    class TargetBuffer : public TargetBufferBase {
    public:
        TargetBuffer(TargetChannel &channel) : TargetBufferBase(buffer, B, channel) {}

    protected:
        alignas(4) uint8_t buffer[B];
    };

    /// @brief Channel for target (slave) mode.
    ///
    class TargetChannel : public BufferDevice, public IntrusiveListNode, public coco::Loop_Queue::CompletionHandler {
        friend class I2c_I2C_DMA;
        friend class TargetBufferBase;
    public:
        /// @brief Constructor.
        /// @param device the I2C master to operate on
        /// @param address 7 bit I2C address of the slave, 0x00 - 0x7f
        TargetChannel(I2c_I2C_DMA &device, int address);
        ~TargetChannel() override;

        // BufferDevice methods
        int getBufferCount() override;
        TargetBufferBase &getBuffer(int index) override;

        /// @brief Wait until an event request occurs.
        /// @return use co_await on return value
        COCO_NODISCARD coco::Awaitable<coco::CoroutineTask<Events>> untilRequest() {
            return {this->tasks_, Events::REQUEST};
        }

        coco::Buffer::Op requestedOp();

    protected:
        Registers &registers() {return device_.registers_;}

        // start transfer of data (header or buffer)
        void start(BufferBase::Op op, volatile void *data, int size);

        // CompletionHandler methods
        void onCompletion() override;


        I2c_I2C_DMA &device_;
        uint16_t address_;

        std::atomic<bool> requestInProgress_;

        // this channel is currently addressed by the controller and needs to transfer data
        std::atomic<bool> active_;

        // list of buffers
        IntrusiveList<TargetBufferBase> buffers_;

        // list of active transfers
        //InterruptQueue<TargetBufferBase> readTransfers_;
        //InterruptQueue<TargetBufferBase> writeTransfers_;
    };


    /// @brief handle I2C interrupt, needs to be called from I2C interrupt handler.
    /// See startup_stm32XXX.c, e.g. I2Cx_IRQHandler() or I2Cx_EV_IRQHandler() where x=1,2... (I2C instance index)
    void I2C_IRQHandler();

protected:
    void startRecover();


    Loop_Queue &loop_;

    Registers registers_;
    int i2cIrq_;
    int recoverCount_ = 0;
    bool recovering_ = false;

    uint32_t reload_;

    // list of active transfers
    InterruptQueue<BufferBase> transfers_;
    int transferCount_;

    IntrusiveList<TargetChannel> targetChannels_;
    TargetBufferBase *targetTransfer_ = nullptr;
};
COCO_ENUM(I2c_I2C_DMA::Flags)

} // namespace coco
