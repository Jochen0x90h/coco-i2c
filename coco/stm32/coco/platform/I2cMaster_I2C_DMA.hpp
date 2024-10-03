#pragma once

#include <coco/I2cMaster.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/align.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/dma.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/i2c.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/**
 * Implementation of I2C hardware interface for stm32f0 and stm32g4 with multiple virtual channels.
 *
 * Reference manual:
 *   f0:
 *     https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *       I2C: Section 26
 *         DMA: Section 10, Table 29
 *         Code Examples: Section A.14
 *     g4:
 *       https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *         I2C: Section 41
 * Data sheet:
 *   f0:
 *     https://www.st.com/resource/en/datasheet/stm32f042f6.pdf
 *       Alternate Functions: Section 4, Tables 14-16, Page 37
 *     https://www.st.com/resource/en/datasheet/dm00039193.pdf
 *       Alternate Functions: Section 4, Tables 14+15, Page 37
 *     g4:
 *       https://www.st.com/resource/en/datasheet/stm32g431rb.pdf
 *         Alternate Functions: Section 4.11, Table 13, Page 61
 * Resources:
 *   I2C
 *   DMA
 */
class I2cMaster_I2C_DMA : public I2cMaster {
public:
    /**
     * Constructor
     * @param loop event loop
     * @param sclPin clock pin and alternate function (SCL, see data sheet), configure as open drain and maybe pull-up
     * @param sdaPin data pin and alternate function (SDA, see data sheet), configure as open drain and maybe pull-up
     * @param i2cInfo info of I2C instance to use
     * @param dmaInfo info of DMA channels to use
     * @param timing timing configuration for register I2C_TIMINGR, use STM32CubeMX tool to calculate it
     */
    I2cMaster_I2C_DMA(Loop_Queue &loop, gpio::Config sclPin, gpio::Config sdaPin,
        const i2c::Info &i2cInfo, const dma::Info2 &dmaInfo, uint32_t timing);
    ~I2cMaster_I2C_DMA() override;

    // I2cMaster methods
    void recover() override;


    class Channel;

    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class I2cMaster_I2C_DMA;
    public:
        /**
         * Constructor
         * @param data data of the buffer
         * @param capacity capacity of the buffer
         * @param channel channel to attach to
         */
        BufferBase(uint8_t *data, int capacity, Channel &channel);
        ~BufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void start();
        void handle() override;

        Channel &channel;

        Op op;
    };

    /**
     * Virtual channel to a slave device using a dedicated address
     */
    class Channel : public BufferDevice {
        friend class BufferBase;
    public:
        /**
         * Constructor
         * @param device the I2C master to operate on
         * @param address I2C address of the slave
         */
        Channel(I2cMaster_I2C_DMA &device, int address);
        ~Channel();

        // BufferDevice methods
        int getBufferCount() override;
        BufferBase &getBuffer(int index) override;

    protected:
        // list of buffers
        IntrusiveList<BufferBase> buffers;

        I2cMaster_I2C_DMA &device;
        int address;
    };

    /**
     * Buffer for transferring data to/from a I2C slave.
     * @tparam C capacity of buffer
     */
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(Channel &channel) : BufferBase(data, C, channel) {}

    protected:
        alignas(4) uint8_t data[C];
    };

    /**
     * I2C interrupt handler, needs to be called from USART/UART interrupt handler (e.g. I2C1_IRQHandler() or I2C1_EV_IRQHandler() for Resources::I2C1_DMA1_CHANNELS12)
     */
    void I2C_IRQHandler();
protected:
    void startRecover();

    Loop_Queue &loop;

    // i2c
    I2C_TypeDef *i2c;
    int i2cIrq;

    // dma
    dma::Channel rxChannel;
    dma::Channel txChannel;

    int recoverCount = 0;
    bool recovering = false;

    // list of active transfers
    InterruptQueue<BufferBase> transfers;
    int transferCount;
};

} // namespace coco
