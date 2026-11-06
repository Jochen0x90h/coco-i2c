#pragma once

#include <coco/InterruptQueue.hpp>
#include <coco/PackedValue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/dma.hpp>
#include <coco/platform/flash.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/i2c.hpp>


namespace coco {

/// @param I2C EEPROM emulator.
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
class I2cEeEmu_I2C_DMA : public Loop_Queue::TimeoutHandler {
public:
    /// @param Constructor
    /// @param loop event loop
    /// @param i2cInfo info of I2C instance to use
    /// @param sclPin clock pin and alternate function (SCL, see data sheet), configure as open drain and maybe pull-up
    /// @param sdaPin data pin and alternate function (SDA, see data sheet), configure as open drain and maybe pull-up
    /// @param dmaInfo info of DMA channels to use
    /// @param timing timing configuration for register I2C_TIMINGR, use STM32CubeMX tool to calculate it
    /// @param targetAddressFilter Address filter for target mode, enables target mode if set (currently only single address)
    I2cEeEmu_I2C_DMA(Loop_Queue &loop, const i2c::Info &i2cInfo, gpio::Config sclPin, gpio::Config sdaPin,
        const dma::DualInfo<> &dmaInfo, uint32_t timing, int targetAddress, uint32_t flashAddress, int size);
    ~I2cEeEmu_I2C_DMA() override;


    /// @brief handle I2C interrupt, needs to be called from I2C interrupt handler.
    /// See startup_stm32XXX.c, e.g. I2Cx_IRQHandler() or I2Cx_EV_IRQHandler() where x=1,2... (I2C instance index)
    void I2C_IRQHandler();

protected:
    void onTimeout() override;


    Loop_Queue &loop_;

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

    Registers registers_;
    int i2cIrq_;

    // eeprom data in flash
    uint32_t flashAddress_;

    // eeprom size
    int eeSize_;

    // current eeprom address
    int eeAddress_;

    // flash page in ram
    volatile uint8_t eePage_[flash::PAGE_SIZE];

    uint32_t reload_;
    int size_;
    int reloadCount_;
    bool setAddress_;
};

} // namespace coco
