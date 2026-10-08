#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/I2c_I2C_DMA.hpp>
#include <coco/board/config.hpp>



using namespace coco;


/// @brief Drivers for I2cTargetTest on STM32G491 Nucleo board
/// schematic: https://www.st.com/content/ccc/resource/technical/layouts_and_diagrams/schematic_pack/group1/98/d2/70/60/b1/cb/44/4c/mb1367-g431rb-c04_schematic/files/mb1367-g431rb-c04_schematic.pdf/jcr:content/translations/en.mb1367-g431rb-c04_schematic.pdf
/// I2C1 is controller, I2C2 is target
/// Connect as follows:
/// SCL: PA15 (CN7 17) -> PA9 (CN10 21)
/// SDA: PB9 (CN10 5) -> PA8 (CN10 23)
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using I2c = I2c_I2C_DMA;

    // controller
    I2c i2cController{loop,
        i2c::I2C1_INFO,
        gpio::PA15 | gpio::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN,
        gpio::PB9 | gpio::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN,
        dma::DMA1_CH1_CH2_INFO,

        0x00303D5B}; // timing for 100kHz I2C and 16MHz clock from STM32Cube
        //0x00100822}; // timing for 400kHz I2C and 20MHz clock from STM32Cube
    I2c::Channel controllerChannel{i2cController, 0x50};
    I2c::Buffer<2, 16> controllerBuffer{controllerChannel};

    // target
    I2c i2cTarget{loop,
        i2c::I2C2_INFO,
        gpio::PA9 | gpio::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN,
        gpio::PA8 | gpio::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN,
        dma::DMA1_CH3_CH4_INFO,

        0x00303D5B, // timing for 100kHz I2C and 16MHz clock from STM32Cube
        0x50}; // slave address filter
    I2c::TargetChannel targetChannel{i2cTarget, 0x50};
    I2c::TargetBuffer<16> targetBuffer{targetChannel};

};

Drivers drivers;

extern "C" {
void I2C1_EV_IRQHandler() {
    drivers.i2cController.I2C_IRQHandler();
}
void I2C2_EV_IRQHandler() {
    drivers.i2cTarget.I2C_IRQHandler();
}
}
