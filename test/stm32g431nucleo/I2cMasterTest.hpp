#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/I2cMaster_I2C_DMA.hpp>
#include <coco/board/config.hpp>



using namespace coco;


// drivers for I2cMasterTest on STM32G431 Nucleo board
// https://www.st.com/content/ccc/resource/technical/layouts_and_diagrams/schematic_pack/group1/98/d2/70/60/b1/cb/44/4c/mb1367-g431rb-c04_schematic/files/mb1367-g431rb-c04_schematic.pdf/jcr:content/translations/en.mb1367-g431rb-c04_schematic.pdf
struct Drivers {
	Loop_TIM2 loop{APB1_TIMER_CLOCK};

	using I2cMaster = I2cMaster_I2C_DMA;
	I2cMaster i2c{loop,
		gpio::Config::PA15 | gpio::Config::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN, // I2C1 SCL (CN7 38) (don't forget to lookup the alternate function number in the data sheet!)
		gpio::Config::PB7 | gpio::Config::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN, // I2C1 SDA  (CN7 21)
		i2c::I2C1_INFO,
		dma::DMA1_CH1_CH2_INFO,

		//0x00303D5B}; // timing for 100kHz I2C and 16MHz clock from STM32Cube
		0x00100822}; // timing for 400kHz I2C and 20MHz clock from STM32Cube
	I2cMaster::Channel channel1{i2c, 0x3c}; // SSD1306 display
	I2cMaster::Channel channel2{i2c, 0xf};
	I2cMaster::Buffer<16> buffer1{channel1};
	I2cMaster::Buffer<16> buffer2{channel2};
};

Drivers drivers;

extern "C" {
void I2C1_EV_IRQHandler() {
	drivers.i2c.I2C_IRQHandler();
}
}
