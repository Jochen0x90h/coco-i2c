#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/I2cMaster_I2C_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


// drivers for I2cMasterTest
struct Drivers {
	Loop_TIM2 loop{APB1_TIMER_CLOCK};

	using I2cMaster = I2cMaster_I2C_DMA;
	I2cMaster i2c{loop,
		//gpio::PB(6, 4), // I2C1 SCL (don't forget to lookup the alternate function number in the data sheet!)
		//gpio::PB(7, 4), // I2C1 SDA
		gpio::Config::PB6 | gpio::Config::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN, // I2C1 SCL (don't forget to lookup the alternate function number in the data sheet!)
		gpio::Config::PB7 | gpio::Config::AF4 | gpio::Config::PULL_UP | gpio::Config::SPEED_LOW | gpio::Config::DRIVE_DOWN, // I2C1 SDA
		i2c::I2C1_INFO,
		dma::DMA1_CH3_CH2_INFO,

		//0x2000090E}; // timing for 100kHz I2C and 8MHz clock from STM32Cube
		0x2000090E}; // timing for 100kHz I2C and 20MHz clock from STM32Cube
		//0x0000020B}; // timing for 400kHz I2C and 20MHz clock from STM32Cube
		//0x00000001}; // timing for 1MHz I2C and 20MHz clock from STM32Cube
	I2cMaster::Channel channel1{i2c, 0xa};
	I2cMaster::Channel channel2{i2c, 0xf};
	I2cMaster::Buffer<16> buffer1{channel1};
	I2cMaster::Buffer<16> buffer2{channel2};
};

Drivers drivers;

extern "C" {
void I2C1_IRQHandler() {
	drivers.i2c.I2C_IRQHandler();
}
}
