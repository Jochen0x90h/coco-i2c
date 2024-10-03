#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/BufferDevice_cout.hpp>
#include <coco/platform/I2cMaster_cout.hpp>


using namespace coco;


// drivers for I2cMasterTest
struct Drivers {
	Loop_native loop;

	using Device = BufferDevice_cout;
	I2cMaster_cout i2c;
	Device channel1{loop, "channel1"};
	Device channel2{loop, "channel2"};
	Device::Buffer buffer1{16, channel1};
	Device::Buffer buffer2{16, channel2};
};

Drivers drivers;
