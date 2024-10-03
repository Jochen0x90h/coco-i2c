#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/Frequency.hpp>
#include <coco/String.hpp>
#include <cstdint>


namespace coco {

/**
	Inter-integrated-circuit (I2C) abstraction.
*/
class I2cMaster {
public:

	virtual ~I2cMaster() {}

	/**
		Recover the I2C bus by sending stat/stop condition.
	*/
	virtual void recover() = 0;


	using Buffer = coco::Buffer;
};

} // namespace coco
