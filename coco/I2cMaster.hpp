#pragma once

#include <coco/BufferDevice.hpp>


namespace coco {

/// @brief Inter-integrated-circuit (I2C) abstraction.
///
class I2cMaster {
public:

    virtual ~I2cMaster() {}

    /// @brief Recover the I2C bus by sending stat/stop condition.
    ///
    virtual void recover() = 0;


    using Buffer = coco::Buffer;
};

} // namespace coco
