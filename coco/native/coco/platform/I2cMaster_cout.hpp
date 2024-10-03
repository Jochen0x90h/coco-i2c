#include <coco/I2cMaster.hpp>


namespace coco {

/**
    Dummy implementation of an I2C master that simply writes to std::cout
    Use in conjunction with BufferDevice_cout (coco/platform/BufferDevice_cout.hpp)
*/
class I2cMaster_cout : public I2cMaster {
public:
    ~I2cMaster_cout() override;

    // I2cMaster methods
    void recover() override;
};

} // namespace coco
