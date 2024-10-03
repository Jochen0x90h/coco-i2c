#include "I2cMaster_cout.hpp"
#include <iostream>


namespace coco {

I2cMaster_cout::~I2cMaster_cout() {
}

void I2cMaster_cout::recover() {
    std::cout << "recover" << std::endl;
}

} // namespace coco
