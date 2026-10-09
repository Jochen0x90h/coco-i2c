#include <coco/convert.hpp>
#include <coco/debug.hpp>
#include <coco/PackedValue.hpp>
#include <I2cTargetTest.hpp>


using namespace coco;

// emulated eeprom
struct {
    uint8_t data[1024] = {0x00, 0x11, 0x22, 0x33, 0x44};
    int address = 0;
} ee;


Coroutine target(Loop &loop, Drivers::I2c::TargetChannel &channel, Buffer &buffer) {
    while (buffer.ready()) {
        debug::toggleGreen();

        debug::out << "wait request\n";
        co_await channel.untilRequest();

        auto op = channel.requestedOp();
        debug::out << "request " << dec(op) << "\n";

        if (op == Buffer::Op::READ) {
            // target (master) reads data, controller (master) writes address followed by data
            co_await buffer.read();

            debug::out << "target read: ";
            for (int i = 0; i < buffer.size(); i++) {
                debug::out << "0x" << hex(buffer.data()[i]) << " ";
            }
            debug::out << "\n";

            ee.address = buffer.cast<U16B>();
            debug::out << "ee address: " << hex(ee.address, 4) << '\n';
            for (int i = 2; i < buffer.size(); i++) {
                ee.data[ee.address & 1023] = buffer.data()[i];
                ++ee.address;
            }
        } else if (op == Buffer::Op::WRITE) {
            // target (slave) writes data, controller (master) reads data

/*
            // write in one go
            debug::out << "target write " << dec(buffer.capacity()) << "\n";
            for (int i = 0; i < buffer.capacity(); i++) {
                buffer.data()[i] = ee.data[(ee.address + i) & 1023];
            }
            co_await buffer.write(buffer.capacity());
            debug::out << "target written: " << dec(buffer.size()) << '\n';
            ee.address += buffer.size();
*/

            // write in several sections
            do {
                for (int i = 0; i < 2; i++) {
                    buffer.data()[i] = ee.data[(ee.address + i) & 1023];
                }
                co_await buffer.write(2);
                debug::out << "target written: " << dec(buffer.size()) << '\n';
                ee.address += buffer.size();
            } while (buffer.size() == 2);

        }

    }
}

const uint8_t command[] = {0x00, 0x01};
const uint8_t data[] = {0x37, 0x13};

Coroutine controller(Loop &loop, Buffer &buffer) {
    //while (buffer.ready()) {
        buffer.setHeader(command);
        debug::out << "controller write\n";

        // write
        //co_await buffer.write(data);
        //debug::out << "controller written\n";

        // read
        debug::out << "controller read\n";
        co_await buffer.read(4);
        debug::out << "controller read: ";
        for (int i = 0; i < buffer.size(); i++) {
            debug::out << "0x" << hex(buffer.data()[i]) << " ";
        }
        debug::out << "\n";


        co_await loop.sleep(1s);
        debug::toggleBlue();
    //}
}


int main() {
    debug::out << "\nI2cTargetTest\n";

    target(drivers.loop, drivers.targetChannel, drivers.targetBuffer);

    controller(drivers.loop, drivers.controllerBuffer);

    drivers.loop.run();
    return 0;
}
