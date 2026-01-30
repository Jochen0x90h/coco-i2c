#include <I2cEepromTest.hpp>
#include <coco/StreamOperators.hpp>
#include <coco/debug.hpp>


using namespace coco;


/*
    This test writes data into an EEPROM (e.g. 24LC16) and reads it back
*/

const uint8_t data[] = {0x33, 0x55, 0xAA};

Coroutine test(Loop &loop, Buffer &buffer) {
    // wait until the buffer is ready (i.e. the device the buffer belongs to is ready)
    co_await buffer.untilReady();

    co_await loop.sleep(1s);

    // set write address
    buffer.setHeader<uint8_t>(0);

    // write data
    debug::out << "Write ";
    co_await buffer.writeArray(data);

    // indicate that data has been written
    if (buffer.result() == Buffer::Result::SUCCESS) {
        debug::out << "Success!\n";
        debug::set(debug::BLUE);
    } else {
        debug::out << "Error!\n";
        debug::set(debug::RED);
        co_return;
        loop.exit();
    }

    // try to read data until the eeprom is ready
    for (int i = 1; i <= 100; ++i) {
        co_await loop.sleep(1ms);

        // set read address
        buffer.setHeader<uint8_t>(0);

        // read data
        co_await buffer.read(3);

        if (buffer.result() == Buffer::Result::SUCCESS) {
            debug::out << "Read success after " << dec(i) << " tries\n";
            break;
        }
    }

    // check
    debug::out << "Check\n";
    if (buffer.array<uint8_t>() == Array<const uint8_t>(data)) {
        debug::out << "Success!\n";
        debug::set(debug::GREEN);
    } else {
        debug::out << "Error!\n";
        if (buffer.size() == 0)
            debug::set(debug::RED);
        else
            debug::set(debug::MAGENTA);
    }

    loop.exit();
}


int main() {
    debug::out << "I2cEepromTest\n";

    test(drivers.loop, drivers.buffer1);

    drivers.loop.run();
    return 0;
}
