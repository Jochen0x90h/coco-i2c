#include <coco/debug.hpp>
#include <I2cMasterTest.hpp>


using namespace coco;

Coroutine recover(Loop &loop, I2cMaster &master) {
	while (true) {
		master.recover();
		//master.recover();

		debug::toggleGreen();
		co_await loop.sleep(1s);
	}
}

Coroutine read(Loop &loop, Buffer &buffer) {
	while (buffer.ready()) {
		co_await buffer.read(10);

		co_await loop.sleep(1s);
		debug::toggleGreen();
	}
}


const uint8_t command[] = {0x00, 0xff};
const uint8_t data[] = {0x33, 0x55};

Coroutine write(Loop &loop, Buffer &buffer) {
	while (buffer.ready()) {
		buffer.setHeader(command);
		co_await buffer.writeArray(data);

		co_await loop.sleep(1s);
		debug::toggleBlue();
	}
}


int main() {
	//recover(drivers.loop, drivers.i2c);

	read(drivers.loop, drivers.buffer1);
	write(drivers.loop, drivers.buffer2);

	drivers.loop.run();
	return 0;
}
