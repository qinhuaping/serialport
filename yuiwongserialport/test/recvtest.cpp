/* ========================================================================
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ======================================================================== */
#include <stdio.h>
#include "yuiwong/serialport.hpp"
#define NAME "yuiwongserialporttest"
#define DEVICE "/dev/ttyUSB0"
namespace
{
class TestSerialPort: public yuiwong::SerialPort {
public:
	TestSerialPort(): yuiwong::SerialPort() {}
	~TestSerialPort() {}
	virtual ssize_t shouldRecv(size_t const maxAvailable) override {
		ssize_t const ret = this->recvAll(this->buffer);
		std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__
			<< ") recvAll ret " << ret << "\n";
		if (ret == static_cast<ssize_t>(maxAvailable)) {
			return ret;
		} else {
			return -1;
		}
	}
private:
	std::vector<uint8_t> buffer;
};
static void case1(int argc, char** argv)
{
	TestSerialPort serialPort;
	int ret;
	if (argc < 2) {
		std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__
			<< ") open default device " DEVICE "\n";
		serialPort.setPortName(DEVICE);
		ret = serialPort.open();
		std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__
			<< ") open ret " << ret << "\n";
		usleep(1e6);
		ret = serialPort.close();
		std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__
			<< ") close ret " << ret << "\n";
	} else {
		for (int i = 1; i < argc; ++i) {
			serialPort.setPortName(argv[i]);
			ret = serialPort.open();
			std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__
				<< ") open ret " << ret << "\n";
			usleep(1e6);
		}
		serialPort.close();
	}
	std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__ <<
		") will done\n";
	fflush(stdout);
	fflush(stderr);
	usleep(1e6);
}
static void case2()
{
	TestSerialPort serialPort;
	serialPort.setPortName(DEVICE);
	serialPort.open();
	usleep(50e3);
	serialPort.close();
	serialPort.open();
	usleep(50e3);
	serialPort.close();
	std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__ <<
		") will done\n";
	fflush(stdout);
	fflush(stderr);
	usleep(1e6);
}
}
int main(int argc, char** argv)
{
	case1(argc, argv);
	case2();
	return 0;
}