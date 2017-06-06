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
#ifndef YUIWONGSERIALPORT_SERIALPORT_HPP
#define YUIWONGSERIALPORT_SERIALPORT_HPP
#include <stdint.h> /* uint8_t .. */
#include <termios.h> /* termios */
#include <sys/types.h> /* ssize_t */
#include "ev++.h" /* ev_io */
#include <string> /* string */
#include "boost/thread.hpp"
#include "boost/thread/shared_mutex.hpp" /*  shared_mutex */
namespace yuiwong {
#if defined(B76800) && (!defined(YUIWONGSERIALPORT_B76800))
#	define YUIWONGSERIALPORT_B76800 1
#endif
#ifndef YUIWONGSERIALPORT_B76800
#	define YUIWONGSERIALPORT_B76800 0
#endif
#if defined(B230400) && (!defined(YUIWONGSERIALPORT_B230400))
#	define YUIWONGSERIALPORT_B230400 1
#endif
#if defined(B4000000) && (!defined(YUIWONGSERIALPORT_B4000000))
#	define YUIWONGSERIALPORT_B4000000 1
#endif
#if (!defined(YUIWONGSERIALPORT_B230400)) && \
	(!defined(YUIWONGSERIALPORT_B4000000))
#	define YUIWONGSERIALPORT_B230400 1
#	define YUIWONGSERIALPORT_B4000000 1
#endif
#if defined(_POSIX_VDISABLE) && (!defined(YUIWONGSERIALPORT_POSIXVDISABLE))
#	define YUIWONGSERIALPORT_POSIXVDISABLE _POSIX_VDISABLE
#endif
#ifndef YUIWONGSERIALPORT_POSIXVDISABLE
#	define YUIWONGSERIALPORT_POSIXVDISABLE '\0'
#endif
#define YUIWONGSERIALPORT_DEFAULTREADBUFSIZE (sizeof(size_t) * 1024)
enum class BaudRate: uint32_t {
	_50 = 50,/* posix only */
	_75 = 75,/* posix only */
	_134 = 134,/* posix only */
	_150 = 150,/* posix only */
	_200 = 200,/* posix only */
	_1800 = 1800,/* posix only */
#	if defined(YUIWONGSERIALPORT_B76800) && (YUIWONGSERIALPORT_B76800)
	_76800 = 76800,/* posix only */
#	endif
#	if defined(YUIWONGSERIALPORT_B230400) && \
	defined(YUIWONGSERIALPORT_B4000000)
	_230400 = 230400,/* posix only */
	_460800 = 460800,/* posix only */
	_500000 = 500000,/* posix only */
	_576000 = 576000,/* posix only */
	_921600 = 921600,/* posix only */
	_1000000 = 1000000,/* posix only */
	_1152000 = 1152000,/* posix only */
	_1500000 = 1500000,/* posix only */
	_2000000 = 2000000,/* posix only */
	_2500000 = 2500000,/* posix only */
	_3000000 = 3000000,/* posix only */
	_3500000 = 3500000,/* posix only */
	_4000000 = 4000000,/* posix only */
#	endif
	_110 = 110,
	_300 = 300,
	_600 = 600,
	_1200 = 1200,
	_2400 = 2400,
	_4800 = 4800,
	_9600 = 9600,
	_19200 = 19200,
	_38400 = 38400,
	_57600 = 57600,
	_115200 = 115200
};
enum class DataBit: uint8_t {
	_5 = 5,
	_6 = 6,
	_7 = 7,
	_8 = 8
};
enum class ParityType: int {
	NONE,
	ODD,
	EVEN,
	SPACE
};
enum class StopBit: int {
	_1,
	_2
};
enum class FlowControl: int {
	OFF,
	HARDWARE,
	XONXOFF
};
/**
 * @name PortSetting
 * structure to contain port settings
 */
struct PortSetting {
	BaudRate baudRate;
	DataBit dataBit;
	ParityType parityType;
	StopBit stopBit;
	FlowControl flowControl;
	ssize_t timeoutMillisec;
};
class SerialPort {
public:
	enum class QueryMode {
		Polling,
		EventDriven
	};
	enum class Dirty: uint32_t {
		BaudRate = 0x0001,
		Parity = 0x0002,
		StopBit = 0x0004,
		DataBit = 0x0008,
		Flow = 0x0010,
		TimeOut = 0x0100,
		ALL = 0x0fff,
		SettingMask = 0x00ff,/* without TimeOut */
	};
	enum class OpenFlag: uint32_t {
		NotOpen = 0x0000,
		Read = 0x0004,
		Write = 0x0002,
		ReadWrite = Read | Write,
		Append = 0x0008,
		Truncate = 0x0010,
		Text = 0x0020,
	};
	/**
	 * @brief ctor / other params use default
	 * @param queryMode QueryMode const& default EventDriven
	 */
	SerialPort(QueryMode const& queryMode = QueryMode::EventDriven);
	/**
	 * @brief ctor / other params use default
	 * @param portName std::string const&: serial port name
	 * @param queryMode QueryMode const& default EventDriven
	 */
	SerialPort(
		std::string const& portName,
		QueryMode const& queryMode = QueryMode::EventDriven);
	SerialPort(
		PortSetting const& portSetting,
		QueryMode const& mode = QueryMode::EventDriven);
	SerialPort(
		std::string const& portName,
		PortSetting const& portSetting,
		QueryMode const& queryMode = QueryMode::EventDriven);
	~SerialPort();
	inline QueryMode const& getQueryMode() const { return this->queryMode; }
	std::string getPortName() const;
	void setPortName(std::string const& portName);
	BaudRate getBaudRate() const;
	void setBaudRate(BaudRate const& baudRate, bool const update = true);
	/** @brief set callback rate for read when EventDriven */
	bool setRate(double const rate);
	bool isOpen() const;
	/**
	 * @brief
	 * opens a serial port and sets its OpenFlag to @a openFlag
	 * (if not NotOpen)
	 * @param readBufferSz when EventDriven and @a readBufferSz > 0 read
	 * buffer will be alloced
	 * @note that this function does not specify which device to open
	 * @return
	 * - return 0 if no any error
	 * - return <0 (-errno) when fail
	 * - return >0 (errno) when alreay done
	 * @details
	 * - this function has no effect if the port associated with the class is
	 *   already open and return is EBUSY
	 * - the port is also configured to the current settings,
	 *   as stored in the settings structure
	 * - if do open start will be called
	 */
	int open(
		uint32_t const openFlag = static_cast<uint32_t>(OpenFlag::ReadWrite),
		bool const asyncSend = false,
		size_t const readBufferSz = YUIWONGSERIALPORT_DEFAULTREADBUFSIZE);
	/** @brief start event loop when EventDriven also open if not */
	int start(
		bool const asyncSend = false,
		size_t const readBufferSz = YUIWONGSERIALPORT_DEFAULTREADBUFSIZE);
	/** @brief stop (if started) and close */
	int close();
	int stop();
	/**
	 * @brief override to read data when EventDriven
	 * e.x. use recv or recvAll
	 * @note
	 * - if rate enabled shouldRecv be callbacked when cache full or
	 * no cache or time to rate.
	 * - if rate disabled shouldRecv be callbacked once data available
	 * @return >= 0 when success else fail and will not callback shouldSend
	 */
	virtual ssize_t shouldRecv(size_t const maxAvailable);
	/**
	 * @brief override to send to fd when just fd writeable
	 * when EventDriven
	 * e.x. use send to send
	 */
	virtual ssize_t shouldSend();
	ssize_t hasData() const;
	/**
	 * @param buffer std::vector<uint8_t>&: should has min @a maxRecv capacity
	 */
	ssize_t recv(
		std::vector<uint8_t>& buffer,
		size_t const maxRecv = YUIWONGSERIALPORT_DEFAULTREADBUFSIZE * 2,
		long const timeoutMillisec = -1);
	std::vector<uint8_t> recv(
		size_t const maxRecv = YUIWONGSERIALPORT_DEFAULTREADBUFSIZE * 2,
		long const timeoutMillisec = -1);
	ssize_t recvAll(std::vector<uint8_t>& buffer);
	ssize_t send(
		std::vector<uint8_t> const& buffer,
		size_t const each = SIZE_MAX);
	ssize_t send(
		uint8_t const* const buffer,
		size_t const count,
		size_t const each = SIZE_MAX);
	int flush();
	void updatePortSettings();
	static std::string getFullPortName(std::string const& portName);
	static struct termios& baudRateToTermios(
		uint32_t const& baudRate,
		struct termios& ter);
private:
	static void rwcallback(struct ev_loop* mainLoop, ev_io* rwio, int event);
	void rwioloop();
	inline bool __isOpen() const { return this->device.fd >= 0; }
	/**
	 * @name __open
	 * @brief real open
	 */
	int __open(
		uint32_t const openFlag = static_cast<uint32_t>(OpenFlag::ReadWrite),
		bool const asyncSend = false,
		size_t const readBufferSz = YUIWONGSERIALPORT_DEFAULTREADBUFSIZE);
	/** @brief real start */
	int __start(
		bool const asyncSend = false,
		size_t const readBufferSz = YUIWONGSERIALPORT_DEFAULTREADBUFSIZE);
	ssize_t __hasData() const;
	int __flush();
	void __updatePortSettings();
	enum class RecvMode {
		Free,
		User,
	};
	struct EventDriven {
		bool running;
		bool terminate;
		double rate;/* if 0.0 as fast as possible */
		double lastCallbackRecv;/* nsec if < 0 not need update */
		RecvMode recvMode;
		ev_io rwio;
		struct ev_loop* mainLoop;
		boost::shared_ptr<boost::thread> thread;
		boost::shared_ptr<std::vector<uint8_t> > readBuffer;
		EventDriven(size_t const readBufferSz):
			running(false),
			terminate(false),
			rate(0.0),
			lastCallbackRecv(-1.0),
			recvMode(RecvMode::Free),
			mainLoop(nullptr),
			thread(nullptr),
			readBuffer((readBufferSz <= 0) ? nullptr :
				new std::vector<uint8_t>(readBufferSz, 0)) {
				if (this->readBuffer) {
					this->readBuffer->resize(0);
				}
			}
	};
	mutable boost::shared_mutex rwlock;
	QueryMode const queryMode;
	boost::shared_ptr<EventDriven> eventDriven;
	PortSetting portSetting;
	uint32_t portSettingDirty;
	struct Device {
		int fd;/* serial device fd */
		std::string portName;
		uint32_t openFlag;
		struct termios oldTermios;
		struct termios currentTermios;
		std::vector<uint8_t> buffer;
		Device(): fd(-1), portName(""), openFlag(0) {}
		~Device() {}
	} device;
};
}
#endif