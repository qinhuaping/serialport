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
#include "yuiwong/serialport.hpp" /* this header */
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "boost/make_shared.hpp"
#define NAME "yuiwongserialport"
namespace yuiwong
{
SerialPort::SerialPort(QueryMode const& queryMode): queryMode(queryMode)
{
	this->portSetting.baudRate = BaudRateType::_9600;
	this->portSetting.parityType = ParityType::NONE;
	this->portSetting.flowType = FlowType::OFF;
	this->portSetting.dataBits = DataBitsType::_8;
	this->portSetting.stopBitsType =  StopBitsType::_1;
	this->portSetting.timeoutMillisec = 10;
	this->portSettingDirty = static_cast<uint32_t>(Dirty::ALL);
	this->device.fd = -1;
	this->device.portName.assign("/dev/ttyUSB0");
	this->eventDrive.mainLoop = nullptr;
	this->eventDrive.running = false;
}
SerialPort::~SerialPort()
{
	this->close();
}
std::string SerialPort::getPortName() const
{
	boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
	(void)(rLock);
	return this->device.portName;
}
/**
 * set @a portName of the device associated with the object, e.g. "/dev/ttyS0"
 */
void SerialPort::setPortName(std::string const& portName)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	this->device.portName = portName;
}
void SerialPort::handlerw(
	struct ev_loop* /* mainLoop */,
	ev_io* rwio,
	int event)
{
	if (!rwio) {
		std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__ << ") "
			"no rwio\n";
		return;
	}
	SerialPort *const self = reinterpret_cast<SerialPort *>(rwio->data);
	if (!self) {
		std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__ << ") "
			"no instance\n";
		return;
	}
	{
		boost::shared_lock<boost::shared_mutex> rLock(self->rwlock);
		(void)(rLock);
		if (self->eventDrive.terminate) {
			std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__ <<
				") terminate called\n";
			return;
		}
	}
	if (event & EV_READ) {
		ssize_t const ret = self->shouldRecv(self->device.fd);
		if (ret < 0) {
			std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__ <<
				") recv fail: break\n";
			return;
		}
	}
	{
		boost::shared_lock<boost::shared_mutex> rLock(self->rwlock);
		(void)(rLock);
		if (self->eventDrive.terminate) {
			std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__ <<
				") terminate called\n";
			return;
		}
	}
	if (event & EV_WRITE) {
		ssize_t const ret = self->shouldSend(self->device.fd);
		if (ret < 0) {
			std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__ <<
				") send fail!\n";
		}
	}
}
void SerialPort::rwioloop()
{
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		this->eventDrive.running = true;
	}
	ev_run(this->eventDrive.mainLoop, 0);
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		this->eventDrive.running = false;
	}
}
bool SerialPort::isOpen() const
{
	boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
	(void)(rLock);
	return (this->device.fd >= 0);
}
/**
 * @brief
 * opens a serial port and sets its OpenFlag to @a openFlag (if not NotOpen)
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
int SerialPort::open(uint32_t const openFlag, bool const asyncSend)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if (this->__isOpen()) {
		return EBUSY;
	}
	if (openFlag == static_cast<uint32_t>(OpenFlag::NotOpen)) {
		return -EINVAL;
	}
	return this->__open(openFlag, asyncSend);
}
int SerialPort::__open(uint32_t const openFlag, bool const asyncSend)
{
	/** @note linux 2.6.21 seems to ignore O_NDELAY flag */
	int copenFlag = O_NOCTTY | O_NDELAY;
	if (openFlag & static_cast<uint32_t>(OpenFlag::ReadWrite)) {
		copenFlag |= O_RDWR;
	} else if (openFlag & static_cast<uint32_t>(OpenFlag::Read)) {
		copenFlag |= O_RDONLY;
	} else if (openFlag & static_cast<uint32_t>(OpenFlag::Write)) {
		copenFlag |= O_WRONLY;
	}
	if ((openFlag & static_cast<uint32_t>(OpenFlag::Write)) &&
		(openFlag & static_cast<uint32_t>(OpenFlag::Append))) {
		copenFlag |= O_APPEND;
	}
	int const fd = ::open(
		getFullPortName(this->device.portName).c_str(),
		/* O_RDWR | O_NOCTTY | O_NDELAY */ copenFlag);
	if (-1 == fd) {
		int const e = errno;
		std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__ << ") "
			"open `" << this->device.portName << "' fail " << strerror(e) <<
			"\n";
		if (e) {
			return -e;
		} else {
			return -EPERM;
		}
	}
	this->device.fd = fd;
	this->device.openFlag = openFlag;/* flag the port as opened */
	::tcgetattr(fd, &this->device.oldTermios);/* save the old termios */
	/* make a working copy */
	this->device.currentTermios = this->device.oldTermios;
	::cfmakeraw(&this->device.currentTermios);/* enable raw access */
	/* set up other port settings */
	this->device.currentTermios.c_cflag |= CREAD|CLOCAL;
	this->device.currentTermios.c_lflag &=
		(~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG));
	this->device.currentTermios.c_iflag &=
		(~(INPCK | IGNPAR | PARMRK | ISTRIP | ICRNL | IXANY));
	this->device.currentTermios.c_oflag &= (~OPOST);
	this->device.currentTermios.c_cc[VMIN] = 0;
	/* is a disable character available on this system? */
#	ifdef YUIWONGSERIALPORT_POSIXVDISABLE
	/*
	 * some systems allow for per-device disable-characters,
	 * so get the proper value for the configured device
	 */
	long const vdisable = ::fpathconf(fd, _PC_VDISABLE);
	this->device.currentTermios.c_cc[VINTR] = vdisable;
	this->device.currentTermios.c_cc[VQUIT] = vdisable;
	this->device.currentTermios.c_cc[VSTART] = vdisable;
	this->device.currentTermios.c_cc[VSTOP] = vdisable;
	this->device.currentTermios.c_cc[VSUSP] = vdisable;
#	endif /* YUIWONGSERIALPORT_POSIXVDISABLE */
	this->portSettingDirty = static_cast<uint32_t>(Dirty::ALL);
	this->__updatePortSettings();
	return this->__start(asyncSend);
}
/** @brief stop (if started) and close */
int SerialPort::close()
{
	this->stop();
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->device.fd > 0) {
			this->__flush();
			int const ret = ::close(this->device.fd);
			this->device.fd = -1;
			return ret;
		} else {
			return ENOENT;
		}
	}
}
int SerialPort::start(bool const asyncSend)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if (this->device.fd < 0) {
		std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__ << ") "
			<< "device not open open it now with rw\n";
		return this->__open(static_cast<uint32_t>(OpenFlag::ReadWrite));
	}
	return this->__start(asyncSend);
}
int SerialPort::__start(bool const asyncSend)
{
	if (QueryMode::EventDriven == this->queryMode) {
		if (this->eventDrive.mainLoop) {
			return EBUSY;
		}
		this->eventDrive.mainLoop = ev_default_loop(0);
		/* io 监控器的初始化 */
		ev_init(&this->eventDrive.rwio, handlerw);
		this->eventDrive.rwio.data = this;
		if (asyncSend && (this->device.openFlag &
			(static_cast<uint32_t>(OpenFlag::Write)))) {
			ev_io_set(
				&this->eventDrive.rwio,
				this->device.fd,
				EV_READ | EV_WRITE);
		} else {
			ev_io_set(
				&this->eventDrive.rwio,
				this->device.fd,
				EV_READ);
		}
		this->eventDrive.terminate = false;
		ev_io_start(this->eventDrive.mainLoop, &this->eventDrive.rwio);
		this->eventDrive.thread = boost::make_shared<boost::thread>(
			boost::bind(&SerialPort::rwioloop, this));
	}
	return 0;
}
int SerialPort::stop()
{
	{
		boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
		(void)(rLock);
		if (QueryMode::EventDriven != this->queryMode) {
			return 0;
		}
	}
	struct ev_loop* mainLoop;
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (!this->eventDrive.mainLoop) {
			return ENOENT;
		}
		this->eventDrive.terminate = true;
		mainLoop = this->eventDrive.mainLoop;
		this->eventDrive.mainLoop = nullptr;
		/*
		 * for one-shot events, one must manually stop the watcher
         * with its corresponding stop function
		 */
		ev_io_stop(mainLoop, &this->eventDrive.rwio);
		/* this causes all nested ev_run's to stop iterating */
		ev_break(mainLoop, EVBREAK_ALL);
		/*
		ev_clear_pending(mainLoop, &this->eventDrive.rwio);
		*/
	}
	{
		bool r;
		{
			r = this->eventDrive.running;
			boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
			(void)(rLock);
		}
		while (r) {
			usleep(10 * 1e3);
			{
				boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
				(void)(rLock);
				r = this->eventDrive.running;
			}
		}
		this->eventDrive.thread->join();
		{
			boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
			(void)(wLock);
			this->eventDrive.thread.reset();
		}
		ev_loop_destroy(/* EV_DEFAULT */ mainLoop);/* FIXME: release */
		mainLoop = nullptr;
		return 0;
	}
}
/** @brief override to send to fd when just fd writeable */
ssize_t SerialPort::shouldSend(int const fd)
{
	std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__ << ") "
		<< "defualt implements nothing to send to fd `" << fd << "' flush\n";
	this->flush();
	return 0;
}
ssize_t SerialPort::hasData() const
{
	boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
	(void)(rLock);
	return this->__hasData();
}
ssize_t SerialPort::__hasData() const
{
	long bytes;
	if (::ioctl(this->device.fd, FIONREAD, &bytes) == -1) {
		int const e = errno;
		std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__
			<< ") ioctl fail " << strerror(e) << "\n";
		if (e) {
			return -e;
		} else {
			return -1;
		}
	}
    return bytes;
}
ssize_t SerialPort::recvAll(std::vector<uint8_t>& buffer)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	ssize_t const avail = this->__hasData();
	if (avail <= 0) {
		return avail;
	}
	buffer.resize(avail);
	ssize_t const ret = ::read(this->device.fd, buffer.data(), avail);
	if (ret > 0) {
		buffer.resize(avail);
	} else {
		buffer.resize(0);
	}
	return ret;
}
ssize_t SerialPort::send(std::vector<uint8_t> const& buffer, size_t const each)
{
	return this->send(buffer.data(), buffer.size(), each);
}
ssize_t SerialPort::send(
	uint8_t const* const buffer,
	size_t const count,
	size_t const each)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if (this->device.fd < 0) {
		return -ENOENT;
	}
	if (!buffer) {
		return -EINVAL;
	}
	size_t eachw;
	if (each > count) {
		eachw = count;
	} else {
		eachw = each;
	}
	ssize_t ret;
	size_t sum = 0;
	while (sum < count) {
		if (eachw <= (count - sum)) {
			ret = ::write(this->device.fd, buffer + sum, eachw);
		} else {
			ret = ::write(this->device.fd, buffer + sum, count - sum);
		}
		if (ret != static_cast<ssize_t>(eachw)) {
			int const e = errno;
			std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__
				<< ") write fail " << strerror(e) << "\n";
			if (e) {
				return -e;
			} else {
				return -1;
			}
		}
		sum += static_cast<size_t>(ret);
	}
	return sum;
}
int SerialPort::flush()
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	return this->__flush();
}
int SerialPort::__flush()
{
	if (this->device.fd < 0) {
		return -ENODEV;
	}
	/* ::fsync(this->device.fd); */
	return ::tcdrain(this->device.fd);
}
void SerialPort::updatePortSettings()
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if ((!this->__isOpen()) || (!this->portSettingDirty)) {
		return;
	} else {
		return this->__updatePortSettings();
	}
}
void SerialPort::__updatePortSettings()
{
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::BaudRate)) {
		switch (this->portSetting.baudRate) {
		case BaudRateType::_50:
			baudRateToTermios(B50, this->device.currentTermios);
			break;
		case BaudRateType::_75:
			baudRateToTermios(B75, this->device.currentTermios);
			break;
		case BaudRateType::_110:
			baudRateToTermios(B110, this->device.currentTermios);
			break;
		case BaudRateType::_134:
			baudRateToTermios(B134, this->device.currentTermios);
			break;
		case BaudRateType::_150:
			baudRateToTermios(B150, this->device.currentTermios);
			break;
		case BaudRateType::_200:
			baudRateToTermios(B200, this->device.currentTermios);
			break;
		case BaudRateType::_300:
			baudRateToTermios(B300, this->device.currentTermios);
			break;
		case BaudRateType::_600:
			baudRateToTermios(B600, this->device.currentTermios);
			break;
		case BaudRateType::_1200:
			baudRateToTermios(B1200, this->device.currentTermios);
			break;
		case BaudRateType::_1800:
			baudRateToTermios(B1800, this->device.currentTermios);
			break;
		case BaudRateType::_2400:
			baudRateToTermios(B2400, this->device.currentTermios);
			break;
		case BaudRateType::_4800:
			baudRateToTermios(B4800, this->device.currentTermios);
			break;
		case BaudRateType::_9600:
			baudRateToTermios(B9600, this->device.currentTermios);
			break;
		case BaudRateType::_19200:
			baudRateToTermios(B19200, this->device.currentTermios);
			break;
		case BaudRateType::_38400:
			baudRateToTermios(B38400, this->device.currentTermios);
			break;
		case BaudRateType::_57600:
			baudRateToTermios(B57600, this->device.currentTermios);
			break;
#		if defined(YUIWONGSERIALPORT_B76800) && (YUIWONGSERIALPORT_B76800)
		case BaudRateType::_76800:
			baudRateToTermios(B76800, this->device.currentTermios);
			break;
#		endif
		case BaudRateType::_115200:
			baudRateToTermios(B115200, this->device.currentTermios);
			break;
#		if defined(YUIWONGSERIALPORT_B230400) && \
			defined(YUIWONGSERIALPORT_B4000000)
		case BaudRateType::_230400:
			baudRateToTermios(B230400, this->device.currentTermios);
			break;
		case BaudRateType::_460800:
			baudRateToTermios(B460800, this->device.currentTermios);
			break;
		case BaudRateType::_500000:
			baudRateToTermios(B500000, this->device.currentTermios);
			break;
		case BaudRateType::_576000:
			baudRateToTermios(B576000, this->device.currentTermios);
			break;
		case BaudRateType::_921600:
			baudRateToTermios(B921600, this->device.currentTermios);
			break;
		case BaudRateType::_1000000:
			baudRateToTermios(B1000000, this->device.currentTermios);
			break;
		case BaudRateType::_1152000:
			baudRateToTermios(B1152000, this->device.currentTermios);
			break;
		case BaudRateType::_1500000:
			baudRateToTermios(B1500000, this->device.currentTermios);
			break;
		case BaudRateType::_2000000:
			baudRateToTermios(B2000000, this->device.currentTermios);
			break;
		case BaudRateType::_2500000:
			baudRateToTermios(B2500000, this->device.currentTermios);
			break;
		case BaudRateType::_3000000:
			baudRateToTermios(B3000000, this->device.currentTermios);
			break;
		case BaudRateType::_3500000:
			baudRateToTermios(B3500000, this->device.currentTermios);
			break;
		case BaudRateType::_4000000:
			baudRateToTermios(B4000000, this->device.currentTermios);
			break;
#		endif
		default:
			std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__
				<< ") bad baudRate no update " <<
				static_cast<uint32_t>(this->portSetting.baudRate) << "\n";
			break;
		}
	}
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::Parity)) {
		switch (this->portSetting.parityType) {
		case ParityType::SPACE:
			/*
			 * space parity not directly supported - add an extra data bit to
			 * simulate it
			 */
			this->portSettingDirty |= static_cast<uint32_t>(Dirty::DataBits);
			break;
		case ParityType::NONE:
			this->device.currentTermios.c_cflag &= (~PARENB);
			break;
		case ParityType::EVEN: {
			this->device.currentTermios.c_cflag &= (~PARODD);
			this->device.currentTermios.c_cflag |= PARENB;
		} break;
		case ParityType::ODD:
			this->device.currentTermios.c_cflag |= (PARENB|PARODD);
			break;
		}
	}
	/* must after parity settings */
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::DataBits)) {
		if (this->portSetting.parityType != ParityType::SPACE) {
			this->device.currentTermios.c_cflag &= (~CSIZE);
			switch(this->portSetting.dataBits) {
			case DataBitsType::_5:
				this->device.currentTermios.c_cflag |= CS5;
				break;
			case DataBitsType::_6:
				this->device.currentTermios.c_cflag |= CS6;
				break;
			case DataBitsType::_7:
				this->device.currentTermios.c_cflag |= CS7;
				break;
			case DataBitsType::_8:
				this->device.currentTermios.c_cflag |= CS8;
				break;
			}
		} else {
			/*
			 * space parity not directly supported - add an extra data bit to
			 * simulate it
			 */
			this->device.currentTermios.c_cflag &= ~(PARENB|CSIZE);
			switch(this->portSetting.dataBits) {
			case DataBitsType::_5:
				this->device.currentTermios.c_cflag |= CS6;
				break;
			case DataBitsType::_6:
				this->device.currentTermios.c_cflag |= CS7;
				break;
			case DataBitsType::_7:
				this->device.currentTermios.c_cflag |= CS8;
				break;
			case DataBitsType::_8:
				/* this will never happen, put here to suppress an warning */
				std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__
					<< ") DataBits _8 no update\n";
				break;
			}
		}
	}
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::StopBits)) {
		switch (this->portSetting.stopBitsType) {
		case StopBitsType::_1:
			this->device.currentTermios.c_cflag &= (~CSTOPB);
			break;
		case StopBitsType::_2:
			this->device.currentTermios.c_cflag |= CSTOPB;
			break;
		}
	}
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::Flow)) {
		switch(this->portSetting.flowType) {
		case FlowType::OFF:
			this->device.currentTermios.c_cflag &= (~CRTSCTS);
			this->device.currentTermios.c_iflag &= (~(IXON|IXOFF|IXANY));
			break;
		case FlowType::XONXOFF:
			/* software (XON/XOFF) flow control */
			this->device.currentTermios.c_cflag &= (~CRTSCTS);
			this->device.currentTermios.c_iflag |= (IXON|IXOFF|IXANY);
			break;
		case FlowType::HARDWARE:
			this->device.currentTermios.c_cflag |= CRTSCTS;
			this->device.currentTermios.c_iflag &= (~(IXON|IXOFF|IXANY));
			break;
		}
	}

	/* if any thing in currentTermios changed, flush */
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::SettingMask)) {
		::tcsetattr(this->device.fd, TCSAFLUSH, &this->device.currentTermios);
	}
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::TimeOut)) {
		long const millisec = this->portSetting.timeoutMillisec;
		if (millisec == -1) {
			::fcntl(this->device.fd, F_SETFL, O_NDELAY);
		} else {
			/*
			 * O_SYNC should enable blocking ::write()
			 * however this seems not working on Linux 2.6.21
			 * (works on OpenBSD 4.2)
			 */
			::fcntl(this->device.fd, F_SETFL, O_SYNC);
		}
		::tcgetattr(this->device.fd, &this->device.currentTermios);
		this->device.currentTermios.c_cc[VTIME] = millisec / 100;
		::tcsetattr(this->device.fd, TCSAFLUSH, &this->device.currentTermios);
	}
	this->portSettingDirty = 0;
}
std::string SerialPort::getFullPortName(std::string const& portName)
{
	char const* const p = strstr(portName.c_str(), "/");
	if (p && (p == portName.c_str())) {
		return portName;
	} else {
		return "/dev/" + portName;
	}
}
struct termios& SerialPort::baudRateToTermios(
	uint32_t const& baudRate,
	struct termios& ter)
{
#	ifdef CBAUD
	ter.c_cflag &= (~CBAUD);
	ter.c_cflag |= baudRate;
#	else
	::cfsetispeed(&ter, baudRate);
	::cfsetospeed(&ter, baudRate);
#	endif
	return ter;
}
}