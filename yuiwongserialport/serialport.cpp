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
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "boost/make_shared.hpp"
#define NAME "yuiwongserialport"
namespace yuiwong
{
static double NowNanosec()
{
	timespec tp;
	if (!clock_gettime(CLOCK_MONOTONIC, &tp)) {
		return (tp.tv_nsec + (tp.tv_sec * 1e9));
	} else {
		return 0.0;
	}
}
/**
 * @brief ctor / other params use default
 * @param queryMode QueryMode const& default EventDriven
 */
SerialPort::SerialPort(QueryMode const& queryMode):
	queryMode(queryMode),
	eventDriven(nullptr)
{
	this->portSetting.baudRate = BaudRate::_115200;
	this->portSetting.parityType = ParityType::NONE;
	this->portSetting.flowControl = FlowControl::OFF;
	this->portSetting.dataBit = DataBit::_8;
	this->portSetting.stopBit = StopBit::_1;
	this->portSetting.timeoutMillisec = 10;
	this->portSettingDirty = static_cast<uint32_t>(Dirty::ALL);
	this->device.fd = -1;
	this->device.portName.assign("/dev/ttyUSB0");
}
/**
 * @brief ctor / other params use default
 * @param portName std::string const&: serial port name
 * @param queryMode QueryMode const& default EventDriven
 */
SerialPort::SerialPort(
	std::string const& portName,
	QueryMode const& queryMode):
	queryMode(queryMode),
	eventDriven(nullptr)
{
	this->portSetting.baudRate = BaudRate::_115200;
	this->portSetting.parityType = ParityType::NONE;
	this->portSetting.flowControl = FlowControl::OFF;
	this->portSetting.dataBit = DataBit::_8;
	this->portSetting.stopBit = StopBit::_1;
	this->portSetting.timeoutMillisec = 10;
	this->portSettingDirty = static_cast<uint32_t>(Dirty::ALL);
	this->device.fd = -1;
	this->device.portName = portName;
}
SerialPort::SerialPort(
	std::string const& portName,
	PortSetting const& portSetting,
	QueryMode const& queryMode):
	queryMode(queryMode),
	eventDriven(nullptr),
	portSetting(portSetting),
	portSettingDirty(static_cast<uint32_t>(Dirty::ALL))
{
	this->portSettingDirty = static_cast<uint32_t>(Dirty::ALL);
	this->device.fd = -1;
	this->device.portName = portName;
}
SerialPort::~SerialPort()
{
	this->close();
	std::cout << "[" NAME "][INFO](" << __FILE__ << "+" << __LINE__
		<< ") ~SerialPort\n";
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
BaudRate SerialPort::getBaudRate() const
{
	boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
	(void)(rLock);
	return this->portSetting.baudRate;
}
void SerialPort::setBaudRate(BaudRate const& baudRate, bool const update)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if (baudRate == this->portSetting.baudRate) {
		return;
	}
	switch(baudRate) {
	case BaudRate::_50:
	case BaudRate::_75:
	case BaudRate::_134:
	case BaudRate::_150:
	case BaudRate::_200:
	case BaudRate::_1800:
#	if defined(YUIWONGSERIALPORT_B76800) && (YUIWONGSERIALPORT_B76800)
	case BaudRate::_76800:
#	endif
#	if defined(YUIWONGSERIALPORT_B230400) && \
		defined(YUIWONGSERIALPORT_B4000000)
	case BaudRate::_230400:
	case BaudRate::_460800:
	case BaudRate::_500000:
	case BaudRate::_576000:
	case BaudRate::_921600:
	case BaudRate::_1000000:
	case BaudRate::_1152000:
	case BaudRate::_1500000:
	case BaudRate::_2000000:
	case BaudRate::_2500000:
	case BaudRate::_3000000:
	case BaudRate::_3500000:
	case BaudRate::_4000000:
#	endif
	case BaudRate::_110:
	case BaudRate::_300:
	case BaudRate::_600:
	case BaudRate::_1200:
	case BaudRate::_2400:
	case BaudRate::_4800:
	case BaudRate::_9600:
	case BaudRate::_19200:
	case BaudRate::_38400:
	case BaudRate::_57600:
	case BaudRate::_115200: {
		this->portSetting.baudRate = baudRate;
		this->portSettingDirty |= static_cast<uint32_t>(Dirty::BaudRate);
		if (update && this->__isOpen()) {
			this->__updatePortSettings();
		}
	} break;
	default:
		std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__ << ") "
			"does not support baudRate " << static_cast<uint32_t>(baudRate) <<
			"\n";
	}
}
/** @brief set callback rate for read when EventDriven */
bool SerialPort::setRate(double const rate)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if (!this->eventDriven) {
		return false;
	}
	this->eventDriven->rate = fabs(rate);
	if (fabs(rate) >= 1e-6) {
		this->eventDriven->lastCallbackRecv = 0.0;
	} else {
		this->eventDriven->lastCallbackRecv = -1.0;
	}
	return true;
}
void SerialPort::rwcallback(struct ev_loop*, ev_io* rwio, int event)
{
	if (!rwio) {
		return;
	}
	SerialPort *const self = reinterpret_cast<SerialPort *>(rwio->data);
	if (!self) {
		return;
	}
	boost::shared_ptr<EventDriven> eventDriven;
	/* chk terminate */
	{
		boost::shared_lock<boost::shared_mutex> rLock(self->rwlock);
		(void)(rLock);
		eventDriven = self->eventDriven;
		if ((!eventDriven) || self->eventDriven->terminate) {
			std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__ <<
				") terminate called\n";
			return;
		}
	}
	if (event & EV_READ) {
		bool callbackable = false;
		/* cache to buffer when buffer enabled */
		{
			boost::upgrade_lock<boost::shared_mutex> uplock(self->rwlock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
			(void)(wLock);
			if (eventDriven->readBuffer) {
				size_t const oldsz = eventDriven->readBuffer->size();
				ssize_t const bufsz =
					eventDriven->readBuffer->capacity() - oldsz;
				if (bufsz > 0) {
					/* pre size .. max */
					eventDriven->readBuffer->resize(
						eventDriven->readBuffer->capacity());
					ssize_t const r = ::read(
						self->device.fd,
						eventDriven->readBuffer->data() + oldsz,
						bufsz);
					/* narrow size */
					if (r <= 0) {
						eventDriven->readBuffer->resize(oldsz);
					} else {
						eventDriven->readBuffer->resize(oldsz + r);
					}
				}
				if (eventDriven->readBuffer->size() >=
					eventDriven->readBuffer->capacity()) {
					callbackable = true;/* cache full */
				}
			}
		}
		ssize_t maxAvailable = 0;
		bool updateTime;
		/* chk and setup for callback */
		{
			boost::shared_lock<boost::shared_mutex> rLock(self->rwlock);
			(void)(rLock);
			updateTime = (fabs(eventDriven->rate) >= 1e-6);
			if (!callbackable) {
				/* cache not full or no cache */
				if (!updateTime) {
					/* cache not full or no cache and no limit rate */
					callbackable = true;
				} else {
					/* cache not full or no cache and has limit rate */
					double const each = (1.0 / fabs(eventDriven->rate)) * 1e9;
					double const nownsec = NowNanosec();
					if ((nownsec - eventDriven->lastCallbackRecv) >= each) {
						callbackable = true;
					}
				}
			}
			if (callbackable) {
				if (eventDriven->recvMode != RecvMode::Free) {
					/* cache full / timeup, and not free */
					callbackable = false;
				} else {
					maxAvailable = self->__hasData();
				}
			}
		}
		/* callback when ok */
		if (callbackable && (maxAvailable >= 0)) {
			ssize_t const ret = self->shouldRecv(maxAvailable);
			/* chk and update time */
			if (updateTime) {
				boost::upgrade_lock<boost::shared_mutex> uplock(self->rwlock);
				boost::upgrade_to_unique_lock<boost::shared_mutex>
					wLock(uplock);
				(void)(wLock);
				double const nownsec = NowNanosec();
				eventDriven->lastCallbackRecv = nownsec;
			}
			/* chk ret */
			if (ret < 0) {
				std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" <<
					__LINE__ << ") recv fail: break\n";
				return;
			}
		}
		if (maxAvailable < 0 && self->device.fd >= 0) {
			{
				boost::upgrade_lock<boost::shared_mutex> uplock(self->rwlock);
				boost::upgrade_to_unique_lock<boost::shared_mutex>
					wLock(uplock);
				(void)(wLock);
				::close(self->device.fd);
				self->device.fd = -1;
			}
			ev_io_stop(eventDriven->mainLoop, &eventDriven->rwio);
			ev_break(eventDriven->mainLoop, EVBREAK_ALL);
		}
	}
	if (event & EV_WRITE) {
		self->shouldSend();
	}
}
void SerialPort::rwioloop()
{
	boost::shared_ptr<EventDriven> eventDriven;
	{
		boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
		(void)(rLock);
		eventDriven = this->eventDriven;
	}
	while (!eventDriven->terminate) {
		if (this->device.fd >= 0) {
			std::cerr << __LINE__ << " loop\n";
			ev_run(eventDriven->mainLoop, 0);
		}
		if (!eventDriven->terminate) {
			std::cerr << __LINE__ << " TRY REOPEN `" <<
				getFullPortName(this->device.portName) <<
				"' each second ...\n";
		}
		if (this->device.fd >= 0) {
			boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
			(void)(wLock);
			::close(this->device.fd);
			this->device.fd = -1;
		}
		double lastNanoSec = 0;
		while (!eventDriven->terminate) {
			if ((NowNanosec() - lastNanoSec) >= 1e9) {
				boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
				boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(
					uplock);
				(void)(wLock);
				this->device.fd = ::open(
					getFullPortName(this->device.portName).c_str(),
					this->device.openFlag);
				/* FIXME notify to release device when open fail */
				if (this->device.fd >= 0) {
					ev_io_set(
						&eventDriven->rwio,
						this->device.fd,
						this->device.ioFlag);
					ev_io_start(
						eventDriven->mainLoop,
						&eventDriven->rwio);
					break;
				}
				lastNanoSec = NowNanosec();
			}
			::usleep(10e3);
		}
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
 * @param readBufferSz when EventDriven and @a readBufferSz > 0 read
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
int SerialPort::open(
	uint32_t const openFlag,
	bool const asyncSend,
	size_t const readBufferSz)
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
	return this->__open(openFlag, asyncSend, readBufferSz);
}
/**
 * @name __open
 * @brief real open
 */
int SerialPort::__open(
	uint32_t const openFlag,
	bool const asyncSend,
	size_t const readBufferSz)
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
	return this->__start(asyncSend, readBufferSz);
}
/** @brief start event loop when EventDriven also open if not */
int SerialPort::start(bool const asyncSend, size_t const readBufferSz)
{
	boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
	boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
	(void)(wLock);
	if (this->device.fd < 0) {
		std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__ << ") "
			<< "open device now with rw\n";
		/* open and start */
		return this->__open(
			static_cast<uint32_t>(OpenFlag::ReadWrite),
			readBufferSz);
	}
	return this->__start(asyncSend, readBufferSz);
}
static void syserr(char const* msg)
{
	std::cerr << "serialport.cpp +" << __LINE__ << " "  << msg << "\n";
}
/** @brief real start */
int SerialPort::__start(bool const asyncSend, size_t const readBufferSz)
{
	if ((QueryMode::EventDriven != this->queryMode) || this->eventDriven) {
		return 0;
	}
	this->eventDriven = boost::make_shared<EventDriven>(readBufferSz);
	/* this->eventDriven->mainLoop = ev_default_loop(0); */
	this->eventDriven->mainLoop = ev_loop_new(0);
	/* io 监控器的初始化 */
	ev_init(&this->eventDriven->rwio, rwcallback);
	ev_set_syserr_cb(syserr);
	this->eventDriven->rwio.data = this;
	if (asyncSend && (this->device.openFlag &
		(static_cast<uint32_t>(OpenFlag::Write)))) {
		this->device.ioFlag = EV_READ | EV_WRITE;
	} else {
		this->device.ioFlag = EV_READ;
	}
	ev_io_set(
		&this->eventDriven->rwio,
		this->device.fd,
		this->device.ioFlag);
	ev_io_start(this->eventDriven->mainLoop, &this->eventDriven->rwio);
	this->eventDriven->thread = boost::make_shared<boost::thread>(
		boost::bind(&SerialPort::rwioloop, this));
	return 0;
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
int SerialPort::stop()
{
	{
		boost::shared_lock<boost::shared_mutex> rLock(this->rwlock);
		(void)(rLock);
		if (QueryMode::EventDriven != this->queryMode) {
			return 0;
		}
	}
	boost::shared_ptr<EventDriven> eventDriven;
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		eventDriven = this->eventDriven;
		if (!eventDriven) {
			std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__
				<< ") already nil\n";
			return ENODEV;
		}
		this->eventDriven = nullptr;
		eventDriven->terminate = true;
		/*
		 * for one-shot events, one must manually stop the watcher
		 * with its corresponding stop function
		 */
		ev_io_stop(eventDriven->mainLoop, &eventDriven->rwio);
		/* this causes all nested ev_run's to stop iterating */
		ev_break(eventDriven->mainLoop, EVBREAK_ALL);
	}
	eventDriven->thread->join();
	struct ev_loop* mainLoop;
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		eventDriven->thread.reset();
		mainLoop = eventDriven->mainLoop;
		eventDriven->mainLoop = nullptr;
		eventDriven->readBuffer.reset();
	}
	/* release */
	ev_loop_destroy(/* EV_DEFAULT */ mainLoop);
	eventDriven.reset();
	return 0;
}
/**
 * @brief override to read data when EventDriven
 * e.x. use recv or recvAll
 * @note
 * - if rate enabled shouldRecv be callbacked when cache full or
 * no cache or time to rate.
 * - if rate disabled shouldRecv be callbacked once data available
 * @return >= 0 when success else fail and will not callback shouldSend
 */
ssize_t SerialPort::shouldRecv(size_t const maxAvailable)
{
	std::vector<uint8_t> buffer;
	/* recv to eat all when dft */
	return this->recv(buffer, maxAvailable, 1000);
}
/**
 * @brief override to send to fd when just fd writeable
 * when EventDriven
 */
ssize_t SerialPort::shouldSend()
{
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
	if (this->device.fd < 0) {
		return -ENODEV;
	}
	long bytes = 0;
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
	if (this->eventDriven && this->eventDriven->readBuffer) {
		return bytes + this->eventDriven->readBuffer->size();
	} else {
		return bytes;
	}
}
/**
 * @param buffer std::vector<uint8_t>&: should has min @a maxRecv capacity
 * (e.x. reserve) but not required .. here will resize
 * @param timeoutMillisec long const: if is 0: read once till ok or fail
 * @returns
 * - full success return received bytes and return equals maxRecv
 * - else if return > 0 but < maxRecv means time out
 * - return -errno when fail
 */
ssize_t SerialPort::recv(
	std::vector<uint8_t>& buffer,
	size_t const maxRecv,
	long const timeoutMillisec)
{
	if (maxRecv <= 0 || this->device.fd < 0) {
		return 0;
	}
	/* chk busy */
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven) {
			if (this->eventDriven->recvMode != RecvMode::Free) {
				return -EBUSY;
			}
			this->eventDriven->recvMode = RecvMode::User;/* get */
		}
	}
	/* recv to maxRecv or till timeout */
	size_t offset;
	double nownsec, lastnsec;
	ssize_t recvbytes, r;
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven && this->eventDriven->readBuffer) {
			if (this->eventDriven->readBuffer->size() > maxRecv) {
				/*
				 * 1 ensure size after copy is maxRecv
				 * 2 be large enough for cpy(caller should ensure first)
				 */
				buffer.resize(maxRecv);
				::memcpy(
					buffer.data(),
					this->eventDriven->readBuffer->data(),
					maxRecv);
				/* erase cache count: maxRecv */
				this->eventDriven->readBuffer->erase(
					this->eventDriven->readBuffer->begin(),
					this->eventDriven->readBuffer->begin() + maxRecv);
				recvbytes = maxRecv;
				goto restore;
			} else {
				buffer = *(this->eventDriven->readBuffer);/* copy out */
				this->eventDriven->readBuffer->clear();/* clear cache */
				offset = buffer.size();
			}
		} else {
			offset = 0;
		}
	}
	buffer.resize(maxRecv);/* ensure size after read more */
	recvbytes = offset;
	nownsec = NowNanosec();
	lastnsec = (timeoutMillisec < 0) ? -1.0 :
		(nownsec + (1e6 * timeoutMillisec));
	while ((recvbytes < static_cast<ssize_t>(maxRecv)) &&
		((timeoutMillisec < 0) || (nownsec <= lastnsec))) {
		/* read more */
		{
			boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
			(void)(wLock);
			r = ::read(
				this->device.fd,
				buffer.data() + recvbytes,
				maxRecv - recvbytes);
			if (r < 0) {
				int const e = errno;
				std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << __LINE__
					<< ") write fail " << strerror(e) << "\n";
				if (e) {
					recvbytes = -e;
				} else {
					recvbytes = -1;
				}
				goto restore;
			} else if (r > 0) {
				recvbytes += r;
				continue;/* fast read again */
			}
		}
		usleep(10 * 1e3);/* wait more data */
		if (timeoutMillisec >= 0) {
			nownsec = NowNanosec();
		}
	}
	if (recvbytes < static_cast<ssize_t>(maxRecv)) {
		buffer.resize(recvbytes);/* incase timeout: size should be recvbytes */
		/**caller manage capacity so no shrink_to_fit
		buffer.shrink_to_fit();
		@note if recvbytes != maxRecv => ETIME so not return -ETIME
		recvbytes = -ETIME*/
	}
restore:
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven) {
			this->eventDriven->recvMode = RecvMode::Free;/* release */
		}
	}
	return recvbytes;
}
std::vector<uint8_t> SerialPort::recv(
	size_t const maxRecv,
	long const timeoutMillisec)
{
	if (maxRecv <= 0 || this->device.fd < 0) {
		return std::vector<uint8_t>();
	}
	/* chk busy */
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven) {
			if (this->eventDriven->recvMode != RecvMode::Free) {
				return std::vector<uint8_t>();
			}
			this->eventDriven->recvMode = RecvMode::User;/* get */
		}
	}
	/* recv to maxRecv or till timeout */
	size_t offset;
	double nownsec, lastnsec;
	ssize_t recvbytes, r;
	std::vector<uint8_t> buffer;
	buffer.reserve(maxRecv);
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven && this->eventDriven->readBuffer) {
			if (this->eventDriven->readBuffer->size() > maxRecv) {
				/*
				 * 1 ensure size after copy is maxRecv
				 * 2 be large enough for cpy(caller should ensure first)
				 */
				buffer.resize(maxRecv);
				memcpy(
					buffer.data(),
					this->eventDriven->readBuffer->data(),
					maxRecv);
				/* erase cache count: maxRecv */
				this->eventDriven->readBuffer->erase(
					this->eventDriven->readBuffer->begin(),
					this->eventDriven->readBuffer->begin() + maxRecv);
				recvbytes = maxRecv;
				goto restore;
			} else {
				buffer = *(this->eventDriven->readBuffer);/* copy out */
				this->eventDriven->readBuffer->clear();/* clear cache */
				offset = buffer.size();
			}
		} else {
			offset = 0;
		}
	}
	buffer.resize(maxRecv);/* ensure size after read more */
	recvbytes = offset;
	nownsec = NowNanosec();
	lastnsec = (timeoutMillisec < 0) ? -1.0 :
		(nownsec + (1e6 * timeoutMillisec));
	while ((recvbytes < static_cast<ssize_t>(maxRecv)) &&
		((timeoutMillisec < 0) || (nownsec <= lastnsec))) {
		/* read more */
		{
			boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
			boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
			(void)(wLock);
			r = ::read(
				this->device.fd,
				buffer.data() + recvbytes,
				maxRecv - recvbytes);
			if (r < 0) {
				int const e = errno;
				std::cerr << "[" NAME "][ERRO](" << __FILE__ << "+" << \
					__LINE__ << ") write fail " << strerror(e) << "\n";
				if (e) {
					recvbytes = -e;
				} else {
					recvbytes = -1;
				}
				goto restore;
			} else if (r > 0) {
				recvbytes += r;
				continue;/* fast read again */
			}
		}
		usleep(10 * 1e3);/* wait more data */
		if (timeoutMillisec >= 0) {
			nownsec = NowNanosec();
		}
	}
	if (recvbytes < static_cast<ssize_t>(maxRecv)) {
		buffer.resize(recvbytes);/* incase timeout: size should be recvbytes */
		/* caller manage capacity so no shrink_to_fit */
		/* buffer.shrink_to_fit(); */
		recvbytes = -ETIMEDOUT;
	}
restore:
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven) {
			this->eventDriven->recvMode = RecvMode::Free;/* release */
		}
	}
	buffer.shrink_to_fit();
	return buffer;
}
ssize_t SerialPort::recvAll(std::vector<uint8_t>& buffer)
{
	/* chk busy */
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven) {
			if (this->eventDriven->recvMode != RecvMode::Free) {
				return -EBUSY;
			}
			this->eventDriven->recvMode = RecvMode::User;/* get */
		}
	}
	ssize_t recvbytes;
	/* recv */
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		ssize_t const avail = this->__hasData();
		if (avail <= 0) {
			recvbytes = avail;
			goto restore;
		}
		size_t offset;
		if (this->eventDriven && this->eventDriven->readBuffer &&
			(this->eventDriven->readBuffer->size() > 0)) {
			buffer = *(this->eventDriven->readBuffer);
			this->eventDriven->readBuffer->clear();
			offset = buffer.size();
		} else {
			offset = 0;
		}
		buffer.resize(offset + avail);/* pre size max */
		ssize_t const ret = ::read(
			this->device.fd,
			buffer.data() + offset,
			avail);
		if (ret > 0) {
			buffer.resize(offset + ret);/* incase ret < avail */
		} else {
			buffer.resize(offset);
		}
		buffer.shrink_to_fit();
		recvbytes = buffer.size();
	}
restore:
	{
		boost::upgrade_lock<boost::shared_mutex> uplock(this->rwlock);
		boost::upgrade_to_unique_lock<boost::shared_mutex> wLock(uplock);
		(void)(wLock);
		if (this->eventDriven) {
			this->eventDriven->recvMode = RecvMode::Free;/* release */
		}
	}
	return recvbytes;
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
		case BaudRate::_50:
			baudRateToTermios(B50, this->device.currentTermios);
			break;
		case BaudRate::_75:
			baudRateToTermios(B75, this->device.currentTermios);
			break;
		case BaudRate::_110:
			baudRateToTermios(B110, this->device.currentTermios);
			break;
		case BaudRate::_134:
			baudRateToTermios(B134, this->device.currentTermios);
			break;
		case BaudRate::_150:
			baudRateToTermios(B150, this->device.currentTermios);
			break;
		case BaudRate::_200:
			baudRateToTermios(B200, this->device.currentTermios);
			break;
		case BaudRate::_300:
			baudRateToTermios(B300, this->device.currentTermios);
			break;
		case BaudRate::_600:
			baudRateToTermios(B600, this->device.currentTermios);
			break;
		case BaudRate::_1200:
			baudRateToTermios(B1200, this->device.currentTermios);
			break;
		case BaudRate::_1800:
			baudRateToTermios(B1800, this->device.currentTermios);
			break;
		case BaudRate::_2400:
			baudRateToTermios(B2400, this->device.currentTermios);
			break;
		case BaudRate::_4800:
			baudRateToTermios(B4800, this->device.currentTermios);
			break;
		case BaudRate::_9600:
			baudRateToTermios(B9600, this->device.currentTermios);
			break;
		case BaudRate::_19200:
			baudRateToTermios(B19200, this->device.currentTermios);
			break;
		case BaudRate::_38400:
			baudRateToTermios(B38400, this->device.currentTermios);
			break;
		case BaudRate::_57600:
			baudRateToTermios(B57600, this->device.currentTermios);
			break;
#		if defined(YUIWONGSERIALPORT_B76800) && (YUIWONGSERIALPORT_B76800)
		case BaudRate::_76800:
			baudRateToTermios(B76800, this->device.currentTermios);
			break;
#		endif
		case BaudRate::_115200:
			baudRateToTermios(B115200, this->device.currentTermios);
			break;
#		if defined(YUIWONGSERIALPORT_B230400) && \
			defined(YUIWONGSERIALPORT_B4000000)
		case BaudRate::_230400:
			baudRateToTermios(B230400, this->device.currentTermios);
			break;
		case BaudRate::_460800:
			baudRateToTermios(B460800, this->device.currentTermios);
			break;
		case BaudRate::_500000:
			baudRateToTermios(B500000, this->device.currentTermios);
			break;
		case BaudRate::_576000:
			baudRateToTermios(B576000, this->device.currentTermios);
			break;
		case BaudRate::_921600:
			baudRateToTermios(B921600, this->device.currentTermios);
			break;
		case BaudRate::_1000000:
			baudRateToTermios(B1000000, this->device.currentTermios);
			break;
		case BaudRate::_1152000:
			baudRateToTermios(B1152000, this->device.currentTermios);
			break;
		case BaudRate::_1500000:
			baudRateToTermios(B1500000, this->device.currentTermios);
			break;
		case BaudRate::_2000000:
			baudRateToTermios(B2000000, this->device.currentTermios);
			break;
		case BaudRate::_2500000:
			baudRateToTermios(B2500000, this->device.currentTermios);
			break;
		case BaudRate::_3000000:
			baudRateToTermios(B3000000, this->device.currentTermios);
			break;
		case BaudRate::_3500000:
			baudRateToTermios(B3500000, this->device.currentTermios);
			break;
		case BaudRate::_4000000:
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
			this->portSettingDirty |= static_cast<uint32_t>(Dirty::DataBit);
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
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::DataBit)) {
		if (this->portSetting.parityType != ParityType::SPACE) {
			this->device.currentTermios.c_cflag &= (~CSIZE);
			switch(this->portSetting.dataBit) {
			case DataBit::_5:
				this->device.currentTermios.c_cflag |= CS5;
				break;
			case DataBit::_6:
				this->device.currentTermios.c_cflag |= CS6;
				break;
			case DataBit::_7:
				this->device.currentTermios.c_cflag |= CS7;
				break;
			case DataBit::_8:
				this->device.currentTermios.c_cflag |= CS8;
				break;
			}
		} else {
			/*
			 * space parity not directly supported - add an extra data bit to
			 * simulate it
			 */
			this->device.currentTermios.c_cflag &= ~(PARENB|CSIZE);
			switch(this->portSetting.dataBit) {
			case DataBit::_5:
				this->device.currentTermios.c_cflag |= CS6;
				break;
			case DataBit::_6:
				this->device.currentTermios.c_cflag |= CS7;
				break;
			case DataBit::_7:
				this->device.currentTermios.c_cflag |= CS8;
				break;
			case DataBit::_8:
				/* this will never happen, put here to suppress an warning */
				std::cerr << "[" NAME "][WARN](" << __FILE__ << "+" << __LINE__
					<< ") DataBit _8 no update\n";
				break;
			}
		}
	}
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::StopBit)) {
		switch (this->portSetting.stopBit) {
		case StopBit::_1:
			this->device.currentTermios.c_cflag &= (~CSTOPB);
			break;
		case StopBit::_2:
			this->device.currentTermios.c_cflag |= CSTOPB;
			break;
		}
	}
	if (this->portSettingDirty & static_cast<uint32_t>(Dirty::Flow)) {
		switch(this->portSetting.flowControl) {
		case FlowControl::OFF:
			this->device.currentTermios.c_cflag &= (~CRTSCTS);
			this->device.currentTermios.c_iflag &= (~(IXON|IXOFF|IXANY));
			break;
		case FlowControl::XONXOFF:
			/* software (XON/XOFF) flow control */
			this->device.currentTermios.c_cflag &= (~CRTSCTS);
			this->device.currentTermios.c_iflag |= (IXON|IXOFF|IXANY);
			break;
		case FlowControl::HARDWARE:
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
