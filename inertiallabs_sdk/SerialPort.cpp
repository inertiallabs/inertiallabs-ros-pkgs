#include "SerialPort.h"
#ifdef __linux__
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <cstdio>
#include <fcntl.h>
#include <cerrno>
#endif // __linux__

SerialPort::SerialPort()
{
#ifdef __linux__
	fd = -1;
#endif // __linux__
}

SerialPort::~SerialPort()
{
	close();
}

int SerialPort::open(const char *path)
{
#ifdef __linux__
	fd = ::open(path, O_NOCTTY | O_RDWR);
	if (fd < 0) return errno;
#endif // __linux__
	return 0;
}

bool SerialPort::isOpen()
{
#ifdef __linux__
		return fd >= 0;
#endif // __linux__
}

void SerialPort::close()
{
#ifdef __linux__
	if (fd >= 0) ::close(fd);
	fd = -1;
#endif // __linux__
}

int SerialPort::setBaudrate(int baud)
{
#ifdef __linux__
	struct termios config;
	if (tcgetattr(fd, &config) < 0)
	{
		return -1;
	}
	cfmakeraw(&config);
	switch (baud)
	{
	case 4800	:config.c_cflag = B4800		; break;
	case 9600	:config.c_cflag = B9600		; break;
	case 19200	:config.c_cflag = B19200	; break;
	case 38400	:config.c_cflag = B38400	; break;
	case 57600	:config.c_cflag = B57600	; break;
	case 115200 :config.c_cflag = B115200	; break;
	case 230400 :config.c_cflag = B230400	; break;
	case 460800 :config.c_cflag = B460800	; break;
	case 921600 :config.c_cflag = B921600	; break;
	case 2000000:config.c_cflag = B2000000	; break;
	default:
		return -2;
	}
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	if (tcsetattr(fd, TCSANOW, &config) < 0)
	{
		return -3;
	}
#endif // __linux__

	return 0;
}

int SerialPort::read(char* buf, unsigned int size, int timeout)
{
#ifdef __linux__
	if (!isatty(fd))
		return -2;
	pollfd fds;
	fds.fd = fd;
	fds.events = POLLIN;
	fds.revents = 0;
	int result = poll(&fds, 1, timeout > 0 ? timeout : -1);
	if (result < 0)
		return -1;
	if (result)
		return ::read(fd, buf, size);
	return 0;
#endif
}

int SerialPort::write(char* buf, unsigned int size)
{
	int result;
#ifdef __linux__
	result = ::write(fd, buf, size);
	if (result < 0)
		printf("Write failed: %i\n", errno);
#endif
	return result;
}
