#include "SerialPort.h"
#if defined __linux__
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <cerrno>
#elif defined _WIN32 || defined _WIN64
#include <string>
#include <sstream>
#endif // __linux__
#include <cstdio>

SerialPort::SerialPort()
	: timeout(1000)
{
#if defined __linux__
	fd = -1;
#elif defined _WIN32 || defined _WIN64
	hCom = INVALID_HANDLE_VALUE;
#endif // __linux__
}

SerialPort::~SerialPort()
{
	close();
}

int SerialPort::open(const char *path)
{
#if defined __linux__
	fd = ::open(path, O_NOCTTY | O_RDWR);
	if (fd < 0) return -errno;
#elif defined _WIN32 || defined _WIN64
	hCom = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, nullptr);
	if (hCom == INVALID_HANDLE_VALUE)
		return -static_cast<int>(GetLastError());
	COMMTIMEOUTS timeouts = { MAXDWORD,MAXDWORD,MAXDWORD,1,1 };
	timeouts.ReadTotalTimeoutConstant = timeout;
	if (!SetCommTimeouts(hCom, &timeouts))
		return -2;
#endif // __linux__
	return 0;
}

bool SerialPort::isOpen()
{
#if defined __linux__
	return fd >= 0;
#elif defined _WIN32 || defined _WIN64
	return hCom != INVALID_HANDLE_VALUE;
#endif // __linux__
}

void SerialPort::close()
{
	if (isOpen())
	{
#if defined __linux__
		::close(fd);
		fd = -1;
#elif defined _WIN32 || defined _WIN64
		CloseHandle(hCom);
		hCom = INVALID_HANDLE_VALUE;
#endif // __linux__
	}
}

int SerialPort::setBaudrate(int baud)
{
#if defined __linux__
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
	config.c_cflag |= CS8 | CREAD | CLOCAL;
	config.c_lflag |= IEXTEN;
	if (tcsetattr(fd, TCSANOW, &config) < 0)
	{
		return -3;
	}
#elif defined _WIN32 || defined _WIN64

	COMMPROP commProp;
	if (!GetCommProperties(hCom, &commProp))
		return -1;
	if (!(commProp.dwSettableBaud & BAUD_USER))
		return -2;
	_DCB config;
	config.DCBlength = sizeof(config);
	if (!GetCommState(hCom, &config))
		return -1;
	std::stringstream commstr;
	commstr << "baud=" << baud << " parity=N data=8 stop=1 to=off xon=off odsr=off octs=off dtr=off rts=off idsr=off";
	if (!BuildCommDCBA(commstr.str().c_str(), &config))
		return -3;
	if (!SetCommState(hCom, &config))
	{
		auto err = GetLastError();
		return -3;
	}
#endif // __linux__
	return 0;
}

int SerialPort::read(char* buf, unsigned int size)
{
#if defined __linux__
	if (!isatty(fd))
		return -1;
	pollfd fds;
	fds.fd = fd;
	fds.events = POLLIN;
	fds.revents = 0;
	int result = poll(&fds, 1, timeout);
	if (result < 0)
		return -errno;
	if (!result) return 0;
	result = ::read(fd, buf, size);
	if (result < 0)
		return -errno;
	else
		return result;
#elif defined _WIN32 || defined _WIN64
	DWORD bytesRead = 0;
	if (!ReadFile(hCom, buf, size, &bytesRead, nullptr))
		return -static_cast<int>(GetLastError());
	return bytesRead;
#endif
}

int SerialPort::write(char* buf, unsigned int size)
{
#if defined __linux__
	int result = ::write(fd, buf, size);
	if (result < 0)
		return -errno;
	else
		return result;
#elif defined _WIN32 || defined _WIN64
	DWORD result;
	if (WriteFile(hCom, buf, size, reinterpret_cast<LPDWORD>(&result), nullptr))
		return result;
	result = GetLastError();
	return -static_cast<int>(result);
#endif
}
