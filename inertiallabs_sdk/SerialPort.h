#pragma once
#ifdef _WIN32
#include <windows.h>
#endif // __WIN32__


class SerialPort
{
public:
	SerialPort();
	~SerialPort();
	int open(const char *path);
	bool isOpen();
	void close();
	int setBaudrate(int baud);
	int read(char *buf, unsigned int size);
	int write(char* buf, unsigned int size);

private:
#if defined __linux__
	int fd;
#elif defined _WIN32 || defined _WIN64
	HANDLE hCom;
#endif
	int timeout;
};

