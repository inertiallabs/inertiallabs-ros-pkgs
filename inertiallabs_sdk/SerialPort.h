#pragma once

class SerialPort
{
public:
	SerialPort();
	~SerialPort();
	int open(const char *path);
	bool isOpen();
	void close();
	int setBaudrate(int baud);
	int read(char *buf, unsigned int size, int timeout = 0);
	int write(char* buf, unsigned int size);

private:
#ifdef __linux__
	int fd;
#endif
#ifdef __WIN32__
#endif
};

