#pragma once
#include <thread>
#include <mutex>
#include <fstream>
#include "dataStructures.h"

class ILDriver
{
public:
	ILDriver();
	~ILDriver();
	int connect(const char* path, int baudrate);
	void disconnect();
	int start(unsigned char mode, bool onRequest = false, const char* logname = nullptr);
	int request(unsigned char mode, int timeout);
	int stop();
	INSDataStruct getLatestData();
	INSDeviceInfo getDeviceInfo();
	bool isStarted() { return sessionState == 4;  }

private:
	INSDataStruct latestData;
	INSDeviceInfo deviceInfo;
	INSDevicePar deviceParam;
	void *port;
	std::thread *workerThread;
	std::mutex dataMutex;
	bool quit;
	bool devInfoRead;
	bool onRequestMode;
	char requestCode;
	bool requestFulfilled;
	int sessionState;
	int sendPacket(char type, const char* payload, unsigned int size);
	int readDevInfo();
	void readerLoop();
	static void threadFunc(ILDriver* instance) { instance->readerLoop(); }
	std::ofstream log;
};

