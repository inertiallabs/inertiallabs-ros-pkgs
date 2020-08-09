#include <WS2tcpip.h>
#include <string>
#include "../../NetClient.h"

#pragma comment(lib, "Ws2_32.lib")

namespace IL {
	NetClient::NetClient() 
		: timeout(1000)
		, hCom(INVALID_SOCKET)
	{
		WSADATA data;
		WSAStartup(MAKEWORD(2, 2), &data);
	}

	NetClient::~NetClient()
	{
		close();
		WSACleanup();
	}

	int NetClient::open(const char* url)
	{
		std::string urlStr(url);
		size_t pos = urlStr.find(':');
		if (pos == std::string::npos) return 1;
		std::string addrStr = urlStr.substr(pos + 1);
		std::string protoStr = urlStr.substr(0, pos);
		pos = addrStr.find(':');
		if (pos == std::string::npos) return 1;
		std::string portStr = addrStr.substr(pos + 1);
		std::string hostStr = addrStr.substr(0, pos);
		struct addrinfo hints = {};
		struct addrinfo* result, * rp;
		hints.ai_family = AF_INET;
		if (protoStr == "tcp") hints.ai_socktype = SOCK_STREAM;
		else if (protoStr == "udp") hints.ai_socktype = SOCK_DGRAM;
		else return 1;
		hints.ai_flags = 0;
		hints.ai_protocol = 0;
		int res = getaddrinfo(hostStr.c_str(), portStr.c_str(), &hints, &result);
		if (res != 0) return -res;
		for (rp = result; rp != NULL; rp = rp->ai_next) {
			hCom = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
			if (hCom == INVALID_SOCKET) continue;
			if (connect(hCom, rp->ai_addr, rp->ai_addrlen) != SOCKET_ERROR)	break;
			closesocket(hCom);
			hCom = INVALID_SOCKET;
		}
		if (hCom == INVALID_SOCKET) return 2;
		return 0;
	}

	bool NetClient::isOpen()
	{
		return hCom != INVALID_SOCKET;
	}

	void NetClient::close()
	{
		if (isOpen()) {
			closesocket(hCom);
			hCom = INVALID_SOCKET;
		}
	}

	int NetClient::read(char* buf, unsigned int size)
	{
		if (hCom == INVALID_SOCKET) return -1;
		fd_set readfds = { 1, {hCom} };
		TIMEVAL tv_timeout = {timeout / 1000, (timeout % 1000) * 1000};
		int result = select(1, &readfds, nullptr, nullptr, &tv_timeout);
		if (result == SOCKET_ERROR) return -WSAGetLastError();
		if (!result) return 0;
		result = recv(hCom, buf, size, 0);
		int err;
		if (result == SOCKET_ERROR && (err = WSAGetLastError()) != WSAEMSGSIZE)	return -err;
		return result;
	}

	int NetClient::write(char* buf, unsigned int size)
	{
		int result = send(hCom, buf, size, 0);
		if (result == SOCKET_ERROR)	return -WSAGetLastError();
		return result;
	}


}