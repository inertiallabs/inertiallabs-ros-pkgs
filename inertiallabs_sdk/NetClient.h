#pragma once
#include "Transport.h"

namespace IL {
    class NetClient :
        public Transport
    {
    public:
        NetClient();
        virtual ~NetClient();
        virtual int open(const char* url);
        virtual bool isOpen();
        virtual void close();
        virtual int read(char* buf, unsigned int size);
        virtual int write(char* buf, unsigned int size);

    private:
        int fd;
        int64_t hCom;
        int timeout;
    };
}

