#ifndef ILSDK_H
#define ILSDK_H

#ifdef WIN32
    #ifdef ILSDK_EXPORT
        #define ILSDK_API  __declspec(dllexport)
    #else
        #define ILSDK_API  __declspec(dllimport)
    #endif
#else
    #define ILSDK_API
#endif

#include "dataStructures.h"
#include "Transport.h"
#include "ILDriver.h"

#endif //ILSDK_H
