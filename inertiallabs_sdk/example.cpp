#include "ILDriver.h"
#include <iostream>

ILDriver driver;

int main()
{
    if (driver.connect("/dev/ttyUSB0", 460800) != 0)
    { 
        std::cout << "IL Driver failed to connect!" << std::endl;
        return 1;                       // Failed to connect
    }
    if (!driver.isStarted()) {
        if (driver.start(0x58))
        {
            std::cout << "INS failed to start!" << std::endl;
            driver.disconnect();
            return 2;                   // Failed to start
        }
    }
    for (int i = 0; i < 5; ++i)
    {
            auto data = driver.getLatestData();
            std::cout << "Time: " << data.ms_gps << ", GNSSInfo2 = " << data.GNSSInfo2 << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    driver.stop();
    driver.disconnect();
    return 0;
}
