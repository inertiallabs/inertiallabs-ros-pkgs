# Inertiallabs_sdk

Cross-platform C/C++ SDK for the Inertial Labs Products.

[![alt text](https://readthedocs.org/projects/docs/badge/?version=latest "Documentation Status")](https://oblivione.gitlab.io/inertiallabs_ros_package/index.html)



The `Inertiallabs_sdk` is a Cross-platform c/c++ SDK for GPS-Aided Inertial Navigation Systems (INS) and IMU-P of [Inertial Labs](https://inertiallabs.com/). Currently the SDK is developed for Linux and Windows . The user manual for the device can be found [here](https://inertiallabs.com/static/pdf/INS-Datasheet.rev3.2_Nov_2018.pdf).

The SDK is tested on both Linux(Ubuntu 16.04 LTS & 18.04 LTS)  and Windows(Visual Studio 2017).To use the SDK in windows you have to install Visual Studio version 2012 > .

## License

* The license for the official SDK is the MIT license which is included in the `inertiallabs_sdk/ LICENSE.txt`
* The license for the other codes is Apache 2.0 whenever not specified.

## Compiling

**For Linux**

The following steps will walk you through adding the Inertial Labs C/C++ Library to your Linux project to access a INS/IMU-P device. You can also find an example usage of the library at imu_linux_basic.c & ins_linux_basic.c .

```
$ git clone https://gitlab.com/oblivione/inertiallabs_sdk
$ cd inertiallabs_sdk/examples/<example_code_dir>
$ make clean
$ make

```
To include this project to your own make file , have a look at the example make file provied by us in the expample folder ins_linux_basic .

**For Windows**

The following steps will walk you through including the Inertial Labs C/C++ Library into your existing C or C++ project to access a INS/IMU-P device. You can also find an example usage of the library at ins_windows_basic.c & imu_windows_basic.c.

1. Add the code files src/InertialLabs_IMU.c or InertialLabs_INS.c and src/arch/win32/InertialLabs_services.c to your project.
    - Right-click on your project file and select Add -> Existing Item... and browse to where you extracted the library files and select the two file.
2. Add an additional include directory to the libraries include folder.
    - Right-click on your project file and select Properties. On the property pages, browse to Configuration Properties -> C/C++ -> General. For the property field Additional Include Directories, add a link to the library's include folder.
3. Disable usage of precompiled headers for your project.
    - Right-click on your project file and select Properties. Browse to the section Configuration Properties -> C/C++ -> Precompiled Header and select the option Not Using Precompiled Headers.
4. Add the include line #include "InertialLabs_IMU.h" or #include "InertialLabs_INS.h"  to the top of your code file to get access to all of the types and functions provided by the library.

## FAQ

1. The driver can't open my device?\
Make sure you have ownership of the device in `/dev`.

2. Why I have permission error during the initialization process of the driver?\
Most often, this is because the baud rate you set does not match the package size to be received. Try increase the baud rate.

3. Why is the IMU data output rate much lower than what is set?\
This may be due to a recent change in the FTDI USB-Serial driver in the Linux kernel, the following shell script might help:
    ```bash
    # Reduce latency in the FTDI serial-USB kernel driver to 1ms
    # This is required due to https://github.com/torvalds/linux/commit/c6dce262
    for file in $(ls /sys/bus/usb-serial/devices/); do
      value=`cat /sys/bus/usb-serial/devices/$file/latency_timer`
      if [ $value -gt 1 ]; then
        echo "Setting low_latency mode for $file"
        sudo sh -c "echo 1 > /sys/bus/usb-serial/devices/$file/latency_timer"
      fi
    done
    ```

## Bug Report

Prefer to open an issue. You can also send an E-mail to omprakashpatro@gmail.com.

