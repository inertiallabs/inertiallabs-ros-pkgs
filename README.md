# inertiallabs_ros_pkgs

ROS Packages for Inertial Labs Products . Current supported devices INS , IMU-P , AHRS , AHRS-10 .

[![alt text](https://readthedocs.org/projects/docs/badge/?version=latest "Documentation Status")](https://gitlab.com/oblivione/inertiallabs_ros_pkgs)


![Picture of IMU](https://inertiallabs.com/static/assets/img/products/INS-D.jpg)

The `inertiallabs_ros_pkgs` package is a linux ROS driver for GPS-Aided Inertial Navigation Systems (INS), IMU-P ,AHRS and AHRS-10 of [Inertial Labs](https://inertiallabs.com/). The package is developed based on the official [SDK v0.2](https://gitlab.com/oblivione/inertiallabs_sdk) for Linux.

The package is tested on Ubuntu 16.04 LTS & 18.04 LTS  with ROS Kinetic & ROS Melodic . You can install ROS from [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). 

## License

* The license for the official SDK is the MIT license which is included in the `ins_ros/inertiallabs_sdk`
* The license for the other codes is Apache 2.0 whenever not specified.

## Compiling

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
$ cd <your_work_space>/src
$ $ git colne https://gitlab.com/oblivione/inertiallabs_ros_pkgs
$ cd <your_work_space>
$ catkin_make_isolated
$ source devel/setup.bash

```

Why catkin_make_isolated ?

We developed a package for multiple devices with same dependencies , so to avoid confilcts we are compiling using catkin_make_isolated . If you need 
to use ins only , So you get the subpackages like inertiallabs_ins , inertiallabs_sdk , intertiallans_msgs and intertiallabs_ros_pkgs to your workspace and use your own build system to compile it . You have to change the intertiallabs_ros_pkgs/intertiallabs_ros_pkgs/package.xml 

```
<exec_depend>inertiallabs_ins</exec_depend>
 <!-- <exec_depend>inertiallabs_imu</exec_depend> -->
 <!-- <exec_depend>inertiallabs_ahrs</exec_depend> -->
 <!-- <exec_depend>inertiallabs_ahrs_10</exec_depend> -->

```

**Node**

example rosnodes

for ins on request mode 
```
 rosrun inertiallabs_ins ins_onrequest_mode _serial_baud:=460800 _ins_output_format:=3 

```

for ins continues mode 
```
 rosrun inertiallabs_ins ins_continues_mode _serial_baud:=460800 _ins_output_format:=3 

```


for imp-p on request mode 
```
 rosrun inertiallabs_imu imu_onrequest_mode _serial_baud:=460800 _imu_output_format:=3 

```

for imp-p continues mode 
```
 rosrun inertiallabs_imu imu_continues_mode _serial_baud:=460800 _imu_output_format:=3 

```

for ahrs-10 on request mode 
```
 rosrun inertiallabs_ahrs_10 ahrs10_onrequest_mode _serial_baud:=460800 _imu_output_format:=3 

```

for ahrs-10 continues mode 
```
 rosrun inertiallabs_ahrs_10 ahrs10_continues_mode _serial_baud:=460800 _imu_output_format:=3 

```
## Example Usage

**Parameters**

`serial_port` (`string`, `default: /dev/ttyUSB0`)

Port which the device connected to. This can be checked by command `dmesg`.

`serial_baud` (`int`, `460800`)

The baud rate of the serial port. The available baud rates can be checked on the user manual. It is suggested that the baud rate is kept at `460800` to ensure the high frequency transmission. The device will send `permission denied` error code if the baud rate cannot support the desired data package at the desired frequency.The sdk supports 7 baud rates.


`ins_output_format` (`int`, `2`)

The output data format of the INS data.

```
 IL_OPVT_RECEIVE           2      
 IL_QPVT_RECEIVE      		       3     
 IL_OPVT2A_RECEIVE    		       4      
 IL_OPVT2AW_RECEIVE   		       5      
 IL_OPVT2AHR_RECEIVE  		       6       
 IL_OPVTAD_RECEIVE    		       7    
 IL_MINIMAL_DATA_RECEIVE 	       8 
 IL_SENSOR_DATA_RECEIVE          9
 IL_OPVT_RAWIMU_DATA_RECEIVE    11
 IL_OPVT_GNSSEXT_DATA_RECEIVE   12

```
`imu_output_format` (`int`, `3`)

The output data format of the IMP-P data.

```
 IL_IMU_CLB_DATA_RECEIVE      2     
 IL_IMU_GA_DATA_RECEIVE       3      
 IL_IMU_ORIENTATION_RECEIVE     4
 IL_IMU_PSTABILIZATION_RECEIVE  5
 IL_IMU_NMEA_RECEIVE      6
 IL_IMU_GET_DEV_INFO_RECEIVE      9
```
**Published Topics**

`/Inertial_Labs/sensor_data` (`ins_ros/sensor_data`)
 
Publish Gyro(x,y,z) , Accelation(x,y,z) , Magnetic (x,y,z) , Temprature , Input Voltage , Pressure , Barometric height .

`/Inertial_Labs/ins_data` (`ins_ros/ins_data`)
 
 Publish Heading , Pitch , Roll values .

`/Inertial_Labs/gps_data` (`ins_ros/gps_data`)

 Publish Latitute, Longitute , Altitude , East Speed , North Speed  Vertial Speed values .

`/Inertial_Labs/quat_data` (`ins_ros/quat_data`)

 Publish  Quaternion of orientation values  . Only published in IL_QPVT_RECEIVE data output type . So the commmand line input will be _ins_output_format:=3 .  

 `/Inertial_Labs/imu_data` (`ins_ros/imu_data`)
  
Publish  imu data  for while you start the imu nodes in the arc folser . 

**Debug Mode**
To see the debug data from the sdk you can enable the debug mode in IL_common.h .The file located inside `intertiallabs_sdk/include/IL_common.h` .

```
#define IL_DBG 0                   /**< 0 - SDK Debug mode off.
                                        1 - SDK Debug mode on */


```
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
