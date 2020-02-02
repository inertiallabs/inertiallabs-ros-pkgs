#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>

//Inertial Labs source header
#include "InertialLabs_IMU.h"
#include <ros/ros.h>
#include <inertiallabs_msgs/imu_data.h>


//Publishers
ros::Publisher pubimu_data;


//global variables
IL_IMU imu;
int imu_output_format;
std::string imu_frame_id;
ros::Timer pub_timer;
IL_ERROR_CODE il_err;         
std::string il_error_msg;


IMUCompositeData imu_data;


inertiallabs_msgs::imu_data  msg_imu_data;

void ilerror_msg(IL_ERROR_CODE il_error,std::string &msg);

void publish_device()
{

	static int seq=0;
	seq++;

	ros::Time timestamp=ros::Time::now();

	if(pubimu_data.getNumSubscribers()>0)
	{
		ROS_INFO("subscribed");
		
		il_err = IMU_YPR(&imu,&imu_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());exit(EXIT_FAILURE);
		}
		il_err = IMU_getGyroAccMag(&imu,&imu_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL("%s" ,il_error_msg.c_str()); exit(EXIT_FAILURE);
		}
		il_err = IMU_getSensorData(&imu,&imu_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" , il_error_msg.c_str());exit(EXIT_FAILURE);
		}
	
			msg_imu_data.header.seq=seq;
			msg_imu_data.header.stamp=timestamp;
			msg_imu_data.header.frame_id=imu_frame_id;
			msg_imu_data.YPR.x=imu_data.ypr.yaw;
			msg_imu_data.YPR.y=imu_data.ypr.pitch;
			msg_imu_data.YPR.z=imu_data.ypr.roll;
			msg_imu_data.Mag.x=imu_data.magnetic.c0;
			msg_imu_data.Mag.y=imu_data.magnetic.c1;
			msg_imu_data.Mag.z=imu_data.magnetic.c2;
			msg_imu_data.Accel.x=imu_data.acceleration.c0;
			msg_imu_data.Accel.y=imu_data.acceleration.c1;
			msg_imu_data.Accel.z=imu_data.acceleration.c2;
			msg_imu_data.Gyro.x=imu_data.gyro.c0;
			msg_imu_data.Gyro.y=imu_data.gyro.c1;
			msg_imu_data.Gyro.z=imu_data.gyro.c2;
			msg_imu_data.Temp=imu_data.Temper;
			msg_imu_data.Vinp=imu_data.Vinp;
			pubimu_data.publish(msg_imu_data);
	
	}

}
//defining timer publication
void publish_timer(const ros::TimerEvent&)
{
	publish_device();
}
 
void ilerror_msg(IL_ERROR_CODE il_error,std::string &msg)
{
	switch(il_error)
	{
		case ILERR_NO_ERROR:
		  msg="No Error";
		  break;
		case ILERR_UNKNOWN_ERROR:
		  msg="UnKnownError";
		  break;
		case ILERR_NOT_IMPLEMENTED:
		  msg="Not implemented";
		  break;
		case ILERR_TIMEOUT:
		  msg="TimeOut";
		  break;
		case ILERR_INVALID_VALUE:
		  msg="Invalid value";
		  break;
		case ILERR_FILE_NOT_FOUND:
		  msg="File not Found";
		  break;
		case ILERR_NOT_CONNECTED:
		  msg="Not Connected";
 		 break;
		case ILERR_MEMORY_ERROR:
		   msg ="No Data In the Buffer";
		   break;
		default:
		  msg="Undefined error";
		  break;
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"inertiallabs_imu");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(100); // 100 hz
	std::string port;

	IMUSetInternalData data;

	//commadn line varibales

	int baudrate,publish_rate,async_output_rate,async_output_type;
	np.param<std::string>("serial_port",port,"/dev/ttyUSB0");
	np.param<int>("serial_baud",baudrate,115200);
	np.param<int>("publish_rate",publish_rate,10);
	np.param<int>("async_output_type",async_output_type,0);
	np.param<int>("async_output_rate",async_output_rate,6);
	np.param<int>("imu_output_format",imu_output_format,1);

	//Initializing Publishers


	ROS_INFO("Ready to answer your queries regarding ins data");

	ROS_INFO("connecting to INS. port: %s at a baudrate:%d\n",port.c_str(),baudrate);

	il_err=IMU_connect(&imu,port.c_str(),baudrate);
	std:: cout << "after INS_connect \n";
	if(il_err!=ILERR_NO_ERROR) 
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("Could not connect to the sensor on this %s port error:%s\n did you add the user to the dialout group???",
       	        port.c_str(),
	          il_error_msg.c_str() 
		);
		exit(EXIT_FAILURE);
	}


	il_err= IMU_Stop(&imu);
	ros::Duration(5).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("stop command error");exit(EXIT_FAILURE);
	}

	il_err= ReadIMUpar(&imu);
	ros::Duration(2).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("read command error"); exit(EXIT_FAILURE);
	}

	IMU_ReadInternalParameters(&imu,&data);


	/*
	 * \brief Set the data output mode of the INS.
	 */
	il_err= IMU_SetMode(&imu,IL_SET_CONTINUES_MODE); 

	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("SetMode command error");exit(EXIT_FAILURE);
	}
	/*
	 *	\brief  wait for 2 seconds till the communication light off in the INS
	 */
	ros::Duration(2).sleep();

	imu.cmd_flag = imu_output_format;
	switch (imu_output_format)
	{
		case IL_IMU_CLB_DATA_RECEIVE:
			il_err = IMU_ClbData_Receive(&imu);
			break;

		case IL_IMU_GA_DATA_RECEIVE:
			il_err = IMU_GAdata_Receive(&imu);
			break;

		case IL_IMU_ORIENTATION_RECEIVE:
			il_err = IMU_Orientation_Receive(&imu);
			break;

		case IL_IMU_PSTABILIZATION_RECEIVE:
			il_err = IMU_PStabilization_Receive(&imu);
			break;
		case IL_IMU_NMEA_RECEIVE:
			il_err = IMU_NMEA_Receive(&imu) ;
			break;
		
		default:
			ROS_INFO("this output data format is not supported by this IMU-P");
			break;
	}

	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL(" command error");exit(EXIT_FAILURE);
	}

  	if(async_output_type ==0)
	{
		ROS_INFO("publishing at %d Hz\n",publish_rate);
		pub_timer=np.createTimer(ros::Duration(1.0/(double)publish_rate),publish_timer);
	}

	ros::spin();
	IMU_disconnect(&imu);
	return 0;
}