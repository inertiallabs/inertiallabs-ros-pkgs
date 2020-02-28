#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>

//Inertial Labs source header
#include "InertialLabs_AHRS_10.h"

#include <ros/ros.h>

//adding message type headers
#include <inertiallabs_msgs/ahrs_data.h>


//Publishers
ros::Publisher pubahrs_data;


//global variables
IL_AHRS_10 ahrs;
int ahrs_output_format;
std::string ahrs_frame_id;
ros::Timer pub_timer;
IL_ERROR_CODE il_err;
IL_ERROR_CODE il_error;         
std::string il_error_msg;


AHRS_10_CompositeData ahrs_data;


inertiallabs_msgs::ahrs_data  msg_ahrs_data;

void ilerror_msg(IL_ERROR_CODE il_error,std::string &msg);

void publish_device()
{

	static int seq=0;
	seq++;

	ros::Time timestamp=ros::Time::now();

	if(pubahrs_data.getNumSubscribers()>0)
	{
		ROS_INFO("subscribed");
		
		il_err = AHRS_10_YPR(&ahrs,&ahrs_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());exit(EXIT_FAILURE);
		}
		il_err = AHRS_10_getGyroAccMag(&ahrs,&ahrs_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL("%s" ,il_error_msg.c_str()); exit(EXIT_FAILURE);
		}
		il_err = AHRS_10_getSensorData(&ahrs,&ahrs_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" , il_error_msg.c_str());exit(EXIT_FAILURE);
		}
	
			msg_ahrs_data.header.seq=seq;
			msg_ahrs_data.header.stamp=timestamp;
			msg_ahrs_data.header.frame_id=ahrs_frame_id;
			msg_ahrs_data.YPR.x=ahrs_data.ypr.yaw;
			msg_ahrs_data.YPR.y=ahrs_data.ypr.pitch;
			msg_ahrs_data.YPR.z=ahrs_data.ypr.roll;
			msg_ahrs_data.Mag.x=ahrs_data.magnetic.c0;
			msg_ahrs_data.Mag.y=ahrs_data.magnetic.c1;
			msg_ahrs_data.Mag.z=ahrs_data.magnetic.c2;
			msg_ahrs_data.Accel.x=ahrs_data.acceleration.c0;
			msg_ahrs_data.Accel.y=ahrs_data.acceleration.c1;
			msg_ahrs_data.Accel.z=ahrs_data.acceleration.c2;
			msg_ahrs_data.Gyro.x=ahrs_data.gyro.c0;
			msg_ahrs_data.Gyro.y=ahrs_data.gyro.c1;
			msg_ahrs_data.Gyro.z=ahrs_data.gyro.c2;
			msg_ahrs_data.Temp=ahrs_data.Temper;
			msg_ahrs_data.Vinp=ahrs_data.Vinp;
			pubahrs_data.publish(msg_ahrs_data);
	
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
	ros::init(argc,argv,"continues_inertiallabs_ahrs_10");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(100); // 100 hz
	std::string port;

	AHRS_10_SetInternalData data;

	//commadn line varibales

	int baudrate,publish_rate,async_output_rate,async_output_type;
	np.param<std::string>("serial_port",port,"/dev/ttyUSB0");
	np.param<int>("serial_baud",baudrate,115200);
	np.param<int>("publish_rate",publish_rate,10);
	np.param<int>("async_output_type",async_output_type,0);
	np.param<int>("async_output_rate",async_output_rate,6);
	np.param<int>("ahrs_output_format",ahrs_output_format,1);

	//Initializing Publishers


	ROS_INFO("Ready to answer your queries regarding ins data");

	ROS_INFO("connecting to INS. port: %s at a baudrate:%d\n",port.c_str(),baudrate);

	il_err=AHRS_10_connect(&ahrs,port.c_str(),baudrate);
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


	il_err= AHRS_10_Stop(&ahrs);
	ros::Duration(5).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("stop command error");exit(EXIT_FAILURE);
	}

	il_err= ReadAHRS10par(&ahrs);
	ros::Duration(2).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("read command error"); exit(EXIT_FAILURE);
	}

	AHRS_10_ReadInternalParameters(&ahrs,&data);


	/*
	 * \brief Set the data output mode of the INS.
	 */
	il_err= AHRS_10_SetMode(&ahrs,IL_AHRS_10_ONREQUEST_CMD); 

	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("SetMode command error");exit(EXIT_FAILURE);
	}
	/*
	 *	\brief  wait for 2 seconds till the communication light off in the INS
	 */
	ros::Duration(2).sleep();

	ahrs.cmd_flag = ahrs_output_format;
	switch (ahrs_output_format)
	{
		case IL_AHRS_10_CLB_DATA_RECEIVE:
			il_error = AHRS_10_ClbData_Receive(&ahrs);
			break;

		case IL_AHRS_10_CLB_HR_DATA_RECEIVE:
			il_error = AHRS_10_ClbHRData_Receive(&ahrs);
			break;

		case IL_AHRS_10_QUATERNION_RECEIVE:
			il_error = AHRS_10_QuatData_Receive(&ahrs);
			break;

		case IL_READ_AHRS_10_PAR_RECEIVE:
			il_error = ReadAHRS10par(&ahrs);
			break;
		case IL_AHRS_10_NMEA_RECEIVE:
			il_error = AHRS_10_NMEA_Receive(&ahrs) ;
			break;
		
		default:
			ROS_INFO("this output data format is not supported by this AHRS_10");
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
	AHRS_10_disconnect(&ahrs);
	return 0;
}