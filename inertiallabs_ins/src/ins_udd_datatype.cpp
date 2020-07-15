/**
 * \cond INCLUDE_PRIVATE
 * \file
 *
 * \section DESCRIPTION
 * This file implements the inertial labs SDK functions for interfacing with Inertial Labs INS in ROS enviroment.
 * OnRequest mode implmented in this file. It will publish YPR(yaw , pitch , roll) ,Gyro(x,y,z) , Acceleration(x,y,z) ,
 * Magnetic(x,y,z) , Temprature , Input Voltage , Position and GNSS Position data in rostopic.
 * 
 * \section Rostopic List
 * 
 * - /Inertial_Labs/ins_data
 * - /Inertial_Labs/sensor_data
 * - /Inertial_Labs/gps_data
 * 
 * 
 *  */

#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>

//Inertial Labs source header
#include "InertialLabs_INS.h"
#include <ros/ros.h>


//adding message type headers
#include <inertiallabs_msgs/udd_data.h>


void ilerror_msg(IL_ERROR_CODE il_error,std::string &msg);

ros::Publisher pubudd_data;

//Device
IL_INS ins;
int ins_output_format;
ros::Timer pub_timer;
IL_ERROR_CODE il_err; 
std::string udd_frame_id;        
std::string il_error_msg;

INSCompositeData composite_data;
INSPositionData  position_data;

void publish_device()
{
    static int seq=0;
	seq++;

    inertiallabs_msgs::udd_data msg_udd_data;
    ros::Time timestamp=ros::Time::now();

    switch (ins_output_format)
	{
        case IL_USERDEF_DATA_RECEIVE:
			il_err = UserDef_Data_Receive(&ins);
			break;

        default:
			ROS_INFO("this output data format is not supported by this INS");
			break;
    }

    if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL(" input data type error command error");exit(EXIT_FAILURE);
	}

    ros::Duration(1).sleep();

    il_err = INS_UDD(&ins,&composite_data , &position_data);
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL( "%s" ,il_error_msg.c_str());exit(EXIT_FAILURE);
	}

    if(pubudd_data.getNumSubscribers()>0)
	{
        msg_udd_data.header.seq=seq;
		msg_udd_data.header.stamp=timestamp;
		msg_udd_data.header.frame_id=udd_frame_id;
		msg_udd_data.udd_sensor.Mag.x=composite_data.magnetic.c0;
		msg_udd_data.udd_sensor.Mag.y=composite_data.magnetic.c1;
		msg_udd_data.udd_sensor.Mag.z=composite_data.magnetic.c2;

		msg_udd_data.udd_sensor.Accel.x=composite_data.acceleration.c0;
		msg_udd_data.udd_sensor.Accel.y=composite_data.acceleration.c1;
		msg_udd_data.udd_sensor.Accel.z=composite_data.acceleration.c2;
		msg_udd_data.udd_sensor.Gyro.x=composite_data.gyro.c0;
		msg_udd_data.udd_sensor.Gyro.y=composite_data.gyro.c1;
		msg_udd_data.udd_sensor.Gyro.z=composite_data.gyro.c2;
		msg_udd_data.udd_sensor.Temp=composite_data.Temper;
		msg_udd_data.udd_sensor.Vinp=composite_data.Vinp;
		msg_udd_data.udd_sensor.Pressure=composite_data.H_bar;
		msg_udd_data.udd_sensor.Barometric_Height=composite_data.P_bar;
        msg_udd_data.udd_ins.YPR.x=composite_data.ypr.yaw;
		msg_udd_data.udd_ins.YPR.y=composite_data.ypr.pitch;
		msg_udd_data.udd_ins.YPR.z=composite_data.ypr.roll;

		msg_udd_data.udd_gps.Latitude=position_data.Latitude;
		msg_udd_data.udd_gps.Longitude=position_data.Longitude;
		msg_udd_data.udd_gps.Altitude=position_data.Altitude;
		msg_udd_data.udd_gps.East_Speed=position_data.East_Speed;
		msg_udd_data.udd_gps.North_Speed=position_data.North_Speed;
		msg_udd_data.udd_gps.Vertical_Speed=position_data.Vertical_Speed;

        msg_udd_data.udd_gnss.GNSS_Latitude=position_data.GNSS_Latitude;
		msg_udd_data.udd_gnss.GNSS_Longitude=position_data.GNSS_Longitude;
		msg_udd_data.udd_gnss.GNSS_Altitude=position_data.GNSS_Altitude;
		msg_udd_data.udd_gnss.GNSS_Horizontal=position_data.GNSS_Horizontal_Speed;
		msg_udd_data.udd_gnss.GNSS_Track_Over=position_data.GNSS_Trackover_Ground;
		msg_udd_data.udd_gnss.GNSS_Vertical_Speed=position_data.GNSS_Vertical_Speed;

        msg_udd_data.udd_gnss.GNSS_info_1.data=position_data.GNSS_info_1;
		msg_udd_data.udd_gnss.GNSS_info_2.data=position_data.GNSS_info_2;

		msg_udd_data.udd_gnss.GNSS_Velocity_Latency=position_data.GNSS_Velocity_Latency;
		msg_udd_data.udd_gnss.GNSS_Angles_Position_Type.data=position_data.GNSS_Angles_Position_Type;

		msg_udd_data.udd_gnss.GNSS_GDOP=position_data.GNSS_GDOP;
		msg_udd_data.udd_gnss.GNSS_PDOP=position_data.GNSS_PDOP;
        msg_udd_data.udd_gnss.GNSS_HDOP=position_data.GNSS_HDOP;
		msg_udd_data.udd_gnss.GNSS_VDOP=position_data.GNSS_VDOP;
        msg_udd_data.udd_gnss.GNSS_TDOP=position_data.GNSS_TDOP;

		msg_udd_data.udd_gnss.Number_Sat.data=position_data.Number_Sat;

        // msg_udd_data.udd_gnss.GNSS_YPR.x=position_data.ypr.yaw;
		// msg_udd_data.udd_gnss.GNSS_YPR.y=position_data.ypr.pitch;
		// msg_udd_data.udd_gnss.GNSS_YPR.z=position_data.ypr.roll;


        pubudd_data.publish(msg_udd_data);

    }


}
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
		   msg ="Buffer acess erros";
		   break;
		default:
		  msg="Undefined error";
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"onrequest_inertiallabs_ins");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(100); // 100 hz
	std::string port;

	INSSetInternalData data;
	

	//assigning params to variables

	int baudrate,publish_rate,async_output_rate,async_output_type;
	np.param<std::string>("serial_port",port,"/dev/ttyUSB0");
	np.param<int>("serial_baud",baudrate,460800);
	np.param<int>("publish_rate",publish_rate,10);
	np.param<int>("async_output_type",async_output_type,0);
	np.param<int>("async_output_rate",async_output_rate,6);
	np.param<int>("ins_output_format",ins_output_format,IL_USERDEF_DATA_RECEIVE);

	//Initializing Publishers

	pubudd_data   =np.advertise<inertiallabs_msgs::udd_data> ("/Inertial_Labs/udd_data",1);
	//ros::ServiceServer service=n.advertiseService("query_ins_data",send_data);
	
	ROS_INFO("Ready to answer your queries regarding ins data");

	ROS_INFO("connecting to INS. port: %s at a baudrate:%d\n",port.c_str(),baudrate);

	il_err=INS_connect(&ins,port.c_str(),baudrate);
	if(il_err!=ILERR_NO_ERROR) 
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("Could not connect to the sensor on this %s port error:%s\n did you add the user to the dialout group???",
       	    port.c_str(),
	        il_error_msg.c_str() 
		);
		exit(EXIT_FAILURE);
	}


	il_err= INS_Stop(&ins);
	ros::Duration(5).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("stop command error"); exit(EXIT_FAILURE);
	}

	
	il_err= INS_ReadINSpar(&ins);
	ros::Duration(2).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("read command error"); exit(EXIT_FAILURE);
	}

	il_err =INS_ReadInternalParameters(&ins,&data);
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("memory error , buffer have no data left in that"); exit(EXIT_FAILURE);
	}

	ROS_INFO("stoped here ");

	
	/*
	 * \brief Set the data output mode of the INS.
	 */
	il_err= INS_SetMode(&ins,IL_SET_ONREQUEST_MODE); 

	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("SetMode command error");exit(EXIT_FAILURE);
	}

		/*
	 *	\brief  wait for 2 seconds till the communication light off in the INS
	 */
	if(ins.mode)
	{
		ROS_INFO("On Request mode calibaration running ");
		ros::Duration(20).sleep();
	}
    // ins.cmd_flag = IL_USERDEF_DATA_RECEIVE;
	// UserDef_Data_Receive(&ins);
	// ros::Duration(4).sleep();

	 //DataListener();


	ins.cmd_flag = ins_output_format;
  	if(async_output_type ==0)
	{
		ROS_INFO("publishing at %d Hz\n",publish_rate);
		ROS_INFO("rostopic echo the topics to see the data");
		pub_timer=np.createTimer(ros::Duration(1.0/(double)publish_rate),publish_timer);
	}
	

	ros::spin();
	INS_disconnect(&ins);
	return 0;
}
