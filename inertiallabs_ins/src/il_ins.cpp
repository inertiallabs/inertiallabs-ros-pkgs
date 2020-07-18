#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>

//Inertial Labs source header
#include "ILDriver.h"
#include <ros/ros.h>


//adding message type headers
#include <inertiallabs_msgs/gps_data.h>
#include <inertiallabs_msgs/sensor_data.h>
#include <inertiallabs_msgs/ins_data.h>
#include <inertiallabs_msgs/gnss_data.h>

//Publishers
ros::Publisher pubsens_data;
ros::Publisher pubins_data;
ros::Publisher pubgps_data;
ros::Publisher pubgnss_data;

//Device
ILDriver ins;
int ins_output_format;
std::string imu_frame_id;
ros::Timer pub_timer;

void publish_device()
{
	static int seq=0;
	seq++;

	inertiallabs_msgs::gps_data msg_gps_data;
	inertiallabs_msgs::gnss_data msg_gnss_data;
	inertiallabs_msgs::ins_data msg_ins_data;
	inertiallabs_msgs::sensor_data msg_sensor_data;

	ros::Time timestamp=ros::Time::now();
	
	auto data = ins.getLatestData();

	if(pubsens_data.getNumSubscribers()>0)
	{
		msg_sensor_data.header.seq=seq;
		msg_sensor_data.header.stamp=timestamp;
		msg_sensor_data.header.frame_id=imu_frame_id;
		msg_sensor_data.Mag.x=data.Mag[0];
		msg_sensor_data.Mag.y=data.Mag[0];
		msg_sensor_data.Mag.z=data.Mag[0];
		msg_sensor_data.Accel.x=data.Acc[0];
		msg_sensor_data.Accel.y=data.Acc[1];
		msg_sensor_data.Accel.z=data.Acc[2];
		msg_sensor_data.Gyro.x=data.Gyro[0];
		msg_sensor_data.Gyro.y=data.Gyro[1];
		msg_sensor_data.Gyro.z=data.Gyro[2];
		msg_sensor_data.Temp=data.Temp;
		msg_sensor_data.Vinp=data.VSup;
		msg_sensor_data.Pressure=data.hBar;
		msg_sensor_data.Barometric_Height=data.pBar;
		pubsens_data.publish(msg_sensor_data);
	}
	if(pubins_data.getNumSubscribers()>0)
	{
		msg_ins_data.header.seq=seq;
		msg_ins_data.header.stamp=timestamp;
		msg_ins_data.header.frame_id=imu_frame_id;
		msg_ins_data.YPR.x=data.Heading;
		msg_ins_data.YPR.y=data.Pitch;
		msg_ins_data.YPR.z=data.Roll;
		msg_ins_data.LLH.x=data.Latitude;
		msg_ins_data.LLH.y=data.Longitude;
		msg_ins_data.LLH.z=data.Altitude;
		msg_ins_data.Vel_ENU.x=data.VelENU[0];
		msg_ins_data.Vel_ENU.y=data.VelENU[1];
		msg_ins_data.Vel_ENU.z=data.VelENU[2];
		msg_ins_data.GPS_INS_Time=data.GPS_INS_Time;
		msg_ins_data.GPS_mSOW.data=data.ms_gps;
		pubins_data.publish(msg_ins_data);
	}
	if(pubgps_data.getNumSubscribers()>0)
	{
		msg_gps_data.header.seq=seq;
		msg_gps_data.header.stamp=timestamp;
		msg_gps_data.header.frame_id=imu_frame_id;
		msg_gps_data.Latitude=data.LatGNSS;
		msg_gps_data.Longitude=data.LonGNSS;
		msg_gps_data.Altitude=data.AltGNSS;
		msg_gps_data.HorSpeed=data.V_Hor;
		msg_gps_data.SpeedDir=data.Trk_gnd;
		msg_gps_data.VerSpeed=data.V_ver;
		pubgps_data.publish(msg_gps_data);
    }
	if(pubgnss_data.getNumSubscribers()>0)
	{
		msg_gnss_data.header.seq=seq;
		msg_gnss_data.header.stamp=timestamp;
		msg_gnss_data.header.frame_id=imu_frame_id;
		msg_gnss_data.GNSS_info_1=data.GNSSInfo1;
		msg_gnss_data.GNSS_info_2=data.GNSSInfo2;
		msg_gnss_data.Number_Sat=data.SVsol;
		msg_gnss_data.GNSS_Velocity_Latency=data.GNSSVelLatency;
		msg_gnss_data.GNSS_Angles_Position_Type=data.AnglesType;
		msg_gnss_data.GNSS_Heading=data.Heading_GNSS;
		msg_gnss_data.GNSS_Pitch=data.Pitch_GNSS;
		msg_gnss_data.GNSS_GDOP=data.GDOP;
		msg_gnss_data.GNSS_PDOP=data.PDOP;
		msg_gnss_data.GNSS_HDOP=data.HDOP;
		msg_gnss_data.GNSS_VDOP=data.VDOP;
		msg_gnss_data.GNSS_TDOP=data.TDOP;
		pubgnss_data.publish(msg_gnss_data);
	}
}
//defining timer publication
void publish_timer(const ros::TimerEvent&)
{
	publish_device();
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"il_ins");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(100); // 100 hz
	std::string port;

	//command line varibales

	int baudrate,publish_rate,async_output_rate,async_output_type;
	np.param<std::string>("serial_port",port,"/dev/ttyUSB0");
	np.param<int>("serial_baud",baudrate,460800);
	np.param<int>("publish_rate",publish_rate,100);
	np.param<int>("ins_output_format",ins_output_format,0x58);

	//Initializing Publishers

	pubsens_data   =np.advertise<inertiallabs_msgs::sensor_data> ("/Inertial_Labs/sensor_data",1);
	pubins_data    =np.advertise<inertiallabs_msgs::ins_data> ("/Inertial_Labs/ins_data",1);
	pubgps_data    =np.advertise<inertiallabs_msgs::gps_data> ("/Inertial_Labs/gps_data",1);
	pubgnss_data   =np.advertise<inertiallabs_msgs::gnss_data>("/Inertial_Labs/gnss_data",1); 

	
	ROS_INFO("Ready to answer your queries regarding ins data");

	ROS_INFO("connecting to INS. port: %s at a baudrate:%d\n",port.c_str(),baudrate);

	auto il_err=ins.connect(port.c_str(),baudrate);
	if(il_err!=0) 
	{
		ROS_FATAL("Could not connect to the sensor on this %s port error:\n did you add the user to the dialout group???",
       	        port.c_str()
		);
		exit(EXIT_FAILURE);
	}
	
	if (!ins.isStarted())
	{
	    auto devInfo = ins.getDeviceInfo();
	    std::string SN(reinterpret_cast<const char *>(devInfo.IDN),8);
	    ROS_INFO("Found INS S/N %s\n", SN.c_str());
	    il_err = ins.start(ins_output_format);
	    if(il_err!=0) 
	    {
		    ROS_FATAL("Could not start the sensor: %i\n", il_err);
		    ins.disconnect();
		    exit(EXIT_FAILURE);
	    }

 	ROS_INFO("publishing at %d Hz\n",publish_rate);
	ROS_INFO("rostopic echo the topics to see the data");

	pub_timer=np.createTimer(ros::Duration(1.0/(double)publish_rate),publish_timer);
	}

	ros::spin();
	ins.stop();
	ROS_INFO("Disconnecting the INS");
	ins.disconnect();
	return 0;
}
