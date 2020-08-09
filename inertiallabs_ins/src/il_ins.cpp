#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>
#include <ros/ros.h>

//Inertial Labs source header
#include "ILDriver.h"

//adding message type headers
#include <inertiallabs_msgs/ins_data.h>

//Publishers

struct Context {
	ros::Publisher publisher;
	std::string imu_frame_id;
};

void publish_device(IL::INSDataStruct *data, void* contextPtr)
{
	Context * context = reinterpret_cast<Context*>(contextPtr);
	static int seq=0;
	seq++;

	inertiallabs_msgs::ins_data msg_ins_data;

	ros::Time timestamp=ros::Time::now();
	
	if(context->publisher.getNumSubscribers()>0)
	{
		msg_ins_data.header.seq=seq;
		msg_ins_data.header.stamp=timestamp;
		msg_ins_data.header.frame_id=context->imu_frame_id;
		msg_ins_data.GPS_INS_Time=data->GPS_INS_Time;
		msg_ins_data.GPS_IMU_Time=data->GPS_IMU_Time;
		msg_ins_data.Gyro.x=data->Gyro[0];
		msg_ins_data.Gyro.y=data->Gyro[1];
		msg_ins_data.Gyro.z=data->Gyro[2];
		msg_ins_data.Acc.x=data->Acc[0];
		msg_ins_data.Acc.y=data->Acc[1];
		msg_ins_data.Acc.z=data->Acc[2];
		msg_ins_data.OriQuat.w=data->Quat[0];
		msg_ins_data.OriQuat.x=data->Quat[1];
		msg_ins_data.OriQuat.y=data->Quat[2];
		msg_ins_data.OriQuat.z=data->Quat[3];
		msg_ins_data.INS_LLH.x=data->Latitude;
		msg_ins_data.INS_LLH.y=data->Longitude;
		msg_ins_data.INS_LLH.z=data->Altitude;
		msg_ins_data.Vel_ENU.x=data->VelENU[0];
		msg_ins_data.Vel_ENU.y=data->VelENU[1];
		msg_ins_data.Vel_ENU.z=data->VelENU[2];
		msg_ins_data.GNSS_LLH.x=data->LatGNSS;
		msg_ins_data.GNSS_LLH.y=data->LonGNSS;
		msg_ins_data.GNSS_LLH.z=data->AltGNSS;
		msg_ins_data.GDOP = data->GDOP;
		msg_ins_data.PDOP = data->PDOP;
		msg_ins_data.HDOP = data->HDOP;
		msg_ins_data.VDOP = data->VDOP;
		msg_ins_data.TDOP = data->TDOP;
		msg_ins_data.Satellites = data->SVsol;
		msg_ins_data.GNSS_Info1 = data->GNSSInfo1;
		msg_ins_data.GNSS_Info2 = data->GNSSInfo2;
		msg_ins_data.Solution_Status = data->INSSolStatus;
		msg_ins_data.GNSS_Heading_Type = data->AnglesType;
		msg_ins_data.New_GNSS_Flags = data->NewGPS;
		msg_ins_data.GNSS_Track_GND = data->Trk_gnd;
		msg_ins_data.Diff_Age = data->DiffAge;
		msg_ins_data.USW = data->USW;
		context->publisher.publish(msg_ins_data);
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"il_ins");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(100); // 100 hz
	std::string port;
	IL::Driver ins;
	int ins_output_format;
	std::string imu_frame_id;
	Context context;

	//command line varibales

	np.param<std::string>("ins_url",port,"serial:/dev/ttyUSB0:460800");
	np.param<int>("ins_output_format",ins_output_format,0x95);

	//Initializing Publishers

	context.publisher = np.advertise<inertiallabs_msgs::ins_data> ("/Inertial_Labs/ins_data",1);


	ROS_INFO("connecting to INS at URL %s\n",port.c_str());

	auto il_err=ins.connect(port.c_str());
	if(il_err!=0) 
	{
		ROS_FATAL("Could not connect to the INS on this URL %s\n",
       	        port.c_str()
		);
		exit(EXIT_FAILURE);
	}
	
	if (ins.isStarted())
	{
		ins.stop();
	}
	auto devInfo = ins.getDeviceInfo();
	auto devParams = ins.getDeviceParams();
	std::string SN(reinterpret_cast<const char *>(devInfo.IDN),8);
	ROS_INFO("Found INS S/N %s\n", SN.c_str());
	context.imu_frame_id = SN;
	il_err = ins.start(ins_output_format);
	if(il_err!=0) 
	{
		ROS_FATAL("Could not start the INS: %i\n", il_err);
		ins.disconnect();
		exit(EXIT_FAILURE);
	}
	ins.setCallback(&publish_device, &context);
 	ROS_INFO("publishing at %d Hz\n",devParams.dataRate);
	ROS_INFO("rostopic echo the topics to see the data");
	ros::spin();
	std::cout << "Stopping INS..." << std::flush;
	ins.stop();
	std::cout << "Disconnecting..." << std::flush;
	ins.disconnect();
	std::cout << "Done." << std::endl;
	return 0;
}
