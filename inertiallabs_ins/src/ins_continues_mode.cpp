#include<iostream>
#include<unistd.h>
#include<math.h>
#include<stdlib.h>

//Inertial Labs source header
#include "InertialLabs_INS.h"
#include <ros/ros.h>


//adding message type headers
#include <inertiallabs_msgs/gps_data.h>
#include <inertiallabs_msgs/sensor_data.h>
#include <inertiallabs_msgs/ins_data.h>

//Publishers
ros::Publisher pubsens_data;
ros::Publisher pubins_data;
ros::Publisher pubgps_data;
ros::Publisher pubquat_data;
ros::Publisher pubpressure_barometric_data;
ros::Publisher pubgnss_data;

//Device
IL_INS ins;
int ins_output_format;
std::string imu_frame_id;
IL_ERROR_CODE il_err;         
std::string il_error_msg;
ros::Timer pub_timer;

void ilerror_msg(IL_ERROR_CODE il_error,std::string &msg);

#if 0
void asyncDataListener(IL_INS* ins,INSCompositeData* data)
{

	ros::Time timestamp=ros::Time::now();
	static int seq =0;
	ROS_INFO("\nASYNC DATA:\n"
	         "YPR.Yaw: %+#7.2f\n"
	         "YPR.pitch: %+#7.2f\n"
	         "YPR.roll: %+#7.2f\n",
	         data->ypr.yaw,
	         data->ypr.pitch, 
	         data->ypr.roll);
	ROS_INFO(
	     "\n quaternion.X:  %+#7.2f\n"
	     "   quaternion.Y:  %+#7.2f\n"
	     "   quaternion.Z:  %+#7.2f\n"
	     "   quaternion.W:  %+#7.2f\n", 
	        data->quaternion.x,
	        data->quaternion.y,
	        data->quaternion.z,
	        data->quaternion.w);
	ROS_INFO(
	       "\n            {Value,Voltage}\n"
	       " magnetic X:  %+#7.2f, %+#7.2f\n"
	       " magnetic Y:  %+#7.2f, %+#7.2f\n"
	       " magnetic Z:  %+#7.2f, %+#7.2f\n",
	         data->magnetic.c0,data->magneticVoltage.c0, 
	         data->magnetic.c1,data->magneticVoltage.c1,
	         data->magnetic.c2,data->magneticVoltage.c2);
	ROS_INFO(
	       "\n acceleration X:    %+#7.2f,%+#7.2f\n"
	       "  acceleration Y:     %+#7.2f,%+#7.2f\n"
	       "  acceleration Z:     %+#7.2f,%+#7.2f\n",
	          data->acceleration.c0,
	          data->accelerationVoltage.c0,
	          data->acceleration.c1,
	          data->accelerationVoltage.c1,
	          data->acceleration.c2,
	          data->accelerationVoltage.c2);
	ROS_INFO(
	       "\n                   {Value ,Voltage,Bias,BiasVariance}"
       	"  angularRate X:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Y:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Z:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n",
		  data->angularRate.c0,
	          data->angularRateVoltage.c0, 
		  data->angularRateBias.c0,
	          data->angularRateBiasVariance.c0, 
		  data->angularRate.c1,    
	          data->angularRateVoltage.c1, 
		  data->angularRateBias.c1, 
	          data->angularRateBiasVariance.c1, 
		  data->angularRate.c2,    
	          data->angularRateVoltage.c2,
		  data->angularRateBias.c2, data->angularRateBiasVariance.c2);
	 ROS_INFO(
		  "\n  Attitude Variance X:    %+#7.2f\n"
		  "  Attitude Variance Y:    %+#7.2f\n"
		  "  Attitude Variance Z:    %+#7.2f\n",
		  data->attitudeVariance.c0, 
		  data->attitudeVariance.c1, 
		  data->attitudeVariance.c2);
	
	 ROS_INFO(
		  "\n  Direction Cosine Matrix:\n"
		  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
		  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
		  "    %+#7.2f, %+#7.2f, %+#7.2f\n",
		  data->dcm.c00, data->dcm.c01, data->dcm.c02,
		  data->dcm.c10, data->dcm.c11, data->dcm.c12,
		  data->dcm.c20, data->dcm.c21, data->dcm.c22);
	
	    ROS_INFO(
		  "\n  Temperature:            %+#7.2f\n"
		  "  Temperature Voltage:    %+#7.2f\n",
		  data->temperature,
		  data->temperatureVoltage);
	

}
#endif
void publish_device()
{
	static int seq=0;
	seq++;
	//IL_ERROR_CODE il_error ;


	INSCompositeData sensor_data;
	INSPositionData pos_data;

	inertiallabs_msgs::gps_data msg_gps_data;
	inertiallabs_msgs::ins_data msg_ins_data;
	inertiallabs_msgs::sensor_data msg_sensor_data;

	ros::Time timestamp=ros::Time::now();

	if(pubsens_data.getNumSubscribers()>0)
	{
		//ROS_INFO("subscribed");
		
		il_err = INS_getGyroAccMag(&ins,&sensor_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());
			
			//exit(EXIT_FAILURE);
		}
		il_err = INS_getPressureBarometricData(&ins,&sensor_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());//exit(EXIT_FAILURE);
		}

		msg_sensor_data.header.seq=seq;
		msg_sensor_data.header.stamp=timestamp;
		msg_sensor_data.header.frame_id=imu_frame_id;
		msg_sensor_data.Mag.x=sensor_data.magnetic.c0;
		msg_sensor_data.Mag.y=sensor_data.magnetic.c1;
		msg_sensor_data.Mag.z=sensor_data.magnetic.c2;
		msg_sensor_data.Accel.x=sensor_data.acceleration.c0;
		msg_sensor_data.Accel.y=sensor_data.acceleration.c1;
		msg_sensor_data.Accel.z=sensor_data.acceleration.c2;
		msg_sensor_data.Gyro.x=sensor_data.gyro.c0;
		msg_sensor_data.Gyro.y=sensor_data.gyro.c1;
		msg_sensor_data.Gyro.z=sensor_data.gyro.c2;
		msg_sensor_data.Temp=sensor_data.Temper;
		msg_sensor_data.Vinp=sensor_data.Vinp;
		msg_sensor_data.Pressure=sensor_data.H_bar;
		msg_sensor_data.Barometric_Height=sensor_data.P_bar;
		pubsens_data.publish(msg_sensor_data);
		
	}
	if(pubins_data.getNumSubscribers()>0)
	{
			
		//ROS_INFO("subscribed");
		il_err = INS_YPR(&ins,&sensor_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());//exit(EXIT_FAILURE);
		}
				
		msg_ins_data.header.seq=seq;
		msg_ins_data.header.stamp=timestamp;
		msg_ins_data.header.frame_id=imu_frame_id;
		msg_ins_data.YPR.x=sensor_data.ypr.yaw;
		msg_ins_data.YPR.y=sensor_data.ypr.pitch;
		msg_ins_data.YPR.z=sensor_data.ypr.roll;
		pubins_data.publish(msg_ins_data);
			
	}
	if(pubgps_data.getNumSubscribers()>0)
	{
		//INSPositionData pos_data;
		
		il_err = INS_getPositionData(&ins,&pos_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());//exit(EXIT_FAILURE);
		}

		msg_gps_data.header.seq=seq;
		msg_gps_data.header.stamp=timestamp;
		msg_gps_data.header.frame_id=imu_frame_id;
		msg_gps_data.Latitude=pos_data.Latitude;
		msg_gps_data.Longitude=pos_data.Longitude;
		msg_gps_data.Altitude=pos_data.Altitude;
		msg_gps_data.East_Speed=pos_data.East_Speed;
		msg_gps_data.North_Speed=pos_data.North_Speed;
		msg_gps_data.Vertical_Speed=pos_data.Vertical_Speed;
		pubgps_data.publish(msg_gps_data);

	}
	if(pubquat_data.getNumSubscribers()>0 && ins_output_format == 3 )
	{
		il_err = INS_getQuaternionData(&ins,&sensor_data);
		if(il_err!=ILERR_NO_ERROR)
		{
			ilerror_msg(il_err,il_error_msg);
			ROS_FATAL( "%s" ,il_error_msg.c_str());//exit(EXIT_FAILURE);
		}
		
		msg_ins_data.header.seq=seq;
		msg_ins_data.header.stamp=timestamp;
		msg_ins_data.header.frame_id=imu_frame_id;
		msg_ins_data.quat_data[0]=sensor_data.quaternion.Lk0;
		msg_ins_data.quat_data[1]=sensor_data.quaternion.Lk1;
		msg_ins_data.quat_data[2]=sensor_data.quaternion.Lk2;
		msg_ins_data.quat_data[3]=sensor_data.quaternion.Lk3;
		pubquat_data.publish(msg_ins_data);

	

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
	ros::init(argc,argv,"continues_inertiallabs_ins");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Rate r(100); // 100 hz
	std::string port;

	//INSSetInternalData data;

	//commadn line varibales

	int baudrate,publish_rate,async_output_rate,async_output_type;
	np.param<std::string>("serial_port",port,"/dev/ttyUSB0");
	np.param<int>("serial_baud",baudrate,460800);
	np.param<int>("publish_rate",publish_rate,100);
	np.param<int>("async_output_type",async_output_type,0);
	np.param<int>("async_output_rate",async_output_rate,6);
	np.param<int>("ins_output_format",ins_output_format,IL_USERDEF_DATA_RECEIVE);

	//Initializing Publishers

	pubsens_data   =np.advertise<inertiallabs_msgs::sensor_data> ("/Inertial_Labs/sensor_data",1);
	pubins_data    =np.advertise<inertiallabs_msgs::ins_data> ("/Inertial_Labs/ins_data",1);
	pubgps_data    =np.advertise<inertiallabs_msgs::gps_data> ("/Inertial_Labs/gps_data",1);
	pubquat_data   =np.advertise<inertiallabs_msgs::ins_data>("/Inertial_Labs/quat_data",1); 

	
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

	//ins.cmd_flag = IL_STOP_CMD;
	il_err= INS_Stop(&ins);
	ros::Duration(5).sleep();
	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("stop command error");exit(EXIT_FAILURE);
	}

	// il_err= INS_ReadINSpar(&ins);
	// ros::Duration(2).sleep();
	// if(il_err!=ILERR_NO_ERROR)
	// {
	// 	ilerror_msg(il_err,il_error_msg);
	// 	ROS_FATAL("read command error"); exit(EXIT_FAILURE);
	// }

	//INS_ReadInternalParameters(&ins,&data);


	/*
	 * \brief Set the data output mode of the INS.
	 */
	il_err= INS_SetMode(&ins,IL_SET_CONTINUES_MODE); 

	if(il_err!=ILERR_NO_ERROR)
	{
		ilerror_msg(il_err,il_error_msg);
		ROS_FATAL("SetMode command error");exit(EXIT_FAILURE);
	}
	/*
	 *	\brief  wait for 2 seconds till the communication light off in the INS
	 */
	ros::Duration(2).sleep();

	ins.cmd_flag = ins_output_format;
	switch (ins_output_format)
	{	
		case IL_OPVT_RECEIVE:
			il_err = INS_OPVTdata_Receive(&ins);
			break;

		case IL_QPVT_RECEIVE:
			il_err = INS_QPVTdata_Receive(&ins);
			break;

		case IL_OPVT2A_RECEIVE:
			il_err = INS_OPVT2Adata_Receive(&ins);
			break;

		case IL_OPVT2AW_RECEIVE:
			il_err = INS_OPVT2AWdata_Receive(&ins);;
			break;

		case IL_OPVT2AHR_RECEIVE:
			il_err = INS_OPVT2AHRdata_Receive(&ins);
			break;

		case IL_OPVTAD_RECEIVE:
			il_err = INS_OPVTADdata_Receive(&ins);
			break;

		case IL_MINIMAL_DATA_RECEIVE:
			il_err  = INS_Minimaldata_Receive(&ins);
			break;
		
		case IL_OPVT_GNSSEXT_DATA_RECEIVE:
			il_err = INS_OPVT_GNSSextdata_Receive(&ins);
			break;

		case IL_OPVT_RAWIMU_DATA_RECEIVE:
			il_err = INS_OPVT_rawIMUdata_Receive(&ins);
			break;
		case IL_SENSOR_DATA_RECEIVE:
			il_err = INS_SensorsData_Receive(&ins);
			break;

		case IL_NMEA_RECEIVE:
			il_err = INS_NMEA_Receive(&ins);
			break;

		case IL_NMEA_SENSORS_RECEIVE:
			il_err = INS_Sensors_NMEA_Receive(&ins);
			break;
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
		ROS_FATAL(" command error");exit(EXIT_FAILURE);
	}

  	if(async_output_type ==0)
	{
		ROS_INFO("publishing at %d Hz\n",publish_rate);
		ROS_INFO("rostopic echo the topics to see the data");

		pub_timer=np.createTimer(ros::Duration(1.0/(double)publish_rate),publish_timer);
	}


	//INS_registerDataReceivedListener(&ins,&asyncDataListener);
	ros::spin();
	INS_disconnect(&ins);
	ROS_INFO("Disconnecting the INS");
	return 0;
}