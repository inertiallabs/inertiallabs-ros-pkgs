#pragma once
#include <cstdint>

namespace IL {

	namespace PacketType {
		const uint8_t Cobham_UAV200_Satcom = 0x46;
		const uint8_t INS_Sensors = 0x50;
		const uint8_t INS_OPVT = 0x52;
		const uint8_t INS_min = 0x53;
		const uint8_t INS_NMEA = 0x54;
		const uint8_t INS_Sensors_NMEA = 0x55;
		const uint8_t INS_QPVT = 0x56;
		const uint8_t INS_OPVT2A = 0x57;
		const uint8_t INS_OPVT2AHR = 0x58;
		const uint8_t INS_OPVT2AW = 0x59;
		const uint8_t INS_OPVTAD = 0x61;
		const uint8_t INS_OPVT_rawIMU = 0x66;
		const uint8_t INS_OPVT_GNSSext = 0x67;
		const uint8_t SPAN_rawIMU = 0x68;
		const uint8_t INS_UDD = 0x95;

	}

	struct INSDataStruct
	{
		unsigned int ms_gps;
		double GPS_INS_Time;
		double GPS_IMU_Time;
		int UTC_Hour;
		int UTC_Minute;
		int UTC_Second;
		double UTC_DecSec;
		int UTC_Day;
		int UTC_Month;
		int UTC_Year;
		uint64_t UTCSecSinceEpoch;
		double Heading, Pitch, Roll;
		double Quat[4];
		double Latitude, Longitude, Altitude;
		double VelENU[3];
		double Gyro[3];
		double Acc[3];
		double AccPVPoint[3];
		double Mag[3];
		double pBar, hBar;
		double GBias[3];
		double ABias[3];
		double LatGNSS, LonGNSS, AltGNSS;
		double V_Hor, Trk_gnd, V_ver;
		double Heading_GNSS, Pitch_GNSS;
		double LatGNSSStd, LonGNSSStd, AltGNSSStd;
		double HeadingGNSSStd, PitchGNSSStd;
		int GNSSInfo1, GNSSInfo2;
		int SVtrack, SVsol, SVsolL1, SVSolMulti, GalBD, GPSGlo, TimeStatus, ExtSolStatus;
		int GNSSSolStatus;
		int GNSSSolType;
		int AnglesType;
		int Week;
		int GNSSVelLatency, GNSSPosMs, GNSSVelMs, GNSSHdgMs;
		int NewGPS;
		double GDOP, PDOP, HDOP, VDOP, TDOP;
		double GNSS_PACC, GNSS_VACC;
		double VSup;
		double VStab;
		double Temp;
		int USW;
		int INSSolStatus;
		double KFLatStd, KFLonStd, KFAltStd, KFHdgStd, KFVelStd[3];
		double Odometer, AirSpeed, WindN, WindE, WindNStd, WindEStd;
		double LatExt, LonExt, AltExt, LatExtStd, LonExtStd, AltExtStd, ExtPosLatency;
		double LocLat, LocLon, LocAlt, LocDopplerShift, LocDopplerShiftStd;
		double ExtAntPri[3], ExtAntSec[3];
		int NewAiding;
		double HdgExt, HdgExtStd, HdgExtLatency;
		double DVLRight, DVLFwd, DVLUp, DVLRightStd, DVLFwdStd, DVLUpStd, DVLLatency, DVLPressure;
		double GBExt[3], ABExt[3];
		double PitchExt, RollExt;
		int Latency_ms_pos, Latency_ms_vel, Latency_ms_head;
		int UP, UT;
		double GNSS_ECEF_X, GNSS_ECEF_Y, GNSS_ECEF_Z, GNSS_ECEF_VX, GNSS_ECEF_VY, GNSS_ECEF_VZ, GNSS_ECEF_VXStd, GNSS_ECEF_VYStd, GNSS_ECEF_VZStd, DiffAge;
		int LatencyECEF;
		int PPPStore, PPPApp;
		uint64_t dataPresent[8];
	};


#ifdef _MSC_VER
#pragma pack(push,1)
#endif
	struct INSDeviceInfo {
		char IDN[8];
		char FW[40];
		uint8_t pressSensor;
		uint8_t imuType;
		char imuSN[8];
		char imuFW[40];
		char GNSSmodel[16];
		char GNSSsn[16];
		char GNSShw[16];
		char GNSSfw[16];
		uint16_t week;
		uint8_t GNSSmaxRate;
		uint8_t reserved;
#ifdef __GNUC__
	} __attribute__((packed));
#else
};
#endif
	struct INSDevicePar {
		uint16_t dataRate;
		uint16_t initAlignmentTime;
		int32_t magDeclination;
		int32_t Lat, Lon, Alt;
		uint8_t YearSince1900, Month, Day;
		int16_t AlignmentAngles[3], COGtoINSleverArm[3], AntennaToINSleverArm[3];
		uint8_t GeoidAltitude;
		char reserved1[8];
		char IDN[8];
		uint8_t enableBaroAltimeter;
		char reserved2;
#ifdef __GNUC__
	} __attribute__((packed));
#else
};
#endif
#ifdef _MSC_VER
#pragma pack(pop)
#endif

}
