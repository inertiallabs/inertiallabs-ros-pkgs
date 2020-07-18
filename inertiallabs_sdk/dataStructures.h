#pragma once
#include <cstdint>

struct INSDataStruct
{
	uint32_t ms_gps;
	double GPS_INS_Time;
	double GPS_IMU_Time;
	int32_t UTC_Hour;
	int32_t UTC_Minute;
	int32_t UTC_Second;
	double UTC_DecSec;
	int32_t UTC_Day;
	int32_t UTC_Month;
	int32_t UTC_Year;
	uint64_t UTCSecSinceEpoch;
	double Heading, Pitch, Roll;
	double Quat[4];
	double Latitude, Longitude, Altitude;
	double VelENU[3];
	double Gyro[3];
	double Acc[3];
	double Mag[3];
	double pBar, hBar;
	double GBias[3];
	double ABias[3];
	double LatGNSS, LonGNSS, AltGNSS;
	double V_Hor, Trk_gnd, V_ver;
	double Heading_GNSS, Pitch_GNSS;
	double LatGNSSStd, LonGNSSStd, AltGNSSStd;
	double HeadingGNSSStd, PitchGNSSStd;
	int32_t GNSSInfo1, GNSSInfo2;
	int32_t SVtrack, SVsol, SVsolL1, SVSolMulti, GalBD, GPSGlo, TimeStatus, ExtSolStatus;
	int32_t GNSSSolStatus;
	int32_t GNSSSolType;
	int32_t AnglesType;
	int32_t Week;
	int32_t GNSSVelLatency, GNSSPosMs, GNSSVelMs, GNSSHdgMs;
	int32_t NewGPS;
	double GDOP, PDOP, HDOP, VDOP, TDOP;
	double GNSS_PACC, GNSS_VACC;
	double VSup;
	double VStab;
	double Temp;
	int32_t USW;
	int32_t INSSolStatus;
	double KFLatStd, KFLonStd, KFAltStd, KFHdgStd;
	double Odometer, AirSpeed, WindN, WindE, WindNStd, WindEStd;
	double LatExt, LonExt, AltExt, LatExtStd, LonExtStd, AltExtStd, ExtPosLatency;
	double LocLat, LocLon, LocAlt, LocDopplerShift, LocDopplerShiftStd;
	int32_t NewAiding;
	double HdgExt, HdgExtStd, HdgExtLatency;
	double DVLRight, DVLFwd, DVLUp, DVLRightStd, DVLFwdStd, DVLUpStd, DVLLatency, DVLPressure;
	double GBExt[3], ABExt[3];
	double PitchExt, RollExt;
	int32_t Latency_ms_pos, Latency_ms_vel, Latency_ms_head;
	int32_t UP, UT;
	double GNSS_ECEF_X, GNSS_ECEF_Y, GNSS_ECEF_Z, GNSS_ECEF_VX, GNSS_ECEF_VY, GNSS_ECEF_VZ, DiffAge;
	int32_t LatencyECEF;
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
