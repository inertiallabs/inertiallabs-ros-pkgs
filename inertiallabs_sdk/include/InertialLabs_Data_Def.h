#ifndef _IL_DATA_DEF_H_
#define _IL_DATA_DEF_H_


#include "IL_kinematics.h"
// enum user_data_defination {

//     = 4,

// // };
//  6 1 2 3 4 5 6  1234r45554 1122
// 12
// 7 + 6 + 12 25
typedef struct {
		double          GPS_INS_Time_round;                  
		double          GPS_INS_Time;                       
		double 			GPS_IMU_Time;						
	  //UTC_Struct 		UTC; 
                   
} UDD_TimeData;

typedef struct {
		IlYpr           ypr;                        
		IlQuaternion    quaternion;                     
		IlYpr           ypr_HR;                        
                   
} UDD_OrientationData;

typedef struct {
		IlYpr           ypr;                        
		IlQuaternion    quaternion;                     
		IlYpr           ypr_HR;                        
                   
} UDD_NavigationData;


int udd_size_aaray[112] = {
    -1 , 4 , 8 , 8 , 9 , -1 , -1,
     6 , 12 , 8 ,-1 , -1 ,-1 , -1, 
    -1 , 12 , 20 ,12 , -1 , -1 , -1 ,
    -1 , -1 , -1  , -1 , -1 , 12 , -1 ,
    -1 , -1 , -1 , 6 , 12 , 6  ,12 , 6 ,
     6 ,  7 , 12 , -1 , -1 , -1 , -1 , -1 ,
    -1 , -1 , -1 , 12 , 20 , 10 ,4 ,6, 4, 2,
     8 ,  1 , 1 ,1 ,1 ,2 ,2 ,4 ,4 ,4 ,1 ,10 , 4,
     4 ,  2 , -1 , 2, 6, 2 , -1 , -1 , -1 ,-1 , -1 ,
     2 ,  2 , 2 , 2 , 1 , 3 , 1 , 6 , 3 , -1 , -1 ,-1 ,
    -1 , -1 ,-1 , 4 ,2 ,8, 20 ,16 , 2 ,6, 24, 7, 4, 12,
    -1 , -1 , -1 , -1 , -1 
}
#endif /* _IL_DATA_DEF_H_ */