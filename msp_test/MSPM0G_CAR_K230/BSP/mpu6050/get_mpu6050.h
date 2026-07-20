#ifndef	_APP_MPU6050_H_
#define _APP_MPU6050_H_

#include "AllHeader.h"
#include "bsp_mpu6050.h"
#include "inv_mpu.h"
#include "delay.h"




extern volatile float pitch, roll, yaw;
extern volatile float Filter_out;
extern volatile uint32_t mpu_yaw_update_count;
extern volatile uint32_t mpu_yaw_last_ms;
extern volatile uint8_t mpu_yaw_valid;

extern volatile float yawBias,pitchBias,rollBias;
extern volatile float calibratedYaw, calibratedPitch, calibratedRoll;
extern volatile short angle[3];
extern volatile short accel[3];
//extern volatile float get_yaw;
typedef struct
{
	float Xoffset;
	float Yoffset;
	float Zoffset;
} Bias_t;



float get_Filter(float Encoder_Err)	;
void Get_EulerAngles(void);
float Dir_PID(float error);
float Yaw_To_Speed(float angle_error);
float navigetion_0_360_limit(float angle);
float get_minor_arc(float azimuth,float headingAngle);
void AngleOffsetCalc(void) ;
void Get_CalibratedAngles(void);
void Get_Angle(uint8_t way);

float get_minor_arc(float azimuth,float headingAngle) ;
float navigetion_0_360_limit(float angle);
#endif

