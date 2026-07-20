#include "get_mpu6050.h"
#include "Filter.h"
#define Pi 3.14159265
volatile short angle[3];
volatile short accel[3];
volatile float pitch, roll, yaw;

volatile float kal_mpu_out;

volatile float Filter_out;

/* MPU6050航向数据状态，供直线控制器判断数据是否新鲜。 */
volatile uint32_t mpu_yaw_update_count = 0;
volatile uint32_t mpu_yaw_last_ms = 0;
volatile uint8_t mpu_yaw_valid = 0;

volatile float Filter_out_last;

volatile float Yaw_Err_Lowout_last;
volatile float Yaw_Err_Lowout;

volatile float erro_sumr;
volatile float error;

volatile float gyro_x,gyro_y,gyro_z,accel_z;
volatile float Accel_Angle_X,Accel_Angle_Y;
uint8_t Way_Angle=2;
Bias_t Angle;

const int CALIB_SAMPLES = 2;
float Angle_Balance,Gyro_Balance,Gyro_Turn;
float Acceleration_Z;

volatile float yawBias = 0, pitchBias = 0, rollBias = 0;

volatile float calibratedYaw = 0, calibratedPitch = 0, calibratedRoll = 0;

void AngleOffsetCalc(void)
{


    for (int i = 0; i < CALIB_SAMPLES; i++)
    {



//        printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n", yaw, pitch, roll);
        calibratedYaw += calibratedYaw;


//        delay_ms(20);
    }
//    printf("yawsum:%d\r\n", (int)yawSum);

    calibratedYaw = calibratedYaw / CALIB_SAMPLES;


//    printf("yawBias:%.2f, pitchBias:%.2f, rollBias:%.2f", yawBias, pitchBias, rollBias);
}

void Get_CalibratedAngles(void)
{
    if (!mpu_yaw_valid)
    {
        return;
    }

    if (Filter_out < 0.0f)
    {
        calibratedYaw = -Filter_out;
    }
    else
    {
        calibratedYaw = 360.0f - Filter_out;
        if (calibratedYaw >= 360.0f)
        {
            calibratedYaw = 0.0f;
        }
    }
}

float get_Filter(float Yaw_Err)
{
	/* 一阶低通滤波：保留65%的历史量，吸收35%的新测量。 */
	Yaw_Err_Lowout = 0.35f * Yaw_Err + 0.65f * Yaw_Err_Lowout_last;
	Yaw_Err_Lowout_last = Yaw_Err_Lowout;
//			if(error<00.55)
//	{
//		erro_sumr+=error;

//		error=Yaw_Err_Lowout-Yaw_Err_Lowout_last;

//		Yaw_Err_Lowout-=error;
//}
	return Yaw_Err_Lowout;
}




void Get_EulerAngles(void)
{
    float new_pitch;
    float new_roll;
    float new_yaw;





//	    for (int i = 0; i < CALIB_SAMPLES; i++)
//    {
    /* 只有DMP返回成功时才发布新航向，避免错误帧参与控制。 */
    if (mpu_dmp_get_data(&new_pitch, &new_roll, &new_yaw) == 0)
    {
        pitch = new_pitch;
        roll = new_roll;
        yaw = new_yaw;
        Filter_out = new_yaw;
        mpu_yaw_last_ms = Get_Time();
        mpu_yaw_update_count++;
        mpu_yaw_valid = 1;
    }


//			calibratedYaw= Filter_out+Filter_out_last;
//			Filter_out_last=Filter_out;


//        printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n", yaw, pitch, roll);
//        calibratedYaw += calibratedYaw;


////        delay_ms(20);
////    }
//	  calibratedYaw = calibratedYaw / CALIB_SAMPLES;













//		calibratedYaw=Filter_out+180;


//	printf("pitch :%3.2f roll :%3.2f yaw:%3.2f\r\n",pitch,roll,Filter_out);

}


float dir_kp = 7.50,dir_ki=0.00,dir_kd = 5.00;
int Integral_Max = 300; //300
int pid_max = 1000; //3000
float Dir_PID(float error)
{ //5*20/1000*188
    float result = 0;
    static int16_t err_last = 0;
    static float Integral = 0;

//   if(error == 0)
//   {

//   }
    Integral += error;
    if (Integral > Integral_Max) Integral = Integral_Max;
    if (Integral < -Integral_Max) Integral = -Integral_Max;


    result = dir_kp * error +dir_ki*Integral+ dir_kd * (error - err_last);

	 err_last = error;


    if (result > Integral_Max) result = pid_max;
    if (result < -Integral_Max) result = -pid_max;
    return -result;
}


float Yaw_To_Speed(float angle_error)
{
    static float integral = 0;
    #define YS_KP 3.0f
    #define YS_KI 0.005f

    integral += angle_error;
    if (integral > 80)  integral = 80;
    if (integral < -80) integral = -80;

    float correction = YS_KP * angle_error + YS_KI * integral;
    if (correction > 150)  correction = 150;
    if (correction < -150) correction = -150;

    return correction;
}

//Limit the heading_angle to 0-360 degrees(to prevent the range of heading_angle over 0-360 degrees beacuse of Addition or subtraction operations )
float navigetion_0_360_limit(float angle)
{
		float temp = 0;
		 if(angle > 360)
		{
			temp = angle - 360;
		}
		else
		{
			temp = angle;
		}
		return temp;
}


//Calculate the minor arc deviation in the 0-360 Navigation Coordinate System (Counterclockwise negative arc Angle is Positive, Clockwise is Negative)

//Calculate the minor arc deviation in the 0-360 Navigation Coordinate System (Counterclockwise negative arc Angle is Positive, Clockwise is Negative)
float get_minor_arc(float azimuth,float headingAngle)
{
    float angle_err = 0.0;
    if(azimuth >= 180 + headingAngle)
    {
        angle_err = azimuth - headingAngle - 360;
    }
    else if(headingAngle > 180 + azimuth)
    {
        angle_err = azimuth - headingAngle + 360;
    }
    else
    {
        angle_err =  azimuth - headingAngle;
    }
    return -angle_err;
}



void Get_Angle(uint8_t way)
{

	if(way==1)
	{
		if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
        {
          //  printf("\r\npitch =%d\r\n", (int)pitch);
           // printf("\r\nroll =%d\r\n", (int)roll);
//            printf("\r\nyaw =%3.2f\r\n", yaw);
        }
        delay_ms(10);
	}
	else
	{

		MPU6050ReadGyro(angle);
		MPU6050ReadAcc(accel);
		Accel_Angle_X=atan2(accel[0],accel[2])*180/Pi;
		Accel_Angle_Y=atan2(accel[1],accel[2])*180/Pi;
		accel_z=accel[2]*1.962/32768;
		gyro_z = angle[2] * 2000 / 32768;
		delay_ms(10);
		if(Way_Angle==2)
		{
			pitch= -Kalman_Filter_x(Accel_Angle_X,gyro_x);//鍗″皵鏇兼护娉?
			roll = -Kalman_Filter_y(Accel_Angle_Y,gyro_y);


		}
		else if(Way_Angle==3)
		{
			 pitch = -Complementary_Filter_x(Accel_Angle_X,gyro_x);
			 roll = -Complementary_Filter_y(Accel_Angle_Y,gyro_y);
		}
		Angle_Balance=pitch;
		Gyro_Turn=angle[2];
		Acceleration_Z=accel[2];

	}

}


