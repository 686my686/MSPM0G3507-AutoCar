#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_



#include "AllHeader.h"

// 450RPM���������תһ��Ȧ����������õ�������=���ٱ�*��������*���������壨45*13*4��
#define ENCODER_CIRCLE_450 (2340.0f)

// Half of the sum of the distances between the car chassis motors С�����̵�����֮�͵�һ��
#define MSPM0Car_APB (157.09f) //(143.8+170.38)/2

// The displacement of a wheel in one complete revolution, measured in meters ����תһ��Ȧ��λ�ƣ���λΪ��
#define MECANUM_CIRCLE_MM (210.486f)

// ֹͣģʽ��STOP_FREE��ʾ����ֹͣ��STOP_BRAKE��ʾɲ����
//Stop mode, STOP_ FREE stands for free stop, STOP_ BRAKE stands for braking.
typedef enum _stop_mode
{
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;

typedef enum _motion_state
{
    MOTION_STOP = 0,
    MOTION_RUN,
    MOTION_BACK,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_SPIN_LEFT,
    MOTION_SPIN_RIGHT,
    MOTION_BRAKE,

    MOTION_MAX_STATE
} motion_state_t;

typedef struct _car_data
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} car_data_t;


void *Motion_Get_Data(uint8_t index);
void Motion_Get_Motor_Speed(float *speed);
void Motion_Set_Yaw_Adjust(uint8_t adjust);
uint8_t Motion_Get_Yaw_Adjust(void);
void Motion_Stop(uint8_t brake);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2);
void Motion_Yaw_Calc(float yaw);
void Wheel_Yaw_Calc(float yaw);
void Motion_Get_Speed(car_data_t *car);
float Motion_Get_APB(void);
float Motion_Get_Circle_MM(void);
void Motion_Get_Encoder(void);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);
void Motion_Ctrl_State(uint8_t state, uint16_t speed, uint8_t adjust);
void wheel_State_YAW(uint8_t state, uint16_t speed, uint8_t adjust);
void wheel_State(uint8_t state, uint16_t speed);
void wheel_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);
void Motion_Handle(void);





#endif

