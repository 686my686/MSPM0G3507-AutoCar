#ifndef __CONTROL_H
#define __CONTROL_H

#include "bsp.h"
//#include "AllHeader.h"

extern int encoder_odometry_flag ;
extern volatile int odometry_sum;
extern volatile int RGB_BEEP_flag;


extern volatile float Second_yaw;


#define yaw_p  (-10.0f)
#define yaw_i  (-0.05)
#define yaw_d  (00.0f)

extern uint8_t mode3_stop ;



//State_Machine
struct state_machine
{
    int Main_State;
	int Q1_State;
	int Q2_State;
	int Q3_State;
	int Q4_State;
};




extern volatile float first_yaw_flag2;
extern volatile float first_yaw_flag3;
extern volatile float first_yaw_flag4;
extern volatile float first_yaw_flag5;


extern volatile float second_yaw_flag1;
extern volatile float second_yaw_flag2;
extern volatile float second_yaw_flag3;
extern volatile float second_yaw_flag4;

uint8_t switch_mode();
float abs_float(float p);

void mode_1(void);
void Straight_Control_Start(float target_yaw);
void Straight_Control_Stop(void);
void Straight_Control_20ms(void);

extern volatile uint8_t straight_control_active;
extern volatile uint8_t straight_control_fault;
extern volatile float straight_yaw_error;
extern volatile float straight_speed_correction;
void mode_2(void);
void mode_3(void);
void mode_3old(void);
void mode_4(void);
void mode_5();

int get_targ(int now, int angle);
int Turn(int gyro_Z,int RC);

int calc_min_angle_direction(int now, int targ);

void turn_pid(int16_t dir, int8_t v);
int My_abs_float(int x);






























#endif /* __CONTROL_H */
