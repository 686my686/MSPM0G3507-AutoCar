#ifndef __BSP_PID_MOTOR_H
#define __BSP_PID_MOTOR_H

#include "bsp.h"

#define PI (3.1415926f)

//#define PID_MOTOR_KP (1.5f)
//#define PID_MOTOR_KI (0.08f)
//#define PID_MOTOR_KD (0.5f)

//#define PID_DEF_KP (-000.8f) //0.8
//#define PID_DEF_KI (-0.04f) //0.06
//#define PID_DEF_KD (-001.6f) //0.5


#define PID_DEF_KP (-000.8f) //0.8
#define PID_DEF_KI (-0.030f) //0.06
#define PID_DEF_KD (-000.8f) //0.5



//#define PID_DEF_KP (1.2f) //0.8
//#define PID_DEF_KI (0.000f) //0.06
//#define PID_DEF_KD (0.0f) //0.5

#define PID_YAW_DEF_KP (5.0)
#define PID_YAW_DEF_KI (0.00)
#define PID_YAW_DEF_KD (0.50)

#define veer_p  (-046.0f)
#define veer_i  (-0.00f)
#define veer_d  (-0.00f) 


#define k230_p  (00.90f)
#define k230_i  (0.00f)
#define k230_d  (0.01f) 

#define k230_p1  (00.5f)
#define k230_i1  (0.00f)
#define k230_d1  (0.001f) 

#define k230_p2  (005.5f)
#define k230_i2  (0.00f)
#define k230_d2  (0.001f) 


#define k230_p3  (000.3f)
#define k230_i3  (0.00f)
#define k230_d3  (0.001f) 
#define RIGHT	 (-90)
#define LEFT	(90)
#define BACK	 (180)

typedef struct _pid
{
    float target_val; // 目标值
    float output_val; // 输出值
    float pwm_output; // PWM输出值
    float Kp, Ki, Kd; // 定义比例、积分、微分系数
    float err;        // 定义偏差值
    float err_last;   // 定义上一个偏差值

    float err_next; // 定义下一个偏差值, 增量式
    float integral; // 定义积分值，位置式
} PID_t;

typedef struct
{
    float SetPoint;   // 设定目标Desired value
    float Proportion; // 比例常数Proportional Const
    float Integral;   // 积分常数Integral Const
    float Derivative; // 微分常数Derivative Const
    float LastError;  // Error[-1]
    float PrevError;  // Error[-2]
    float SumError;   // Sums of Errors
} PID;

typedef struct _motor_data_t
{
    float speed_mm_s[2];  // 输入值，编码器计算速度
    float speed_pwm[2];   // 输出值，PID计算出PWM值
    int16_t speed_set[2]; // 速度设置值
} motor_data_t;

typedef struct{
    float Kp;    //比例系数
    float Ki;   //积分系数
	float Kd;     //微分系数
    float Err; //偏差值
	float LastErr;//上一个偏差
	float PenultErr; //下一个偏差
    float Integral;//积分和
    float Target;//目标值
	float PID_out; //PID输出
	
	int8_t KP_polarity;
	int8_t KI_polarity;
	int8_t KD_polarity;
}PID_TypeDef;

extern PID_TypeDef  veer_pid;

extern int8_t veer;

void PID_param_init(PID_TypeDef *pid);

void PID_Param_Init(void);

float PID_Location_Calc(PID_t *pid, float actual_val);
void PID_Calc_Motor(motor_data_t *motor);
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);
float PID_Incre_Calc(PID_t *pid, float actual_val);

//void Set_PID_Motor(float set_l ,float set_r);
void Set_PID_motor(void);


void set_pid_target(PID_TypeDef *pid, float target);
float get_pid_target(PID_TypeDef *pid);
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d);

void Wheel_Yaw_PID(float yaw,float l_motor,float r_motor);

float PID_Calculate(PID_TypeDef *PID,float CurrentValue);
void Set_PID_Veer(void);
//int16_t get_targ(int16_t now, int16_t angle);
//int calc_min_angle_direction(int now, int targ);
//int get_targ(int now, int angle);
void Set_PID_Motor(float set_l ,float set_r,float turn_out);
void PID_Yaw_Reset(float yaw);
float PID_Yaw_Calc(float NextPoint);
void PID_Yaw_Set_Parm(float kp, float ki, float kd);
int My_abs(int x);

#endif
