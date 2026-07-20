#include "control.h"

extern motor_data_t motor_data;
extern int g_Encoder_All_Offset[2];

/* 旧模式保留的固定补偿系数；Mode 1的速度闭环不使用这两个系数。 */
#define LEFT_COMP  1.00f
#define RIGHT_COMP 1.00f

/* 旧的固定PWM偏置已归零，防止与闭环控制互相叠加。 */
#define RIGHT_BIAS  0

/* Mode 1串级控制参数：航向外环输出左右轮目标速度差。 */
#define STRAIGHT_BASE_SPEED_MM_S       (200.0f)
#define STRAIGHT_YAW_KP                (4.0f)
#define STRAIGHT_YAW_KI                (0.15f)
#define STRAIGHT_YAW_MAX_CORRECTION    (60.0f)
#define STRAIGHT_YAW_SLEW_STEP         (3.0f)
#define STRAIGHT_SPEED_RAMP_STEP       (10.0f)
#define STRAIGHT_MIN_RUNNING_SPEED     (80.0f)
#define STRAIGHT_IMU_TIMEOUT_MS        (80U)

volatile uint8_t straight_control_active = 0;
volatile uint8_t straight_control_fault = 0;
volatile float straight_yaw_error = 0.0f;
volatile float straight_speed_correction = 0.0f;

static float straight_target_yaw = 0.0f;
static float straight_yaw_integral = 0.0f;
static float straight_ramped_speed = 0.0f;

int encoder_odometry_flag = 0;
extern volatile float gyro_x,gyro_y,gyro_z,accel_z;
extern volatile float Accel_Angle_X,Accel_Angle_Y;
//uint8_t mode = NO_SELECT_MODE;
extern volatile float kal_mpu_out;
volatile float balance_yaw;
volatile int RGB_BEEP_flag=0;
volatile int odometry_sum = 0;


volatile float first_yaw_flag2;
volatile float first_yaw_flag3;
volatile float first_yaw_flag4;
volatile float first_yaw_flag5;



volatile float second_yaw_flag1;
volatile float second_yaw_flag2;
volatile float second_yaw_flag3;
volatile float second_yaw_flag4;
#define WHITE_ODM   10

float object_angle = 0;
float Angle_error = 0;

//volatile float Second_yaw;
//volatile float first_yaw;

float object_yaw = 0;
//void (*mode_task)(void) = mode_0;
uint8_t mode1_flag =0;
uint8_t mode2_flag =0;
uint8_t mode3_flag =0;
uint8_t mode4_flag =0;

uint8_t yaw_flag =0;
uint8_t mode1_stop =0;
uint8_t mode2_stop =0;
uint8_t mode3_stop =0;
uint8_t mode4_stop =0;

uint8_t Line_flag =0;

uint8_t line_stop=0;

uint8_t mode4_Circle=0;

float
	Turn_Kd=1.00,
	Turn_Kp=1.00;
int	yaw_out  ;//-150

int yaw_out2;









float abs_float(float p)
{
	float q;
	q=p>0?p:(-p);
	return q;
}

/* 将任意角度误差限制到-180°到180°。 */
static float wrap_yaw_error(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static float limit_float(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

/* 启动直线串级控制，只在这里初始化速度环参数和历史状态。 */
void Straight_Control_Start(float target_yaw)
{
    straight_target_yaw = target_yaw;
    straight_yaw_integral = 0.0f;
    straight_ramped_speed = 0.0f;
    straight_yaw_error = 0.0f;
    straight_speed_correction = 0.0f;
    straight_control_fault = 0;

    PID_Clear_Motor(MAX_MOTOR);
    PID_Set_Motor_Parm(MAX_MOTOR, SPD_KP_POS, SPD_KI_POS, SPD_KD_POS);
    Motion_Set_Speed(0, 0);
    straight_control_active = 1;
}

void Straight_Control_Stop(void)
{
    straight_control_active = 0;
    straight_yaw_integral = 0.0f;
    straight_speed_correction = 0.0f;
    Motion_Stop(STOP_BRAKE);
}

/* 每20ms运行一次：航向PI外环生成速度差，两个独立速度环再生成PWM。 */
void Straight_Control_20ms(void)
{
    float requested_correction;
    float correction_delta;
    float left_target;
    float right_target;

    if (!straight_control_active)
    {
        return;
    }

    if (!mpu_yaw_valid || (uint32_t)(Get_Time() - mpu_yaw_last_ms) > STRAIGHT_IMU_TIMEOUT_MS)
    {
        straight_control_fault = 1;
        Motion_Set_Speed(0, 0);
        return;
    }

    straight_control_fault = 0;
    if (straight_ramped_speed < STRAIGHT_BASE_SPEED_MM_S)
    {
        straight_ramped_speed += STRAIGHT_SPEED_RAMP_STEP;
        straight_ramped_speed = limit_float(straight_ramped_speed, 0.0f,
                                             STRAIGHT_BASE_SPEED_MM_S);
    }

    straight_yaw_error = wrap_yaw_error(straight_target_yaw - Filter_out);

    /* 只在误差较小时积分，防止启动或碰撞后的积分饱和。 */
    if (straight_yaw_error > -20.0f && straight_yaw_error < 20.0f)
    {
        straight_yaw_integral += straight_yaw_error * 0.020f;
        straight_yaw_integral = limit_float(straight_yaw_integral, -100.0f, 100.0f);
    }

    requested_correction = STRAIGHT_YAW_KP * straight_yaw_error
                         + STRAIGHT_YAW_KI * straight_yaw_integral;
    requested_correction = limit_float(requested_correction,
                                       -STRAIGHT_YAW_MAX_CORRECTION,
                                       STRAIGHT_YAW_MAX_CORRECTION);

    /* 限制每周期变化量，避免方向校正突然跳变。 */
    correction_delta = requested_correction - straight_speed_correction;
    correction_delta = limit_float(correction_delta,
                                   -STRAIGHT_YAW_SLEW_STEP,
                                   STRAIGHT_YAW_SLEW_STEP);
    straight_speed_correction += correction_delta;

    left_target = straight_ramped_speed - straight_speed_correction;
    right_target = straight_ramped_speed + straight_speed_correction;

    if (straight_ramped_speed >= STRAIGHT_MIN_RUNNING_SPEED)
    {
        left_target = limit_float(left_target, STRAIGHT_MIN_RUNNING_SPEED, 350.0f);
        right_target = limit_float(right_target, STRAIGHT_MIN_RUNNING_SPEED, 350.0f);
    }
    else
    {
        left_target = limit_float(left_target, 0.0f, 350.0f);
        right_target = limit_float(right_target, 0.0f, 350.0f);
    }

    Motion_Set_Speed((int16_t)left_target, (int16_t)right_target);
}


void mode_1(void)
{
    static uint32_t start_yaw_count = 0;
    static uint32_t last_line_check_ms = 0;
    static uint32_t run_start_ms = 0;
    static uint8_t white_count = 0;
    static uint8_t black_count = 0;
    static uint8_t left_start_line = 0;

    if (mode1_flag == 0 && mode1_stop == 0)
    {
        Motion_Stop(STOP_BRAKE);
        start_yaw_count = mpu_yaw_update_count;
        mode1_flag = 10;
    }
    else if (mode1_flag == 10 && mode1_stop == 0)
    {
        /* 等待20个新DMP样本稳定后，再锁定当前车头方向。 */
        if (mpu_yaw_valid && (uint32_t)(mpu_yaw_update_count - start_yaw_count) >= 20U)
        {
            Straight_Control_Start(Filter_out);
            run_start_ms = Get_Time();
            last_line_check_ms = run_start_ms;
            white_count = 0;
            black_count = 0;
            left_start_line = 0;
            mode1_flag = 1;
        }
    }
    else if (mode1_flag == 1 && mode1_stop == 0)
    {
        /* 10ms读取一次巡线模块，先确认离开A线，再允许B线触发停车。 */
        if ((uint32_t)(Get_Time() - last_line_check_ms) >= 10U)
        {
            int on_black = LineCheck();
            last_line_check_ms = Get_Time();

            if (!left_start_line)
            {
                white_count = on_black ? 0 : (uint8_t)(white_count + 1U);
                if (white_count >= 8U)
                {
                    left_start_line = 1;
                }
            }
            else if ((uint32_t)(Get_Time() - run_start_ms) > 500U)
            {
                black_count = on_black ? (uint8_t)(black_count + 1U) : 0;
                if (black_count >= 3U)
                {
                    mode1_stop = 1;
                }
            }
        }

        if (straight_control_fault)
        {
            mode1_stop = 2;
        }
    }
    else if (mode1_flag == 1 && (mode1_stop == 1 || mode1_stop == 2))
    {
        Straight_Control_Stop();
        Buzzer_open_state();
        delay_ms(10);
        Buzzer_close_state();
        Control_RGB_ALL(OFF);
        mode1_stop = 5;
    }
    else if (mode1_flag == 1 && mode1_stop == 5)
    {
        Motor_Stop(STOP_BRAKE);
    }
}


void mode_2(void)
{
    static float first_yaw;
    static float yaw_at_C;
    static float yaw_at_D;


    if(mode2_flag==0&&mode2_stop==0&&Line_flag==0&&yaw_flag==0)
    {
        delay_ms(2000);
        first_yaw = calibratedYaw;
        object_yaw = navigetion_0_360_limit(first_yaw);
        mode2_flag=1;
    }


    else if(mode2_flag==1&&mode2_stop==0&&Line_flag==0&&yaw_flag==0)
    {
        balance_yaw = get_minor_arc(object_yaw, calibratedYaw);
        yaw_out = Dir_PID(balance_yaw);
        Set_PID_Motor(230 ,230, yaw_out);
        mode2_stop = LineCheck();
    }


    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==0&&yaw_flag==0)
    {
        yaw_out = 0;
        object_yaw = 0;
        Motor_Stop(1);
        Buzzer_open_state();
        Control_RGB_ALL(Cyan_RGB);
        delay_ms(10);
        Buzzer_close_state();
        Control_RGB_ALL(OFF);
        delay_ms(1000);
        first_yaw_flag2 = calibratedYaw;
        encoder_odometry_flag = 1;
        Line_flag = 1;
    }


    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==0)
    {
        Line_Tracke(1);
        line_stop = LineCheck();
        if(odometry_sum > 40 && line_stop==0 &&
           abs_float(first_yaw_flag2-calibratedYaw) > 160 &&
           abs_float(first_yaw_flag2-calibratedYaw) < 180)
        {

            Motor_Stop(1);
            Buzzer_open_state();
            Control_RGB_ALL(Cyan_RGB);
            delay_ms(10);
            Buzzer_close_state();
            Control_RGB_ALL(OFF);
            delay_ms(1000);

            yaw_at_C = calibratedYaw;
            odometry_sum = 0;
            object_yaw = navigetion_0_360_limit(yaw_at_C);
            yaw_flag = 1;
        }
        else
        {
            Line_Tracke(1);
        }
    }


    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==1)
    {
        Line_Tracke(0);
        balance_yaw = get_minor_arc(object_yaw, calibratedYaw);
        yaw_out = Dir_PID(balance_yaw);
        Set_PID_Motor(230 ,230, yaw_out);
        encoder_odometry_flag = 1;
        line_stop = LineCheck();
        if(odometry_sum > 60 && line_stop == 1)
        {
            Motor_Stop(1);
            Buzzer_open_state();
            Control_RGB_ALL(Cyan_RGB);
            delay_ms(10);
            Buzzer_close_state();
            Control_RGB_ALL(OFF);
            delay_ms(1000);

            yaw_at_D = calibratedYaw;
            yaw_flag = 2;
        }
    }

    /* 阶段5：清零里程计，为第二个半圆弯道做准备。 */
    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==2)
    {
        encoder_odometry_flag = 0;
        odometry_sum = 0;
        yaw_flag = 3;
    }


    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==3)
    {
        Line_Tracke(1);
        line_stop = LineCheck();
        encoder_odometry_flag = 1;
        if(odometry_sum > 40 && line_stop==0 &&
           abs_float(yaw_at_D-calibratedYaw) > 160 &&
           abs_float(yaw_at_D-calibratedYaw) < 180)
        {

            Motor_Stop(1);
            Buzzer_open_state();
            Control_RGB_ALL(Cyan_RGB);
            delay_ms(10);
            Buzzer_close_state();
            Control_RGB_ALL(OFF);
            yaw_flag = 5;
        }
        else
        {
            Line_Tracke(1);
        }
    }


    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==5)
    {
        Motor_Stop(1);
    }
}
void mode_3(void)
{


		static float Second_yaw;
		static float first_yaw;



if(mode3_flag==0&&mode3_stop==0&&Line_flag==0&&yaw_flag==0)
	{
		delay_ms(2000);
		first_yaw =calibratedYaw;
	  object_yaw  = navigetion_0_360_limit(first_yaw-38.7);



		mode3_flag=1;
	}

else if(mode3_flag==1&&mode3_stop==0&&Line_flag==0&&yaw_flag==0)
	{





		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);


		mode3_stop= LineCheck();
	}

else if(mode3_flag==1&&mode3_stop==1&&Line_flag==0&&yaw_flag==0)
{

	yaw_out=0;
	object_yaw=0;

	 Motor_Stop(1) ;

	Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	Control_RGB_ALL(OFF);
	Motor_Back(100,100);
	delay_ms(1000);
	first_yaw_flag2=calibratedYaw;

	Line_flag =1;
	encoder_odometry_flag = 1;
}



else if(mode3_flag==1&&mode3_stop==1&&Line_flag==1&&yaw_flag==0)
{

//		RGB_BEEP_flag=0;
	Line_Tracke(1);

	line_stop=LineCheck();


	if((odometry_sum>40)&&line_stop==0&&abs_float(first_yaw_flag2-calibratedYaw)>160&&abs_float(first_yaw_flag2-calibratedYaw)<180)
	{
		yaw_flag=1;
		Motor_Stop(1) ;

		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);

		delay_ms(1000);

	Second_yaw=calibratedYaw;
		odometry_sum=0;
	mode3_stop= 0;

	object_yaw  = navigetion_0_360_limit(Second_yaw+38.7);







	}else
	{

		yaw_flag=0;

	}

}






else if(mode3_flag==1&&mode3_stop==0&&Line_flag==1&&yaw_flag==1)
{

		Line_Tracke(0);
		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out2=Dir_PID(balance_yaw);

		Set_PID_Motor(230 ,230,yaw_out2);
//		encoder_odometry_flag = 1;
		line_stop=LineCheck();
		if(odometry_sum>60&&line_stop==1)
		{
		Motor_Stop(1) ;
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
//			Motor_Back(100,100);
				delay_ms(1000);

			second_yaw_flag1=calibratedYaw;
		yaw_flag=2;
		mode3_stop=1;


		}


}
else if(mode3_flag==1&&mode3_stop==1&&Line_flag==1&&yaw_flag==2)
{
	object_yaw=0;
	yaw_out2=0;
	encoder_odometry_flag = 0;
	odometry_sum=0;


	yaw_flag=3;

}
else if(mode3_flag==1&&mode3_stop==1&&Line_flag==1&&yaw_flag==3)
{

		line_stop=LineCheck();
		encoder_odometry_flag = 1;
	if(odometry_sum>40&&line_stop==0&&abs_float(second_yaw_flag1-calibratedYaw)>168&&abs_float(second_yaw_flag1-calibratedYaw)<190)
	{



		Motor_Stop(1) ;
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
		delay_ms(1000);


		mode3_flag=1;
		mode3_stop=0;
		Line_flag=0;
		yaw_flag=5;




	}



	else{

		Line_Tracke(1);

	}
}
else if(mode3_flag==1&&mode3_stop==0&&Line_flag==0&&yaw_flag==5)
{


	Motor_Stop(1) ;


}
}




void mode_4(void)
{
		static float Second_yaw;
		static float first_yaw;
		static float Second_yaw2;
		static float first_yaw2;
		static float Second_yaw3;
		static float first_yaw3;
		static float Second_yaw4;
		static float first_yaw4;




if(mode4_flag==0&&mode4_stop==0&&Line_flag==0&&yaw_flag==0&&mode4_Circle==0)
	{
		delay_ms(2000);
			first_yaw =calibratedYaw;
	  object_yaw  = navigetion_0_360_limit(first_yaw-38.7);



		mode4_flag=1;
	}

else if(mode4_flag==1&&mode4_stop==0&&Line_flag==0&&yaw_flag==0&&mode4_Circle==0)
	{





		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);


		mode4_stop= LineCheck();
	}

else if(mode4_flag==1&&mode4_stop==1&&Line_flag==0&&yaw_flag==0&&mode4_Circle==0)
{

	yaw_out=0;
	object_yaw=0;

	 Motor_Stop(1) ;

	Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	Control_RGB_ALL(OFF);
	Motor_Back(100,100);
	delay_ms(1000);
	first_yaw_flag2=calibratedYaw;

	Line_flag =1;
	encoder_odometry_flag = 1;
}



else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==0&&mode4_Circle==0)
{

//		RGB_BEEP_flag=0;
	Line_Tracke(1);

	line_stop=LineCheck();


	if((odometry_sum>40)&&line_stop==0&&abs_float(first_yaw_flag2-calibratedYaw)>160&&abs_float(first_yaw_flag2-calibratedYaw)<180)
	{
		yaw_flag=1;
		Motor_Stop(1) ;

		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);

		delay_ms(1000);

	Second_yaw=calibratedYaw;
		odometry_sum=0;
	mode4_stop= 0;

	object_yaw  = navigetion_0_360_limit(Second_yaw+38.7);







	}else
	{

		yaw_flag=0;

	}

}






else if(mode4_flag==1&&mode4_stop==0&&Line_flag==1&&yaw_flag==1&&mode4_Circle==0)
{

		Line_Tracke(0);
		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out2=Dir_PID(balance_yaw);

		Set_PID_Motor(230 ,230,yaw_out2);
//		encoder_odometry_flag = 1;
		line_stop=LineCheck();
		if(odometry_sum>60&&line_stop==1)
		{
		Motor_Stop(1) ;
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);

				delay_ms(1000);

			second_yaw_flag1=calibratedYaw;
		yaw_flag=2;
		mode4_stop=1;


		}


}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==2&&mode4_Circle==0)
{
	object_yaw=0;
	yaw_out2=0;
	encoder_odometry_flag = 0;
	odometry_sum=0;


	yaw_flag=3;

}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==3&&mode4_Circle==0)
{

		line_stop=LineCheck();
		encoder_odometry_flag = 1;
	if(odometry_sum>40&&line_stop==0&&abs_float(second_yaw_flag1-calibratedYaw)>168&&abs_float(second_yaw_flag1-calibratedYaw)<190)
	{



		Motor_Stop(1) ;
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
		delay_ms(1000);
		first_yaw2=calibratedYaw;
		mode4_Circle=1;
		mode4_flag=1;
		mode4_stop=0;
		Line_flag=0;
		yaw_flag=0;
		object_yaw  = navigetion_0_360_limit(first_yaw2-38.7);



	}



	else{

		Line_Tracke(1);

	}
}

else if(mode4_flag==1&&mode4_stop==0&&Line_flag==0&&yaw_flag==0&&mode4_Circle==1)
	{



		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);


		mode4_stop= LineCheck();
	}

else if(mode4_flag==1&&mode4_stop==1&&Line_flag==0&&yaw_flag==0&&mode4_Circle==1)
{

	yaw_out=0;
	object_yaw=0;
//	encoder_odometry_flag = 0;
	odometry_sum=0;
	 Motor_Stop(1) ;
//	Line_Tracke(1);
	Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	Control_RGB_ALL(OFF);
	Motor_Back(100,100);
	delay_ms(1000);

	first_yaw_flag3=calibratedYaw;
	Line_flag =1;

}



else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==0&&mode4_Circle==1)
{


	Line_Tracke(1);
	line_stop=LineCheck();
if((odometry_sum>40)&&line_stop==0&&abs_float(first_yaw_flag3-calibratedYaw)>160&&abs_float(first_yaw_flag3-calibratedYaw)<180)
	{
	yaw_flag=1;
		 Motor_Stop(1) ;
			Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
			delay_ms(1000);

	Second_yaw2=calibratedYaw;
		odometry_sum=0;
	mode4_stop= 0;

		object_yaw  = navigetion_0_360_limit(Second_yaw2+38.7);



	}
	else
	{

		yaw_flag=0;

	}

}






else if(mode4_flag==1&&mode4_stop==0&&Line_flag==1&&yaw_flag==1&&mode4_Circle==1)
{

	Line_Tracke(0);

	balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);

		yaw_out2=Dir_PID(balance_yaw);

		Set_PID_Motor(230 ,230,yaw_out2);

		encoder_odometry_flag = 1;
		line_stop=LineCheck();
		if(odometry_sum>50&&line_stop==1)
		{
			 Motor_Stop(1) ;
			Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
//					Motor_Back(100,100);
				delay_ms(1000);

			second_yaw_flag2=calibratedYaw;
			yaw_flag=2;
			mode4_stop=1;


		}


}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==2&&mode4_Circle==1)
{

	encoder_odometry_flag = 0;
	odometry_sum=0;
	yaw_out2=0;
	object_yaw=0;
	yaw_flag=3;

}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==3&&mode4_Circle==1)
{

	line_stop=LineCheck();
		encoder_odometry_flag = 1;
	if(odometry_sum>40&&line_stop==0&&abs_float(second_yaw_flag2-calibratedYaw)>160&&abs_float(second_yaw_flag2-calibratedYaw)<183)
	{

		 Motor_Stop(1) ;
			Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
		delay_ms(1000);
		first_yaw3=calibratedYaw;
		mode4_Circle=2;
		mode4_flag=1;
		mode4_stop=0;
		Line_flag=0;
		yaw_flag=0;
		object_yaw  = navigetion_0_360_limit(first_yaw3-38.7);

//


	}



	else{

		Line_Tracke(1);

	}
}

else if(mode4_flag==1&&mode4_stop==0&&Line_flag==0&&yaw_flag==0&&mode4_Circle==2)
	{


		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);


		mode4_stop= LineCheck();
	}

else if(mode4_flag==1&&mode4_stop==1&&Line_flag==0&&yaw_flag==0&&mode4_Circle==2)
{

	yaw_out=0;
	object_yaw=0;
//	encoder_odometry_flag = 0;
	odometry_sum=0;
	delay_ms(50);
	 Motor_Stop(1) ;
//Line_Tracke(1);
		Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
			Motor_Back(100,100);
				delay_ms(1000);

	first_yaw_flag4=calibratedYaw;
	Line_flag =1;

}



else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==0&&mode4_Circle==2)
{


	Line_Tracke(1);
	line_stop=LineCheck();
	if((odometry_sum>40)&&line_stop==0&&abs_float(first_yaw_flag4-calibratedYaw)>177&&abs_float(first_yaw_flag4-calibratedYaw)<193)
	{
	yaw_flag=1;
	 Motor_Stop(1) ;
		Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
	delay_ms(1000);

	Second_yaw3=calibratedYaw;
		odometry_sum=0;
	mode4_stop= 0;

		object_yaw  = navigetion_0_360_limit(Second_yaw3+38.7);

//

	}else
	{

		yaw_flag=0;

	}

}






else if(mode4_flag==1&&mode4_stop==0&&Line_flag==1&&yaw_flag==1&&mode4_Circle==2)
{

	Line_Tracke(0);

	balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);

		yaw_out2=Dir_PID(balance_yaw);

		Set_PID_Motor(230 ,230,yaw_out2);
		encoder_odometry_flag = 1;
		line_stop=LineCheck();
		if(odometry_sum>50&&line_stop==1)
		{

		Motor_Stop(1) ;
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
//			Motor_Back(100,100);
				delay_ms(1000);
			second_yaw_flag3=calibratedYaw;
		yaw_flag=2;
		mode4_stop=1;


		}


}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==2&&mode4_Circle==2)
{

	encoder_odometry_flag = 0;
	odometry_sum=0;
	yaw_out2=0;
	yaw_flag=3;

}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==3&&mode4_Circle==2)
{

	line_stop=LineCheck();
		encoder_odometry_flag = 1;
	if(odometry_sum>40&&line_stop==0&&abs_float(second_yaw_flag3-calibratedYaw)>170&&abs_float(second_yaw_flag3-calibratedYaw)<195)
	{

		 Motor_Stop(1) ;
			Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
		delay_ms(1000);


		first_yaw4=calibratedYaw;
		mode4_Circle=3;
		mode4_flag=1;
		mode4_stop=0;
		Line_flag=0;
		yaw_flag=0;
		 object_yaw  = navigetion_0_360_limit(first_yaw4-38.7);




	}



	else{

		Line_Tracke(1);

	}
}
else if(mode4_flag==1&&mode4_stop==0&&Line_flag==0&&yaw_flag==0&&mode4_Circle==3)
	{


		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);


		mode4_stop= LineCheck();
	}

else if(mode4_flag==1&&mode4_stop==1&&Line_flag==0&&yaw_flag==0&&mode4_Circle==3)
{

	yaw_out=0;
	object_yaw=0;

	odometry_sum=0;
	encoder_odometry_flag = 1;
	delay_ms(50);

 Motor_Stop(1) ;
//	Line_Tracke(1);
		Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
		Motor_Back(100,100);
	delay_ms(1000);
	first_yaw_flag5=calibratedYaw;
		yaw_out=0;
	Line_flag =1;

}



else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==0&&mode4_Circle==3)
{


	Line_Tracke(1);
		line_stop=LineCheck();
		if((odometry_sum>30)&&line_stop==0&&abs_float(first_yaw_flag5-calibratedYaw)>160&&abs_float(first_yaw_flag5-calibratedYaw)<180)
	{
	yaw_flag=1;
		 Motor_Stop(1) ;
			Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
		delay_ms(1000);
		Second_yaw4=calibratedYaw;
	mode4_stop= 0;
	object_yaw  = navigetion_0_360_limit(Second_yaw4+38.7);

	}else
	{

		yaw_flag=0;

	}

}






else if(mode4_flag==1&&mode4_stop==0&&Line_flag==1&&yaw_flag==1&&mode4_Circle==3)
{

	Line_Tracke(0);

	balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);

		yaw_out2=Dir_PID(balance_yaw);

		Set_PID_Motor(230 ,230,yaw_out2);
		encoder_odometry_flag = 1;
		line_stop=LineCheck();
		if(odometry_sum>50&&line_stop==1)
		{
			 Motor_Stop(1) ;
				Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
//					Motor_Back(100,100);
				delay_ms(1000);
second_yaw_flag4=calibratedYaw;
			yaw_flag=2;
			mode4_stop=1;


		}


}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==2&&mode4_Circle==3)
{

	encoder_odometry_flag = 0;
	odometry_sum=0;
	yaw_out2=0;
	yaw_flag=3;

}
else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==3&&mode4_Circle==3)
{

	line_stop=LineCheck();
		encoder_odometry_flag = 1;
	if(odometry_sum>40&&line_stop==0&&abs_float(second_yaw_flag4-calibratedYaw)>170&&abs_float(second_yaw_flag4-calibratedYaw)<185)
	{

			 Motor_Stop(1) ;
			Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);

	delay_ms(10);
	Buzzer_close_state();
	 Control_RGB_ALL(OFF);
	mode4_Circle=4;
	}



	else{

		Line_Tracke(1);

	}
}

else if(mode4_flag==1&&mode4_stop==1&&Line_flag==1&&yaw_flag==3&&mode4_Circle==4)
{

 Motor_Stop(1) ;

}

}


/* 按KEY1切换模式，按KEY2确认。等待期间持续读取MPU6050，避免DMP FIFO堆积。 */
uint8_t switch_mode()
{
	uint8_t	select_mode = 0, key_num = 0;
	OLED_Clear();
	OLED_ShowString(0, 0, "Select Mode:", 8, 1);
	OLED_ShowNum(80,0,select_mode,1,24,1);
	OLED_Refresh();
	while(1){
		Scheduler_Run();
		key_num = KEY_Scan();
		if(key_num){
			if(key_num == KEY1_PRES){
				select_mode +=1;
			}
			else if(key_num == KEY2_PRES){
				OLED_ShowString(92,0,"OK!!!",8,1);
				OLED_Refresh();
				OLED_Clear();
				break;
			}
			if(select_mode > 7 ){
				select_mode = 0;
			}
			OLED_ShowNum(80,0,select_mode,1,24,1);
			OLED_Refresh();
		}
	}
	return select_mode;
}
