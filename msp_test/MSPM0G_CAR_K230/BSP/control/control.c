#include "control.h"

extern int g_Encoder_All_Offset[4];  /* 编码器脉冲，[0]=左轮 [1]=右轮 */
extern motor_data_t motor_data;      /* 编码器速度 mm/s，用于速差补偿 */

/* 电机转速补偿系数：实测后填入
 * 例如左轮偏慢（车左偏）→ LEFT_COMP > 1.0
 * 用法：跑mode_1看OLED上的L/R脉冲数，比值即补偿系数 */
#define LEFT_COMP  1.00f
#define RIGHT_COMP 1.00f

int encoder_odometry_flag = 0;
extern volatile float gyro_x,gyro_y,gyro_z,accel_z;
extern volatile float Accel_Angle_X,Accel_Angle_Y;
//uint8_t mode = NO_SELECT_MODE;
extern volatile float kal_mpu_out;
volatile int balance_yaw;
volatile int RGB_BEEP_flag=0;
volatile int odometry_sum = 0;                      //Odometer value(��̼���ֵ)


volatile float first_yaw_flag2;
volatile float first_yaw_flag3;
volatile float first_yaw_flag4;
volatile float first_yaw_flag5;



volatile float second_yaw_flag1;
volatile float second_yaw_flag2;
volatile float second_yaw_flag3;
volatile float second_yaw_flag4;
#define WHITE_ODM   10

float object_angle = 0;                    //The target heading_Angle of the car movement(�����˶���Ŀ�꺽���)
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
	Turn_Kd=1.00,//ת��KP��KD1.0
	Turn_Kp=1.00;
int	yaw_out  ;//-150

int yaw_out2;
//H��mode1








int abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}


void mode_1(void)
{
	static float first_yaw;
	if(mode1_flag == 0 && mode1_stop == 0)
	{
		delay_ms(2000);
		first_yaw = calibratedYaw;
		object_yaw  = navigetion_0_360_limit(first_yaw);
		mode1_flag = 1;
	}
	if(mode1_flag == 1 && mode1_stop == 0)
	{
		/* === 角度误差 → yaw校正（兜底） === */
		balance_yaw = get_minor_arc(object_yaw, calibratedYaw);
		float yaw_correction = Dir_PID(balance_yaw);

		/* === 编码器速差直接补偿（不等角度偏了再反应） === */
		float speed_l = motor_data.speed_mm_s[0];
		float speed_r = motor_data.speed_mm_s[1];
		float speed_diff = speed_l - speed_r;     /* 正值=左快右慢 */
		float trim = speed_diff * 2.0f;           /* 补偿量：速差×系数 */

		/* === 合成PWM输出 === */
		int base_pwm = 350;
		int left_pwm  = base_pwm + (int)yaw_correction - (int)trim;
		int right_pwm = base_pwm - (int)yaw_correction + (int)trim;

		/* 限幅 */
		if (left_pwm  > 850) left_pwm  = 850;
		if (left_pwm  < 40)  left_pwm  = 40;
		if (right_pwm > 850) right_pwm = 850;
		if (right_pwm < 40)  right_pwm = 40;

		PWM_Control_Car(left_pwm, right_pwm);

		mode1_stop = LineCheck();
	}
	else if(mode1_flag == 1 && mode1_stop == 1)
	{
		Motor_Stop(STOP_BRAKE);
		Buzzer_open_state();
		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
		mode1_stop = 5;
	}
	else if(mode1_flag == 1 && mode1_stop == 5)
	{
		Motor_Stop(1);
	}
}


//H��mode2
void mode_2(void)
{
    static float first_yaw;
    static float yaw_at_C;
    static float yaw_at_D;

    // �׶�0: ��ʼ��
    if(mode2_flag==0&&mode2_stop==0&&Line_flag==0&&yaw_flag==0)
    {
        delay_ms(2000);
        first_yaw = calibratedYaw;
        object_yaw = navigetion_0_360_limit(first_yaw);
        mode2_flag=1;
    }

    // �׶�1: A��Bֱ�� (Yaw��������)
    else if(mode2_flag==1&&mode2_stop==0&&Line_flag==0&&yaw_flag==0)
    {
        balance_yaw = get_minor_arc(object_yaw, calibratedYaw);
        yaw_out = Dir_PID(balance_yaw);
        Set_PID_Motor(230 ,230, yaw_out);
        mode2_stop = LineCheck();
    }

    // �׶�2: ����B�� �� ͣ������Ѳ��׼��
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
        first_yaw_flag2 = calibratedYaw;  // ��¼����B��ʱ��IMU����
        encoder_odometry_flag = 1;
        Line_flag = 1;
    }

    // �׶�3: ����B��C (IRѲ��, ת180��)
    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==0)
    {
        Line_Tracke(1);
        line_stop = LineCheck();
        if(odometry_sum > 40 && line_stop==0 &&
           abs(first_yaw_flag2-calibratedYaw) > 160 &&
           abs(first_yaw_flag2-calibratedYaw) < 180)
        {
            // ������ɣ�����C��
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

    // �׶�4: C��Dֱ�� (Yaw��������)
    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==1)
    {
        Line_Tracke(0);  // ֻ�����ﴫ��������������
        balance_yaw = get_minor_arc(object_yaw, calibratedYaw);
        yaw_out = Dir_PID(balance_yaw);
        Set_PID_Motor(230 ,230, yaw_out);
        encoder_odometry_flag = 1;
        line_stop = LineCheck();
        if(odometry_sum > 60 && line_stop == 1)  // ����D��
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

    // �׶�5: ��λ��̱̣�׼���ڶ�������
    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==2)
    {
        encoder_odometry_flag = 0;
        odometry_sum = 0;
        yaw_flag = 3;
    }

    // �׶�6: ����D��A (IRѲ��, ת180��)
    else if(mode2_flag==1&&mode2_stop==1&&Line_flag==1&&yaw_flag==3)
    {
        Line_Tracke(1);
        line_stop = LineCheck();
        encoder_odometry_flag = 1;
        if(odometry_sum > 40 && line_stop==0 &&
           abs(yaw_at_D-calibratedYaw) > 160 &&
           abs(yaw_at_D-calibratedYaw) < 180)
        {
            // �����ڶ����������ص�A
            Motor_Stop(1);
            Buzzer_open_state();
            Control_RGB_ALL(Cyan_RGB);
            delay_ms(10);
            Buzzer_close_state();
            Control_RGB_ALL(OFF);
            yaw_flag = 5;  // ��������ͣ��
        }
        else
        {
            Line_Tracke(1);
        }
    }

    // �׶�7: ����ͣ��
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
		first_yaw =calibratedYaw; //��ȡ��ʱimu����������б�� Obtain the IMU data at this moment in order to calculate the slanting angle.
	  object_yaw  = navigetion_0_360_limit(first_yaw-38.7);	//������б�߽Ƕȣ����б����ƫ���޸�����ط��Ĳ��� Calculate the angle of the diagonal line; if there is any deviation in the diagonal line, adjust the parameters in this area accordingly.



		mode3_flag=1;
	}
	
else if(mode3_flag==1&&mode3_stop==0&&Line_flag==0&&yaw_flag==0)
	{



	

		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);

			/* OLED显示编码器脉冲 L/R，用于测量两轮速差 */
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
	first_yaw_flag2=calibratedYaw;//��ȡ��ʱimu�Ƕȣ�������imu�����жϴӶ�����С������ֹͣ Obtain the current IMU angle, which will be used to make judgments in conjunction with the IMU data in order to control the vehicle to stop offline.

	Line_flag =1;
	encoder_odometry_flag = 1;
}

	

else if(mode3_flag==1&&mode3_stop==1&&Line_flag==1&&yaw_flag==0)
{

//		RGB_BEEP_flag=0;
	Line_Tracke(1);

	line_stop=LineCheck();


	if((odometry_sum>40)&&line_stop==0&&abs(first_yaw_flag2-calibratedYaw)>160&&abs(first_yaw_flag2-calibratedYaw)<180)//�ж�С������ֹͣλ�ã����ͣ��׼�����޸�������� Determine the offline stopping position of the cart; if the stopping position is inaccurate, this parameter can be adjusted accordingly.
	{
		yaw_flag=1;
		Motor_Stop(1) ;
		
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
	
		delay_ms(1000);

	Second_yaw=calibratedYaw;//��ȡ��ʱimu����������б�� Obtain the IMU data at this moment in order to calculate the slanting angle.
		odometry_sum=0;
	mode3_stop= 0;
		
	object_yaw  = navigetion_0_360_limit(Second_yaw+38.7);//������б�߽Ƕȣ����б����ƫ���޸�����ط��Ĳ��� Calculate the angle of the diagonal line; if there is any deviation in the diagonal line, adjust the parameters in this area accordingly.





	

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
	if(odometry_sum>40&&line_stop==0&&abs(second_yaw_flag1-calibratedYaw)>168&&abs(second_yaw_flag1-calibratedYaw)<190)
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



//H��mode4
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
			first_yaw =calibratedYaw; //��ȡ��ʱimu����������б�� Obtain the IMU data at this moment in order to calculate the slanting angle.
	  object_yaw  = navigetion_0_360_limit(first_yaw-38.7);	//������б�߽Ƕȣ����б����ƫ���޸�����ط��Ĳ��� Calculate the angle of the diagonal line; if there is any deviation in the diagonal line, adjust the parameters in this area accordingly.



		mode4_flag=1;
	}
	
else if(mode4_flag==1&&mode4_stop==0&&Line_flag==0&&yaw_flag==0&&mode4_Circle==0)
	{



	

		balance_yaw	=get_minor_arc(object_yaw,calibratedYaw);
		yaw_out=Dir_PID(balance_yaw);
		Set_PID_Motor(230 * LEFT_COMP, 230 * RIGHT_COMP, yaw_out);

			/* OLED显示编码器脉冲 L/R，用于测量两轮速差 */
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


	if((odometry_sum>40)&&line_stop==0&&abs(first_yaw_flag2-calibratedYaw)>160&&abs(first_yaw_flag2-calibratedYaw)<180)
	{
		yaw_flag=1;
		Motor_Stop(1) ;
		
		Buzzer_open_state();
		Control_RGB_ALL(Cyan_RGB);

		delay_ms(10);
		Buzzer_close_state();
		Control_RGB_ALL(OFF);
	
		delay_ms(1000);

	Second_yaw=calibratedYaw;//��ȡ��ʱimu����������б�� Obtain the IMU data at this moment in order to calculate the slanting angle.
		odometry_sum=0;
	mode4_stop= 0;
		
	object_yaw  = navigetion_0_360_limit(Second_yaw+38.7);//������б�߽Ƕȣ����б����ƫ���޸�����ط��Ĳ��� Calculate the angle of the diagonal line; if there is any deviation in the diagonal line, adjust the parameters in this area accordingly.





	

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
//			Motor_Back(100,100);//������С������б��֮��Ҷ����ǳ����ߣ������޸������С������ If, after your car moves along the diagonal line, the gray level always exceeds the specified limit, you can modify the settings to make the car move back.
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
	if(odometry_sum>40&&line_stop==0&&abs(second_yaw_flag1-calibratedYaw)>168&&abs(second_yaw_flag1-calibratedYaw)<190)
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

			/* OLED显示编码器脉冲 L/R，用于测量两轮速差 */
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
if((odometry_sum>40)&&line_stop==0&&abs(first_yaw_flag3-calibratedYaw)>160&&abs(first_yaw_flag3-calibratedYaw)<180)
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
	if(odometry_sum>40&&line_stop==0&&abs(second_yaw_flag2-calibratedYaw)>160&&abs(second_yaw_flag2-calibratedYaw)<183)
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

			/* OLED显示编码器脉冲 L/R，用于测量两轮速差 */
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
	if((odometry_sum>40)&&line_stop==0&&abs(first_yaw_flag4-calibratedYaw)>177&&abs(first_yaw_flag4-calibratedYaw)<193)
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
	if(odometry_sum>40&&line_stop==0&&abs(second_yaw_flag3-calibratedYaw)>170&&abs(second_yaw_flag3-calibratedYaw)<195)
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

			/* OLED显示编码器脉冲 L/R，用于测量两轮速差 */
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
		if((odometry_sum>30)&&line_stop==0&&abs(first_yaw_flag5-calibratedYaw)>160&&abs(first_yaw_flag5-calibratedYaw)<180)
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
	if(odometry_sum>40&&line_stop==0&&abs(second_yaw_flag4-calibratedYaw)>170&&abs(second_yaw_flag4-calibratedYaw)<185)
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


//����ѡ��ģʽ
uint8_t switch_mode()
{
	uint8_t	select_mode = 0, key_num = 0;
	OLED_Clear();
	OLED_ShowString(0, 0, "Select Mode:", 8, 1);
	OLED_ShowNum(80,0,select_mode,1,24,1);
	OLED_Refresh();
	while(1){
		key_num = KEY_Scan();
		if(key_num){
			if(key_num == KEY1_PRES){
				select_mode +=1;
			}
			else if(key_num == KEY2_PRES){
				OLED_ShowString(92,0,"OK!!!",8,1);
				OLED_Refresh();
				delay_ms(700);
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



