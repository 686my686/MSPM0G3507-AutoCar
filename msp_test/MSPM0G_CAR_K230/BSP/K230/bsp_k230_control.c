#include "bsp_K230_control.h"



int pid_output,pid_output1 = 0;
int Wait_Flag = 0;//等待识别稳定的标志位    Waiting for identification of stable flag
int SetTarget_Flag = 0;//记录追踪目标的面积的标志位 A flag that records the area of ??the tracking target
int target_area = 0;//目标面积  Target area



void Limit(int pidout)
{
	if(pidout>out_MAX) pidout=out_MAX;
	if(pidout<out_MIN) pidout=out_MIN;
	

}

/******************************************************************
 * 函 数 名 称：Visual_Line_Track
 * 函 数 说 明：颜色巡线程序
 * 函 数 形 参：x：横坐标,w：宽
 * 函 数 返 回：无
 * 备       注：负责颜色巡线
******************************************************************/
void Visual_Line_Track(int x,int w)
{
	PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p, 0, k230_d);
	
    if(!Wait_Flag)
    {
        delay_ms(1000);delay_ms(1000);delay_ms(1000);//delay 3s
        Wait_Flag = 1;
        return;
    }
    
    int16_t err = 300 - (x + w/2);//获取x轴上的差值 Get the difference on the x-axis

    if(err<5 && err>-5)err=0;
    
    pid_output = PID_Calculate(&k230_pid,err);//计算差值的PID  PID to calculate the difference
//    DEBUG_PRINT("pid_output:%d\r\n", pid_output);
    
   	Set_PID_Motor(100 ,100,pid_output);
}


//颜色追踪 Color Tracking
void Color_Trace(int x,int w,int h)
{
	
	PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);
set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
//	set_p_i_d(&k230_pid, k230_p2, 0.001, k230_d2);
    if(!Wait_Flag)
    {
        delay_ms(1000);delay_ms(1000);delay_ms(1000);//delay 3s
        Wait_Flag = 1;
        return;
    }
    
    if(Wait_Flag && !SetTarget_Flag)
    {
        SetTarget_Flag = 1;
        target_area = w*h/40;
    }

    int now_area = w*h/40;
    
    int16_t err = 290 - x;

    pid_output = PID_Calculate(&k230_pid,err);//计算差值的PID  PID to calculate the difference

    if(pid_output>70) pid_output=70;//对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output<-70) pid_output=-70;


    int16_t err_area = target_area - now_area;//计算要追踪的面积差值    Calculate the difference in area to be tracked

    
    if(err_area<10 && err_area>-10)err_area=0;//设置要追踪的面积限幅，减少因为微弱变化而引起小车频繁的抖动  Set the area limit to be tracked to reduce the frequent shaking of the car caused by slight changes
    pid_output1 = PID_Calculate(&k230_pid,err_area);

    if(pid_output1>50) pid_output1=50;//对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output1<-50) pid_output1=-50;
    

     	Set_PID_Motor(-pid_output1 ,-pid_output1,pid_output);

}


//二维码检测 QR code detection
void QRCode_Action(const char* msg)
{
    if(strcmp(msg,"FORWARD")==0 || strcmp(msg,"forward")==0)//如果识别到的内容为forward或者FORWARD  If the recognized content is forward or FORWARD
    {
//        DEBUG_PRINT("FORWARD!\r\n");
        Set_PID_Motor(50,50,0);//前进两秒后停下    Go forward for two seconds and then stop
        delay_ms(1000);
        delay_ms(1000);
       Motor_Stop(1) ;
    }
    else if(strcmp(msg,"BACKWARD")==0 || strcmp(msg,"backward")==0)
    {
		        OLED_Clear();


      OLED_Draw_Line("backward!",1,true,true);
			
//        DEBUG_PRINT("BACKWARD!\r\n");
        Set_PID_Motor(-50,-50,0);
        delay_ms(1000);
        delay_ms(1000);
       Motor_Stop(1) ;
			
    }
    else if (strcmp(msg, "LEFT") == 0 || strcmp(msg, "left") == 0) 
    {
//        DEBUG_PRINT("LEFT!\r\n");
			        OLED_Clear();
	
      OLED_Draw_Line("left!",1,true,true);
			  Set_PID_Motor(50,50,-70);
        delay_ms(1000);
        delay_ms(1000);
				Motor_Stop(1) ;
    
    }
    else if (strcmp(msg, "RIGHT") == 0 || strcmp(msg, "right") == 0) 
    {
			
		 OLED_Clear();

      OLED_Draw_Line("right!",1,true,true);

			  Set_PID_Motor(50,50,70);
        delay_ms(1000);
        delay_ms(1000);
     		Motor_Stop(1) ;
  
    }
    else if (strcmp(msg, "ROTATE LEFT") == 0 || strcmp(msg, "rotate left") == 0) 
    {
        OLED_Clear();

				OLED_Draw_Line("rotate left!",1,true,true);
			  Set_PID_Motor(0,0,-70);
        delay_ms(1000);
        delay_ms(1000);
        Motor_Stop(1) ;
    
    }
    else if (strcmp(msg, "ROTATE RIGHT") == 0 || strcmp(msg, "rotate right") == 0) 
    {
		    OLED_Clear();

				OLED_Draw_Line("rotate right!",1,true,true);
				Set_PID_Motor(0,0,70);
        delay_ms(1000);
        delay_ms(1000);
        Motor_Stop(1) ;
   
    }
    else {

    }
}


//人脸追踪案例  Face tracking case
void Human_Face_Track(int x,int w,int h)
{
	PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
    if(!Wait_Flag)
    {
        delay_ms(1000);delay_ms(1000);delay_ms(1000);//delay 3s
        Wait_Flag = 1;
        return;
    }
    
    if(Wait_Flag && !SetTarget_Flag)
    {
        SetTarget_Flag = 1;
        target_area = w*h/5;
    }

    int now_area = w*h/5;
    
    int16_t err = 270 - x;

    
    pid_output = PID_Calculate(&k230_pid,err);//计算差值的PID  PID to calculate the difference

    if(pid_output>70) pid_output=70;//对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output<-70) pid_output=-70;
    

//    
    int16_t err_area = target_area - now_area;//计算要追踪的面积差值    Calculate the difference in area to be tracked

    
    if(err_area<150 && err_area>-150)err_area=0;//设置要追踪的面积限幅，减少因为微弱变化而引起小车频繁的抖动  Set the area limit to be tracked to reduce the frequent shaking of the car caused by slight changes
    pid_output1 = PID_Calculate(&k230_pid,err_area);

    if(pid_output1>50) pid_output1=50;//对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output1<-50) pid_output1=-50;
    

   	Set_PID_Motor(-pid_output1 ,-pid_output1,pid_output);
}


//AprilTag追踪 AprilTag Track
void AprilTag_Track(int x,int w,int h)
{
	PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);
		set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);

    if(!Wait_Flag)
    {
        delay_ms(1000);delay_ms(1000);delay_ms(1000);//delay 3s
        Wait_Flag = 1;
        return;
    }
    
    if(Wait_Flag && !SetTarget_Flag)
    {
        SetTarget_Flag = 1;
        target_area = w*h/5;
    }

    int now_area = w*h/5;
    
    int16_t err = 190 - x;

    

    pid_output = PID_Calculate(&k230_pid,err);//计算差值的PID  PID to calculate the difference


    if(pid_output>50) pid_output=50;//对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output<-50) pid_output=-50;
		
//    
    int16_t err_area = target_area - now_area;//计算要追踪的面积差值    Calculate the difference in area to be tracked

    if(err_area<10 && err_area>-10)err_area=0;//设置要追踪的面积限幅，减少因为微弱变化而引起小车频繁的抖动  Set the area limit to be tracked to reduce the frequent shaking of the car caused by slight changes
    pid_output1 = PID_Calculate(&k230_pid,err_area);

    if(pid_output1>50) pid_output1=50;//对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output1<-50) pid_output1=-50;
    

        
   	Set_PID_Motor(-pid_output1 ,-pid_output1,pid_output);
}

//车牌追踪  Licence Track
void Licence_Track(int x1,int y1,int x2,int y2,int x3)
{
  PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
if(!Wait_Flag)
    {
        delay_ms(1000);delay_ms(1000);delay_ms(1000);//delay 3s
        Wait_Flag = 1;
        return;
    }
    
    if(Wait_Flag && !SetTarget_Flag)
    {
        SetTarget_Flag = 1;
        target_area = (x3-x1)*(y2-y1)/80;
    }
    
    int now_area = (x3-x1)*(y2-y1)/80;
    
    int16_t err = 160 - x1;
   
    if(err<15 && err>-15)err=0;
    
    pid_output = PID_Calculate(&k230_pid,err);//计算差值的PID  PID to calculate the difference

 
    if(pid_output>100) pid_output=100;    //对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    if(pid_output<-100) pid_output=-100;
    
    
    int16_t err_area = target_area - now_area;//计算要追踪的面积差值    Calculate the difference in area to be tracked

    
    if(err_area<10 && err_area>-10)err_area=0;  //设置要追踪的面积限幅，减少因为微弱变化而引起小车频繁的抖动  Set the area limit to be tracked to reduce the frequent shaking of the car caused by slight changes
    pid_output1 = PID_Calculate(&k230_pid,err_area);
 
    if(pid_output1>70) pid_output1=70;
    if(pid_output1<-70) pid_output1=-70;   //对pid输出值进行限幅，防止小车跑过头   Limit the pid output value to prevent the car from running too far
    

        
	Set_PID_Motor(-pid_output1 ,-pid_output1,pid_output);
}

//自主避让案例 Autonomous avoidance 
void Autonomous_Avoid(int w,int h)
{
    target_area = 200;

    int now_area = w*h/100;
    

    
    if(now_area >= target_area)
    {
        Set_PID_Motor(0, 0, 0);
    }
    delay_ms(100);
}

//路标识别案例  Road sign recognition
void RoadSign_Rec(const char* msg)
{
    if(strcmp(msg,"go_straight")==0)
    {
			Write_Data(0x04);
			
//        DEBUG_PRINT("go straight!\r\n");
        Set_PID_Motor(70,70,0);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
    }
    else if(strcmp(msg,"turn_right")==0)
    {
			
					Write_Data(0x07);

        Set_PID_Motor(70,70,80);
        delay_ms(1000);
			 Motor_Stop(1) ;
//        delay_ms(1000);
    }
    else if (strcmp(msg, "speed_limit")==0) 
    {
				Write_Data(0x02);
			

        Set_PID_Motor(50,50,0);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
    }
    else if (strcmp(msg, "stop")==0) 
    {
				Write_Data(0x01);

        Set_PID_Motor(0,0,0);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
    }
    else {
			 Motor_Stop(1) ;

    }
}

//颜色反应案例 Color reaction case
void Color_Rec(const char* msg)
{
    if(strcmp(msg,"RED")==0)
    {
			Write_Data(0x0B);

  	 Control_RGB_ALL(Red_RGB);
		delay_ms(100);
       
    }
    else if(strcmp(msg,"GREEN")==0)
    {
			
			Write_Data(0x0C);

       	Control_RGB_ALL(Green_RGB);
				delay_ms(100);
    
    }
    else if (strcmp(msg, "BLUE")==0) 
    {
			Write_Data(0x0D);

     	Control_RGB_ALL(Blue_RGB);
		delay_ms(100);
        
    }
    else if (strcmp(msg, "YELLOW")==0){
			
		Write_Data(0x0E);
		Control_RGB_ALL(Yellow_RGB);
		delay_ms(100);

    }
}
//注视方向案例 Gaze Direction Case
void GazeDire_Track(int x_start, int x_end)
{
	
	PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
    int err = x_start - x_end;
    if(err<30 && err>-30)
    {
        err=0;
        Motor_Stop(1) ;
    }


    // 边界检查：禁止任何越界方向的转向 Boundary check: any turn in the out-of-bounds direction is prohibited
    bool allow_motion = true;  // 默认允许运动  Movement is allowed by default

    if (x_start < 270 && err < 0) {
        // 左边界且需要向右转 → 禁止    Left boundary and need to turn right → No
//        DEBUG_PRINT("STOP: x_start=%d < 270, err=%d (no right turn)\r\n", x_start, err);
        allow_motion = false;
    } 
    else if (x_start > 400 && err > 0) {
        // 右边界且需要向左转 → 禁止    Right boundary and need to turn left → No
//        DEBUG_PRINT("STOP: x_start=%d > 400, err=%d (no left turn)\r\n", x_start, err);
        allow_motion = false;
    }

    // 根据条件执行动作 Perform actions based on conditions
    if (allow_motion && (err > 30 || err < -30)) {
        pid_output = PID_Calculate(&k230_pid,err);

        // 限制输出范围 Limit output range
        if (pid_output > 70) pid_output = 70;
        if (pid_output < -70) pid_output = -70;

//        DEBUG_PRINT("pid_output:%d\r\n", pid_output);
//        DEBUG_PRINT("\r\n");
        Set_PID_Motor(0, 0, pid_output);
    } 
    else if (!allow_motion) {
           Motor_Stop(1) ;
    }
}

//手掌追踪案例 Hand Track
void Hand_Track(int x,int w,int h)
{
	
		PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
    if(!Wait_Flag)
    {
        delay_ms(1000);delay_ms(1000);delay_ms(1000);//delay 3s
        Wait_Flag = 1;
        return;
    }
    
    if(Wait_Flag && !SetTarget_Flag)
    {
        SetTarget_Flag = 1;
        target_area = w*h/200;

    }

    int now_area = w*h/200;
    
    int16_t err = 180 - x;

    
    pid_output = PID_Calculate(&k230_pid,err);
    if(pid_output>100) pid_output=100;
    if(pid_output<-100) pid_output=-100;

    int16_t err_area = target_area - now_area;

    
    if(err_area<20 && err_area>-20)err_area=0;
    pid_output1 =PID_Calculate(&k230_pid,err_area);

    if(pid_output1>70) pid_output1=70;
    if(pid_output1<-70) pid_output1=-70;
    
 
        
    Set_PID_Motor(-pid_output1, -pid_output1, pid_output);
}

//目标追踪案例 Target Track
void Target_Track(int x)
{
	
	PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
    int16_t err = 280 - x;

    if(err<5 && err>-5)err=0;

    pid_output = PID_Calculate(&k230_pid,err);

    if(pid_output>70) pid_output=70;
    if(pid_output<-70) pid_output=-70;
    
    
   Set_PID_Motor(0, 0, pid_output);
}

//人体追踪案例 HumanBody Track
void HumanBody_Track(int x,int w,int h)
{
	
		PID_TypeDef k230_pid;
	PID_param_init(&k230_pid);

	set_p_i_d(&k230_pid, k230_p3, 0, k230_d3);
    if(!Wait_Flag)
    {
        delay_ms(1000);//delay 1s
        Wait_Flag = 1;
        return;
    }
    
    if(Wait_Flag && !SetTarget_Flag)
    {
        SetTarget_Flag = 1;
        target_area = w*h/60;
     
    }

    int now_area = w*h/60;
    
    int16_t err = 270 - x;

    if(err<5 && err>-5)err=0;


    pid_output = PID_Calculate(&k230_pid,err);

    if(pid_output>100) pid_output=100;
    if(pid_output<-100) pid_output=-100;
    
   
    
    int16_t err_area = target_area - now_area;

    
    if(err_area<30 && err_area>-30)err_area=0;
		
    pid_output1 = PID_Calculate(&k230_pid,err_area);
  

    if(pid_output1>70) pid_output1=70;
    if(pid_output1<-70) pid_output1=-70;
    
  
        
    Set_PID_Motor(-pid_output1, -pid_output1, pid_output);
}

//OCR字符识别案例 OCR character recognition
void OCRrec_Actions(const char* msg)
{
    if(strcmp(msg,"FORWARD")==0 || strcmp(msg,"forward")==0)
    {

        Set_PID_Motor(70,70,0);
        delay_ms(1000);
        delay_ms(1000);
       Motor_Stop(1) ;
    }
    else if(strcmp(msg,"BACKWARD")==0 || strcmp(msg,"backward")==0)
    {

        Set_PID_Motor(-70,-70,0);
        delay_ms(1000);
        delay_ms(1000);
       Motor_Stop(1) ;
    }
    else if (strcmp(msg, "LEFT") == 0 || strcmp(msg, "left") == 0) 
    {
			
				Set_PID_Motor(70,70,-100);
				delay_ms(1000);
				delay_ms(1000);
				Motor_Stop(1) ;

  
    }
    else if (strcmp(msg, "RIGHT") == 0 || strcmp(msg, "right") == 0) 
    {
				Set_PID_Motor(70,70,100);
        delay_ms(1000);
        delay_ms(1000);
        Motor_Stop(1) ;

    }
    else if (strcmp(msg, "ROTATELEFT") == 0 || strcmp(msg, "rotateleft") == 0) 
    {
		    Set_PID_Motor(0,0,-100);
        delay_ms(1000);
        delay_ms(1000);
        Motor_Stop(1) ;
    }
    else if (strcmp(msg, "ROTATERIGHT") == 0 || strcmp(msg, "rotateright") == 0) 
    {

			
			
        Set_PID_Motor(0,0,100);
        delay_ms(1000);
        delay_ms(1000);
				Motor_Stop(1) ;
 
    }
    else {
			
			Motor_Stop(1) ;

    }
}

//动态手势识别案例 //Dynamic gesture recognition case
void hand_Rec(const char* msg)
{
    if(strcmp(msg,"UP")==0)
    {
			Write_Data(0x27);
			

        Set_PID_Motor(70,70,0);
        delay_ms(1000);

			 Motor_Stop(1) ;
    }
    else if(strcmp(msg,"DOWN")==0)
    {
			
					Write_Data(0x28);

        Set_PID_Motor(-70,-70,0);
        delay_ms(1000);
			 Motor_Stop(1) ;

    }
    else if (strcmp(msg, "RIGHT")==0) 
    {
			
			Write_Data(0x2A);

        Set_PID_Motor(70,70,50);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
    }
    else if (strcmp(msg, "LEFT")==0) 
    {
			
			Write_Data(0x29);

        Set_PID_Motor(70,70,-50);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
    }
    else {
			
			 Motor_Stop(1) ;

    }
}


//石头剪刀布案例 Rock, Paper, Scissors
void Rock_Result(const char* msg)
{
    if(strcmp(msg,"lose")==0)
    {
			Write_Data(0x58);
			
//     
    Control_RGB_ALL(Red_RGB);
		delay_ms(1000);
			delay_ms(1000);
			 Control_RGB_ALL(OFF);
			delay_ms(100);
			
    }
    else if(strcmp(msg,"win")==0)
    {
				Write_Data(0x57);
		Control_RGB_ALL(Green_RGB);
			
				delay_ms(1000);
			delay_ms(1000);
			 Control_RGB_ALL(OFF);
			delay_ms(100);
    }

    else {
					Write_Data(0x5A);

     	Control_RGB_ALL(Yellow_RGB);
						delay_ms(1000);
			delay_ms(1000);
			 Control_RGB_ALL(OFF);
			delay_ms(100);

    }
}

