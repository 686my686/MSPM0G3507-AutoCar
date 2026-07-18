#include "app_ir_track.h"


//巡线探头的处理
static void track_deal_four(u8 *s1,u8 *s2,u8 *s3,u8 *s4)
{
    //从左往右开始
	*s1 = DL_GPIO_readPins(IR_Track_PORT, IR_Track_X2_PIN) > 0 ? 1 : 0;;
	*s2 = DL_GPIO_readPins(IR_Track_PORT, IR_Track_X1_PIN) > 0 ? 1 : 0;;
	*s3 = DL_GPIO_readPins(IR_Track_PORT, IR_Track_X3_PIN) > 0 ? 1 : 0;;
	*s4 = DL_GPIO_readPins(IR_Track_PORT, IR_Track_X4_PIN) > 0 ? 1 : 0;;
	
}



#define IRTrack_Trun_KP (70)
#define IRTrack_Trun_KI (0.0) 
#define IRTrack_Trun_KD (1.40) //110


int pid_output_IRR = 0;

#define IRR_SPEED 			  250  //巡线速度
#define IRTrack_Minddle    0 //中间的值

//巡线pid
float  Track_PID(int8_t actual_value)
{
    float IRTrackTurn = 0;
	int8_t error;
	static int8_t error_last=0;
	static float IRTrack_Integral;//积分
	

	error=actual_value-IRTrack_Minddle;
	
	IRTrack_Integral +=error;
    
    if(IRTrack_Integral>100)
        IRTrack_Integral =100;
    else if(IRTrack_Integral<-100)
        IRTrack_Integral = -100;
	
	//	//位置式pid
	IRTrackTurn=error*IRTrack_Trun_KP
							+IRTrack_Trun_KI*IRTrack_Integral
							+(error - error_last)*IRTrack_Trun_KD;
	return IRTrackTurn;

}

int LineStop()

{
	
	int ret;
	static u8 x1,x2,x3,x4;
	track_deal_four(&x1,&x2,&x3,&x4);

  if(x1 == 0 || x2 == 0|| x3 ==0 || x4==0)
    {
					ret=1;
    }
		else  
		{
			
			
			ret=0;
		}
		return ret;
}




void LineWalking(int line_l,int line_r,int state)
{
	static int8_t err = 0;
	static u8 x1,x2,x3,x4;
	
	track_deal_four(&x1,&x2,&x3,&x4);
    
//    printf("%d\t%d\t%d\t%d\t \r\n",x1,x2,x3,x4);
    
    
    //简单巡圈 这里没添加直角
    if(x1 == 1 && x2 == 1&& x3 ==1 && x4==0)
    {
        err = 2;
    }
    else if(x1 == 0 && x2 == 1&& x3 ==1 && x4==1)
    {
        err = -2;
    }
    
//    else if(x1 == 1 && x2 == 1&& x3 ==0 && x4==0)
//    {
//        err = 2;
//    }
//    else if(x1 == 0 && x2 == 0&& x3 ==1 && x4==1 )
//    {
//        err = -2;
//    }
    
    
    else if(x1 == 1 && x2 == 1&& x3 ==0 && x4==1)
    {
        err = 1;
    }
    else if(x1 == 1 && x2 == 0&& x3 ==1 && x4==1 )
    {
        err = -1;
    }
    else if(x2 == 0&& x3 ==0)
    {
        err = 0;
    }
    
//   
//	pid_output_IRR = (int)(Track_PID(err));
//    
		if(state==1)
		{
			pid_output_IRR = (int)(Track_PID(err));
			Set_PID_Motor(line_l+pid_output_IRR ,line_r-pid_output_IRR,0);
		}
		else 
		{
				
			track_deal_four(&x1,&x2,&x3,&x4);
		
		}

}





