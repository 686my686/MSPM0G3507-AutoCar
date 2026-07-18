#include "bsp_usart_voice.h"
#include "bsp_usart3.h"

volatile uint8_t voice_data1[10];
volatile uint8_t voice_data2[10];
//volatile uint8_t recv_buf[4]={0}; //接收buf Receive buf
char voice[5];
//volatile uint8_t g_index = 0;

//extern uint8_t get_v;
extern uint8_t get_v ;
extern uint8_t get_mode ;


extern void uart3_send_char(char str);


void get_voice()   //ai语音助手处理函数
{
//	printf("get_v :%d\r\n",get_v);
	
	 if(get_v == 0x04){
		 		Control_RGB_ALL(OFF);
		delay_ms(100);
//		if((lable2 == 0x04)){
//		Motor_Run(200,200);
		   Set_PID_Motor(70,70,0);
        delay_ms(1000);

			 Motor_Stop(1) ;
		get_v=0;
//			ret=1;
//		 printf("x:%2x\r\n",x);
		}
	 
		else if(get_v == 0x05){
	Control_RGB_ALL(OFF);
		delay_ms(100);

	 Set_PID_Motor(-50,-50,0);
        delay_ms(1000);

			 Motor_Stop(1) ;
				get_v=0;
//			ret=1;
//		 printf("x:%2x\r\n",x);
		}
	 
		
	else if(get_v == 0x06){
//		Motor_Left(200,200);
//		
			Control_RGB_ALL(OFF);
		delay_ms(100);
		    Set_PID_Motor(70,70,-50);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
				get_v=0;
//			ret=1;
//		 printf("x:%2x\r\n",x);
		}
	
			else if(get_v == 0x01){
		Control_RGB_ALL(OFF);
		delay_ms(100);
		Motor_Stop(1);
		
//			ret=1;
//		 printf("x:%2x\r\n",x);
		}
				else if(get_v == 0x07){
//	 Motor_Right(200,200);
						Control_RGB_ALL(OFF);
		delay_ms(100);
		    Set_PID_Motor(70,70,50);
        delay_ms(1000);
      
			 Motor_Stop(1) ;
							get_v=0;
//			ret=1;
//		 printf("x:%2x\r\n",x);
		}
			else if(get_v == 0x26){
					Control_RGB_ALL(OFF);
		delay_ms(100);
	 Buzzer_open_state();
		delay_ms(1000); 
 Buzzer_close_state();
		
		}
			else if(get_v == 0x08){
				Control_RGB_ALL(OFF);
				delay_ms(100);
//			Motor_Left(200,0);
			 Set_PID_Motor(0,-70,-70);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
					get_v=0;
			}
				else if(get_v == 0x09){
				Control_RGB_ALL(OFF);
				delay_ms(100);
			 Set_PID_Motor(-70,0,70);
        delay_ms(1000);
        delay_ms(1000);
			 Motor_Stop(1) ;
					get_v=0;
			}
				
			else if(get_v == 0x0B){
			
	
  	 Control_RGB_ALL(Red_RGB);
		delay_ms(100);
			}
						
			else if(get_v == 0x0C){
			
	
       	Control_RGB_ALL(Green_RGB);
				delay_ms(100);
					
			}
	
		else if(get_v == 0x0D){

 	Control_RGB_ALL(Blue_RGB);
		delay_ms(100);
		}
				else if(get_v == 0x0E){
			
		Control_RGB_ALL(Yellow_RGB);
		delay_ms(100);
			}
}
	




void Processing_Data(uint8_t GET_data,uint8_t *data1)
{
	
	
	static uint8_t recv_buf[6];
	static uint8_t g_index ;
	
	recv_buf[g_index] =GET_data;
	if((g_index==0) &&(recv_buf[g_index]!=0xAA))
	{
		
		recv_buf[g_index]=0;
		g_index=0;
	}
	else if(g_index==1 &&recv_buf[g_index]!=0x55)
	{
	
	recv_buf[g_index]=0;
	recv_buf[g_index-1]=0;
		g_index=0;
	
	}
		else if(g_index==2 &&recv_buf[g_index]!=0x00)
	{
	g_index=0;
	recv_buf[g_index] = 0;
	recv_buf[g_index-1] = 0;
	recv_buf[g_index-2] = 0;
	}
	else if((g_index==4 )&&(recv_buf[g_index]!=0xFB))
	{

	recv_buf[g_index]=0;
	recv_buf[g_index-1]=0;
	recv_buf[g_index-2]=0;
	recv_buf[g_index-3]=0;
	recv_buf[g_index-4]=0;
	g_index=0;
	}
////	
//	
	else{
		
		if(g_index == 4){
//		
//		
	
			*data1 = recv_buf[3];	

	printf("data1 :%d\r\n",*data1);

}
		g_index++;
		if (g_index ==5)
		{
			g_index = 0;

		}
	}

}
void Write_Data(uint8_t dat)
{

	voice[0] = 0xAA;
	voice[1] = 0x55;
	voice[2] = 0x00;
	voice[3] = dat;  // 直接使用传入的数据
	voice[4] = 0xFB;

	for(int j = 0; j<5 ;j++)
	{
		uart3_send_char(voice[j]);
	}
   

}

