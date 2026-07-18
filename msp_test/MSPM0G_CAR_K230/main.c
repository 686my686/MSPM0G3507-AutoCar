#include "AllHeader.h"
#include "app_ir_track.h"


//选择是否在初始化完成后，小车自主向前走，0为否，1为是(自主避让案例使用)
//Select whether the car moves forward autonomously after initialization is completed, 0 for no, 1 for yes (used in autonomous avoidance cases)

int Car_Auto_Drive = 0;

ColorMode color_mode = 2;  //1:巡线、2:跟踪	1: Patrol, 2: Tracking


int main(void)
{
    
		bsp_init();//需要的外设初始化  Required peripheral initialization
   	Control_RGB_ALL(OFF);
		delay_ms(100);
    OLED_ShowString(0,0,"pid_output:",8,1);
		OLED_ShowString(0,20,"pid_output1:",8,1);


    OLED_Refresh();
				
			
    while (1) 
    {	


				Pto_Loop();

				
    		OLED_ShowSNum(75,0, pid_output, 3, 8, 1);

				OLED_ShowSNum(75,20,pid_output1,3,8,1);
				OLED_Refresh();
				
    }
}














