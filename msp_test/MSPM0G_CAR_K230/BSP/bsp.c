#include "bsp.h"




void board_init(void)
{
	// SYSCFG初始化
	SYSCFG_DL_init();
	//清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	
	printf("Board GPIO Init \r\n");
}


void bsp_init(void)
{
    board_init();
  
		usart2_init();
	
		usart3_init();
    ADC_Init();

    infrared_config();

		encoder_init();//初始化编码器配置 Initialize the encoder configuration
		Init_Motor_PWM();//初始化电机配置 Initialize the motor configuration
		Timer_20ms_Init();//初始化定时器  Initialize the timer
    OLED_Init();  //初始化OLED        Initialize the OLED

		PID_Param_Init();//初始化PID配置  Initialize the PID configuration



	
	
	


}
