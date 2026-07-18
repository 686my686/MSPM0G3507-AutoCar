#include "bsp_usart3.h"

//暂时使用9600



char cmd[5];
volatile uint8_t  recv3_flag = 0;

volatile uint8_t get_v;

void usart3_init(void)
{
	//清除串口中断标志
	NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
	//使能串口中断
	NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
	
	
	
}




//串口发送单个字符
void uart3_send_char(char ch)
{
	//当串口1忙的时候等待，不忙的时候再发送传进来的字符
	while( DL_UART_isBusy(UART_3_INST) == true );
	//发送单个字符
	DL_UART_Main_transmitData(UART_3_INST, ch);

}

//串口发送字符串
void uart3_send_string(char* str)
{
	//当前字符串地址不在结尾 并且 字符串首地址不为空
	while(*str!=0&&str!=0)
	{
		//发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart3_send_char(*str++);
	}
}





//串口中断服务函数
void UART_3_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;
	
	//如果产生了串口中断
	switch( DL_UART_getPendingInterrupt(UART_3_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断
			
			// 接收发送过来的数据保存
		
		
		receivedData = DL_UART_Main_receiveData(UART_3_INST);
		
		
		Processing_Data(receivedData,&get_v);
		

        
    
		break;
		
		default://其他的串口中断
			break;
    }


}


