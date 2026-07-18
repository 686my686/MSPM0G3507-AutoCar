#include "bsp_usart2.h"

//暂时使用9600


#define RE_2_BUFF_LEN_MAX	200

volatile uint8_t  recv2_buff[RE_2_BUFF_LEN_MAX] = {0};
volatile uint16_t recv2_length = 0;
volatile uint8_t  recv2_flag = 0;
volatile uint8_t receivedData2 = 0;



volatile uint8_t get_mode ;

void usart2_init(void)
{
	//清除串口中断标志
	NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
	//使能串口中断
	NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
	

}






//串口发送单个字符
void uart2_send_char(char ch)
{
	//当串口1忙的时候等待，不忙的时候再发送传进来的字符
	while( DL_UART_isBusy(UART_2_INST) == true );
	//发送单个字符
	DL_UART_Main_transmitData(UART_2_INST, ch);

}
//串口发送字符串
void uart2_send_string(char* str)
{
	//当前字符串地址不在结尾 并且 字符串首地址不为空
	while(*str!=0&&str!=0)
	{
		//发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart2_send_char(*str++);
	}
}







void UART_2_INST_IRQHandler(void)
{
	uint8_t receivedData2 = 0;
	
	//如果产生了串口中断

	switch( DL_UART_getPendingInterrupt(UART_2_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			// 接收发送过来的数据保存	Receive and save the data sent

		receivedData2 = DL_UART_Main_receiveData(UART_2_INST);
        
		Pto_Data_Receive(receivedData2);//处理数据   Processing Data
//		
//		uart0_send_char(receivedData2);
		
		
////		
//			// 每次接收到数据时重置计数器	Reset the counter each time data is received
			lost_count = 50;
			break;

		default://其他的串口中断	Other serial port interrupts
			break;
		

    }


}


