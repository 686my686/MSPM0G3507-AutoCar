#include "usart0.h"

#define RE_0_BUFF_LEN_MAX	128
//RingBuff_t Uart2_RingBuff,Uart1_RingBuff,Uart3_RingBuff,Uart0_RingBuff;//????ringBuff????
volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;





//串口发送单个字符
void uart0_send_char(char ch)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while( DL_UART_isBusy(UART_0_INST) == true );
	//发送单个字符
	DL_UART_Main_transmitData(UART_0_INST, ch);

}
//串口发送字符串
void uart0_send_string(char* str)
{
	//当前字符串地址不在结尾 并且 字符串首地址不为空
	while(*str!=0&&str!=0)
	{
		//发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart0_send_char(*str++);
	}
}

#ifdef COMPILE_ENT == 0

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif


//printf函数重定义
int fputc(int ch, FILE *stream)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}



#else

//重定向fputc函数
int fputc(int ch, FILE *stream)
{
    while( DL_UART_isBusy(UART_0_INST) == true );
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

//重定向fputs函数
int fputs(const char* restrict s, FILE* restrict stream) {

    uint16_t char_len=0;
    while(*s!=0)
    {
        while( DL_UART_isBusy(UART_0_INST) == true );
        DL_UART_Main_transmitData(UART_0_INST, *s++);
        char_len++;
    }
    return char_len;
}
int puts(const char* _ptr)
{
 return 0;
}

#endif






////串口的中断服务函数
//void UART_0_INST_IRQHandler(void)
//{
//	uint8_t receivedData = 0;
//	
//	//如果产生了串口中断
//	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
//	{
//		case DL_UART_IIDX_RX://如果是接收中断
//			
//			// 接收发送过来的数据保存
//			receivedData = DL_UART_Main_receiveData(UART_0_INST);

//			// 检查缓冲区是否已满
//			if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
//			{
//				recv0_buff[recv0_length++] = receivedData;

//				// 将保存的数据再发送出去，不想回传可以注释掉
//				uart0_send_char(receivedData);
//			}
//			else
//			{
//				recv0_length = 0;
//			}

//			// 标记接收标志
//			recv0_flag = 1;
//		
//			break;
//		
//		default://其他的串口中断
//			break;
//	}
//}




