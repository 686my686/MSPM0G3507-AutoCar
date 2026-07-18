#include "bsp_ir.h"
#include "stdio.h"
#include "string.h"

#define USART_RECEIVE_LENGTH 200
volatile u8 IR_Data_number[IR_Num];//数字值
char IR_recv_buff[USART_RECEIVE_LENGTH];    // 接收缓冲区
volatile int IR_recv_length = 3;            // 接收数据长度
volatile u8 IR_recv_complete_flag = 0;    // 接收数据完成标志位
volatile u8 oledbuf[13] = {0};  
volatile u8 IR_data_parsed_flag;  //数据解析完成标志




IR_RecvState ir_state = IR_STATE_FIND_START; // 初始状态
uint16_t ir_start_idx = 0; // 起始符 '$' 的后一位索引
uint16_t ir_end_idx = 0;   // 结束符 '#' 的索引

void IR_usart_config(void)
{
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

}



void IRDataAnalysis(void)
{
    char temp[60] = {0};
    char *buff = NULL;
    int head = 0;
    int end = 0;


    // 没有接收到数据 或者 数据没有接收完成 则不进行处理
    if (IR_recv_complete_flag == 0) return;

    // 找到格式的头的第一个 '$'
    while ((IR_recv_buff[head] != '$') && (head < IR_recv_length))
    {
        head++;
    }
    if (head == IR_recv_length)
    {
        // 清除接收完成标志位，等待下一次接收
        IR_recv_complete_flag = 0;
        IR_recv_length = 0;
        return;
    }
    buff = &IR_recv_buff[head];

    // 找到结尾 '#'
    while ((buff[end] != '#') && (end < IR_recv_length))
    {
        end++;
    }
    if ((head + end) == IR_recv_length)
    {
        // 清除接收完成标志位，等待下一次接收
        IR_recv_complete_flag = 0;
        IR_recv_length = 0;
        return;
    }

    // 复制数据到 temp 缓冲区
    if (end + 1 < sizeof(temp))
    {
        strncpy(temp, buff, end + 1);
        temp[end + 1] = '\0';  // 确保字符串终止

        // 检查数据格式是否正确（$D,...）
        if (temp[0] == '$' && temp[1] == 'D')
        {
            char *token = strtok(temp, ",");  // 分割字符串
            int index = 0;

            while (token != NULL && index < IR_Num)
            {
                // 解析 x1:1, x2:1, ..., x8:1
                if (strstr(token, "x") != NULL)
                {
                    char *colon = strchr(token, ':');
                    if (colon != NULL)
                    {
                        IR_Data_number[index] = (colon[1] - '0');  // 提取 '0' 或 '1'
                        index++;
                    }
                }
                token = strtok(NULL, ",");
            }

        }
    }
    // 清除接收完成标志位，等待下一次接收
    IR_recv_complete_flag = 0;
    IR_recv_length = 0;
    memset(IR_recv_buff, 0, USART_RECEIVE_LENGTH);
}



void UART_0_INST_IRQHandler(void)
{
    uint8_t RecvDATA = 0;

    // 检查中断来源
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX: // 接收中断
            RecvDATA = DL_UART_Main_receiveData(UART_0_INST);

            // 检查缓冲区是否已满
            if (IR_recv_length >= USART_RECEIVE_LENGTH - 1)
            {
                // 缓冲区满，丢弃数据或处理错误
                IR_recv_complete_flag = 0;
                IR_recv_length = 0;
                break;
            }

            // 存储数据
            IR_recv_buff[IR_recv_length++] = RecvDATA;
            IR_recv_buff[IR_recv_length] = '\0'; // 确保字符串终止

            // 收到 '#' 时标记接收完成
            if (RecvDATA == '#')
            {
                IR_recv_complete_flag = 1;
							 DL_UART_disableInterrupt(UART_0_INST, DL_UART_IIDX_RX);
            }
            break;
				case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
						uart0_send_string("$0,0,1#");
						DL_UART_disableInterrupt(UART_0_INST, DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR); // 关闭超时中断确保超时中断只触发一次
						break;
        default: // 其他中断（如发送中断）
            break;
    }
}
void IR_DATA(void)
{
	
		if( IR_recv_complete_flag)
			{
				IRDataAnalysis();
//				OLED_ShowString(0,25,(uint8_t *)"IR:",8,1);
//				for (int i = 0; i < 8; i++) 
//						OLED_ShowNum(20 + i * 10, 25, IR_Data_number[i], 1, 8, 1);        
				 DL_UART_enableInterrupt(UART_0_INST, DL_UART_IIDX_RX);
			}

//				OLED_Refresh();

	
}
