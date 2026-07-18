
#ifndef _BSP_OPENMV4_H_
#define _BSP_OPENMV4_H_

#include "AllHeader.h"

/* 串口缓冲区的数据长度 */
#define USART_RECEIVE_LENGTH  200
#define IR_Num 8 // 探头数量

// 解析状态枚举
typedef enum {
    IR_STATE_FIND_START,  // 寻找起始符 '$'
    IR_STATE_EXTRACT_DATA, // 提取 '$' 和 '#' 之间的有效数据
    IR_STATE_PARSE_DATA    // 解析有效数据
} IR_RecvState;

extern volatile u8 IR_recv_complete_flag;
extern volatile u8 IR_Data_number[];
extern volatile u8 oledbuf[];  

extern volatile int ir_data_index;            // 当前解析的数据索引
extern volatile int ir_token_index;           // 当前token位置
extern volatile char ir_current_token[10];    // 当前解析的token
extern volatile u8 IR_data_parsed_flag;  //数据解析完成标志
extern volatile u8 IR_data_parsed_flag;  //数据解析完成标志
void uart1_send_char(char ch);
void uart1_send_string(char* str);
void IR_usart_config(void);
void IRDataAnalysis(void);
void IR_DATA(void);
#endif