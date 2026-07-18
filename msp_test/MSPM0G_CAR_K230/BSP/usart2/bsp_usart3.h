#ifndef __BSP_UASRT3_H_
#define __BSP_UASRT3_H_

#include "ALLHeader.h"


#define  RINGBUFF_LEN          (200)     //?????????
#define  RINGBUFF_OK           1     
#define  RINGBUFF_ERR          0   

typedef struct
{
    uint16_t Head;           
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;


void uart3_send_char(char ch);

void uart3_send_string(char* str);

void Write_Data(uint8_t dat);


void usart3_init(void);

#endif
