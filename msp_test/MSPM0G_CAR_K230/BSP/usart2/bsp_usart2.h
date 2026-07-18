#ifndef __BSP_UASRT2_H_
#define __BSP_UASRT2_H_

#include "ALLHeader.h"

void usart2_init(void);

void uart2_send_char(char ch);
void uart2_send_string(char* str);



#endif
