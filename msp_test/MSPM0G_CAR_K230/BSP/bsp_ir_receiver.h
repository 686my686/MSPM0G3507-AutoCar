#ifndef _BSP_IR_RECEIVER_H__
#define _BSP_IR_RECEIVER_H__

#include "AllHeader.h"


#define GET_OUT         ( ( ( DL_GPIO_readPins(IRContorl_PORT,IRContorl_GET_OUT_PIN) & IRContorl_GET_OUT_PIN ) > 0 ) ? 1 : 0 )  

#define 	run_car     '1'//按键前 Before pressing the button
#define 	back_car    '2'//按键后 After pressing the button
#define 	left_car    '3'//按键左 Left button
#define 	right_car   '4'//按键右 Right button
#define 	stop_car    '0'//按键停 Button stop

void infrared_config(void);
uint8_t get_infrared_command(void);
void clear_infrared_command(void);
uint8_t receiving_infrared_data();
void Deal_ir_Motor_Data(void);
#endif


