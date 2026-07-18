#ifndef __BSP_IBFRARED_LIGHT_H_
#define __BSP_IBFRARED_LIGHT_H_


#include "AllHeader.h"



void Open_IR_Switch(void);
void Close_IR_Switch(void);
void ADC_Init(void);

void get_light_distance();
//void Get_ADC_Value(void);
//void Get_ADC_Value(uint16_t gIRLeft,uint16_t gIRRight ,uint16_t gLightLeft ,uint16_t gLightRight);
void Get_ADC_Value();
void IR_Open_run(void);
float  Light_PID(int8_t actual_value);
	
void Get_Sun_Run();
#endif


