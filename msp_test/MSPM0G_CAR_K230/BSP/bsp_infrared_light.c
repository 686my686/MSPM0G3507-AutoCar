//这个是adc采集红外避障和光敏电阻的文件 

#include "bsp_infrared_light.h"

volatile bool gCheckADC;

volatile uint16_t gIRLeft;
volatile uint16_t gIRRight;
volatile uint16_t gLightLeft;
volatile uint16_t gLightRight;


volatile uint16_t gvol;


#define IRLight_Trun_KP (130)//140 
#define IRLight_Trun_KI (0.15) 
#define IRLight_Trun_KD (5) 

int pid_output_IRL = 0;
#define IRL_SPEED 			  200  //行驶速度
#define IRLight_Minddle    0 //中间的值


float  Light_PID(int8_t actual_value)
{
   float IRLightTurn = 0;
	int8_t error;
	static int8_t error_last=0;
	static float IRLight_Integral;//积分
	

	error=actual_value-IRLight_Minddle;
	
	IRLight_Integral +=error;
    
    if(IRLight_Integral>100)
        IRLight_Integral =100;
    else if(IRLight_Integral<-100)
        IRLight_Integral = -100;
	
	//	//位置式pid
	IRLightTurn=error*IRLight_Trun_KP
							+IRLight_Trun_KI*IRLight_Integral
							+(error - error_last)*IRLight_Trun_KD;
	return IRLightTurn;

}
//打开红外避障输出
void Open_IR_Switch(void)
{
    //在syscfg 配置中默认关闭
     DL_GPIO_setPins(Infrared_borad_PORT,Infrared_borad_IR_switch_PIN);
}

//关闭红外避障输出
void Close_IR_Switch(void)
{
    DL_GPIO_clearPins(Infrared_borad_PORT,Infrared_borad_IR_switch_PIN);
    
}


//ADC 初始化
void ADC_Init(void)
{
    NVIC_EnableIRQ(ADC_Senor_INST_INT_IRQN);
    DL_ADC12_enableConversions(ADC_Senor_INST);
    Open_IR_Switch();//打开红外避障
}


//void Get_ADC_Value(uint16_t gIRLeft,uint16_t gIRRight ,uint16_t gLightLeft ,uint16_t gLightRight)
void Get_ADC_Value()
{
    gCheckADC  = false;
    DL_ADC12_startConversion(ADC_Senor_INST);
    
    while (gCheckADC == false) 
    {
        __WFE(); //如果是小车运行中，低功耗不可取
    }
		
		

    
    gIRLeft = DL_ADC12_getMemResult(ADC_Senor_INST, ADC_Senor_ADCMEM_IR_left);//按照配置填写
    gIRRight = DL_ADC12_getMemResult(ADC_Senor_INST, ADC_Senor_ADCMEM_IR_right);
    gLightLeft = DL_ADC12_getMemResult(ADC_Senor_INST, ADC_Senor_ADCMEM_light_left);
    gLightRight = DL_ADC12_getMemResult(ADC_Senor_INST, ADC_Senor_ADCMEM_light_right);
//		gSunLeft =  DL_ADC12_getMemResult(ADC_Senor_INST,ADC_Senor_ADCMEM_light_left);
//		gSunRight = DL_ADC12_getMemResult(ADC_Senor_INST,ADC_Senor_ADCMEM_light_right);
    gvol = DL_ADC12_getMemResult(ADC_Senor_INST, ADC_Senor_ADCMEM_4);
    
    gCheckADC  = false;
    DL_ADC12_enableConversions(ADC_Senor_INST);
    
//    printf("IRL:%d,IRR:%d,LigL:%d,LigR:%d\r\n",gIRLeft,gIRRight,gLightLeft,gLightRight);
		printf("LigL:%d,LigR:%d\r\n",gLightLeft,gLightRight);
//    printf("gvol:%d\r\n",gvol);
    int voltage_value = ((gvol*3.3/4096)* 403);//(10+3.3)/3.3
	
//    printf("voltage value:%d.%d%dv\r\n",voltage_value/100,voltage_value/10%10,voltage_value%10 );
	
}


void get_light_distance()
{

//	Get_ADC_Value(L1, R1 , L2 , R2);
	Get_ADC_Value();
	uint32_t Value=(int)Hcsr04GetLength();
	
	if(Value>40){
		Motor_Run(300,300);
		delay_ms(100);
		}
	else {
		
		if(gIRLeft>gIRRight){
		
		Motor_Back(300,300);
		delay_ms(500);
		Motor_Left(300,300);
		delay_ms(500);
//			printf("1");
		}
	else{

		Motor_Back(300,300);
		delay_ms(500);
		Motor_Right(300,300);
		delay_ms(500);

	}
	}
//	printf("distance:%d\r\n",Value);



}	
	
void Get_Sun_Run()
{
	int LightR_value =(gLightRight/100);
	int LightL_value =(gLightLeft/100);
		Get_ADC_Value();

	if((LightL_value+LightR_value)>15)
	{
			Motor_Run(200,200);
			delay_ms(100);

	}
	else{
	
		if(LightL_value>LightR_value){
		
		Motor_Back(200,200);
		delay_ms(300);
		Motor_Right(200,200);
		delay_ms(500);
//			printf("1");
		}
	else{

		Motor_Back(200,200);
		delay_ms(300);
		Motor_Left(200,200);
		delay_ms(500);

	}
	
	
	
	
	
	
	
	
	}









}	
		




void ADC_Senor_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC_Senor_INST)) 
   {
        case DL_ADC12_IIDX_MEM4_RESULT_LOADED:
				
            gCheckADC = true;
            break;
        default:
            break;
    }
}
