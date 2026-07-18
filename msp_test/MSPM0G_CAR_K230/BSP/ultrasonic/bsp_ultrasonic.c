#include "bsp_ultrasonic.h"
#include "bsp_infrared_light.h"


volatile unsigned int msHcCount = 0;//ms计数
volatile float distance = 0;


volatile uint32_t systick_counter = 0;

/******************************************************************
 * 函 数 名 称：bsp_ultrasonic
 * 函 数 说 明：超声波初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：TRIG引脚负责发送超声波脉冲串
******************************************************************/
void Ultrasonic_Init(void)
{
    //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_1ms_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_1ms_INST_INT_IRQN);
		DL_TimerG_startCounter(TIMER_1ms_INST); 
        
}
/******************************************************************
 * 函 数 名 称：Open_Timer
 * 函 数 说 明：打开定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：
******************************************************************/
void Open_Timer(void)
{
        
    DL_TimerG_setTimerCount(TIMER_1ms_INST, 0);   // 清除定时器计数  
        
    msHcCount = 0;  
        
    DL_TimerG_startCounter(TIMER_1ms_INST);   // 使能定时器
}

/******************************************************************
 * 函 数 名 称：Get_TIMER_Count
 * 函 数 说 明：获取定时器定时时间
 * 函 数 形 参：无
 * 函 数 返 回：数据
 * 备       注：
******************************************************************/
uint32_t Get_TIMER_Count(void)
{
    uint32_t time  = 0;  
    time   = msHcCount*1000;                         // 得到us 
    time  += DL_TimerG_getTimerCount(TIMER_1ms_INST);  // 得到ms 
        
    DL_TimerG_setTimerCount(TIMER_1ms_INST, 0);   // 清除定时器计数  
    delay_ms(1);
    return time ;          
}

/******************************************************************
 * 函 数 名 称：Close_Timer
 * 函 数 说 明：关闭定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：
******************************************************************/
void Close_Timer(void)
{
    DL_TimerG_stopCounter(TIMER_1ms_INST);     // 关闭定时器 
}


/******************************************************************
 * 函 数 名 称：TIMER_0_INST_IRQHandler
 * 函 数 说 明：定时器中断服务函数
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：1ms进入一次
******************************************************************/
void TIMER_1ms_INST_IRQHandler(void)
{
    //如果产生了定时器中断
    switch( DL_TimerG_getPendingInterrupt(TIMER_1ms_INST) )
    {
        case DL_TIMERA_IIDX_LOAD: //重加载
                msHcCount++;
            break;
				
				
				case DL_TIMERG_IIDX_ZERO:
				 systick_counter++; // 每1ms自动+1      +1 per sencond
            break;
				
        
        default://其他的定时器中断
            break;
    }
}





uint32_t Get_Time(void)    
{
    return msHcCount;
}


/******************************************************************
 * 函 数 名 称：Hcsr04GetLength
 * 函 数 说 明：获取测量距离
 * 函 数 形 参：无
 * 函 数 返 回：测量距离
 * 备       注：无
******************************************************************/
volatile float t = 0;
float Hcsr04GetLength(void)
{
        /*测5次数据计算一次平均值*/
        volatile float length = 0;
		t = 0;
        volatile float sum = 0;
        volatile unsigned int  i = 0;
        
		Close_Timer();
        
        while(i != 5)
        {
            SR04_TRIG(0);//trig拉低信号，发出低电平s   
            delay_1us(10);//持续时间超过5us                        
                    
            SR04_TRIG(1);//trig拉高信号，发出高电平
                    
            delay_1us(15);//持续时间超过10us
                    
            SR04_TRIG(0);//trig拉低信号，发出低电平
                    
            /*Echo发出信号 等待回响信号*/
            /*输入方波后，模块会自动发射8个40KHz的声波，与此同时回波引脚（echo）端的电平会由0变为1；
            （此时应该启动定时器计时）；当超声波返回被模块接收到时，回波引 脚端的电平会由1变为0；
            （此时应该停止定时器计数），定时器记下的这个时间即为
                                            超声波由发射到返回的总时长；*/
            
            while(SR04_ECHO() == 0);//echo等待回响
            
            Open_Timer();   //打开定时器 

            i++;
            
            while(SR04_ECHO() > 0);
            
            Close_Timer();   // 关闭定时器  
            
            delay_ms(100);
                            
            t = (float)Get_TIMER_Count();   // 获取时间,分辨率为1us
            length = (float)t / 58.0f;   // cm  
            sum += length;    
                                

        }
        length = sum/5;//五次平均值
        distance = length;
				
        return length;
}





void get_distance()
{
	static int L1 ;
	static int R1 ;
	static int L2 ;
	static int R2 ;
//	Get_ADC_Value(L1, R1 , L2 , R2);
	Get_ADC_Value();
	uint32_t Value=(int)Hcsr04GetLength();
	
	if(Value>30){
		Motor_Run(200,200);
		delay_ms(100);
		}
	else {
		
		if(L1>R1){
		
		Motor_Back(200,200);
		delay_ms(500);
		Motor_Left(200,200);
//		delay_ms(500);
//			printf("1");
		}
	else{

		Motor_Back(200,200);
		delay_ms(500);
		Motor_Right(200,200);
//		delay_ms(500);
//		printf("2");
		printf("R1 :%d L1:%d\r\n",R1,L1);
	}
	}
	
		
//	printf("distance:%d\r\n",Value);



}