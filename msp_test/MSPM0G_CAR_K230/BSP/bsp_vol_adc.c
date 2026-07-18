////这是一个单独采集电池电压的文件，此文件一旦使用 无法在使用adc0采集其它东西，除非改写。
////syscfg 没配置,此工程不使用
//#include "bsp_vol_adc.h"

//volatile bool gCheckADC;        //ADC采集成功标志位


//void Vol_ADC_Init(void)
//{
//    //开启电压采集ADC中断
//    NVIC_EnableIRQ(ADC_Senor_INST_INT_IRQN);

//}

//void Get_Vol(void)
//{
//    static int adc_value =0;
//    static int voltage_value = 0;
//    
//    //获取ADC数据
//    adc_value = adc_getValue();
//    printf("adc value:%d\r\n", adc_value);

//    //将ADC采集的数据换算为电压
//    voltage_value = (int)((adc_value/4095.0*3.3)*403);

//    printf("voltage value:%d.%d%d\r\n",voltage_value/100,voltage_value/10%10,voltage_value%10 );

//}

////读取ADC的数据
//unsigned int adc_getValue(void)
//{
//    unsigned int gAdcResult = 0;

//    //软件触发ADC开始转换
//    DL_ADC12_startConversion(ADC_Senor_INST);
//    
//    //如果当前状态为正在转换中则等待转换结束
//    while (false == gCheckADC) {
//        __WFE(); //如果是巡线模式，不能进入低功耗模式
//    }
//    //获取数据
////    gAdcResult = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_ADC_VOL_0);

//    //清除标志位
//    gCheckADC = false;

//    return gAdcResult;
//}



//////ADC中断服务函数
////void ADC_INST_IRQHandler(void)
////{
////    //查询并清除ADC中断
////    switch (DL_ADC12_getPendingInterrupt(ADC_INST))
////    {
////        //检查是否完成数据采集
////        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
////                        gCheckADC = true;//将标志位置1
////                        break;
////        default:
////                        break;
////    }
////}