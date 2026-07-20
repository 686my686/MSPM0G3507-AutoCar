#include "bsp_encoder.h"
#include "bsp_ir_receiver.h"

static ENCODER_RES motorL_encoder;
static ENCODER_RES motorR_encoder;

int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;


//编码器初始化
void encoder_init(void)
{
	//编码器引脚外部中断
	NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	NVIC_EnableIRQ(GPIOA_INT_IRQn);

     //测速定时器
    Timer_20ms_Init();
}


//获取左边编码器的方向
ENCODER_DIR get_encoderL_dir(void)
{
	return motorL_encoder.dir;
}



//获取右边编码器的方向
ENCODER_DIR get_encoderR_dir(void)
{
	return motorR_encoder.dir;
}


// 获取开机到现在总共的四路编码器计数。
void Encoder_Get_ALL(int *Encoder_all)
{
	Encoder_all[0] = motorL_encoder.ALLcount;
	Encoder_all[1] = motorR_encoder.ALLcount;
}

//获取每x ms的电机值 这里看测速定时器决定
void Encoder_Get_Temp(int *Encoder_temp)
{
  Encoder_temp[0] = motorL_encoder.ALLcount;
	Encoder_temp[1] = motorR_encoder.ALLcount;
}



//编码器数据更新
//请间隔一定时间更新 20ms更新一次
void encoder_update(void)
{
    int32_t left_count;
    int32_t right_count;
    uint32_t primask;

    /* 中断内持续更新脉冲，读取和清零必须作为一个不可分割操作 */
    primask = __get_PRIMASK();
    __disable_irq();
    left_count = motorL_encoder.temp_count;
    right_count = motorR_encoder.temp_count;
    motorL_encoder.temp_count = 0;
    motorR_encoder.temp_count = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

    /* 统一为小车向前时左右编码器都为正数 */
    left_count *= ENCODER_LEFT_FORWARD_SIGN;
    right_count *= ENCODER_RIGHT_FORWARD_SIGN;

	motorL_encoder.count = left_count;
    motorR_encoder.count = right_count;

    motorL_encoder.ALLcount += left_count;
    motorR_encoder.ALLcount += right_count;

	//确定方向
	motorL_encoder.dir = ( motorL_encoder.count >= 0 ) ? FORWARD : REVERSAL;
    motorR_encoder.dir = ( motorR_encoder.count >= 0 ) ? FORWARD : REVERSAL;

    /* 里程在20ms脉冲快照处累计，避免读取已清零的临时计数 */
    if (encoder_odometry_flag)
    {
        odometry_sum += (left_count + right_count) / 2;
    }
}


void Get_Odometry(void)
{
    /* 里程已在encoder_update中按20ms快照累计，此函数保留用于兼容旧任务表 */
}



#if Motor_Switch
//外部中断处理函数
void GROUP1_IRQHandler(void)
{
    uint32_t pendingSource = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    //  获取中断源
    if(pendingSource & DL_INTERRUPT_GROUP1_IIDX_GPIOB)
    {
        //  获取GPIOB所有已使能中断引脚的状态

        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFF);


        // 处理左边电机编码器中断（H1A和H1B引脚）
        if(gpio_status & (GPIO_ENCODER_L_H1A_PIN | GPIO_ENCODER_L_H1B_PIN))
        {
            // A相上升沿触发
            if(gpio_status & GPIO_ENCODER_L_H1A_PIN)
            {
                // 根据B相电平决定计数方向
                if(!DL_GPIO_readPins(GPIOB, GPIO_ENCODER_L_H1B_PIN))
                 {
                    motorL_encoder.temp_count++;
                }
                 else
                {
                    motorL_encoder.temp_count--;
                }
            }
            // B相上升沿触发
            else if(gpio_status & GPIO_ENCODER_L_H1B_PIN)
            {
                // 根据A相电平决定计数方向
                if(!DL_GPIO_readPins(GPIOB, GPIO_ENCODER_L_H1A_PIN))
                 {
                    motorL_encoder.temp_count--;
                }
                 else
                 {
                    motorL_encoder.temp_count++;
                }
            }

            // 清除编码器中断标志
            DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_L_H1A_PIN | GPIO_ENCODER_L_H1B_PIN);
        }

         //处理右电机的其中一条编码器通道
        if(gpio_status & GPIO_ENCODER_R_H2A_PIN) //PB13
        {
            // 根据B相电平决定计数方向
            if(!DL_GPIO_readPins(GPIOA, GPIO_ENCODER_R_H2B_PIN)) //PA31
             {
                motorR_encoder.temp_count--;
            }
             else
            {
                motorR_encoder.temp_count++;
            }
            // 清除编码器中断标志
            DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_R_H2A_PIN);
        }


        // 4. 处理红外中断
        if(gpio_status & IRContorl_GET_OUT_PIN)  // 替换为实际红外引脚
        {
            // 检查红外引脚是否为低电平
            if( GET_OUT == 0 ) {
                receiving_infrared_data();
            }

            // 清除红外中断标志
            DL_GPIO_clearInterruptStatus(GPIOB, IRContorl_GET_OUT_PIN);
        }
    }

     if(pendingSource & DL_INTERRUPT_GROUP1_IIDX_GPIOA)
    {
        // 处理右电机的另一条编码器通道
        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_ENCODER_R_H2B_PIN);
        //B相上升沿触发 PA31
        if(gpio_status & GPIO_ENCODER_R_H2B_PIN)
        {
            // 根据A相电平决定计数方向
            if(!DL_GPIO_readPins(GPIOB, GPIO_ENCODER_R_H2A_PIN)) //PB13
             {
                motorR_encoder.temp_count++;
            }
             else
             {
                motorR_encoder.temp_count--;
            }

            // 清除编码器中断标志
            DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_R_H2B_PIN);
        }

    }



}
#endif
