#include "bsp.h"

/* 初始化完成后小车是否自动前进: 0=否, 1=是（竞赛模式设为0，由按键选择模式后启动） */
int Car_Auto_Drive = 0;

/* 开发板底层初始化：SYSCFG配置 + UART中断使能 */
void board_init(void)
{
    /* SYSCFG初始化（SysConfig生成的时钟和外设配置） */
    SYSCFG_DL_init();

    /* 清除UART0中断标志并使能（用于八路巡线模块通信） */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    printf("Board GPIO Init \r\n");
}

/* 全部外设初始化 */
void bsp_init(void)
{
    board_init();

    /* 红外接收头初始化（遥控调试用） */
    infrared_config();

    /* OLED显示屏初始化 */
    OLED_Init();
    delay_ms(500);

    /* MPU6050陀螺仪初始化 + DMP姿态解算 */
    MPU6050_Init();
    DMP_Init();

    /* 编码器初始化（电机速度反馈） */
    encoder_init();

    /* 电机PWM初始化 */
    Init_Motor_PWM();

    /* 超声波模块初始化 */
    Ultrasonic_Init();

    /* 20ms周期定时器初始化（任务调度时基） */
    Timer_20ms_Init();

    /* 电机PID参数初始化 */
    PID_Param_Init();

    /* 卡尔曼滤波器初始化（陀螺仪数据融合） */
    Kalman_Init(&kfp_mpu);

    /* W25Q64 Flash存储初始化（参数存储） */
    W25Q64_Init();
}
