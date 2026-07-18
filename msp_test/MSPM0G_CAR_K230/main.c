#include "AllHeader.h"

/* 模式选择变量 */
volatile uint8_t get_mode;

/* 外部引用：PID输出和陀螺仪数据 */
extern volatile float Yawpid_out;
extern volatile float kal_mpu_out;
extern int pid_output, pid_output1;

int main(void)
{
    /* 全部外设初始化 */
    bsp_init();

    /* 关闭RGB灯 */
    Control_RGB_ALL(OFF);
    delay_ms(100);

    /* 向八路巡线模块发送初始化指令 */
    uart0_send_string("$0,0,1#");

    /* OLED显示调试信息 */
    OLED_ShowString(0, 0, "yaw:", 8, 1);
    OLED_Refresh();

    /* 按键选择运行模式（1-4对应赛题四项任务） */
    get_mode = switch_mode();

    while (1)
    {
        /* 任务调度器：按周期执行编码器更新、电机控制、姿态获取等 */
        Scheduler_Run();

        /* 根据选择的模式执行对应任务 */
        switch (get_mode)
        {
        case 1:
            mode_1();  /* 任务1: A→B直行，≤15s */
            break;
        case 2:
            mode_2();  /* 任务2: A→B→C(弧)→D→A(弧)，≤30s */
            break;
        case 3:
            mode_3();  /* 任务3: A→C→B(弧)→D→A(弧)，≤40s */
            break;
        case 4:
            mode_4();  /* 任务4: 任务3路径×4圈 */
            break;
        default:
            break;
        }

        /* OLED显示当前Yaw角度 */
        OLED_ShowSNum(75, 0, calibratedYaw, 3, 8, 1);
        OLED_Refresh();
    }
}
