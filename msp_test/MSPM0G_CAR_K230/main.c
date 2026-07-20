#include "AllHeader.h"

/* 当前选择的赛题模式。 */
volatile uint8_t get_mode;


extern volatile float Yawpid_out;
extern volatile float kal_mpu_out;
extern int pid_output, pid_output1;

int main(void)
{
    uint32_t last_oled_ms = 0;

    /* 初始化全部外设。 */
    bsp_init();

    /* 关闭RGB灯。 */
    Control_RGB_ALL(OFF);
    delay_ms(100);

    /* 初始化八路巡线模块。 */
    uart0_send_string("$0,0,1#");

    /* OLED显示航向调试信息。 */
    OLED_ShowString(0, 0, "yaw:", 8, 1);
    OLED_Refresh();

    /* 按键选择1至4号赛题模式。 */
    get_mode = switch_mode();

    while (1)
    {
        /* 周期执行姿态读取、编码器测速和串级控制。 */
        Scheduler_Run();


        switch (get_mode)
        {
        case 1:
            mode_1();
            break;
        case 2:
            mode_2();
            break;
        case 3:
            mode_3();
            break;
        case 4:
            mode_4();
            break;
        default:
            break;
        }

        /* OLED刷新限制为10Hz，避免显示占用扰乱20ms控制周期。 */
        if ((uint32_t)(Get_Time() - last_oled_ms) >= 100U)
        {
            last_oled_ms = Get_Time();
            OLED_ShowSNum(75, 0, calibratedYaw, 3, 8, 1);
            OLED_Refresh();
        }
    }
}
