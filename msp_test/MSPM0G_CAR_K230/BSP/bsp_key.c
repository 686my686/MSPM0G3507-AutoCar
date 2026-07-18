#include "bsp_key.h"

uint16_t g_key1_long_press = 0;
uint16_t g_key2_long_press = 0;
uint16_t g_key3_long_press = 0;

/*
 * 判断按键K1是否被按下
 * 返回: KEY_PRESS(1)=按下, KEY_RELEASE(0)=松开
 * 注意: 硬件按键为低电平有效（按下→GND→读回0→返回KEY_RELEASE）
 *       此函数已做取反处理，按下返回1
 */
static uint8_t Key1_is_Press(void)
{
    if ((DL_GPIO_readPins(KEY_button1_PORT, KEY_button1_PIN) & KEY_button1_PIN) != 0)
        return KEY_PRESS;
    return KEY_RELEASE;
}

/*
 * 按键K1长按检测
 * timeout: 长按时间阈值（秒）
 * 返回: 1=达到长按, 0=未达到
 */
uint8_t Key1_Long_Press(uint16_t timeout)
{
    if (g_key1_long_press > 0)
    {
        if (g_key1_long_press < timeout * 100 + 2)
        {
            g_key1_long_press++;
            if (g_key1_long_press == timeout * 100 + 2)
                return 1;
            return 0;
        }
    }
    return 0;
}

/*
 * 按键K1状态获取
 * mode: 0=连续触发(一直按下一直返回1), 1=单次触发(按下只返回一次1)
 * 返回: KEY_PRESS(1)=触发, KEY_RELEASE(0)=未触发
 */
uint8_t Key1_State(uint8_t mode)
{
    static uint16_t key1_state = 0;

    if (Key1_is_Press() == KEY_PRESS)
    {
        if (key1_state < (mode + 1) * 2)
            key1_state++;
    }
    else
    {
        key1_state = 0;
        g_key1_long_press = 0;
    }
    if (key1_state == 2)
    {
        g_key1_long_press = 1;
        return KEY_PRESS;
    }
    return KEY_RELEASE;
}

/*
 * 判断按键K2是否被按下
 */
static uint8_t Key2_is_Press(void)
{
    if ((DL_GPIO_readPins(KEY_button2_PORT, KEY_button2_PIN) & KEY_button2_PIN) != 0)
        return KEY_PRESS;
    return KEY_RELEASE;
}

/*
 * 按键K2长按检测
 */
uint8_t Key2_Long_Press(uint16_t timeout)
{
    if (g_key2_long_press > 0)
    {
        if (g_key2_long_press < timeout * 100 + 2)
        {
            g_key2_long_press++;
            if (g_key2_long_press == timeout * 100 + 2)
                return 1;
            return 0;
        }
    }
    return 0;
}

/*
 * 按键K2状态获取
 */
uint8_t Key2_State(uint8_t mode)
{
    static uint16_t key2_state = 0;

    if (Key2_is_Press() == KEY_PRESS)
    {
        if (key2_state < (mode + 1) * 2)
            key2_state++;
    }
    else
    {
        key2_state = 0;
        g_key2_long_press = 0;
    }
    if (key2_state == 2)
    {
        g_key2_long_press = 1;
        return KEY_PRESS;
    }
    return KEY_RELEASE;
}

/*
 * 判断按键K3是否被按下
 */
static uint8_t Key3_is_Press(void)
{
    if ((DL_GPIO_readPins(KEY_button3_PORT, KEY_button3_PIN) & KEY_button3_PIN) != 0)
        return KEY_PRESS;
    return KEY_RELEASE;
}

/*
 * 按键K3长按检测
 */
uint8_t Key3_Long_Press(uint16_t timeout)
{
    if (g_key3_long_press > 0)
    {
        if (g_key3_long_press < timeout * 100 + 2)
        {
            g_key3_long_press++;
            if (g_key3_long_press == timeout * 100 + 2)
                return 1;
            return 0;
        }
    }
    return 0;
}

/*
 * 按键K3状态获取
 */
uint8_t Key3_State(uint8_t mode)
{
    static uint16_t key3_state = 0;

    if (Key3_is_Press() == KEY_PRESS)
    {
        if (key3_state < (mode + 1) * 2)
            key3_state++;
    }
    else
    {
        key3_state = 0;
        g_key3_long_press = 0;
    }
    if (key3_state == 2)
    {
        g_key3_long_press = 1;
        return KEY_PRESS;
    }
    return KEY_RELEASE;
}

/*
 * 按键扫描函数（防抖优化版）
 *
 * 原版bug: 进判断后先delay再读键值，如果按键在10ms去抖期间弹回则漏检。
 * 修复: 先去抖前记录哪个键被按下，去抖后验证该键仍按下才返回。
 *
 * 返回: KEY1_PRES(1), KEY2_PRES(2), 0=无按键
 */
uint8_t KEY_Scan(void)
{
    volatile uint8_t ret = 0;
    static uint8_t release = 1;  /* 按键松开标志，防止连按 */

    if (release && (Key1_is_Press() == 0 || Key2_is_Press() == 0))
    {
        /* 先去抖前记录哪个键被按下 */
        uint8_t k1 = (Key1_is_Press() == 0) ? 1 : 0;
        uint8_t k2 = (Key2_is_Press() == 0) ? 1 : 0;

        delay_ms(10);  /* 去抖延时 */

        release = 0;

        /* 去抖后再次确认该键仍按下 */
        if (k1 && Key1_is_Press() == 0)
            ret = KEY1_PRES;
        else if (k2 && Key2_is_Press() == 0)
            ret = KEY2_PRES;
    }
    else if (Key1_is_Press() == 1 && Key2_is_Press() == 1)
    {
        release = 1;  /* 两键都松开后重新使能 */
        ret = 0;
    }
    return ret;
}

/*
 * 获取按键值（备用，供switch_mode调用）
 * 返回: KEY1_PRES(1), KEY2_PRES(2), 0=无按键
 */
uint8_t get_key(void)
{
    uint8_t KEY_Val = 0;

    if (Key1_State(1) == KEY_PRESS)
        KEY_Val = KEY1_PRES;

    if (Key2_State(1) == KEY_PRESS)
        KEY_Val = KEY2_PRES;

    return KEY_Val;
}
