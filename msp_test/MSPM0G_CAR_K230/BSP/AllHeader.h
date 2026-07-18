#ifndef __ALLHEADER_H_
#define __ALLHEADER_H_

/* 类型别名 */
#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t

/* 标准库 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* TI SDK 配置 */
#include "ti_msp_dl_config.h"

/* 基础BSP */
#include "delay.h"
#include "config.h"
#include "bsp.h"
#include "usart0.h"

/* Flash存储 */
#include "bsp_flash_W25Q64.h"

/* 蜂鸣器和LED */
#include "bsp_beep_led.h"

/* 按键 */
#include "bsp_key.h"

/* 定时器 */
#include "bsp_timer.h"

/* RGB灯 */
#include "bsp_RGB.h"
#include "app_rgb.h"

/* 红外接收与遥控 */
#include "bsp_ir_receiver.h"
#include "app_irremote.h"

/* 电机与编码器 */
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "app_motor.h"
#include "bsp_PID_motor.h"

/* OLED显示 */
#include "oled.h"

/* MPU6050陀螺仪 + DMP */
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "bsp_mpu6050.h"
#include "get_mpu6050.h"
#include "Filter.h"

/* 超声波 */
#include "bsp_ultrasonic.h"

/* 任务调度器 */
#include "task.h"

/* 卡尔曼滤波 */
#include "Kalman.h"

/* 八路红外巡线模块（串口版） */
#include "bsp_ir.h"

/* 路径控制（四个Mode状态机） */
#include "control.h"

/* 八路巡线算法 */
#include "app_irtracking_eight.h"

#endif
