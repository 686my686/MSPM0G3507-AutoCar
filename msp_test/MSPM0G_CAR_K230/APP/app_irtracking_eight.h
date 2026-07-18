#ifndef __APP_IRTRACKING_H__
#define __APP_IRTRACKING_H__

#include "AllHeader.h"

/* 黑线/白线检测常量 */
#define BLACK_IR  1  /* 检测到黑线 */
#define WHITE_IR  0  /* 在白色区域 */

/* 巡线速度限制 */
#define IRR_SPEED_LIMIT 30

/* 类型别名 */
#define u8  uint8_t
#define u16 uint16_t

/* 八路传感器原始数据（外部引用） */
extern u8 x1, x2, x3, x4, x5, x6, x7, x8;

/* 传感器数据处理 */
void Copy_HD_Data(void);
void deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8);

/* 黑线检测：返回BLACK_IR或WHITE_IR */
int LineCheck(void);

/* 停车检测：任一传感器检测到黑线返回1 */
int LineStop(void);

/* 八路巡线PID跟踪（加权平均法） */
void Line_Tracke(int state);

/* 巡线行走（兼容control.c状态机，支持自定义基础速度） */
void LineWalking(int line_l, int line_r, int state);

#endif
