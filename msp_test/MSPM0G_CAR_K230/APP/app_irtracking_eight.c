#include "app_irtracking_eight.h"

/* 巡线PID参数（需要在赛道上实测调试） */
#define IRTrack_Trun_KP (-28.00)
#define IRTrack_Trun_KI (-0.000000)
#define IRTrack_Trun_KD (-2.8000)

/* 巡线基础速度 */
#define IRR_SPEED 70

/* 巡线PID中间值 */
#define IRTrack_Minddle 0

/* 八路传感器原始数据 */
u8 x1, x2, x3, x4, x5, x6, x7, x8;

/* 巡线PID输出 */
int pid_output_IRR = 0;

/* 转弯标志 */
u8 trun_flag = 0;

/* 当前偏差值 */
int8_t err = 0;

/*
 * 巡线PID计算
 * actual_value: 加权平均后的偏差值
 * 返回: PID输出（用于左右轮差速修正）
 */
float Track_PID(int8_t actual_value)
{
    float IRTrackTurn = 0;
    int8_t error;
    static int8_t error_last = 0;
    static float IRTrack_Integral;  /* 积分项 */

    error = actual_value - IRTrack_Minddle;

    IRTrack_Integral += error;

    /* 积分限幅 */
    if (IRTrack_Integral > 100)
        IRTrack_Integral = 100;
    else if (IRTrack_Integral < -100)
        IRTrack_Integral = -100;

    /* 位置式PID */
    IRTrackTurn = error * IRTrack_Trun_KP
                + IRTrack_Trun_KI * IRTrack_Integral
                + (error - error_last) * IRTrack_Trun_KD;

    error_last = error;

    return IRTrackTurn;
}

/*
 * 从串口接收缓冲区复制八路巡线数据
 * 数据由 bsp_ir.c 的 UART中断接收并解析
 */
void Copy_HD_Data(void)
{
    x1 = IR_Data_number[0];
    x2 = IR_Data_number[1];
    x3 = IR_Data_number[2];
    x4 = IR_Data_number[3];
    x5 = IR_Data_number[4];
    x6 = IR_Data_number[5];
    x7 = IR_Data_number[6];
    x8 = IR_Data_number[7];
}

/*
 * 检测是否在黑线上
 * 返回值: BLACK_IR(1)=在黑线上, WHITE_IR(0)=在白色区域
 * 任一传感器检测到黑线即判定为在黑线上
 */
int LineCheck(void)
{
    IR_DATA();
    Copy_HD_Data();

    if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 &&
        x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
    {
        return WHITE_IR;  /* 全白，不在线上 */
    }
    else if (x1 + x2 + x3 + x4 + x5 + x6 + x7 + x8 < 8)
    {
        return BLACK_IR;  /* 至少一个检测到黑线 */
    }

    return WHITE_IR;
}

/*
 * 巡线停车检测（用于直行段终点判断）
 * 任一传感器检测到黑线→到达顶点
 * 返回值: 1=到达黑线, 0=还在白色区域
 */
int LineStop(void)
{
    IR_DATA();
    Copy_HD_Data();

    /* 八路中任一位为1（检测到黑线）即表示到达顶点 */
    if (x1 == 1 || x2 == 1 || x3 == 1 || x4 == 1 ||
        x5 == 1 || x6 == 1 || x7 == 1 || x8 == 1)
    {
        return 1;
    }
    return 0;
}

/*
 * 八路巡线PID跟踪（加权平均法）
 *
 * 传感器布局和权重:
 *   X1   X2   X3   X4   X5   X6   X7   X8
 *  左←                              →右
 *  -20  -15  -10   -5   +5  +10  +15  +20
 *
 * state: 1=执行巡线+电机控制, 0=仅读取传感器数据
 */
void Line_Tracke(int state)
{
    IR_DATA();
    Copy_HD_Data();

    /* 加权平均法计算偏差 */
    int weights[8] = {-20, -15, -10, -5, 5, 10, 15, 20};
    int weighted_sum = 0;
    int sensor_active_count = 0;

    /* 检测到黑线时传感器值为1 */
    if (x1 == 1) { weighted_sum += weights[0]; sensor_active_count++; }
    if (x2 == 1) { weighted_sum += weights[1]; sensor_active_count++; }
    if (x3 == 1) { weighted_sum += weights[2]; sensor_active_count++; }
    if (x4 == 1) { weighted_sum += weights[3]; sensor_active_count++; }
    if (x5 == 1) { weighted_sum += weights[4]; sensor_active_count++; }
    if (x6 == 1) { weighted_sum += weights[5]; sensor_active_count++; }
    if (x7 == 1) { weighted_sum += weights[6]; sensor_active_count++; }
    if (x8 == 1) { weighted_sum += weights[7]; sensor_active_count++; }

    if (sensor_active_count > 0)
    {
        /* 计算加权平均偏差 */
        err = weighted_sum / sensor_active_count;
    }
    else
    {
        /* 所有传感器都未检测到黑线，进入寻线模式 */
        if (err > 0)
        {
            err = 20;   /* 上次偏右，继续右转寻线 */
        }
        else if (err < 0)
        {
            err = -20;  /* 上次偏左，继续左转寻线 */
        }
        else
        {
            err = 0;    /* 初始状态，直行 */
        }
    }

    if (state == 1)
    {
        /* PID计算并施加到电机 */
        pid_output_IRR = (int)(Track_PID(err));
        Set_PID_Motor(IRR_SPEED + pid_output_IRR,
                      IRR_SPEED - pid_output_IRR, 0);
    }
    else
    {
        /* 仅读取传感器，不控制电机 */
        IR_DATA();
        Copy_HD_Data();
    }
}

/*
 * 巡线行走函数（兼容control.c的状态机调用）
 *
 * line_l: 左轮基础速度
 * line_r: 右轮基础速度
 * state:  1=执行巡线控制, 0=仅读取传感器
 */
void LineWalking(int line_l, int line_r, int state)
{
    if (state == 1)
    {
        IR_DATA();
        Copy_HD_Data();

        /* 加权平均法计算偏差 */
        int weights[8] = {-20, -15, -10, -5, 5, 10, 15, 20};
        int weighted_sum = 0;
        int sensor_active_count = 0;

        if (x1 == 1) { weighted_sum += weights[0]; sensor_active_count++; }
        if (x2 == 1) { weighted_sum += weights[1]; sensor_active_count++; }
        if (x3 == 1) { weighted_sum += weights[2]; sensor_active_count++; }
        if (x4 == 1) { weighted_sum += weights[3]; sensor_active_count++; }
        if (x5 == 1) { weighted_sum += weights[4]; sensor_active_count++; }
        if (x6 == 1) { weighted_sum += weights[5]; sensor_active_count++; }
        if (x7 == 1) { weighted_sum += weights[6]; sensor_active_count++; }
        if (x8 == 1) { weighted_sum += weights[7]; sensor_active_count++; }

        if (sensor_active_count > 0)
        {
            err = weighted_sum / sensor_active_count;
        }
        else
        {
            if (err > 0)
                err = 20;
            else if (err < 0)
                err = -20;
            else
                err = 0;
        }

        /* PID计算，使用传入的速度作为基础速度 */
        pid_output_IRR = (int)(Track_PID(err));
        Set_PID_Motor(line_l + pid_output_IRR,
                      line_r - pid_output_IRR, 0);
    }
    else
    {
        /* 仅读取传感器数据，不控制电机 */
        IR_DATA();
        Copy_HD_Data();
    }
}
