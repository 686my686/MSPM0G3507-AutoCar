#include "app_irtracking_eight.h"


#define IRTrack_Trun_KP (-28.00)
#define IRTrack_Trun_KI (-0.000000)
#define IRTrack_Trun_KD (-2.8000)


#define IRR_SPEED 70


#define IRTrack_Minddle 0


u8 x1, x2, x3, x4, x5, x6, x7, x8;


int pid_output_IRR = 0;


u8 trun_flag = 0;


int8_t err = 0;


float Track_PID(int8_t actual_value)
{
    float IRTrackTurn = 0;
    int8_t error;
    static int8_t error_last = 0;
    static float IRTrack_Integral;

    error = actual_value - IRTrack_Minddle;

    IRTrack_Integral += error;


    if (IRTrack_Integral > 100)
        IRTrack_Integral = 100;
    else if (IRTrack_Integral < -100)
        IRTrack_Integral = -100;


    IRTrackTurn = error * IRTrack_Trun_KP
                + IRTrack_Trun_KI * IRTrack_Integral
                + (error - error_last) * IRTrack_Trun_KD;

    error_last = error;

    return IRTrackTurn;
}


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


int LineCheck(void)
{
    IR_DATA();
    Copy_HD_Data();

    if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 &&
        x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
    {
        return WHITE_IR;  /* 八路全为1，表示位于白色区域。 */
    }
    else if (x1 + x2 + x3 + x4 + x5 + x6 + x7 + x8 < 8)
    {
        return BLACK_IR;  /* 至少一路为0，表示检测到黑线。 */
    }

    return WHITE_IR;
}


int LineStop(void)
{
    IR_DATA();
    Copy_HD_Data();


    /* 官方模块协议中0表示黑线，1表示白色背景。 */
    if (x1 == 0 || x2 == 0 || x3 == 0 || x4 == 0 ||
        x5 == 0 || x6 == 0 || x7 == 0 || x8 == 0)
    {
        return 1;
    }
    return 0;
}


void Line_Tracke(int state)
{
    IR_DATA();
    Copy_HD_Data();


    /* 从左到右设置递增权重，用加权平均值表示黑线横向偏差。 */
    int weights[8] = {-20, -15, -10, -5, 5, 10, 15, 20};
    int weighted_sum = 0;
    int sensor_active_count = 0;


    if (x1 == 0) { weighted_sum += weights[0]; sensor_active_count++; }
    if (x2 == 0) { weighted_sum += weights[1]; sensor_active_count++; }
    if (x3 == 0) { weighted_sum += weights[2]; sensor_active_count++; }
    if (x4 == 0) { weighted_sum += weights[3]; sensor_active_count++; }
    if (x5 == 0) { weighted_sum += weights[4]; sensor_active_count++; }
    if (x6 == 0) { weighted_sum += weights[5]; sensor_active_count++; }
    if (x7 == 0) { weighted_sum += weights[6]; sensor_active_count++; }
    if (x8 == 0) { weighted_sum += weights[7]; sensor_active_count++; }

    if (sensor_active_count > 0)
    {

        err = weighted_sum / sensor_active_count;
    }
    else
    {

        if (err > 0)
        {
            err = 20;
        }
        else if (err < 0)
        {
            err = -20;
        }
        else
        {
            err = 0;
        }
    }

    if (state == 1)
    {

        pid_output_IRR = (int)(Track_PID(err));
        Set_PID_Motor(IRR_SPEED + pid_output_IRR,
                      IRR_SPEED - pid_output_IRR, 0);
    }
    else
    {

        IR_DATA();
        Copy_HD_Data();
    }
}


void LineWalking(int line_l, int line_r, int state)
{
    if (state == 1)
    {
        IR_DATA();
        Copy_HD_Data();


        int weights[8] = {-20, -15, -10, -5, 5, 10, 15, 20};
        int weighted_sum = 0;
        int sensor_active_count = 0;

        if (x1 == 0) { weighted_sum += weights[0]; sensor_active_count++; }
        if (x2 == 0) { weighted_sum += weights[1]; sensor_active_count++; }
        if (x3 == 0) { weighted_sum += weights[2]; sensor_active_count++; }
        if (x4 == 0) { weighted_sum += weights[3]; sensor_active_count++; }
        if (x5 == 0) { weighted_sum += weights[4]; sensor_active_count++; }
        if (x6 == 0) { weighted_sum += weights[5]; sensor_active_count++; }
        if (x7 == 0) { weighted_sum += weights[6]; sensor_active_count++; }
        if (x8 == 0) { weighted_sum += weights[7]; sensor_active_count++; }

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


        pid_output_IRR = (int)(Track_PID(err));
        Set_PID_Motor(line_l + pid_output_IRR,
                      line_r - pid_output_IRR, 0);
    }
    else
    {

        IR_DATA();
        Copy_HD_Data();
    }
}
