#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include "AllHeader.h"

typedef enum {
    FORWARD,  // 正向
    REVERSAL  // 反向
} ENCODER_DIR;

typedef struct {
    volatile long long temp_count; //保存实时计数值
    int count;         				//根据定时器时间更新的计数值
    ENCODER_DIR dir;            	 //旋转方向
    int ALLcount;  //开机到现在总的编码器计数
} ENCODER_RES;


void encoder_init(void);
ENCODER_DIR get_encoderL_dir(void);
ENCODER_DIR get_encoderR_dir(void);



void Get_Odometry(void);
void Encoder_Get_ALL(int *Encoder_all);
void Encoder_Get_Temp(int *Encoder_temp);
void encoder_update(void);

#endif
