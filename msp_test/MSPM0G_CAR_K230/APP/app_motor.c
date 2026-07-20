#include "app_motor.h"

static float speed_lr = 0;
static float speed_fb = 0;
static float speed_spin = 0;

static int speed_L1_setup = 0;
static int speed_R1_setup = 0;


static int g_offset_yaw = 0;
static uint16_t g_speed_setup = 0;

// ±àÂëÆ÷10ºÁÃëÇ°ºóÊı¾İ
//Encoder data before and after 10 milliseconds
int g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};
int g_Encoder_All_Offset[MAX_MOTOR] = {0};

uint8_t g_start_ctrl = 0;

car_data_t car_data;
motor_data_t motor_data;

uint8_t g_yaw_adjust = 0;

static float Motion_Get_Circle_Pulse(void)
{
    return ENCODER_CIRCLE_450;
}

// ½öÓÃÓÚÌí¼Óµ½µ÷ÊÔÖĞÏÔÊ¾Êı¾İ¡£
//Only used to display data when added to debugging.
void *Motion_Get_Data(uint8_t index)
{
    if (index == 1)
        return (int *)g_Encoder_All_Now;
    if (index == 2)
        return (int *)g_Encoder_All_Last;
    if (index == 3)
        return (int *)g_Encoder_All_Offset;
    return 0;
}

// »ñÈ¡µç»úËÙ¶È
//Obtain motor speed
void Motion_Get_Motor_Speed(float *speed)
{
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        speed[i] = motor_data.speed_mm_s[i];
    }
}

// ÉèÖÃÆ«º½½Ç×´Ì¬£¬Èç¹ûÊ¹ÄÜÔòË¢ĞÂtargetÄ¿±ê½Ç¶È¡£
//Set the yaw angle status, and if enabled, refresh the target target angle.
void Motion_Set_Yaw_Adjust(uint8_t adjust)
{
    if (adjust == 0)
    {
        g_yaw_adjust = 0;
    }
    else
    {
        g_yaw_adjust = 1;
    }
    if (g_yaw_adjust)
    {
        //PID_Yaw_Reset(»ñÈ¡µ±Ç°IMUÆ«º½½Ç-yaw);
    }
}

// ·µ»ØÆ«º½½Çµ÷½Ú×´Ì¬¡£
//Return to yaw angle adjustment status.
uint8_t Motion_Get_Yaw_Adjust(void)
{
    return g_yaw_adjust;
}


// Car Stop Ğ¡³µÍ£Ö¹
void Motion_Stop(uint8_t brake)
{
    Motion_Set_Speed(0, 0);
    g_start_ctrl = 0;
    g_yaw_adjust = 0;
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(brake);
}

// speed_mX=[-1000, 1000], µ¥Î»Îª£ºmm/s
//speed_mX=[-10001000],Unit: mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2)
{
    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    
    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i] * 1.0); 
    }
}

// Ôö¼ÓÆ«º½½ÇĞ£×¼Ğ¡³µÔË¶¯·½Ïò
//Increase yaw angle to calibrate the direction of the car's movement
void Motion_Yaw_Calc(float yaw)
{
    Wheel_Yaw_Calc(yaw);
}


//´øimuĞ£×¼µÄ
void Wheel_Yaw_Calc(float yaw)
{
    float yaw_offset = PID_Yaw_Calc(yaw);
    g_offset_yaw = yaw_offset * g_speed_setup;

    int speed_L1 = speed_L1_setup - g_offset_yaw;
    int speed_R1 = speed_R1_setup + g_offset_yaw;

    if (speed_L1 > 1000)
        speed_L1 = 1000;
    if (speed_L1 < -1000)
        speed_L1 = -1000;
   
    if (speed_R1 > 1000)
        speed_R1 = 1000;
    if (speed_R1 < -1000)
        speed_R1 = -1000;
   
    Motion_Set_Speed(speed_L1,  speed_R1);
}

// ´Ó±àÂëÆ÷¶ÁÈ¡µ±Ç°¸÷ÂÖ×ÓËÙ¶È£¬µ¥Î»mm/s
//Read the current speed of each wheel from the encoder, in mm/s
void Motion_Get_Speed(car_data_t *car)
{
    int i = 0;
    float speed_mm[MAX_MOTOR] = {0};
    float circle_mm = Motion_Get_Circle_MM();
    float circle_pulse = Motion_Get_Circle_Pulse();
    float robot_APB = Motion_Get_APB();

    Motion_Get_Encoder();

    // ¼ÆËãÂÖ×ÓËÙ¶È£¬µ¥Î»mm/s¡£
    //Calculate the wheel speed in mm/s.
    for (i = 0; i < 4; i++)
    {
        speed_mm[i] = (g_Encoder_All_Offset[i]) * 50 * circle_mm / circle_pulse;
    }

    car->Vx = (speed_mm[0] + speed_mm[1] ) / 2;
    car->Vy = -(speed_mm[0] - speed_mm[1] ) / 2;
    car->Vz = -(speed_mm[0] - speed_mm[1]) / 2.0f / robot_APB * 1000;

    /* å§‹ç»ˆæ›´æ–°ç¼–ç å™¨é€Ÿåº¦ï¼ˆç»å¯¹å€¼ï¼‰ï¼Œä¾›é€Ÿåº¦é—­ç¯ä½¿ç”¨
     * æ³¨æ„ï¼šå³è½®ç¼–ç å™¨ææ€§åäº†ï¼ˆH2A/Bç›¸åºä¸å·¦è½®ç›¸åï¼‰ï¼Œå–ç»å¯¹å€¼ä¿®æ­£ */
    for (i = 0; i < MAX_MOTOR; i++)
    {
        motor_data.speed_mm_s[i] = (speed_mm[i] >= 0) ? speed_mm[i] : -speed_mm[i];
    }

    if (g_start_ctrl)
    {
        PID_Calc_Motor(&motor_data);
    }
}

// ·µ»Øµ±Ç°Ğ¡³µÂÖ×ÓÖá¼ä¾àºÍµÄÒ»°ë
//Returns half of the sum of the current wheel spacing of the small car
float Motion_Get_APB(void)
{
    return MSPM0Car_APB;
}

// ·µ»Øµ±Ç°Ğ¡³µÂÖ×Ó×ªÒ»È¦µÄ¶àÉÙºÁÃ×
//Returns the number of millimeters the current car wheel has rotated once
float Motion_Get_Circle_MM(void)
{
    return MECANUM_CIRCLE_MM;
}

// »ñÈ¡±àÂëÆ÷Êı¾İ£¬²¢¼ÆËãÆ«²îÂö³åÊı
//Obtain encoder data and calculate the number of deviation pulses
void Motion_Get_Encoder(void)
{
    Encoder_Get_ALL(g_Encoder_All_Now);

    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        // ¼ÇÂ¼Á½´Î²âÊÔÊ±¼ä²îµÄÂö³åÊı
    	//Record the number of pulses between two test times
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
        // ¼ÇÂ¼ÉÏ´Î±àÂëÆ÷Êı¾İ
        //Record Last Encoder Data
        g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    }
}

// ¿ØÖÆĞ¡³µÔË¶¯
//Control the movement of the car
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
    wheel_Ctrl(V_x, V_y, V_z);
}

void Motion_Ctrl_State(uint8_t state, uint16_t speed, uint8_t adjust)
{
    uint16_t input_speed = speed * 10;
    wheel_State_YAW(state, input_speed, adjust);
}

// ¿ØÖÆĞ¡³µÔË¶¯×´Ì¬
// ËÙ¶È¿ØÖÆ£ºspeed=0~1000¡£
// Æ«º½½Çµ÷½ÚÔË¶¯£ºadjust=1¿ªÆô£¬=0²»¿ªÆô¡£
//Control the movement status of the car.
//Speed control: speed=0-1000.
//Yaw angle adjustment motion: adjust=1 on,=0 not on.
void wheel_State_YAW(uint8_t state, uint16_t speed, uint8_t adjust)
{
    Motion_Set_Yaw_Adjust(adjust);
    g_speed_setup = speed;
    switch (state)
    {
    case MOTION_STOP:
        g_speed_setup = 0;
        Motion_Stop(speed == 0 ? STOP_FREE : STOP_BRAKE);
        break;
    case MOTION_RUN:
        wheel_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        wheel_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
    	wheel_Ctrl(speed/2, 0, -speed*2);
        break;
    case MOTION_RIGHT:
    	wheel_Ctrl(speed/2, 0, speed*2);

        break;
    case MOTION_SPIN_LEFT:
        Motion_Set_Yaw_Adjust(0);
        wheel_Ctrl(0, 0, -speed * 5);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Set_Yaw_Adjust(0);
        wheel_Ctrl(0, 0, speed * 5);
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        break;
    }
}

// ¿ØÖÆĞ¡³µÔË¶¯×´Ì¬
// ËÙ¶È¿ØÖÆ£ºspeed=0~1000¡£
//Control the movement status of the car.
//Speed control: speed=0-1000.
void wheel_State(uint8_t state, uint16_t speed)
{
    g_speed_setup = speed;
    switch (state)
    {
    case MOTION_STOP:
        g_speed_setup = 0;
        Motion_Stop(speed == 0 ? STOP_FREE : STOP_BRAKE);
        break;
    case MOTION_RUN:
        wheel_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        wheel_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
    	wheel_Ctrl(speed/2, 0, -speed*2);
        break;
    case MOTION_RIGHT:
    	wheel_Ctrl(speed/2, 0, speed*2);

        break;
    case MOTION_SPIN_LEFT:
        Motion_Set_Yaw_Adjust(0);
        wheel_Ctrl(0, 0, -speed * 5);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Set_Yaw_Adjust(0);
        wheel_Ctrl(0, 0, speed * 5);
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        break;
    }
}


void wheel_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Motion_Get_APB();
//    speed_lr = -V_y;
    speed_lr = 0;
    speed_fb = V_x;
    speed_spin = (V_z / 1000.0f) * robot_APB;
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    speed_L1_setup = speed_fb + speed_lr + speed_spin;
    speed_R1_setup = speed_fb - speed_lr - speed_spin;

    if (speed_L1_setup > 1000)
        speed_L1_setup = 1000;
    if (speed_L1_setup < -1000)
        speed_L1_setup = -1000;
   
    if (speed_R1_setup > 1000)
        speed_R1_setup = 1000;
    if (speed_R1_setup < -1000)
        speed_R1_setup = -1000;
    
    
//    printf("%d,%d\r\n",speed_L1_setup, speed_R1_setup);
    //µ÷¼ÓpidµÄ¿ØÖÆ£¬·´Ó¦Âı¾ÍÖ±½Ópwm¿ØÖÆ
//    Motion_Set_Speed(speed_L1_setup, speed_R1_setup); //pid
    
    PWM_Control_Car(speed_L1_setup,speed_R1_setup);//Ö±½Ópwm¿Ø
}

// ÔË¶¯¿ØÖÆ¾ä±ú£¬Ã¿20msµ÷ÓÃÒ»´Î£¬Ö÷Òª´¦ÀíËÙ¶ÈÏà¹ØµÄÊı¾İ
//Motion control handle, called every 10ms, mainly processing speed related data
extern PID_t pid_motor[2];
void Motion_Handle(void)
{
    
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        //ÕâÑùĞ§¹û¸üºÃ
        if(motor_data.speed_set[0]==0)
        {
            motor_data.speed_pwm[0] = 0;
            pid_motor[0].pwm_output = 0;
        }
            
        if(motor_data.speed_set[1]==0)
        {
            motor_data.speed_pwm[1] = 0;
            pid_motor[1].pwm_output = 0;
        }
             
        PWM_Control_Car(motor_data.speed_pwm[0], motor_data.speed_pwm[1]);
    }
    
}
