#include "app_rgb.h"

//RGB简单灯效

#define 	Red_ON     1
#define 	Red_OFF     0

uint8_t RGB_flag ;



extern uint8_t ProtocolString[80];//引入备份数据区 Introducing backup data area
static void set_ALL_RGB_COLOR(unsigned long color)
{
    rgb_SetColor(Left_RGB,color);
    rgb_SetColor(Right_RGB,color);
}








void Control_RGB_ALL(RGB_Color_t color)
{
    switch(color)
    {
        case    Red_RGB:     set_ALL_RGB_COLOR(RED);break;
        case    Green_RGB:   set_ALL_RGB_COLOR(GREEN);break;
        case    Blue_RGB:    set_ALL_RGB_COLOR(BLUE);break;
        case    Yellow_RGB:  set_ALL_RGB_COLOR(YELLOW);break;
        case    Purple_RGB:  set_ALL_RGB_COLOR(PURPLE); break;  
        case    Cyan_RGB:    set_ALL_RGB_COLOR(CYAN);break;
        case    OFF  :       set_ALL_RGB_COLOR(BLACK);break;
        
        default : return;
        
    }
    
    rgb_SendArray();//必须发送,才显示
    delay_ms(200);
    
    
}

