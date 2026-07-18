#include "bsp_key.h"

uint16_t g_key1_long_press = 0;
uint16_t g_key2_long_press = 0;
uint16_t g_key3_long_press = 0;


// 判断按键是否被按下，按下返回KEY_PRESS，松开返回KEY_RELEASE
static uint8_t Key1_is_Press(void)
{
    
	if ((DL_GPIO_readPins(KEY_button1_PORT, KEY_button1_PIN)&KEY_button1_PIN)!=0)
	{
		return KEY_PRESS; // 如果按键被按下，则返回KEY_PRESS
	}
	return KEY_RELEASE;   // 如果按键是松开状态，则返回KEY_RELEASE
}


// 读取按键K1的长按状态，累计达到长按时间返回1，未达到返回0.
// timeout为设置时间长度，单位为秒
uint8_t Key1_Long_Press(uint16_t timeout)
{
	if (g_key1_long_press > 0)
	{
		if (g_key1_long_press < timeout * 100 + 2)
		{
			g_key1_long_press++;
			if (g_key1_long_press == timeout * 100 + 2)
			{
				return 1;
			}
			return 0;
		}
	}
	return 0;
}




// 读取按键K1的状态，按下返回1，松开返回0.
// mode:设置模式，0：按下一直返回1；1：按下只返回一次1
uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
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



// 判断按键是否被按下，按下返回KEY_PRESS，松开返回KEY_RELEASE
static uint8_t Key2_is_Press(void)
{
    
	if ((DL_GPIO_readPins(KEY_button2_PORT, KEY_button2_PIN)&KEY_button2_PIN)!=0)
	{
		return KEY_PRESS; // 如果按键被按下，则返回KEY_PRESS
	}
	return KEY_RELEASE;   // 如果按键是松开状态，则返回KEY_RELEASE
}


// 读取按键K1的长按状态，累计达到长按时间返回1，未达到返回0.
// timeout为设置时间长度，单位为秒
uint8_t Key2_Long_Press(uint16_t timeout)
{
	if (g_key2_long_press > 0)
	{
		if (g_key2_long_press < timeout * 100 + 2)
		{
			g_key2_long_press++;
			if (g_key2_long_press == timeout * 100 + 2)
			{
				return 1;
			}
			return 0;
		}
	}
	return 0;
}




// 读取按键K2的状态，按下返回1，松开返回0.
// mode:设置模式，0：按下一直返回1；1：按下只返回一次1
uint8_t Key2_State(uint8_t mode)
{
	static uint16_t key2_state = 0;

	if (Key2_is_Press() == KEY_PRESS)
	{
		if (key2_state < (mode + 1) * 2)
		{
			key2_state++;
		}
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




// 判断按键是否被按下，按下返回KEY_PRESS，松开返回KEY_RELEASE
static uint8_t Key3_is_Press(void)
{
    
	if ((DL_GPIO_readPins(KEY_button3_PORT, KEY_button3_PIN)&KEY_button3_PIN)!=0)
	{
		return KEY_PRESS; // 如果按键被按下，则返回KEY_PRESS
	}
	return KEY_RELEASE;   // 如果按键是松开状态，则返回KEY_RELEASE
}


// 读取按键K1的长按状态，累计达到长按时间返回1，未达到返回0.
// timeout为设置时间长度，单位为秒
uint8_t Key3_Long_Press(uint16_t timeout)
{
	if (g_key3_long_press > 0)
	{
		if (g_key3_long_press < timeout * 100 + 2)
		{
			g_key3_long_press++;
			if (g_key3_long_press == timeout * 100 + 2)
			{
				return 1;
			}
			return 0;
		}
	}
	return 0;
}




// 读取按键K1的状态，按下返回1，松开返回0.
// mode:设置模式，0：按下一直返回1；1：按下只返回一次1
uint8_t Key3_State(uint8_t mode)
{
	static uint16_t key3_state = 0;

	if (Key3_is_Press() == KEY_PRESS)
	{
		if (key3_state < (mode + 1) * 2)
		{
			key3_state++;
		}
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


uint8_t KEY_Scan(void)
{	
	volatile uint8_t ret = 0;
	static uint8_t release=1;//按键按松开标志	  
	if( release && (Key1_is_Press()==0||Key2_is_Press()==0))
	{
		delay_ms(10);//去抖动 
		release = 0;
		if(Key1_is_Press()==0)				ret = KEY1_PRES;
		else if(Key2_is_Press() == 0)		ret = KEY2_PRES;
	}
	else if( Key1_is_Press()==1 && Key2_is_Press()==1 ){
		release = 1;
		ret = 0;
	}
 	return ret;// 无按键按下
}

uint8_t get_key()
{
	uint8_t KEY_Val;
//	if(Key1_is_Press()==KEY_PRESS||Key2_is_Press()==KEY_PRESS)
//	{	
	if(Key1_State(1)==KEY_PRESS)
		{
			
		KEY_Val=KEY1_PRES;
		
		}
	if (Key2_is_Press()==KEY_RELEASE){
		
		
		KEY_Val=KEY2_PRES;
		
		}
	
//	}

return KEY_Val;
}





/*********************************************END OF FILE**********************/
