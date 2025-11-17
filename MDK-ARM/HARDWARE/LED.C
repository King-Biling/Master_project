#include "led.h"
int led_count = 0;
//int Led_Count=500; //LED flicker time control //LED闪烁时间控制
void Led_task(void *pvParameters)
{
		u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
		if(led_count < 1000000)
		{
			led_count++;
			if((angle_calibrated==1)&&(gyro_bias_captured==1))
			{
				if(led_count % 50 == 0) LED_FLASH();
			}
			else
			{
				if(led_count % 20 == 0) LED_FLASH();
			}
			
		}
		

	}
}

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED_SET();
	  else		if(++temp==time)	LED_FLASH(),temp=0;
	
}

//void Buzzer_Alarm(u16 count)
//{
//	int count_time;
//	if(0 == count)
//	{
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);//置低电平，蜂鸣器不响
//	}
//	else if(++count_time >= count)
//	{
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);//置高电平，蜂鸣器响
//		count_time = 0;	
//	}
//}


