#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "system.h"

u8 click(void);
void Delay_ms(void);
u8 click_N_Double (u8 time);
u8 click_N_Double_MPU6050 (u8 time);
//u8 Long_Press(void);
u8  select(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
uint8_t User_Key_Scan(void);

#define KEY_ON	0
#define KEY_OFF	1
	
	
//用户按键返回值状态
#define No_Action 					0
#define Click 						1
#define Long_Press 					3
#define Double_Click				2

/*--------KEY control pin--------*/
#define KEY_PORT	GPIOE
#define KEY_PIN		GPIO_PIN_0
#define KEY			HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) 
/*----------------------------------*/

#endif 
