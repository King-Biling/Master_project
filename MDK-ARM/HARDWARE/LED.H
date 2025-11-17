#ifndef __LED_H
#define __LED_H
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#define LED_TASK_PRIO		3
#define LED_STK_SIZE 		512  

/*--------Buzzer control pin--------*/
//#define Buzzer PAout(8)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)   // RST
#define LED_RESET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)     // RST
#define LED_FLASH() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12)     // RST
/*----------------------------------*/
extern int Time_count;
void Led_Flash(u16 time);
void Led_task(void *pvParameters);
//void Buzzer_Alarm(u16 count);
extern int Led_Count;
#endif
