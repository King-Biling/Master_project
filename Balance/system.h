#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件
//FreeRTOS相关头文件 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
//The associated header file for the peripheral 
//外设的相关头文件
#include "main.h"
#include "led.h"
#include "oled.h"
#include "balance.h"
#include "adc.h"
#include "encoder.h"	   
#include "key.h"
#include "I2C.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "delay.h"
#include "robot_select_init.h"
#include "usartx.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "wifi_task.h"
#include "esp8266_driver.h"
// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8 CCD_ON_Flag, APP_ON_Flag, ELE_ON_Flag, Avoid_ON_Flag, Follow_ON_Flag, Along_wall, PS2_ON_Flag;
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send; 
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern float CCD_KP,CCD_KI;
extern float ELE_KP,ELE_KI;
extern float Omni_Car_CCD_KP,Omni_Car_CCD_KI;
extern float Omni_Car_ELE_KP,Omni_Car_ELE_KI; 
extern float Tank_Car_CCD_KP,Tank_Car_CCD_KI;
extern float Tank_Car_ELE_KP,Tank_Car_ELE_KI; 
void systemInit(void);
/***Macros define***/ /***宏定义***/
//蜂鸣器控制
#define Buzzer_work()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET)
#define Buzzer_silence()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET)


//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(CONTROL_DELAY/50hz)秒后才允许控制小车进行运动
#define CONTROL_DELAY		750
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif      
