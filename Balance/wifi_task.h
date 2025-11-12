#ifndef __WIFI_TASK_H
#define __WIFI_TASK_H

#include "main.h"
#include "esp8266_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "balance.h"

#define WIFI_TASK_STACK_SIZE    512
#define WIFI_TASK_PRIORITY      3

// 连接健康状态
typedef enum {
    CONNECTION_HEALTHY = 0,     // 连接健康
    CONNECTION_DISCONNECTED     // 连接断开
} ConnectionHealth_t;

// WiFi服务函数声明
void WiFi_Task(void *pvParameters);
void WiFi_State_Init(void);
void WiFi_State_Connecting(void);
void WiFi_State_UDP_Reconnecting(void);
void WiFi_State_Ready(void);
void WiFi_State_Error(void);
void WiFi_State_Hard_Reset(void);  // 新增：硬件复位状态


#endif




