#ifndef __SHOW_H
#define __SHOW_H
#include "system.h"

// 显示任务配置
#define SHOW_TASK_PRIO		3       // 显示任务优先级
#define SHOW_STK_SIZE 		512     // 显示任务堆栈大小

static u32 sysTickCnt = 0;          // 系统滴答计数（静态变量）

// 函数声明
void show_task(void *pvParameters);     // OLED显示及相关任务（主任务）
void oled_show(void);                   // OLED显示管理（页面切换+内容刷新）
void oled2_show(void);                  // 预留OLED显示函数
void display_page1(void);               // OLED第1页显示（电机数据）
void display_page2(void);               // OLED第2页显示（基础状态数据）
void display_page3(void);               // OLED第3页显示
void display_page4(void);               // OLED第4页显示
void OLED_ShowCheckConfirming(void);    // OLED显示自检确认
void OLED_ShowChecking(void);           // OLED显示自检中
void OLED_ShowCheckResult(void);        // OLED显示自检结果
void APP_Show(void);                    // 向APP发送数据

#endif

