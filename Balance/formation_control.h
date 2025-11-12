#ifndef __FORMATION_CONTROL_H
#define __FORMATION_CONTROL_H

#include "main.h"
#include "esp8266_driver.h"
#include "balance.h"
#include <math.h>
#include <string.h>
// 编队控制模式
#define FORMATION_MODE_NONE      0
#define FORMATION_MODE_LEADER    1
#define FORMATION_MODE_FOLLOWER  2

// 编队控制参数
#define FORMATION_TOLERANCE      0.08f    // 编队位置容差
#define FORMATION_MAX_SPEED      0.5f     // 编队最大速度
#define FORMATION_LEADER_TIMEOUT 4000     // 领航者数据超时时间(ms)
// 新增运动状态检测参数
#define VELOCITY_THRESHOLD 0.05f
#define ZERO_VELOCITY_COUNT_THRESHOLD 10
#define MOVING_AVERAGE_SIZE 5
// 编队控制变量
extern uint8_t Formation_mode;
extern char Formation_leader[10];
extern float Formation_offset_x;
extern float Formation_offset_y;
extern float Formation_offset_yaw;

// 新增混合控制参数
#define FORMATION_SPEED_FOLLOW_THRESHOLD 0.15f  // 速度跟随阈值
#define FORMATION_VELOCITY_GAIN 0.8f            // 速度跟随增益

extern float Formation_speed_follow_threshold;
extern float Formation_velocity_gain;

// 编队控制参数
extern float Formation_KP;
extern float Formation_KD;
extern float Formation_max_speed;
extern float Formation_tolerance;
extern float Formation_min_speed;


// 函数声明
uint8_t Detect_Leader_Motion_State(float vx, float vy);
// 函数声明
void Formation_Control(void);
void Formation_Follower_Control(void);
void Process_Formation_Command(const char* command);
void Process_Formation_Update(const char* command);

#endif