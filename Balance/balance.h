#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"
#include "esp8266_driver.h"

#define BALANCE_TASK_PRIO		3     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)
#define PWMA1   TIM10->CCR1 
#define PWMA2   TIM11->CCR1 

#define PWMB1   TIM9->CCR1 
#define PWMB2   TIM9->CCR2

#define PWMC1   TIM1->CCR1  
#define PWMC2   TIM1->CCR2 

#define PWMD1   TIM1->CCR3 
#define PWMD2   TIM1->CCR4

#define EN     PDin(3)  

extern float Pos_KP;                     // 位置控制比例系数
extern float max_linear_speed;           // 最大线速度
extern float position_tolerance;         // 位置容差
extern uint8_t position_reached;         // 位置到达标志

extern uint8_t Auto_mode;
extern short test_num;
extern int robot_mode_check_flag;
extern u8 command_lost_count; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制

// 小车速度变量
extern float Current_Vx;      // X轴方向速度 (m/s)
extern float Current_Vy;      // Y轴方向速度 (m/s) 
extern float Current_Vz;      // Z轴旋转角速度 (rad/s)

void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
float float_abs(float insert);
void robot_mode_check(void);
void Auto_Adjust_Yaw(void);
float Yaw_PID_Control(float current_yaw, float target_yaw);
void Auto_Adjust_Position_And_Yaw(void);
// 速度计算函数
void Calculate_Car_Velocity(void);

#endif  

#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

// 任务配置（FreeRTOS）
// Task configuration (FreeRTOS)
#define BALANCE_TASK_PRIO		3     // 任务优先级 | Task priority
#define BALANCE_STK_SIZE 		512   // 任务堆栈大小 | Task stack size

// 全向轮小车运动学分析参数
// Kinematic parameters for omnidirectional trolley
#define X_PARAMETER    (sqrt(3)/2.f)  // X轴运动学参数 | X-axis kinematic parameter
#define Y_PARAMETER    (0.5f)         // Y轴运动学参数 | Y-axis kinematic parameter
#define L_PARAMETER    (1.0f)         // 轮距相关参数 | Wheelbase-related parameter

// PWM输出定义（对应定时器比较寄存器）
// PWM output definitions (corresponding to timer compare registers)
#define PWMA1   TIM10->CCR1  // 电机A正转PWM | Motor A forward PWM
#define PWMA2   TIM11->CCR1  // 电机A反转PWM | Motor A reverse PWM

#define PWMB1   TIM9->CCR1   // 电机B正转PWM | Motor B forward PWM
#define PWMB2   TIM9->CCR2   // 电机B反转PWM | Motor B reverse PWM

#define PWMC1   TIM1->CCR1   // 电机C正转PWM | Motor C forward PWM
#define PWMC2   TIM1->CCR2   // 电机C反转PWM | Motor C reverse PWM

#define PWMD1   TIM1->CCR3   // 电机D正转PWM | Motor D forward PWM
#define PWMD2   TIM1->CCR4   // 电机D反转PWM | Motor D reverse PWM

#define EN     PDin(3)  // 使能开关（输入）| Enable switch (input)

// 外部变量（供其他模块访问）
// External variables (accessible by other modules)
extern short test_num;  // 测试用变量 | Test variable
extern int robot_mode_check_flag;  // 机器人模式检查标志 | Robot mode check flag
extern u8 command_lost_count;  // 控制命令丢失计数（1秒丢失后停止控制）| Command loss counter (stop after 1s loss)

// 函数声明
// Function declarations
void Balance_task(void *pvParameters);  // 平衡控制任务（核心控制）| Balance control task (core control)
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);  // 设置电机PWM | Set motor PWM
void Limit_Pwm(int amplitude);  // 限制PWM输出幅值 | Limit PWM output amplitude
float target_limit_float(float insert,float low,float high);  // 浮点型限幅函数 | Float limit function
int target_limit_int(int insert,int low,int high);  // 整型限幅函数 | Integer limit function
u8 Turn_Off( int voltage);  // 安全关闭检查（电压、使能等）| Safety shutdown check (voltage, enable, etc.)
u32 myabs(long int a);  // 长整型绝对值计算 | Long integer absolute value calculation
int Incremental_PI_A (float Encoder,float Target);  // 电机A增量式PI控制 | Motor A incremental PI control
int Incremental_PI_B (float Encoder,float Target);  // 电机B增量式PI控制 | Motor B incremental PI control
int Incremental_PI_C (float Encoder,float Target);  // 电机C增量式PI控制 | Motor C incremental PI control
int Incremental_PI_D (float Encoder,float Target);  // 电机D增量式PI控制 | Motor D incremental PI control
void Get_RC(void);  // 处理遥控命令 | Process remote control commands
void Drive_Motor(float Vx,float Vy,float Vz);  // 运动学逆解（计算各轮目标速度）| Inverse kinematics (calculate wheel target speeds)
void Get_Velocity_Form_Encoder(void);  // 从编码器获取车轮速度 | Get wheel speed from encoder
void Smooth_control(float vx,float vy,float vz);  // 速度平滑控制 | Speed smoothing control
float float_abs(float insert);  // 浮点型绝对值计算 | Float absolute value calculation
void robot_mode_check(void);  // 机器人模式检查 | Robot mode check

#endif

