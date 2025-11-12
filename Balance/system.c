#include "system.h"



//Robot software fails to flag bits
//机器人软件失能标志位
u8 Flag_Stop=0;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC值分段变量，取决于小车型号数量，目前有6种小车型号
int Divisor_Mode;

// Robot type variable
//机器人型号变量
//0=Mec_Car，1=Omni_Car，2=Akm_Car，3=Diff_Car，4=FourWheel_Car，5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;  

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s
float RC_Velocity=500; 

//The Tank_Car CCD related variables
//全向小车CCD相关变量
float Tank_Car_CCD_KP=35,Tank_Car_CCD_KI=140;

//The Tank_Car ELE related variables
//全向小车ELE相关变量
float Tank_Car_ELE_KP=25,Tank_Car_ELE_KI=200;


//The Omni_Car CCD related variables
//全向小车CCD相关变量
float Omni_Car_CCD_KP=10,Omni_Car_CCD_KI=110;

//The Omni_Car ELE related variables
//全向小车ELE相关变量
float Omni_Car_ELE_KP=9,Omni_Car_ELE_KI=100;

//The other CCD related variables
//其他小车CCD相关变量
float CCD_KP=37,CCD_KI=140;

//The other ELE related variables
//其他小车ELE相关变量
float ELE_KP=30,ELE_KI=200;

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=700,Velocity_KI=700; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/******************* 小车型号相关变量 *******************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus; 
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1 communication control flag bit.
//These 5 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2手柄、蓝牙APP、航模手柄、CAN通信、串口1通信控制标志位。这5个标志位默认都为0，代表串口3控制模式
u8 CCD_ON_Flag=0, APP_ON_Flag=0, ELE_ON_Flag=0, Avoid_ON_Flag=0, Follow_ON_Flag=0, Along_wall=0, PS2_ON_Flag=0; 

//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send; 




