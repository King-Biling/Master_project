#include "balance.h"
#include "formation_control.h"  // 添加编队控制头文件

int Time_count=0; //Time variable //计时变量
uint8_t Auto_mode = 1;
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据

float Pos_KP = 0.5f;                    // 位置控制比例系数，需要调整
float max_linear_speed = 0.3f;          // 最大线速度 m/s
float position_tolerance = 0.05f;       // 5cm容差
uint8_t position_reached = 0;           // 位置到达标志

float Current_Vx = 0.0f;      // X轴方向速度 (m/s) - 前向为正
float Current_Vy = 0.0f;      // Y轴方向速度 (m/s) - 左向为正  
float Current_Vz = 0.0f;      // Z轴旋转角速度 (rad/s) - 逆时针为正



/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
        
	        //Speed smoothing is enabled when moving the omnidirectional trolley
	        //全向移动小车才开启速度平滑处理
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
            //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		
			//Mecanum wheel car
			//麦克纳姆轮小车
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 

}



/**************************************************************************
Function: 计算小车整体速度（运动学正解）
Input   : 无
Output  : 无
函数功能：根据四个轮子的速度计算小车的整体运动速度
**************************************************************************/
void Calculate_Car_Velocity(void)
{
    float v1 = MOTOR_A.Encoder;  // 后左
    float v2 = MOTOR_B.Encoder;  // 前左  
    float v3 = MOTOR_C.Encoder;  // 前右
    float v4 = MOTOR_D.Encoder;  // 后右
    
    float L = (Axle_spacing + Wheel_spacing) / 2.0f;
    
    // 从逆运动学推导正运动学：
    // 逆运动学矩阵：
    // [v1]   [ 1  1 -L] [Vx]
    // [v2] = [ 1 -1 -L] [Vy]
    // [v3]   [ 1  1  L] [Vz]
    // [v4]   [ 1 -1  L]
    
    // 正运动学（解这个线性方程组）：
    Current_Vx = (v1 + v2 + v3 + v4) / 4.0f;
    Current_Vy = (v1 - v2 + v3 - v4) / 4.0f;
    
    if (L > 0.01f) {
        Current_Vz = (-v1 - v2 + v3 + v4) / (4.0f * L);
    } else {
        Current_Vz = 0.0f;
    }
    
    // 滤波处理
    static float filtered_Vx = 0.0f, filtered_Vy = 0.0f, filtered_Vz = 0.0f;
    float filter_factor = 0.7f;
    
    filtered_Vx = filtered_Vx * filter_factor + Current_Vx * (1.0f - filter_factor);
    filtered_Vy = filtered_Vy * filter_factor + Current_Vy * (1.0f - filter_factor);  
    filtered_Vz = filtered_Vz * filter_factor + Current_Vz * (1.0f - filter_factor);
    
    Current_Vx = filtered_Vx; 
    Current_Vy = filtered_Vy;
    Current_Vz = filtered_Vz;
    
    // // 调试输出，验证计算正确性
    // static uint32_t debug_count = 0;
    // debug_count++;
    // if (debug_count % 200 == 0) {
    //     char debug_msg[256];
    //     snprintf(debug_msg, sizeof(debug_msg), 
    //              "[速度计算] 轮速:%.3f,%.3f,%.3f,%.3f -> 车体:Vx=%.3f,Vy=%.3f,Vz=%.3f\r\n",
    //              v1, v2, v3, v4, Current_Vx, Current_Vy, Current_Vz);
    //     usart1_send_cstring(debug_msg);
    // }
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
// 在 Balance_task 函数中修改模式切换逻辑
void Balance_task(void *pvParameters)
{ 
    u32 lastWakeTime = getSysTickCnt();
    static uint32_t control_debug_count = 0;
    static uint32_t last_other_cars_print = 0;
    
    while(1)
    {	
        // 此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

        // 时间计数
        if(Time_count<40000) Time_count++;

        // 获取编码器数据
        Get_Velocity_Form_Encoder();           		

        // // 每10秒打印一次其他小车信息
        // uint32_t current_time = HAL_GetTick();
        // if (control_debug_count % 100 == 0) {
        //     Print_Other_Cars_Info();
        //     last_other_cars_print = current_time;
        // }

        // 新的控制逻辑：编队模式具有最高优先级
        if(Formation_mode > 0) {
            // 编队模式优先，无论是否在自动或遥控模式
            Formation_Control();
            
            // 调试信息
            static uint32_t formation_debug_count = 0;
            formation_debug_count++;
            if(formation_debug_count % 200 == 0) {
                char debug_msg[256];
                if(Formation_mode == FORMATION_MODE_LEADER) {
                    snprintf(debug_msg, sizeof(debug_msg), 
                             "[编队] 领航者模式 - 位置(%.2f,%.2f) 航向%.1f°\r\n", 
                             position[0], position[1], Yaw);
                } else if(Formation_mode == FORMATION_MODE_FOLLOWER) {
                    snprintf(debug_msg, sizeof(debug_msg), 
                             "[编队] 跟随者模式 - 目标(%.2f,%.2f,%.1f) 当前位置(%.2f,%.2f,%.1f)\r\n", 
                             Target_position[0], Target_position[1], Target_Yaw,
                             position[0], position[1], Yaw);
                }
                usart1_send_cstring(debug_msg);
            }
        }
        else if(Auto_mode && newCoordinateReceived) {
            // 原有的自动位置控制（非编队模式）
            Auto_Adjust_Position_And_Yaw();
        }
        else if(APP_ON_Flag) {
            // 手动遥控模式（非编队模式）
            Get_RC();
            newCoordinateReceived = 0;
            position_reached = 0;
        }
        else {
            // 无控制指令，停止小车
            Drive_Motor(0, 0, 0);
        }
        
        
        if((Voltage>10)&&(EN==1)) 
        { 			
            // 速度闭环控制计算各电机PWM值
            MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
            MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
            MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
            MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
            Limit_Pwm(1500);
            
            // 设置PWM
            switch(Car_Mode)
            {
                case Mec_Car: Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, -MOTOR_D.Motor_Pwm, 0); break;
            }
        }
        else
        {
            Set_Pwm(0,0,0,0,0); 
        }
        
        control_debug_count++;
    }
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=1599,PWMA2=1599+motor_a;
	else 	            PWMA2=1599,PWMA1=1599-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=1599,PWMB2=1599+motor_b;
	else 	            PWMB2=1599,PWMB1=1599-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=1599,PWMC2=1599+motor_c;
	else 	            PWMC2=1599,PWMC1=1599-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=1599,PWMD2=1599+motor_d;
	else 	            PWMD2=1599,PWMD1=1599-motor_d;

	
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
          目前没有用上该函数
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC2=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;

	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }


	//Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z; // 从app端得到的速度指令单位为mm/s 但是小车处理m/s的数据
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}



/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据

		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	
	
	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
	
        // 新增：计算小车整体速度
        Calculate_Car_Velocity();
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


/**************************************************************************
Function: 自动航向调整函数
Input   : 无
Output  : 无
函数功能：自动调整小车到目标航向
**************************************************************************/
void Auto_Adjust_Yaw(void)
{
    // 获取当前航向角（需要从陀螺仪或其他传感器获取）
    float current_yaw = Yaw; 
    
    // 计算航向控制量
    float yaw_control = Yaw_PID_Control(current_yaw, Target_Yaw);
    
    // 设置旋转速度，其他方向速度为0
    Drive_Motor(0, 0, yaw_control);

}

/**************************************************************************
Function: 航向控制PID函数
Input   : 当前航向角，目标航向角
Output  : 旋转速度控制量
函数功能：使用PID控制算法计算小车的旋转速度，使小车朝向目标航向
**************************************************************************/
float Yaw_KP = 0.12f;          // 比例系数，需要根据实际调整
float Yaw_KI = 0.001f;         // 积分系数，需要根据实际调整
float Yaw_KD = 0.05f;           // 微分系数，需要根据实际调整
float Yaw_PID_Control(float current_yaw, float target_yaw)
{
    float yaw_error, yaw_output;
    static float yaw_integral = 0;
    static float last_yaw_error = 0;
    
    // 计算航向误差（考虑角度循环特性）
    yaw_error = target_yaw - current_yaw; // 负值：应当顺时针调整  正值：应当逆时针调整
    
    // 将误差规范化到[-180, 180]度范围内
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

	// 计算微分项
    float derivative = yaw_error - last_yaw_error;
    // 计算积分项	
    yaw_integral += yaw_error;

	// 最重要的修改：将积分限幅与输出范围关联
	// 假设最大输出是 1.0，KI是0.01，那么积分项最大允许 (1.0 / 0.01) = 100 是合理的。
	// 这表示积分项单独最大只能贡献 1.0 的输出量。
	float integral_max = 1.0f / Yaw_KI; // 动态计算积分上限
	yaw_integral = target_limit_float(yaw_integral, -integral_max, integral_max);
	// PID计算
    yaw_output = Yaw_KP * yaw_error + Yaw_KI * yaw_integral + Yaw_KD * (yaw_error - last_yaw_error);
    
    
    
    // 保存当前误差用于下一次计算
    last_yaw_error = yaw_error;
    
    // 限制输出范围
    return target_limit_float(yaw_output, -1.0f, 1.0f);
}

/**************************************************************************
Function: 自动位置和航向调整函数
Input   : 无
Output  : 无
函数功能：自动调整小车到目标位置和航向
**************************************************************************/
void Auto_Adjust_Position_And_Yaw(void)
{
    // 获取当前航向角和位置
    float current_yaw = Yaw; 
    
    // 计算位置误差
    float error_x = Target_position[0] - position[0];
    float error_y = Target_position[1] - position[1];
    
    // 计算到目标的距离和方向角（全局坐标系）
    float distance_to_target = sqrtf(error_x * error_x + error_y * error_y);
    float global_target_angle = atan2f(error_y, error_x) * 180.0f / PI;
    
    // 每50次调用打印一次调试信息
    static uint32_t adjust_count = 0;
    adjust_count++;
    if(adjust_count % 50 == 0) {
        char debug_msg[256];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[导航] 误差(%.3f,%.3f) 距离:%.3f 目标角:%.1f 当前角:%.1f\r\n", 
                 error_x, error_y, distance_to_target, global_target_angle, current_yaw);
        usart1_send_cstring(debug_msg);
    }
    
    // 检查是否到达目标位置
    if(distance_to_target < position_tolerance) {
        position_reached = 1;
        // 只进行航向调整，不移动
        float yaw_control = Yaw_PID_Control(current_yaw, Target_Yaw);
        if(adjust_count % 50 == 0) {
            char debug_msg[128];
            snprintf(debug_msg, sizeof(debug_msg), 
                     "[导航] 到达目标，航向控制:%.3f\r\n", yaw_control);
            usart1_send_cstring(debug_msg);
        }
        Drive_Motor(0, 0, yaw_control);
        return;
    } else {
        position_reached = 0;
    }
    
    // 将全局坐标系下的目标方向转换到小车坐标系
    // 小车坐标系：X向前，Y向左
    float relative_target_angle = global_target_angle - current_yaw;
    
    // 规范化角度到[-180, 180]
    while(relative_target_angle > 180.0f) relative_target_angle -= 360.0f;
    while(relative_target_angle < -180.0f) relative_target_angle += 360.0f;
    
    // 计算基础速度（基于距离的比例控制）
    float base_speed = Pos_KP * distance_to_target;
    
    // 限制最大速度并实现平滑接近（距离越近速度越慢）
    if(base_speed > max_linear_speed) {
        base_speed = max_linear_speed;
    }
    
    // 实现接近目标时的减速
    if(distance_to_target < 0.5f) { // 在0.5米内开始减速
        base_speed *= (distance_to_target / 0.5f);
    }
    
    // 分解速度到X和Y方向（小车坐标系）
    float speed_x = base_speed * cosf(relative_target_angle * PI / 180.0f);
    float speed_y = base_speed * sinf(relative_target_angle * PI / 180.0f);
    
    // 同时进行航向控制
    float yaw_control = Yaw_PID_Control(current_yaw, Target_Yaw);
    
    // 每50次调用打印一次控制输出
    if(adjust_count % 50 == 0) {
        char debug_msg[256];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[导航] 控制输出 X:%.3f Y:%.3f Z:%.3f 基础速度:%.3f\r\n", 
                 speed_x, speed_y, yaw_control, base_speed);
        usart1_send_cstring(debug_msg);
    }
    
    // 组合移动和旋转控制
    Drive_Motor(speed_x, speed_y, yaw_control);
}


