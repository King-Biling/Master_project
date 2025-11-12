/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-07-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-07-05

All rights reserved
***********************************************/

#include "lidar.h"
#include <string.h>
#include "sys.h"

float Diff_Along_Distance_KP = -0.030f,Diff_Along_Distance_KD = -0.245f,Diff_Along_Distance_KI = -0.001f;	//差速小车走直线距离调整PID参数
float Akm_Along_Distance_KP = -0.115f*1000,Akm_Along_Distance_KD = -1000.245f*1000,Akm_Along_Distance_KI = -0.001f*1000;	//阿克曼走直线距离调整PID参数
float FourWheel_Along_Distance_KP = -0.115f*1000,FourWheel_Along_Distance_KD = -100.200f*1000,FourWheel_Along_Distance_KI = -0.001f*1000;	//四驱走直线距离调整PID参数
float Along_Distance_KP = 0.163f*1000,Along_Distance_KD = 0.123*1000,Along_Distance_KI = 0.001f*1000;	//麦轮和全向走直线距离调整PID参数

float Distance_KP = -0.150f,Distance_KD = -0.052f,Distance_KI = -0.001;	//跟随距离调整PID参数

float Follow_KP = -2.566f,Follow_KD = -1.368f,Follow_KI = -0.001f;  //转向PID
float Follow_KP_Akm = -0.550f,Follow_KD_Akm = -0.121f,Follow_KI_Akm = -0.001f;
//float Distance_KP = 0.653f*1000,Distance_KD = 2.431f*1000,Distance_KI = 0.001*1000;	//距离调整PID参数

PointDataProcessDef PointDataProcess[1200] ;//更新225个数据
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[1200];      //用于小车避障、跟随、走直线、ELE雷达避障的雷达数据



/**************************************************************************
Function: data_process
Input   : none
Output  : none
函数功能：数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
int data_cnt = 0;
void data_process(void) //数据处理
{
	static int data_cnt = 0;
	int i,m,n;
	u32 distance_sum[32]={0};//2个点的距离和的数组
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//计算32个点的开始角度
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//计算32个点的结束角度
	float area_angle[32]={0};
	
	if(start_angle>end_angle)//结束角度和开始角度被0度分割的情况
		end_angle +=360;

	for(m=0;m<32;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/32*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}
	
	for(i=0;i<32;i++)
	{
		distance_sum[i] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//数据高低8位合并
	}
	

	for(n=0;n<32;n++)
	{
		PointDataProcess[data_cnt+n].angle = area_angle[n];
	    PointDataProcess[data_cnt+n].distance = distance_sum[n];  //一帧数据为32个点
		
	}
	data_cnt +=32;
	if(data_cnt>=1152) //雷达转一圈大概有1152个点（大概数值，每圈的点数都不固定，一帧大约为10度，一圈大约是36帧数据=32*36=1152）
	{
		for(i=0,m=0;i<1152;i++)
		{				
				if(PointDataProcess[i].distance == 0 &&PointDataProcess[i-1].distance == 0)  //两次回波均是0，即采集距离为0
				{
					Dataprocess[m].angle=PointDataProcess[i].angle;
					Dataprocess[m++].distance=PointDataProcess[i].distance;  //将数组PointDataProcess中两次回波均为0的数据传输到数组Dataprocess中
				}
				else if(PointDataProcess[i].distance != 0)  //回波信息一致，只传输一次数据，即非0的数据
				{
					Dataprocess[m].angle=PointDataProcess[i].angle;
					Dataprocess[m++].distance=PointDataProcess[i].distance;  //将数组PointDataProcess中非0的数据传输到数组Dataprocess中
				}
		}
		data_cnt = 0;
	}
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：走直线雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//走直线雷达距离调整pid

float Along_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	Integral_bias+=Bias;	                                 			//求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == FourWheel_Car)
		OutPut=FourWheel_Along_Distance_KP*Bias/100000+FourWheel_Along_Distance_KI*Integral_bias/100000+FourWheel_Along_Distance_KD*(Bias-Last_Bias)/100000;//位置式PID控制器
	else if(Car_Mode == Akm_Car)
		OutPut=Akm_Along_Distance_KP*Bias/100000+Akm_Along_Distance_KI*Integral_bias/100000+Akm_Along_Distance_KD*(Bias-Last_Bias)/1000;//位置式PID控制器
	else if(Car_Mode == Diff_Car || Car_Mode == Tank_Car)
		OutPut=Diff_Along_Distance_KP*Bias/100+Diff_Along_Distance_KI*Integral_bias/100+Diff_Along_Distance_KD*(Bias-Last_Bias);//位置式PID控制器
	else
	  OutPut=-Along_Distance_KP*Bias/100000-Along_Distance_KI*Integral_bias/100000-Along_Distance_KD*(Bias-Last_Bias)/1000;//位置式PID控制器
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                          	
}


/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：跟随雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//跟随雷达距离调整pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	Integral_bias+=Bias;	                                 			//求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=Distance_KP*Bias/100+Distance_KI*Integral_bias/100+Distance_KD*(Bias-Last_Bias)/100;//位置式PID控制器
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
函数功能：跟随雷达转向pid
入口参数: 当前角度和目标角度
返回  值：电机转向速度
**************************************************************************/	 	
//跟随雷达转向pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //计算偏差
	Integral_bias+=Bias;	                                 				 //求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == Akm_Car || Car_Mode == Omni_Car)
		OutPut=(Follow_KP_Akm/100)*Bias+(Follow_KI_Akm/100)*Integral_bias+(Follow_KD_Akm/100)*(Bias-Last_Bias);	//位置式PID控制器
	else
	  OutPut=(Follow_KP/100)*Bias+(Follow_KI/100)*Integral_bias+(Follow_KD/100)*(Bias-Last_Bias);	//位置式PID控制器
	Last_Bias=Bias;                                       					 		//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                           					 	//输出
	
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

//int Incremental_PI_Left (int Encoder,int Target)
//{ 	
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder;                					//计算偏差
//	 Pwm+=11*(Bias-Last_bias)+10*Bias;   	//增量式PI控制器
//	 Last_bias=Bias;	                   					//保存上一次偏差 
//	 return Pwm;                         					//增量输出
//}


//int Incremental_PI_Right (int Encoder,int Target)
//{ 	
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder;                					//计算偏差
//	 Pwm+=11*(Bias-Last_bias)+10*Bias;   	//增量式PI控制器
//	 Last_bias=Bias;	                   					//保存上一次偏差 
//	 return Pwm;                         					//增量输出
//}
