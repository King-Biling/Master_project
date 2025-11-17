#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/

/**************************************************************************
Function: Buttons to scan
Input   : none
Output  : none
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
	//OLED_Clear();// 因为目前单击案按键用于切换显示界面 所以直接把清屏函数放到这里
	//Press the release sign
	//按键按松开标志
	static u8 flag_key=1;
	
	if(flag_key&&KEY==0)
	{
	 flag_key=0; //The key is pressed //按键按下
	 return 1;	
	}
	else if(1==KEY)			
		flag_key=1;
	return 0; //No key is pressed //无按键按下
}


uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*检测是否有按键按下 */
	if(KEY == KEY_ON )  
	{	 
		/*等待按键释放 */
		HAL_Delay(200);
		while(KEY == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}

/**************************************************************************
Function: Delay function
Input   : none
Output  : none
函数功能：延迟函数
入口参数：无
返 回 值：无
**************************************************************************/
void Delay_ms(void)
{
   int ii,i;    
   for(ii=0;ii<50;ii++)
   {
	   for(i=0;i<50;i++);
	 }	
}
/**************************************************************************
Function: Buttons to scan
Input   : Double click wait time
Output  : Button status: 0- no action, 1- click, 2- double click
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态: 0-无动作, 1-单击, 2-双击 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;   
    else        Forever_count=0;
	
		if(KEY == KEY_ON&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click //双击
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click //单击
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Button scan.Because static variables are used, a function with a different name needs to be defined when the keystroke scan function is used multiple times
Input   : none
Output  : Button status: 0- no action, 1- click, 2- double click
函数功能：按键扫描。因为使用到了静态变量，当多处需要使用按键扫描函数时，需要再定义一个不同名函数
入口参数：无
返 回 值：按键状态: 0-无动作, 1-单击, 2-双击 
**************************************************************************/
u8 click_N_Double_MPU6050 (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;  
    else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click //双击
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click //单击
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Long according to the test
Input   : none
Output  : Key state 0: no action 1: long press 3s
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按3s
**************************************************************************/
//u8 Long_Press(void)
//{
//	static u16 Long_Press_count,Long_Press;

//	if(Long_Press==0&&KEY==0)  Long_Press_count++; 
//	else                       Long_Press_count=0;

//	if(Long_Press_count>300)	//3 seconds //3秒	
//	{
//		Long_Press=1;	
//		Long_Press_count=0;
//		return 1;
//	}				
//	 if(Long_Press==1) //Long press position 1 //长按标志位置1
//	{
//			Long_Press=0;
//	}
//	return 0;
//}


/*************************************************************************
Function:User_Key_Scan
Input:None
Output:Key_status
函数功能：用户按键检测
入口参数：无
返回值  ：按键状态
**************************************************************************/
//放在5ms中断中调用
uint8_t User_Key_Scan(void)
{
	static u16 count_time = 0;					//计算按下的时间，每5ms加1
	static u8 key_step = 0;						//记录此时的步骤
	switch(key_step)
	{
		case 0:
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) == KEY_ON )
				key_step++;						//检测到有按键按下，进入下一步
			break;
		case 1:
			if((++count_time) == 5)				//延时消抖
			{
				if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) == KEY_ON )//按键确实按下了
					key_step++,count_time = 0;	//进入下一步
				else
					count_time = 0,key_step = 0;//否则复位
			}
			break;
		case 2:
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) == KEY_ON )
				count_time++;					//计算按下的时间
			else 								//此时已松开手
				key_step++;						//进入下一步
			break;
		case 3:									//此时看按下的时间，来判断是长按还是短按
			if(count_time > 200)				//在5ms中断中调用，故按下时间若大于400*5 = 2000ms（大概值）
			{							
				key_step = 0;					//标志位复位
				count_time = 0;
				return Long_Press;				//返回 长按 的状态 
 			}
			else if(count_time > 5)				//此时是单击了一次
			{
				key_step++;						//此时进入下一步，判断是否是双击
				count_time = 0;					//按下的时间清零
			}
			else
			{
				key_step = 0;
				count_time = 0;	
			}
			break;
		case 4:									//判断是否是双击或单击
			if(++count_time > 30)				//5*50= 250ms内判断按键是否按下
			{
				if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) == KEY_ON )	//按键确实按下了
				{																	//这里双击不能按太快，会识别成单击
					key_step++;														//进入下一步，需要等松手才能释放状态
					count_time = 0;
				}
				else																//190ms内无按键按下，此时是单击的状态
				{
					key_step = 0;				//标志位复位
					count_time = 0;					
					return Click;				//返回单击的状态
				}
			}
			break;
		case 5:
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) == KEY_ON )//按键还在按着
			{
				count_time++;
			}
			else								//按键已经松手
			{
//				if(count_time>400)				//这里第二次的单击也可以判断时间的，默认不判断时间，全部都返回双击
//				{
//				}
				count_time = 0;
				key_step = 0;
				return Double_Click;
			}
			break;
		default:break;
	}
	return No_Action;							//无动作
}
/**************************************************************************
函数功能：选择运行的模式
入口参数：无
返回  值：无
**************************************************************************/
//u8  select(void)
//{
//	  int Flag_Next=0;
//	  static u8 flag=1;
//	
//	  oled_show_once();  //OLED显示
//	  if(click_N_Double(150) == 1)
//		{
//			Move+=1;
//			if(Move>6)
//				Move=0;
//		}
//	  if(Move==0)			 ELE_ON_Flag=0,CCD_ON_Flag=1,APP_ON_Flag=0,Along_wall=0,Follow_ON_Flag=0,Avoid_ON_Flag=0,PS2_ON_Flag=0;  //CCD遥控模式 
//		else if(Move==1) ELE_ON_Flag=1,CCD_ON_Flag=0,APP_ON_Flag=0,Along_wall=0,Follow_ON_Flag=0,Avoid_ON_Flag=0,PS2_ON_Flag=0;  //ELE遥控模式
//		else if(Move==2) ELE_ON_Flag=0,CCD_ON_Flag=0,APP_ON_Flag=1,Along_wall=0,Follow_ON_Flag=0,Avoid_ON_Flag=0,PS2_ON_Flag=0;	//oth模式
//	  else if(Move==3) ELE_ON_Flag=0,CCD_ON_Flag=0,APP_ON_Flag=0,Along_wall=1,Follow_ON_Flag=0,Avoid_ON_Flag=0,PS2_ON_Flag=0;  //走直线模式
//	  else if(Move==4) ELE_ON_Flag=0,CCD_ON_Flag=0,APP_ON_Flag=0,Along_wall=0,Follow_ON_Flag=1,Avoid_ON_Flag=0,PS2_ON_Flag=0;  //跟随模式
//	  else if(Move==5) ELE_ON_Flag=0,CCD_ON_Flag=0,APP_ON_Flag=0,Along_wall=0,Follow_ON_Flag=0,Avoid_ON_Flag=1,PS2_ON_Flag=0;  //避障模式
//		else if(Move==6) ELE_ON_Flag=0,CCD_ON_Flag=0,APP_ON_Flag=0,Along_wall=0,Follow_ON_Flag=0,Avoid_ON_Flag=0,PS2_ON_Flag=1;  //ps2模式
//    if(Long_Press())Flag_Next=1;//长按按键3秒进入运行模式
//	  if(Flag_Next==1)OLED_Clear(),flag=0;  //清除OLED屏幕 程序往下运行
//	  return flag;	  
//}
