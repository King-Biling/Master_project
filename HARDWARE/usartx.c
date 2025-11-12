#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
u8 Usart2_Receive_buf[1];          //串口2接收中断数据存放的缓冲区
u8 Usart2_Receive;                 //从串口2读取的数据

UWB_Data uwb_data;
char rx_buffer[256];
uint16_t rx_index = 0;

char rx6_buffer[256];
uint16_t rx6_index = 0;

char rx4_buffer[256];
uint16_t rx4_index = 0;

uint8_t esp8266_rx_buffer[512];
uint16_t esp8266_rx_index = 0;
uint8_t esp8266_receive_byte;

uint8_t newCoordinateReceived = 0; // 标志位表示收到新坐标
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
if (huart->Instance == USART2) {
    int Usart_Receive;
    static u8 Flag_PID, i, j, Receive[50], Last_Usart_Receive;
    static float Data;
    
    // 新增：坐标解析相关变量
    static uint8_t coordBuffer[32];
    static uint8_t coordIndex = 0;
    static uint8_t coordParseState = 0; // 0:等待开始 1:接收X 2:接收Y 3:接收Yaw
    
    Usart_Receive = USART2->DR;

    // 先尝试解析坐标数据
    if (Usart_Receive == '[') {
        coordParseState = 1; // 开始接收X坐标
        coordIndex = 0;
        memset(coordBuffer, 0, sizeof(coordBuffer));
    } else if (coordParseState > 0) {
        if (Usart_Receive == ',' && coordParseState == 1) {
            // 接收到第一个逗号，解析X坐标
            coordBuffer[coordIndex] = '\0';
            Target_position[0] = atof((char*)coordBuffer);
            coordIndex = 0;
            coordParseState = 2; // 开始接收Y坐标
            memset(coordBuffer, 0, sizeof(coordBuffer));
        } else if (Usart_Receive == ',' && coordParseState == 2) {
            // 接收到第二个逗号，解析Y坐标
            coordBuffer[coordIndex] = '\0';
            Target_position[1] = atof((char*)coordBuffer);
            coordIndex = 0;
            coordParseState = 3; // 开始接收Yaw
            memset(coordBuffer, 0, sizeof(coordBuffer));
        } else if (Usart_Receive == ']' && coordParseState == 3) {
            // 接收到结束符，解析Yaw
            coordBuffer[coordIndex] = '\0';
            Target_Yaw = atof((char*)coordBuffer);
            coordParseState = 0;
            newCoordinateReceived = 1; // 设置新坐标标志
            Auto_mode = 1; // 进入自动模式
            // 可选：发送确认回执
            // usart2_send("Coordinate received");
        } else if (coordParseState > 0) {
            // 存储数字和小数点
            if (coordIndex < sizeof(coordBuffer) - 1 && 
               (Usart_Receive == '.' || (Usart_Receive >= '0' && Usart_Receive <= '9') || 
                Usart_Receive == '-' || Usart_Receive == '+')) {
                coordBuffer[coordIndex++] = Usart_Receive;
            }
        }
    } else {
        // 原有控制指令处理逻辑保持不变
        if (Usart_Receive == 0x41 && Last_Usart_Receive == 0x41 && APP_ON_Flag == 0) {
            APP_ON_Flag = 1;
        }
        
        // 关键修改：当接收到任何蓝牙控制指令时，退出自动模式
        // 检查是否是有效的控制指令（方向控制、速度控制等）
        if (Usart_Receive >= 0x41 && Usart_Receive <= 0x48 ||  // 方向指令 A-H
            Usart_Receive <= 8 ||                             // 方向指令 1-8
            Usart_Receive == 0x58 || Usart_Receive == 0x59 || // 速度控制 X,Y
            Usart_Receive == 0x4B ||                          // 转向控制界面 K
            Usart_Receive == 0x49 || Usart_Receive == 0x4A || // 方向控制界面 I,J
            Usart_Receive == 0x43 || Usart_Receive == 0x47)   // 转向控制 C,G
        {
            Auto_mode = 0; // 退出自动模式，切换到手动模式
            APP_ON_Flag = 1; // 确保手动模式使能
        }
        
        Last_Usart_Receive = Usart_Receive;
        
        if(Usart_Receive==0x4B) 
            Turn_Flag=1;
        else if(Usart_Receive==0x49||Usart_Receive==0x4A) 
            Turn_Flag=0;
        
        if(Turn_Flag==0) {
            if(Usart_Receive>=0x41&&Usart_Receive<=0x48) {	
                Flag_Direction=Usart_Receive-0x40;
            } else if(Usart_Receive<=8) {			
                Flag_Direction=Usart_Receive;
            } else Flag_Direction=0;
        } else if(Turn_Flag==1) {
            if(Usart_Receive==0x43) {
                Flag_Left=0,Flag_Right=1;
            } else if(Usart_Receive==0x47) {
                Flag_Left=1,Flag_Right=0;
            } else Flag_Left=0,Flag_Right=0;
            
            if(Usart_Receive==0x41||Usart_Receive==0x45) 
                Flag_Direction=Usart_Receive-0x40;
            else Flag_Direction=0;
        }
        
        if(Usart_Receive==0x58) RC_Velocity=RC_Velocity+50;
        if(Usart_Receive==0x59) RC_Velocity=RC_Velocity-50;
        
        if(Usart_Receive==0x7B) Flag_PID=1;
        if(Usart_Receive==0x7D) Flag_PID=2;

        if(Flag_PID==1) {
            Receive[i]=Usart_Receive;
            i++;
        }
        
        if(Flag_PID==2) {
            if(Receive[3]==0x50) PID_Send=1;
            else if(Receive[1]!=0x23) {								
                for(j=i;j>=4;j--) {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1]) {
                    case 0x30: RC_Velocity=Data; break;
                    case 0x31: Velocity_KP=Data; break;
                    case 0x32: Velocity_KI=Data; break;
                    case 0x33: break;
                    case 0x34: break;
                    case 0x35: break;
                    case 0x36: break;
                    case 0x37: break;
                    case 0x38: break;
                }
            }
            
            Flag_PID=0;
            i=0;
            j=0;
            Data=0;
            memset(Receive, 0, sizeof(u8)*50);
            if(RC_Velocity<0) RC_Velocity=0;
        }
    }

    HAL_UART_Receive_IT(&huart2, Usart2_Receive_buf, sizeof(Usart2_Receive_buf));
    }else if (huart->Instance == UART5) {  // 假设发送：$KT1,1.69,2.93,4.98,NULL,LO=[-2.45,5.44,1.43]
      // 检查是否收到完整的一行（以换行符结束）
        if (rx_buffer[rx_index] == '\n') {
            // 处理完整的数据行
            rx_buffer[rx_index + 1] = '\0'; // 添加字符串结束符
            Parse_UWB_Data(rx_buffer);
            rx_index = 0;
        } else {
            rx_index++;
            if (rx_index >= sizeof(rx_buffer)) {
                rx_index = 0; // 防止缓冲区溢出
            }
        }
        // 重新启动接收
        HAL_UART_Receive_IT(&huart5, (uint8_t*)&rx_buffer[rx_index], 1);    
	}else if(huart->Instance == USART6) {
        // 将接收到的字符存入缓冲区
        if(esp8266_rx_index < sizeof(esp8266_rx_buffer) - 1) {
            esp8266_rx_buffer[esp8266_rx_index++] = esp8266_receive_byte;
            
            // 如果检测到数据包过长，提前处理
            if(esp8266_rx_index > 300) {
                ESP8266_Process(); // 强制处理当前数据
            }
        } else {
            // 缓冲区满，重置
            esp8266_rx_index = 0;
            memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
            debug_print("[警告] 接收缓冲区溢出，已重置\r\n");
        }
        HAL_UART_Receive_IT(&huart6, &esp8266_receive_byte, 1);
    }

	
}


/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**
 * Send a string of arbitrary length via USART1
 * @param str Pointer to the string to be sent
 * @param length Number of bytes to send
 */
void usart1_send_cstring(const char *str)
{
    // Check for null pointer
    if (str == NULL) return;
    
    // Send characters until null terminator is encountered
    while (*str != '\0') {
        usart1_send((u8)*str);
        str++;
    }
}


/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart5_send(u8 data)
{
	USART3->DR = data;
	while((UART5->SR&0x40)==0);	
}
/**
 * Send a string of arbitrary length via USART1
 * @param str Pointer to the string to be sent
 * @param length Number of bytes to send
 */
void usart5_send_cstring(const char *str)
{
    // Check for null pointer
    if (str == NULL) return;
    
    // Send characters until null terminator is encountered
    while (*str != '\0') {
        usart5_send((u8)*str);
        str++;
    }
}

// 解析UWB数据函数

void Parse_UWB_Data(char* data) {
    // 初始化UWB数据结构
    memset(&uwb_data, 0, sizeof(uwb_data));
    for (int i = 0; i < 4; i++) {
        uwb_data.distances[i] = NAN;
    }
    uwb_data.valid_position = 0;
    
    // 检查数据起始符
    if (data[0] != '$') {
        return; // 无效数据
    }
    
    // // 调试：打印原始数据
    // char debug_msg[200];
    // snprintf(debug_msg, sizeof(debug_msg), "原始数据: %s\r\n", data);
    // usart1_send_cstring(debug_msg);
    
    // 提取角色和滤波模式 (如KT0中的K和T0)
    if (strlen(data) >= 3 && data[1] == 'K') {
        strncpy(uwb_data.filter_mode, "K", 1);
        // 提取角色 (T0, T1等)
        if (sscanf(data + 2, "%2s", uwb_data.role) == 1) {
            uwb_data.role[2] = '\0'; // 确保字符串结束
        }
    }
    
    // 提取距离信息
    char* distances_start = strchr(data, ',') + 1; // 找到第一个逗号后的位置
    if (distances_start) {
        int count = sscanf(distances_start, "%f,%f,%f,%f",
                         &uwb_data.distances[0],
                         &uwb_data.distances[1],
                         &uwb_data.distances[2],
                         &uwb_data.distances[3]);
        
        // 计算有效距离数量
        uwb_data.valid_distances = 0;
        for (int i = 0; i < 4; i++) {
            if (!isnan(uwb_data.distances[i])) {
                uwb_data.valid_distances++;
            }
        }
    }
    
    // 提取坐标信息
    char* lo_start = strstr(data, "LO=[");
    if (lo_start) {
        lo_start += 4; // 跳过"LO=["
        char* lo_end = strchr(lo_start, ']');
        if (lo_end) {
            // 临时存储坐标字符串
            char coord_str[50];
            int len = lo_end - lo_start;
            if (len < sizeof(coord_str) - 1) {
                strncpy(coord_str, lo_start, len);
                coord_str[len] = '\0';
                
                // 解析坐标值
                if (sscanf(coord_str, "%f,%f,%f",
                         &uwb_data.position[0],
                         &uwb_data.position[1],
                         &uwb_data.position[2]) == 3) { // 解析到了坐标值
                    uwb_data.valid_position = 1;
					// 存储位置坐标
					position[0] = uwb_data.position[0]; // 存储X坐标
					position[1] = uwb_data.position[1]; // 存储Y坐标
					position[2] = uwb_data.position[2]; // 存储Z坐标
                }
            }
        }
    }
    
    // // 输出解析结果
    // char buffer[128];
    // snprintf(buffer, sizeof(buffer), "Role: %s, Filter: %s\r\n", uwb_data.role, uwb_data.filter_mode);
    // usart1_send_cstring(buffer);
    
    // for (int i = 0; i < 4; i++) {
    //     if (!isnan(uwb_data.distances[i])) {
    //         snprintf(buffer, sizeof(buffer), "Distance to A%d: %.2f m\r\n", i, uwb_data.distances[i]);
    //         usart1_send_cstring(buffer);
    //     }
    // }
    
    // if (uwb_data.valid_position) {
    //     snprintf(buffer, sizeof(buffer), "Position: [%.2f, %.2f, %.2f]\r\n", 
    //             uwb_data.position[0], 
    //             uwb_data.position[1], 
    //             uwb_data.position[2]);
    //     usart1_send_cstring(buffer);
    // } else {
    //     usart1_send_cstring("未能解析到坐标信息\r\n");
    // }
    
    // usart1_send_cstring("--------------------\r\n");
}





