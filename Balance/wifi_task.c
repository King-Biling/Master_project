#include "wifi_task.h"

// WiFi任务状态
typedef enum {
    WIFI_STATE_INIT = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_UDP_RECONNECTING,
    WIFI_STATE_READY,
    WIFI_STATE_ERROR,
    WIFI_STATE_HARD_RESET  // 新增：硬件复位状态
} WiFiState_t;

static WiFiState_t wifi_state = WIFI_STATE_INIT;
static uint32_t last_status_send_time = 0;
static uint32_t wifi_reconnect_time = 0;

// 连接健康状态机相关变量
static ConnectionHealth_t connection_health = CONNECTION_HEALTHY;
static uint8_t consecutive_failures = 0;
static uint32_t last_data_receive_time = 0;
static uint32_t last_health_check_time = 0;
static uint32_t connection_start_time = 0;  // 新增：连接开始时间
static uint8_t hard_reset_count = 0;        // 新增：硬件复位计数

void WiFi_Task(void *pvParameters)
{
    usart1_send_cstring("WiFi任务启动...\r\n");
    u32 lastWakeTime = getSysTickCnt();
    
    // 任务主循环
    while(1) {
        // 此任务以50Hz的频率运行（20ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ)); 
        
        switch(wifi_state) {
            case WIFI_STATE_INIT:
                WiFi_State_Init();
                break;
                
            case WIFI_STATE_CONNECTING:
                WiFi_State_Connecting();
                break;
                
            case WIFI_STATE_UDP_RECONNECTING:
                WiFi_State_UDP_Reconnecting();
                break;
                
            case WIFI_STATE_READY:
                WiFi_State_Ready();
                break;
                
            case WIFI_STATE_ERROR:
                WiFi_State_Error();
                break;
                
            case WIFI_STATE_HARD_RESET:  // 新增：硬件复位状态
                WiFi_State_Hard_Reset();
                break;
        }
    }
}

// 新增：硬件复位状态处理。
void WiFi_State_Hard_Reset(void)
{
    static uint32_t last_reset_attempt = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每10秒尝试一次硬件复位
    if(current_time - last_reset_attempt > 10000) {
        usart1_send_cstring("[硬复位] 尝试硬件复位ESP8266...\r\n");
        
        // 发送AT+RST命令进行硬件复位
        if(ESP8266_SendATCommand_Enhanced("AT+RST", "ready", 10000) == ESP8266_OK) {
            usart1_send_cstring("[硬复位] ESP8266复位成功\r\n");
            HAL_Delay(3000);  // 等待模块启动
            hard_reset_count++;
            wifi_state = WIFI_STATE_INIT;
        } else {
            usart1_send_cstring("[硬复位] ESP8266复位失败\r\n");
        }
        
        last_reset_attempt = current_time;
        
        // 如果连续3次硬件复位都失败，进入错误状态
        if(hard_reset_count >= 3) {
            usart1_send_cstring("[硬复位] 连续复位失败，进入错误状态\r\n");
            wifi_state = WIFI_STATE_ERROR;
            hard_reset_count = 0;
        }
    }
}

// 初始化状态处理 - 增强版本
void WiFi_State_Init(void)
{
    static uint32_t init_start_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 记录初始化开始时间
    if(init_start_time == 0) {
        init_start_time = current_time;
        usart1_send_cstring("初始化WiFi模块...\r\n");
    }
    
    // 检查初始化超时（30秒）
    if(current_time - init_start_time > 30000) {
        usart1_send_cstring("[初始化] 超时，进入硬件复位状态\r\n");
        init_start_time = 0;
        wifi_state = WIFI_STATE_HARD_RESET;
        return;
    }
    
    // 重置关键状态变量
    consecutive_failures = 0;
    connection_health = CONNECTION_HEALTHY;
    
    if(ESP8266_Init() == ESP8266_OK) {
        char init_msg[64];
        snprintf(init_msg, sizeof(init_msg), "WiFi模块初始化成功，小车ID: %s\r\n", CAR_ID);
        usart1_send_cstring(init_msg);
        
        connection_start_time = HAL_GetTick();  // 记录连接开始时间
        wifi_state = WIFI_STATE_READY;
        init_start_time = 0;  // 重置初始化计时
        hard_reset_count = 0; // 重置硬件复位计数
    } else {
        // 初始化失败，短暂延迟后重试
        vTaskDelay(F2T(RATE_10_HZ)); // 100ms延迟
    }
}

// 连接状态处理 - 添加缺失的函数定义
void WiFi_State_Connecting(void)
{
    static uint32_t connecting_start_time = 0;
    static uint8_t connecting_attempts = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 记录连接开始时间
    if(connecting_start_time == 0) {
        connecting_start_time = current_time;
        usart1_send_cstring("WiFi连接中...\r\n");
    }
    
    // 检查连接超时（15秒）
    if(current_time - connecting_start_time > 15000) {
        usart1_send_cstring("[连接] 连接超时，回到初始化状态\r\n");
        connecting_start_time = 0;
        connecting_attempts = 0;
        wifi_state = WIFI_STATE_INIT;
        return;
    }
    
    // 每3秒尝试一次连接
    if(current_time - connecting_start_time > connecting_attempts * 3000) {
        connecting_attempts++;
        usart1_send_cstring("[连接] 尝试连接WiFi网络...\r\n");
        
        // 这里可以添加连接尝试的逻辑
        // 暂时直接回到初始化状态重新尝试
        wifi_state = WIFI_STATE_INIT;
    }
}

// UDP重连状态处理 - 增强版本
void WiFi_State_UDP_Reconnecting(void)
{
    static uint32_t last_udp_reconnect = 0;
    static uint8_t reconnect_attempts = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每3秒尝试重新初始化UDP
    if(current_time - last_udp_reconnect > 3000) {
        reconnect_attempts++;
        debug_print("[UDP] 尝试重新初始化UDP连接...\r\n");
        
        if(ESP8266_InitUDP() == ESP8266_OK) {
            debug_print("[UDP] UDP重连成功\r\n");
            wifi_state = WIFI_STATE_READY;
            connection_health = CONNECTION_HEALTHY;
            consecutive_failures = 0;
            reconnect_attempts = 0;
            connection_start_time = HAL_GetTick();  // 重置连接时间
        } else {
            debug_print("[UDP] UDP重连失败\r\n");
            
            // 如果连续5次UDP重连失败，尝试完全重新初始化
            if(reconnect_attempts >= 5) {
                debug_print("[UDP] 连续重连失败，回到初始化状态\r\n");
                wifi_state = WIFI_STATE_INIT;
                reconnect_attempts = 0;
            }
        }
        
        last_udp_reconnect = current_time;
    }
}

// 在WiFi_State_Ready中增加连接健康检查 - 增强版本
void WiFi_State_Ready(void)
{
    uint32_t current_time = HAL_GetTick();
    
    static uint32_t last_health_check = 0;
    static uint8_t udp_health_counter = 0;
    
    // 检查连接持续时间，如果超过1小时，主动重新初始化（预防内存泄漏）
    if(current_time - connection_start_time > 3600000) { // 1小时
        debug_print("[健康检查] 连接时间过长，主动重新初始化\r\n");
        wifi_state = WIFI_STATE_INIT;
        return;
    }
    
    // 每10秒检查一次UDP连接健康状态
    if(current_time - last_health_check > 10000) {
        // 如果连续多次发送失败，尝试重新初始化UDP
        if(udp_health_counter > 3) {
            debug_print("[健康检查] UDP连接异常，尝试重新初始化...\r\n");
            esp8266_udp_initialized = 0;
            if(ESP8266_InitUDP() == ESP8266_OK) {
                debug_print("[健康检查] UDP重新初始化成功\r\n");
                udp_health_counter = 0;
            } else {
                debug_print("[健康检查] UDP重新初始化失败\r\n");
                connection_health = CONNECTION_DISCONNECTED;
            }
        }
        last_health_check = current_time;
    }
    
    // 健康状态机处理
    switch(connection_health) {
        case CONNECTION_HEALTHY:
            // 健康状态：高频发送（50ms）
            if(current_time - last_status_send_time > 30) {
                ESP8266_Status_t result = ESP8266_SendStatus_UDP_Reliable(
                    position[0], position[1], Yaw, Voltage, 
                    Current_Vx, Current_Vy, Current_Vz
                );
                
                if(result == ESP8266_OK) {
                    // 发送成功，重置失败计数
                    consecutive_failures = 0;
                    udp_health_counter = 0;
                } else {
                    // 发送失败，增加失败计数
                    consecutive_failures++;
                    udp_health_counter++;
                    
                    if(consecutive_failures >= 10) {  // 进一步降低阈值到10次
                        // UDP通信失败，转为断开状态
                        connection_health = CONNECTION_DISCONNECTED;
                        // usart1_send_cstring("[状态机] 转为断开状态\r\n");
                    }
                }
                last_status_send_time = current_time;
            }
            break;
            
        case CONNECTION_DISCONNECTED:
            // 断开状态：开始重连流程
            // usart1_send_cstring("[状态机] 开始UDP重连流程\r\n");
            esp8266_udp_initialized = 0;  // 标记需要重新初始化
            wifi_state = WIFI_STATE_UDP_RECONNECTING;
            connection_health = CONNECTION_HEALTHY;  // 重置为健康状态
            consecutive_failures = 0;                // 重置失败计数
            udp_health_counter = 0;                  // 重置健康计数器
            break;
    }
    
    // 处理接收到的数据（包括广播信息）
    ESP8266_Process();
    
    // // 定期打印连接状态
    // static uint32_t last_status_print = 0;
    // if(current_time - last_status_print > 5000) { // 每5秒打印一次
    //     char status_msg[128];
    //     uint32_t connection_duration = (current_time - connection_start_time) / 1000;
    //     snprintf(status_msg, sizeof(status_msg), 
    //              "[状态] 连接:%ds, UDP:%s, 失败:%d, 健康:%d\r\n",
    //              connection_duration,
    //              esp8266_udp_initialized ? "已初始化" : "未初始化",
    //              consecutive_failures, udp_health_counter);
    //     debug_print(status_msg);
    //     last_status_print = current_time;
    // }
}

// 错误状态处理 - 增强版本
void WiFi_State_Error(void)
{
    static uint32_t error_start_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(error_start_time == 0) {
        error_start_time = current_time;
    }
    
    // 定时重连（5秒后）
    if(current_time > wifi_reconnect_time) {
        usart1_send_cstring("尝试重新连接WiFi...\r\n");
        esp8266_udp_initialized = 0; // 重置UDP状态
        
        // 如果错误状态持续超过60秒，尝试硬件复位
        if(current_time - error_start_time > 60000) {
            usart1_send_cstring("[错误] 长时间无法恢复，尝试硬件复位\r\n");
            wifi_state = WIFI_STATE_HARD_RESET;
            error_start_time = 0;
        } else {
            wifi_state = WIFI_STATE_INIT;
        }
        
        wifi_reconnect_time = current_time + 5000; // 5秒后重试
    }
}

