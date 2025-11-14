#ifndef __ESP8266_DRIVER_H
#define __ESP8266_DRIVER_H

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "usartx.h"
#include "FreeRTOS.h"
#include "task.h"

// FreeRTOS延时宏定义
#define RTOS_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))

// 根据你的系统tick频率定义合适的延时
#define DELAY_QUICK    pdMS_TO_TICKS(1)   // 1ms
#define DELAY_SHORT    pdMS_TO_TICKS(2)   // 2ms  
#define DELAY_MEDIUM   pdMS_TO_TICKS(5)   // 5ms
#define DELAY_LONG     pdMS_TO_TICKS(10)  // 10ms

// 对于需要精确时序的地方，仍然使用HAL_Delay（微秒级）
#define HAL_DELAY_US(us) HAL_Delay(1)  // HAL_Delay最小单位是1ms

// ESP8266状态定义
typedef enum {
    ESP8266_OK = 0,
    ESP8266_ERROR,
    ESP8266_TIMEOUT,
    ESP8266_ALREADY_CONNECTED
} ESP8266_Status_t;

// 路由器wifi配置
#define WIFI_SSID "Xiaomi_1010"
#define WIFI_PASSWORD "12345678"
// #define SERVER_IP "192.168.31.234" //王浩
#define SERVER_IP "192.168.31.46" //白瑞
#define SERVER_PORT 8080
#define BROADCAST_PORT 8081       // 新增广播端口

extern char CAR_ID[10];  // 存储自动分配的小车ID

// 其他小车信息最大数量.
#define MAX_OTHER_CARS 10

// 通信拓扑配置
#define MAX_CARS 4
extern uint8_t communication_topology[MAX_CARS][MAX_CARS];
extern uint8_t topology_enabled;
extern uint8_t car_index;  // 本车在拓扑中的索引

// 其他小车信息结构体
typedef struct {
    char car_id[10];
    float position_x;
    float position_y;
    float velocity_vx;
    float velocity_vy;
    float velocity_vz;
    float yaw;          // 新增：航向角字段
    uint32_t last_update;
    uint8_t valid;
} OtherCarInfo;

extern OtherCarInfo other_cars[MAX_OTHER_CARS];  // 其他小车信息数组
extern uint8_t other_cars_count;  // 其他小车数量

// UDP相关变量
extern uint8_t esp8266_udp_initialized;
extern uint8_t esp8266_broadcast_initialized;

// 函数声明
void debug_print(const char* message);
ESP8266_Status_t ESP8266_Init(void);

// UDP函数
ESP8266_Status_t ESP8266_InitUDP(void);
ESP8266_Status_t ESP8266_InitBroadcast(void);
ESP8266_Status_t ESP8266_ResetModule(void);
ESP8266_Status_t ESP8266_SendStatus_UDP_Reliable(float x, float y, float yaw, float voltage, float vx, float vy, float vz);
ESP8266_Status_t ESP8266_SendATCommand_Enhanced(const char* cmd, const char* expected_response, uint32_t timeout);

// MAC地址和ID分配
ESP8266_Status_t ESP8266_GetMACAddress(char* mac_buffer, uint32_t buffer_size);
void ESP8266_AutoAssignCarID(void);

// 广播数据处理函数
void Process_Compact_Broadcast(const char* data);
void Process_Broadcast_Data(const char* data);
void Update_Other_Car_Info(const char* car_id, float x, float y, float vx, float vy, float vz, float yaw);
void Cleanup_Old_Car_Info(void);
void Init_Other_Cars_Info(void);

// 控制命令处理
void Process_Control_Command(const char* command);
void Process_Formation_Command(const char* command);

// 拓扑处理函数
void Process_Topology_Command(const char* command);
void Update_Topology_Matrix(const char* topology_str);
uint8_t Should_Process_Car_Info(const char* car_id);
void Init_Communication_Topology(void);

// 单播数据处理
void Process_Unicast_Data(const char* data);

// 数据处理
void ESP8266_Process(void);
void Process_Multiple_IPD_Packets(char* data_start, uint32_t data_length);

// 目前还未继续完善，后续会根据需要继续添加功能

#endif




