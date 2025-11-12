#include "esp8266_driver.h"
#include "formation_control.h"  // 添加编队控制头文件

// 全局变量定义
uint8_t esp8266_udp_initialized = 0;  // UDP初始化状态
uint8_t esp8266_broadcast_initialized = 0;  // 广播初始化状态
char CAR_ID[10] = "CAR0";  // 默认ID，会在初始化时更新

// 通信拓扑全局变量
uint8_t communication_topology[MAX_CARS][MAX_CARS] = {
    {0, 1, 1, 1},  // CAR1 可以发送给CAR2,CAR3,CAR4
    {1, 0, 1, 1},  // CAR2 可以发送给CAR1,CAR3,CAR4  
    {1, 1, 0, 1},  // CAR3 可以发送给CAR1,CAR2,CAR4
    {1, 1, 1, 0}   // CAR4 可以发送给CAR1,CAR2,CAR3
};
uint8_t topology_enabled = 0;
uint8_t car_index = 0;  // 根据CAR_ID确定

// 在文件开头添加全局变量
static uint8_t uart_timeout_count = 0;
static uint32_t last_successful_communication = 0;

// 其他小车信息
OtherCarInfo other_cars[MAX_OTHER_CARS];
uint8_t other_cars_count = 0;

// 已知的四个小车的MAC地址
const char* known_mac_addresses[] = {
    "78:1c:3c:8a:a5:00",  // 小车1的MAC
    "78:1c:3c:8a:9e:49",  // 小车2的MAC  
    "78:1c:3c:8a:9e:74",  // 小车3的MAC
    "78:1c:3c:8a:9e:7f"   // 小车4的MAC
};

// 调试信息发送函数
void debug_print(const char* message)
{
    usart1_send_cstring(message);
}

void Init_Communication_Topology(void)
{
    // 根据CAR_ID确定本车索引
    char debug_msg[128];
    snprintf(debug_msg, sizeof(debug_msg), 
             "[拓扑] 初始化通信拓扑，当前CAR_ID: %s\r\n", CAR_ID);
    debug_print(debug_msg);
    
    // 修剪CAR_ID（移除可能的换行符和空格）
    char trimmed_car_id[16];
    strncpy(trimmed_car_id, CAR_ID, sizeof(trimmed_car_id)-1);
    trimmed_car_id[sizeof(trimmed_car_id)-1] = '\0';
    
    // 移除换行符和回车符
    char* newline = strchr(trimmed_car_id, '\r');
    if (newline) *newline = '\0';
    newline = strchr(trimmed_car_id, '\n');
    if (newline) *newline = '\0';
    
    // 设置car_index - 修复索引映射
    if(strcmp(trimmed_car_id, "CAR1") == 0) {
        car_index = 0;
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 本车是CAR1，索引设置为0\r\n");
        debug_print(debug_msg);
    }
    else if(strcmp(trimmed_car_id, "CAR2") == 0) {
        car_index = 1;  // 修复：CAR2应该是索引1，不是0
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 本车是CAR2，索引设置为1\r\n");
        debug_print(debug_msg);
    }
    else if(strcmp(trimmed_car_id, "CAR3") == 0) {
        car_index = 2;
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 本车是CAR3，索引设置为2\r\n");
        debug_print(debug_msg);
    }
    else if(strcmp(trimmed_car_id, "CAR4") == 0) {
        car_index = 3;
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 本车是CAR4，索引设置为3\r\n");
        debug_print(debug_msg);
    }
    else {
        car_index = 0;  // 默认
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 未知CAR_ID: %s，使用默认索引0\r\n", trimmed_car_id);
        debug_print(debug_msg);
    }
    
    // 初始化为全连接拓扑，但对角线为0（自己不能给自己发送）
    for(int i = 0; i < MAX_CARS; i++) {
        for(int j = 0; j < MAX_CARS; j++) {
            communication_topology[i][j] = (i == j) ? 0 : 1;
        }
    }
    topology_enabled = 0;
    
    debug_print("[拓扑] 通信拓扑初始化完成\r\n");
}

uint8_t Should_Process_Car_Info(const char* car_id)
{
    // 如果拓扑功能未启用，处理所有小车信息
    if(!topology_enabled) {
        char debug_msg[128];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 拓扑功能未启用，允许接收 %s 的信息\r\n", car_id);
        debug_print(debug_msg);
        return 1;
    }
    
    // 确定目标小车的索引
    uint8_t target_index = 0;
    if(strcmp(car_id, "CAR1") == 0) target_index = 0;
    else if(strcmp(car_id, "CAR2") == 0) target_index = 1;
    else if(strcmp(car_id, "CAR3") == 0) target_index = 2;
    else if(strcmp(car_id, "CAR4") == 0) target_index = 3;
    else {
        // 未知小车，默认允许接收
        char debug_msg[128];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[拓扑] 未知小车ID: %s，默认允许接收\r\n", car_id);
        debug_print(debug_msg);
        return 1;
    }
    
    // 根据规则：A[i][j] = 1 表示小车i可以给小车j发送信息（j可以看见i）
    // 所以我们要检查：目标车能否发送给本车？即 A[target_index][car_index] == 1?
    uint8_t result = communication_topology[target_index][car_index];
    
    // // 详细的调试信息
    // char debug_msg[256];
    // snprintf(debug_msg, sizeof(debug_msg), 
    //          "[拓扑] 检查通信权限:\r\n"
    //          "[拓扑]   本车: %s (索引%d)\r\n"
    //          "[拓扑]   目标车: %s (索引%d)\r\n"
    //          "[拓扑]   矩阵检查: communication_topology[%d][%d] = %d\r\n"
    //          "[拓扑]   结果: %s\r\n",
    //          CAR_ID, car_index,
    //          car_id, target_index,
    //          target_index, car_index, communication_topology[target_index][car_index],
    //          result ? "允许接收" : "禁止接收");
    // debug_print(debug_msg);
    
    return result;
}


void Process_Topology_Command(const char* command)
{
    // 格式: "TOPOLOGY_TOGGLE:1" 或 "TOPOLOGY_TOGGLE:0" 或 "TOPOLOGY_TOGGLE:True/False"
    const char* toggle_ptr = strstr(command, "TOPOLOGY_TOGGLE:");
    if(toggle_ptr != NULL) {
        toggle_ptr += 16;  // 跳过"TOPOLOGY_TOGGLE:"
        
        // 处理布尔值 True/False 或 1/0
        if (strstr(toggle_ptr, "True") != NULL || (toggle_ptr[0] == '1' && (toggle_ptr[1] == '\0' || toggle_ptr[1] == '\r' || toggle_ptr[1] == '\n'))) {
            topology_enabled = 1;
            debug_print("[拓扑] 拓扑功能已启用\r\n");
        } else if (strstr(toggle_ptr, "False") != NULL || (toggle_ptr[0] == '0' && (toggle_ptr[1] == '\0' || toggle_ptr[1] == '\r' || toggle_ptr[1] == '\n'))) {
            topology_enabled = 0;
            debug_print("[拓扑] 拓扑功能已禁用\r\n");
        } 
        return;
    }
    
    // 格式: "TOPOLOGY:1,1,1,1;1,0,1,0;1,1,0,1;1,0,1,0"
    const char* topology_ptr = strstr(command, "TOPOLOGY:");
    if(topology_ptr != NULL) {
        topology_ptr += 9;  // 跳过"TOPOLOGY:"
        
        // 自动启用拓扑功能
        topology_enabled = 1;
        
        // 复制拓扑字符串进行处理
        char topology_str[128];
        strncpy(topology_str, topology_ptr, sizeof(topology_str)-1);
        topology_str[sizeof(topology_str)-1] = '\0';
        
        // 移除可能的换行符和回车符
        char* newline = strchr(topology_str, '\r');
        if (newline) *newline = '\0';
        newline = strchr(topology_str, '\n');
        if (newline) *newline = '\0';
        
        Update_Topology_Matrix(topology_str);
        return;
    }
    
    // 格式: "TOPOLOGY:START" 或 "TOPOLOGY:ENABLE" - 启用拓扑
    if (strstr(command, "TOPOLOGY:START") != NULL || strstr(command, "TOPOLOGY:ENABLE") != NULL) {
        topology_enabled = 1;
        debug_print("[拓扑] 拓扑功能已启用\r\n");
        return;
    }
    
    // 格式: "TOPOLOGY:STOP" 或 "TOPOLOGY:DISABLE" - 禁用拓扑
    if (strstr(command, "TOPOLOGY:STOP") != NULL || strstr(command, "TOPOLOGY:DISABLE") != NULL) {
        topology_enabled = 0;
        debug_print("[拓扑] 拓扑功能已禁用\r\n");
        return;
    }
    
    debug_print("[拓扑] 未知拓扑指令格式\r\n");
}

void Update_Topology_Matrix(const char* topology_str)
{
    char temp_str[128];
    char temp_row[128];
    strncpy(temp_str, topology_str, sizeof(temp_str)-1);
    temp_str[sizeof(temp_str)-1] = '\0';
    
    debug_print("[拓扑] 开始解析拓扑矩阵字符串\r\n");
    
    char* row_ptr = temp_str;
    int row = 0;
    
    while (row_ptr != NULL && *row_ptr != '\0' && row < MAX_CARS) {
        // 找到当前行的结束（分号或字符串结束）
        char* row_end = strchr(row_ptr, ';');
        if (row_end != NULL) {
            *row_end = '\0'; // 临时终止当前行
        }
        
        // 复制当前行到临时缓冲区
        strncpy(temp_row, row_ptr, sizeof(temp_row)-1);
        temp_row[sizeof(temp_row)-1] = '\0';
        
        char debug_row[64];
        snprintf(debug_row, sizeof(debug_row), 
                 "[拓扑] 解析第%d行: %s\r\n", row, temp_row);
        debug_print(debug_row);
        
        // 解析当前行的数字
        char* col_ptr = temp_row;
        int col = 0;
        
        while (col_ptr != NULL && *col_ptr != '\0' && col < MAX_CARS) {
            // 跳过空格和逗号
            while (*col_ptr == ' ' || *col_ptr == ',') {
                col_ptr++;
            }
            
            if (*col_ptr == '\0') {
                break;
            }
            
            // 解析数字
            int value = atoi(col_ptr);
            communication_topology[row][col] = (uint8_t)value;
            
            char debug_cell[32];
            snprintf(debug_cell, sizeof(debug_cell), 
                     "[拓扑] 设置 [%d][%d] = %d\r\n", row, col, value);
            debug_print(debug_cell);
            
            col++;
            
            // 移动到下一个数字
            col_ptr = strchr(col_ptr, ',');
            if (col_ptr != NULL) {
                col_ptr++; // 跳过逗号
            }
        }
        
        row++;
        
        // 移动到下一行
        if (row_end != NULL) {
            row_ptr = row_end + 1; // 跳过分号
        } else {
            row_ptr = NULL; // 没有更多行了
        }
    }
    
    debug_print("[拓扑] 拓扑矩阵解析完成\r\n");
    
    // // 打印拓扑矩阵用于调试
    // char debug_msg[256];
    // snprintf(debug_msg, sizeof(debug_msg), 
    //          "[拓扑] 当前矩阵:\r\n"
    //          "[%d,%d,%d,%d]\r\n"
    //          "[%d,%d,%d,%d]\r\n"
    //          "[%d,%d,%d,%d]\r\n"
    //          "[%d,%d,%d,%d]\r\n",
    //          communication_topology[0][0], communication_topology[0][1], 
    //          communication_topology[0][2], communication_topology[0][3],
    //          communication_topology[1][0], communication_topology[1][1],
    //          communication_topology[1][2], communication_topology[1][3],
    //          communication_topology[2][0], communication_topology[2][1],
    //          communication_topology[2][2], communication_topology[2][3],
    //          communication_topology[3][0], communication_topology[3][1],
    //          communication_topology[3][2], communication_topology[3][3]);
    // debug_print(debug_msg);
    
    // 修复接收权限验证逻辑
    char car_list[4][8] = {"CAR1", "CAR2", "CAR3", "CAR4"};
    debug_print("[拓扑] 本车接收权限验证:\r\n");
    
    // 先确认本车索引
    char index_msg[64];
    snprintf(index_msg, sizeof(index_msg), 
             "[拓扑] 本车: %s, 索引: %d\r\n", CAR_ID, car_index);
    debug_print(index_msg);
    
    for(int i = 0; i < MAX_CARS; i++) {
        // 根据规则：A[i][j] = 1 表示小车i可以给小车j发送信息（j可以看见i）
        // 所以本车（索引car_index）能否接收小车i的信息，取决于communication_topology[i][car_index]
        uint8_t can_receive = communication_topology[i][car_index];
        
        char perm_msg[64];
        snprintf(perm_msg, sizeof(perm_msg), 
                 "[拓扑]   从 %s 接收: %s (检查 communication_topology[%d][%d] = %d)\r\n", 
                 car_list[i], 
                 can_receive ? "允许" : "禁止",
                 i, car_index, communication_topology[i][car_index]);
        debug_print(perm_msg);
    }
}

// 广播连接初始化函数
ESP8266_Status_t ESP8266_InitBroadcast(void)
{
    debug_print("[广播] 初始化广播连接...\r\n");
    
    // 设置多连接模式
    if(ESP8266_SendATCommand_Enhanced("AT+CIPMUX=1", "OK", 5000) != ESP8266_OK) {
        debug_print("[广播] 设置多连接模式失败\r\n");
        return ESP8266_ERROR;
    }
    
    // 创建广播连接（使用连接1）
    char broadcast_cmd[128];
    snprintf(broadcast_cmd, sizeof(broadcast_cmd), 
             "AT+CIPSTART=1,\"UDP\",\"%s\",%d,%d,0", 
             "192.168.31.255", BROADCAST_PORT, BROADCAST_PORT);
    
    ESP8266_Status_t result = ESP8266_SendATCommand_Enhanced(broadcast_cmd, "OK", 10000);
    
    if(result == ESP8266_OK || result == ESP8266_ALREADY_CONNECTED) {
        esp8266_broadcast_initialized = 1;
        debug_print("[广播] 广播连接初始化成功\r\n");
        return ESP8266_OK;
    } else {
        debug_print("[广播] 广播连接初始化失败\r\n");
        return ESP8266_ERROR;
    }
}

// UDP初始化函数（修改为使用连接0）
ESP8266_Status_t ESP8266_InitUDP(void)
{
    debug_print("[UDP] 初始化UDP连接...\r\n");
    
    // 设置多连接模式
    if(ESP8266_SendATCommand_Enhanced("AT+CIPMUX=1", "OK", 5000) != ESP8266_OK) {
        debug_print("[UDP] 设置多连接模式失败\r\n");
        return ESP8266_ERROR;
    }
    
    char udp_cmd[128];
    snprintf(udp_cmd, sizeof(udp_cmd), 
             "AT+CIPSTART=0,\"UDP\",\"%s\",%d,%d,0", 
             SERVER_IP, SERVER_PORT, 12345);
    
    ESP8266_Status_t result = ESP8266_SendATCommand_Enhanced(udp_cmd, "OK", 10000);
    
    if(result == ESP8266_OK || result == ESP8266_ALREADY_CONNECTED) {
        esp8266_udp_initialized = 1;
        debug_print("[UDP] UDP连接初始化成功\r\n");
        return ESP8266_OK;
    } else {
        debug_print("[UDP] UDP连接初始化失败\r\n");
        return ESP8266_ERROR;
    }
}


ESP8266_Status_t ESP8266_SendStatus_UDP_Reliable(float x, float y, float yaw, float voltage, float vx, float vy, float vz)
{
    static char status_msg[128];
    static uint8_t retry_count = 0;
    
    snprintf(status_msg, sizeof(status_msg), 
             "%s:%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.3f", 
             CAR_ID, x, y, yaw, voltage, vx, vy, vz);
    
    // 如果UDP未初始化，先初始化
    if(!esp8266_udp_initialized) {
        if(ESP8266_InitUDP() != ESP8266_OK) {
            return ESP8266_ERROR;
        }
    }
     
    char send_cmd[32];  //
    int len = sprintf(send_cmd, "AT+CIPSEND=0,%d\r\n", strlen(status_msg));
    
    // 发送命令，最多重试2次（减少重试次数）
    for(retry_count = 0; retry_count < 2; retry_count++) {
        // 清空接收缓冲区
        esp8266_rx_index = 0;
        memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
        
        HAL_UART_Transmit(&huart6, (uint8_t*)send_cmd, len, 100);
        HAL_Delay(3);  // 增加延迟，等待响应
        
        // 检查是否收到">"提示
        uint32_t wait_start = HAL_GetTick();
        while(HAL_GetTick() - wait_start < 1000) {
            if(esp8266_rx_index > 0 && strstr((char*)esp8266_rx_buffer, ">") != NULL) {
                break;
            }
            HAL_Delay(3);
        }
        
        // 发送数据
        HAL_UART_Transmit(&huart6, (uint8_t*)status_msg, strlen(status_msg), 100);
        HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 100);
        
        // 检查发送是否成功
        HAL_Delay(3);  // 等待响应
        if(esp8266_rx_index > 0) {
            if(strstr((char*)esp8266_rx_buffer, "SEND OK") != NULL) {
                last_successful_communication = HAL_GetTick();
                esp8266_rx_index = 0;
                memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
                return ESP8266_OK;
            }
        }   
        
        HAL_Delay(3);
    }
    
    // 发送失败，标记UDP需要重新初始化
    esp8266_udp_initialized = 0;
    esp8266_broadcast_initialized = 0;
    return ESP8266_ERROR;
}  

// 增强的AT指令发送函数
ESP8266_Status_t ESP8266_SendATCommand_Enhanced(const char* cmd, const char* expected_response, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    
    // 清空接收缓冲区
    esp8266_rx_index = 0;
    memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
    
    // 清空UART接收缓冲区
    __HAL_UART_FLUSH_DRREGISTER(&huart6);
    
    debug_print("[ESP8266] 发送: ");
    debug_print(cmd);
    debug_print("\r\n");
    
    // 发送命令
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), 1000);
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 1000);
    
    while(HAL_GetTick() - start_time < timeout) {
        // 处理接收数据
        if(esp8266_rx_index > 0) {
            esp8266_rx_buffer[esp8266_rx_index] = '\0';
            
            if(strstr((char*)esp8266_rx_buffer, "ALREADY CONNECTED") != NULL) {
                debug_print("[ESP8266] 已连接状态\r\n");
                last_successful_communication = HAL_GetTick();
                uart_timeout_count = 0;
                return ESP8266_ALREADY_CONNECTED;
            }
            
            if(strstr((char*)esp8266_rx_buffer, expected_response) != NULL) {
                debug_print("[ESP8266] 收到预期响应\r\n");
                last_successful_communication = HAL_GetTick();
                uart_timeout_count = 0;
                return ESP8266_OK;
            }
            
            if(strstr((char*)esp8266_rx_buffer, "ERROR") != NULL) {
                debug_print("[ESP8266] 错误响应\r\n");
                last_successful_communication = HAL_GetTick();
                uart_timeout_count = 0;
                return ESP8266_ERROR;
            }
        }
        
        // 短暂延迟，避免忙等待
        HAL_Delay(3);
    }
    
    // 超时处理
    uart_timeout_count++;
    debug_print("[ESP8266] 超时，收到: ");
    debug_print((char*)esp8266_rx_buffer);
    debug_print("\r\n");
    
    // 如果连续超时多次，可能需要复位UART
    if(uart_timeout_count > 5) {
        debug_print("[UART] 连续超时，可能需要检查硬件连接\r\n");
        uart_timeout_count = 0;
    }
    
    return ESP8266_TIMEOUT;
}

// 获取MAC地址
ESP8266_Status_t ESP8266_GetMACAddress(char* mac_buffer, uint32_t buffer_size)
{
    debug_print("[MAC] 获取MAC地址...\r\n");
    
    esp8266_rx_index = 0;
    memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
    
    if(ESP8266_SendATCommand_Enhanced("AT+CIPSTAMAC?", "OK", 3000) != ESP8266_OK) {
        debug_print("[MAC] 获取MAC地址失败\r\n");
        return ESP8266_ERROR;
    }
    
    char* mac_start = strstr((char*)esp8266_rx_buffer, "+CIPSTAMAC:");
    if(mac_start != NULL) {
        mac_start += 12;
        int i = 0;
        while(i < buffer_size - 1 && mac_start[i] != '"' && mac_start[i] != '\0') {
            mac_buffer[i] = mac_start[i];
            i++;
        }
        mac_buffer[i] = '\0';
        
        debug_print("[MAC] 获取到MAC地址: ");
        debug_print(mac_buffer);
        debug_print("\r\n");
        
        return ESP8266_OK;
    }
    
    debug_print("[MAC] 解析MAC地址失败\r\n");
    return ESP8266_ERROR;
}

// 根据MAC地址自动分配小车ID
void ESP8266_AutoAssignCarID(void)
{
    char mac_address[18];
    
    if(ESP8266_GetMACAddress(mac_address, sizeof(mac_address)) == ESP8266_OK) {
        for(int i = 0; i < 4; i++) {
            if(strcmp(mac_address, known_mac_addresses[i]) == 0) {
                snprintf(CAR_ID, sizeof(CAR_ID), "CAR%d", i + 1);
                debug_print("[ID] 根据MAC地址分配ID: ");
                debug_print(CAR_ID);
                debug_print("\r\n");
                return;
            }
        }
        
        snprintf(CAR_ID, sizeof(CAR_ID), "CAR%s", mac_address + 12);
        debug_print("[ID] 使用MAC地址生成ID: ");
        debug_print(CAR_ID);
        debug_print("\r\n");
    } else {
        strcpy(CAR_ID, "CAR0");
        debug_print("[ID] 使用默认ID: CAR0\r\n");
    }
}

// 初始化其他小车信息数组（含4辆已知小车的基础信息）
void Init_Other_Cars_Info(void)
{
    // 1. 先清空数组
    for (int i = 0; i < MAX_OTHER_CARS; i++) {
        other_cars[i].valid = 0;
        memset(other_cars[i].car_id, 0, sizeof(other_cars[i].car_id));
        other_cars[i].position_x = 0;
        other_cars[i].position_y = 0;
        other_cars[i].yaw = 0;
        other_cars[i].last_update = 0;
    }
    other_cars_count = 0;

    // 2. 预先添加4辆已知小车（CAR1~CAR4），标记为"未上线"（valid=0）
    const char* known_car_ids[] = {"CAR1", "CAR2", "CAR3", "CAR4"};
    for (int i = 0; i < 4 && i < MAX_OTHER_CARS; i++) {
        strncpy(other_cars[i].car_id, known_car_ids[i], sizeof(other_cars[i].car_id) - 1);
        other_cars[i].valid = 0; // 初始为"未上线"
        other_cars[i].position_x = 0;
        other_cars[i].position_y = 0;
        other_cars[i].yaw = 0;
    }

    debug_print("[广播] 其他小车信息数组初始化完成（含CAR1~CAR4基础信息）\r\n");
}

ESP8266_Status_t ESP8266_Init(void)
{
    debug_print("开始初始化ESP8266...\r\n");
    
    Init_Other_Cars_Info();

    HAL_Delay(1000);
    
    // 先发送AT命令测试连接
    if(ESP8266_SendATCommand_Enhanced("AT", "OK", 3000) != ESP8266_OK) {
        debug_print("ESP8266无响应，请检查连接\r\n");
        return ESP8266_ERROR;
    }
    
    ESP8266_AutoAssignCarID();
    Init_Communication_Topology();  // 初始化通信拓扑
    if(ESP8266_SendATCommand_Enhanced("AT+CWMODE=1", "OK", 3000) != ESP8266_OK) {
        debug_print("设置模式失败\r\n");
        return ESP8266_ERROR;
    }
    
    char wifi_cmd[128];
    snprintf(wifi_cmd, sizeof(wifi_cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
    if(ESP8266_SendATCommand_Enhanced(wifi_cmd, "OK", 20000) != ESP8266_OK) {
        debug_print("连接WiFi失败\r\n");
        return ESP8266_ERROR;
    }
    
    // 初始化UDP连接（单播）
    if(ESP8266_InitUDP() != ESP8266_OK) {
        debug_print("UDP初始化失败\r\n");
        return ESP8266_ERROR;
    }
    
    // 初始化广播连接
    if(ESP8266_InitBroadcast() != ESP8266_OK) {
        debug_print("广播初始化失败\r\n");
        return ESP8266_ERROR;
    }
    
    debug_print("ESP8266 UDP和广播初始化成功！小车ID: ");
    debug_print(CAR_ID);
    debug_print("\r\n");
    
    last_successful_communication = HAL_GetTick();
    return ESP8266_OK;
}

// 处理精简版分段广播数据（修复版本，原始的JSON版本数据）
void Process_Segmented_Broadcast(const char* json_data)
{
    // debug_print("[广播] 处理精简版分段广播数据\r\n");
    
    // 解析小车ID
    char car_id[16] = {0};
    const char* id_ptr = strstr(json_data, "\"i\":\"");
    if (id_ptr != NULL) {
        id_ptr += 5;
        const char* id_end = strchr(id_ptr, '"');
        if (id_end != NULL) {
            int id_len = id_end - id_ptr;
            if (id_len > 0 && id_len < (int)sizeof(car_id)-1) {
                strncpy(car_id, id_ptr, id_len);
                car_id[id_len] = '\0';
            }
        }
    }
    
    if (strlen(car_id) == 0) {
        // debug_print("[广播] 未找到有效小车ID\r\n");
        return;
    }
    
    // 跳过自己的信息
    if (strcmp(car_id, CAR_ID) == 0) {
        return;
    }
    
    // 解析位置信息 [x,y] 数组格式
    float pos_x = 0, pos_y = 0;
    const char* pos_ptr = strstr(json_data, "\"p\":[");
    if (pos_ptr != NULL) {
        sscanf(pos_ptr, "\"p\":[%f,%f]", &pos_x, &pos_y);
    }
    
    // 解析航向角
    float heading = 0;
    const char* h_ptr = strstr(json_data, "\"h\":");
    if (h_ptr != NULL) {
        sscanf(h_ptr, "\"h\":%f", &heading);
    }
    
    // 解析速度信息 [vx,vy,vz] 数组格式
    float vel_vx = 0, vel_vy = 0, vel_vz = 0;
    const char* vel_ptr = strstr(json_data, "\"v\":[");
    if (vel_ptr != NULL) {
        sscanf(vel_ptr, "\"v\":[%f,%f,%f]", &vel_vx, &vel_vy, &vel_vz);
    }
    
    // 拓扑过滤：检查是否应该处理这辆小车的信息
    if(!Should_Process_Car_Info(car_id)) {
        // char filter_msg[64];
        // snprintf(filter_msg, sizeof(filter_msg), 
        //          "[拓扑] 过滤 %s 的信息（拓扑限制）\r\n", car_id);
        // debug_print(filter_msg);
        return;
    }
    
    // 更新其他小车信息
    Update_Other_Car_Info(car_id, pos_x, pos_y, vel_vx, vel_vy, vel_vz, heading);
    
    // // 修复：在调试信息中添加航向角显示
    // char debug_msg[128];
    // snprintf(debug_msg, sizeof(debug_msg), 
    //          "[广播] 更新小车 %s: 位置(%.2f,%.2f) 航向(%.1f°) 速度(%.3f,%.3f,%.3f)\r\n",
    //          car_id, pos_x, pos_y, heading, vel_vx, vel_vy, vel_vz);
    // debug_print(debug_msg);
}

// 处理精简合并广播数据（新格式：去掉电压数据）
void Process_Compact_Broadcast(const char* data)
{    
    // 打印原始数据用于调试
    char debug_raw[256];
    int data_len = strlen(data);
    // snprintf(debug_raw, sizeof(debug_raw), "[广播] 原始数据(%d字节): %s\r\n", data_len, data);
    // debug_print(debug_raw);
    
    // 详细分析数据格式
    // debug_print("[广播] 数据分析:\r\n");
    
    // 检查数据起始和结束标记
    if(data[0] != '[') {
        // debug_print("[广播] ? 数据不是以'['开头\r\n");
        return;
    }
    
    char* end_bracket = strchr(data, ']');
    if(end_bracket == NULL) {
        // debug_print("[广播] ? 数据没有以']'结尾\r\n");
        return;
    }
    
    // debug_print("[广播] ? 数据格式基本正确，有起始和结束标记\r\n");
    
    // 新数据格式: [数量 CAR1 x y 航向角 vx vy vz CAR2 ...] （去掉电压数据）
    // 示例: [4 CAR1 1.21 2.32 78.5 0.120 0.080 0.050 CAR2 2.21 3.32 38.2 1.120 4.900 0.080 CAR3 3.33 3.33 33.3 0.333 0.333 0.033 CAR4 4.44 4.44 44.4 0.444 0.444 0.044]
    
    // 跳过开头的'['
    const char* ptr = data + 1;
    
    // 解析小车数量
    int car_count = 0;
    if (sscanf(ptr, "%d", &car_count) != 1) {
        // debug_print("[广播] ? 解析小车数量失败\r\n");
        return;
    }
    
    char count_msg[32];
    // snprintf(count_msg, sizeof(count_msg), "[广播] ? 小车数量: %d\r\n", car_count);
    // debug_print(count_msg);
    
    if (car_count <= 0 || car_count > MAX_OTHER_CARS) {
        // debug_print("[广播] ? 小车数量无效\r\n");
        return;
    }
    
    // 移动到数量后的空格
    ptr = strchr(ptr, ' ');
    if (ptr == NULL) {
        // debug_print("[广播] ? 数据格式错误，找不到数量后的空格\r\n");
        return;
    }
    ptr++; // 跳过空格
    
    // 解析每辆小车的数据（现在每个小车有7个字段，去掉电压）
    int processed_cars = 0;
    for (int i = 0; i < car_count; i++) {
        char short_id[4] = {0};  // C1, C2等
        float pos_x = 0, pos_y = 0;
        float heading = 0;  // 改为float类型，保留小数
        float vx = 0, vy = 0, vz = 0;
        
        // 解析单个小车数据（7个字段，去掉电压）
        int parsed = sscanf(ptr, "%3s %f %f %f %f %f %f", 
                           short_id, &pos_x, &pos_y, &heading, 
                           &vx, &vy, &vz);
        
        if (parsed == 7) {
            // 转换短ID为完整ID
            char car_id[16];
            snprintf(car_id, sizeof(car_id), "CAR%c", short_id[1]);
            
            // 跳过自己的信息
            if (strcmp(car_id, CAR_ID) != 0) {
                // 拓扑过滤：检查是否应该处理这辆小车的信息
                if(Should_Process_Car_Info(car_id)) {
                    // 更新其他小车信息
                    Update_Other_Car_Info(car_id, pos_x, pos_y, vx, vy, vz, heading);
                    
                    processed_cars++;
                } else {
                }
            } else {
            }
            
            // 移动到下一辆小车数据 - 现在每个小车有7个字段
            for (int j = 0; j < 7; j++) {
                ptr = strchr(ptr, ' ');
                if (ptr == NULL) {
                    if (i < car_count - 1) {
                        // debug_print("[广播] ?? 数据不完整，提前结束\r\n");
                    }
                    break;
                }
                ptr++; // 跳过空格
            }
            
            // 如果已经到字符串末尾，提前退出
            if (ptr == NULL || *ptr == '\0' || *ptr == ']') {
                break;
            }
        } else {
           
            break;
        }
    }
    
}

void Process_Broadcast_Data(const char* data)
{
    // 首先检查是否为拓扑指令
    if (strstr(data, "TOPOLOGY") != NULL) {
        debug_print("[广播] 检测到拓扑指令，转发给拓扑处理函数\r\n");
        Process_Topology_Command(data);
        return;
    }
    
    // 检查是否为编队指令
    if (strstr(data, "FORMATION") != NULL) {
        debug_print("[广播] 检测到编队指令，转发给编队处理函数\r\n");
        Process_Formation_Command(data);
        return;
    }
    
    // 检查是否为新的精简合并格式
    if (data[0] == '[' && strchr(data, ']') != NULL) {
        Process_Compact_Broadcast(data);
    }
    // 原有的JSON格式处理
    else if (strstr(data, "\"type\":\"broadcast_single\"") != NULL) {
        const char* json_start = strstr(data, "{");
        if (json_start != NULL) {
            Process_Segmented_Broadcast(json_start);
        }
    }
    else {
        // 尝试其他格式
        debug_print("[广播] 未知广播数据格式，尝试通用处理\r\n");
        
        // 再次检查是否为拓扑或编队指令（可能有前缀）
        if (strstr(data, "TOPOLOGY") != NULL) {
            Process_Topology_Command(data);
        }
        else if (strstr(data, "FORMATION") != NULL) {
            Process_Formation_Command(data);
        }
        else {
            debug_print("[广播] 无法识别的广播数据格式\r\n");
        }
    }
}

// 处理单播数据（控制指令、编队指令、拓扑指令等）
void Process_Unicast_Data(const char* data)
{
    // debug_print("[单播] 处理单播数据\r\n");
    
    // 检查数据是否为编队指令
    if(strstr(data, "FORMATION:") != NULL) {
        debug_print("[单播] 检测到编队指令\r\n");
        Process_Formation_Command(data);
    }
    
    // 检查控制命令
    else if(strstr(data, "CTRL:") != NULL) {
        debug_print("[单播] 检测到控制命令\r\n");
        Process_Control_Command(data);
    }
    // 检查拓扑指令
    else if(strstr(data, "TOPOLOGY") != NULL) {
        debug_print("[单播] 检测到拓扑指令\r\n");
        Process_Topology_Command(data);
    }
    else {
        debug_print("[单播] 未知单播数据格式\r\n");
    }
}

// 控制命令处理函数 - 确保与wifi_task.c中的定义一致
void Process_Control_Command(const char* command)
{
    // 打印原始指令用于调试
    char debug_msg[256];
    // snprintf(debug_msg, sizeof(debug_msg), 
    //          "[WiFi] 收到原始指令: %s\r\n", command);
    // debug_print(debug_msg);
    
    // 解析控制命令格式: "CTRL:CAR1,TARGET:1.5,2.3,45.0"
    // 或者 "+IPD,0,31:CTRL:CAR1,TARGET:0.00,0.00,0.0"
    const char* ctrl_ptr = strstr(command, "CTRL:");
    if(ctrl_ptr != NULL) {
        char target_car[16];
        float target_x, target_y, target_yaw;
        
        // 跳过可能的IPD前缀
        if(ctrl_ptr != command) {
            // 找到CTRL:之后的位置
            ctrl_ptr = strstr(command, "CTRL:");
        }
        
        // 解析指令
        if(sscanf(ctrl_ptr, "CTRL:%[^,],TARGET:%f,%f,%f", 
                  target_car, &target_x, &target_y, &target_yaw) == 4) {
            
            // 检查是否是发给本车的命令
            if(strcmp(target_car, CAR_ID) == 0) {
                // snprintf(debug_msg, sizeof(debug_msg), 
                //          "[控制] 目标位置(%.2f, %.2f), 航向%.1f\r\n", 
                //          target_x, target_y, target_yaw);
                // debug_print(debug_msg);
                
                // 设置目标位置和航向
                Target_position[0] = target_x;
                Target_position[1] = target_y;
                Target_Yaw = target_yaw;
                newCoordinateReceived = 1;
                Auto_mode = 1;
                
                // 重置位置到达标志
                position_reached = 0;
                
                // snprintf(debug_msg, sizeof(debug_msg), 
                //          "[控制] 已设置自动模式，准备导航\r\n");
                // debug_print(debug_msg);
            } else {
                // snprintf(debug_msg, sizeof(debug_msg), 
                //          "[控制] 指令不是发给本车(%s)，忽略\r\n", CAR_ID);
                // debug_print(debug_msg);
            }
        } else {
            // debug_print("[控制] 指令格式解析失败\r\n");
        }
    } else {
        // debug_print("[控制] 未找到CTRL指令\r\n");
    }
}

// 更新其他小车信息
void Update_Other_Car_Info(const char* car_id, float x, float y, float vx, float vy, float vz, float yaw)
{
    uint32_t current_time = HAL_GetTick();
    
    int existing_index = -1;
    for (int i = 0; i < MAX_OTHER_CARS; i++) {
        if (other_cars[i].valid && strcmp(other_cars[i].car_id, car_id) == 0) {
            existing_index = i;
            break;
        }
    }
    
    if (existing_index >= 0) {
        other_cars[existing_index].position_x = x;
        other_cars[existing_index].position_y = y;
        other_cars[existing_index].velocity_vx = vx;
        other_cars[existing_index].velocity_vy = vy;
        other_cars[existing_index].velocity_vz = vz;
        other_cars[existing_index].yaw = yaw;   // 同步航向角
        other_cars[existing_index].last_update = current_time;
    } else {
        for (int i = 0; i < MAX_OTHER_CARS; i++) {
            if (!other_cars[i].valid) {
                strncpy(other_cars[i].car_id, car_id, sizeof(other_cars[i].car_id) - 1);
                other_cars[i].position_x = x;
                other_cars[i].position_y = y;
                other_cars[i].velocity_vx = vx;
                other_cars[i].velocity_vy = vy;
                other_cars[i].velocity_vz = vz;
                other_cars[i].yaw = yaw;       // 初始化航向角
                other_cars[i].last_update = current_time;
                other_cars[i].valid = 1;
                other_cars_count++;
                break;
            }
        }
    }
}

// 打印所有其他小车信息
void Print_Other_Cars_Info(void)
{
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_print_time < 5000) {
        return;
    }
    last_print_time = current_time;
    
    debug_print("====================\r\n");
    
    int valid_count = 0;
    for (int i = 0; i < MAX_OTHER_CARS; i++) {
        if (other_cars[i].valid) {
            char debug_msg[256];
            uint32_t time_since_update = (current_time - other_cars[i].last_update) / 1000;
            snprintf(debug_msg, sizeof(debug_msg), 
                     "[%d] %s: (%.2f,%.2f) (%.3f,%.3f,%.3f) [%lu秒前]\r\n",
                     valid_count + 1, other_cars[i].car_id,
                     other_cars[i].position_x, other_cars[i].position_y,
                     other_cars[i].velocity_vx, other_cars[i].velocity_vy, other_cars[i].velocity_vz,
                     time_since_update);
            debug_print(debug_msg);
            valid_count++;
        }
    }
    
    if (valid_count == 0) {
        debug_print("[广播] 暂无其他小车信息\r\n");
    } else {
        char count_msg[64];
        snprintf(count_msg, sizeof(count_msg), 
                 "[广播] 总计: %d 辆其他小车\r\n", valid_count);
        debug_print(count_msg);
    }
    debug_print("====================\r\n");
}

// 清理过时的小车信息（30秒未更新）
void Cleanup_Old_Car_Info(void)
{
    uint32_t current_time = HAL_GetTick();
    int removed_count = 0;
    
    for (int i = 0; i < MAX_OTHER_CARS; i++) {
        if (other_cars[i].valid) {
            if (current_time - other_cars[i].last_update > 30000) {
                char debug_msg[64];
                snprintf(debug_msg, sizeof(debug_msg), 
                         "[广播] 移除离线小车: %s\r\n", other_cars[i].car_id);
                debug_print(debug_msg);
                
                other_cars[i].valid = 0;
                other_cars_count--;
                removed_count++;
            }
        }
    }
    
    if (removed_count > 0) {
        char debug_msg[64];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "[广播] 移除了 %d 辆离线小车\r\n", removed_count);
        debug_print(debug_msg);
    }
}

void Process_Multiple_IPD_Packets(char* data_start, uint32_t data_length)
{
    char* current_ptr = data_start;
    uint32_t remaining_length = data_length;
    int processed_packets = 0;
    
    while (remaining_length > 0 && processed_packets < 10) { // 最多处理10个包防止无限循环
        // 查找下一个 +IPD 开头
        char* ipd_ptr = strstr(current_ptr, "+IPD");
        if (ipd_ptr == NULL) {
            break; // 没有更多 +IPD 包
        }
        
        // 计算当前包在缓冲区中的位置
        uint32_t current_offset = ipd_ptr - data_start;
        if (current_offset >= data_length) {
            break; // 超出范围
        }
        
        // 解析这个 +IPD 包
        int link_id = 0;
        int data_len = 0;
        char* colon_ptr = NULL;
        
        // 尝试解析格式: +IPD,<link_id>,<data_len>:<data>
        if (sscanf(ipd_ptr, "+IPD,%d,%d:", &link_id, &data_len) == 2) {
            colon_ptr = strchr(ipd_ptr, ':');
            if (colon_ptr != NULL) {
                char* packet_data_start = colon_ptr + 1;
                
                // 确保数据长度有效且不会越界
                uint32_t packet_end_offset = (packet_data_start + data_len) - data_start;
                if (packet_end_offset <= data_length) {
                    // 临时保存当前数据包
                    char temp_packet[256];
                    int copy_len = data_len < (int)sizeof(temp_packet)-1 ? data_len : sizeof(temp_packet)-1;
                    strncpy(temp_packet, packet_data_start, copy_len);
                    temp_packet[copy_len] = '\0';
                    
                    char debug_msg[128];
                    // snprintf(debug_msg, sizeof(debug_msg), 
                    //          "[多包] 处理粘连包 %d: ID=%d, Len=%d, Data=%.*s\r\n", 
                    //          processed_packets + 1, link_id, data_len, 30, temp_packet);
                    // debug_print(debug_msg);
                    
                    // 在多包处理函数中，修改数据处理部分：
                    if (link_id == 0) {
                        // debug_print("[多包] 处理单播数据\r\n");
                        Process_Unicast_Data(temp_packet);
                    } else if (link_id == 1) {
                        // debug_print("[多包] 处理广播数据\r\n");
                        
                        // 特别处理截断的拓扑数据
                        if (strstr(temp_packet, "TOPOLOGY:") != NULL && data_len == 40) {
                            // debug_print("[多包] 检测到可能被截断的拓扑数据，尝试特殊处理\r\n");
                            // 这里可以添加逻辑来组合多个包，或者记录不完整的数据
                        }
                        
                        Process_Broadcast_Data(temp_packet);
                    }
                    
                    processed_packets++;
                    
                    // 移动到下一个包
                    current_ptr = packet_data_start + data_len;
                    remaining_length = data_length - (current_ptr - data_start);
                    continue;
                } else {
                    // debug_print("[多包] 数据长度超出缓冲区范围\r\n");
                }
            } else {
                // debug_print("[多包] 未找到冒号分隔符\r\n");
            }
        } else {
            // debug_print("[多包] 解析+IPD格式失败\r\n");
        }
        
        // 如果解析失败，尝试备用解析
        char* comma1 = strchr(ipd_ptr, ',');
        if (comma1 != NULL) {
            char* comma2 = strchr(comma1 + 1, ',');
            if (comma2 != NULL) {
                char* colon = strchr(comma2 + 1, ':');
                if (colon != NULL) {
                    // 手动解析
                    link_id = atoi(comma1 + 1);
                    data_len = atoi(comma2 + 1);
                    char* packet_data_start = colon + 1;
                    
                    // 确保数据长度有效
                    uint32_t packet_end_offset = (packet_data_start + data_len) - data_start;
                    if (packet_end_offset <= data_length) {
                        char temp_packet[256];
                        int copy_len = data_len < (int)sizeof(temp_packet)-1 ? data_len : sizeof(temp_packet)-1;
                        strncpy(temp_packet, packet_data_start, copy_len);
                        temp_packet[copy_len] = '\0';
                        
                        char debug_msg[128];
                        // snprintf(debug_msg, sizeof(debug_msg), 
                        //          "[多包] 手动解析包 %d: ID=%d, Len=%d\r\n", 
                        //          processed_packets + 1, link_id, data_len);
                        // debug_print(debug_msg);
                        
                        if (link_id == 0) {
                            Process_Unicast_Data(temp_packet);
                        } else if (link_id == 1) {
                            Process_Broadcast_Data(temp_packet);
                        }
                        
                        processed_packets++;
                        current_ptr = packet_data_start + data_len;
                        remaining_length = data_length - (current_ptr - data_start);
                        continue;
                    }
                }
            }
        }
        
        // 如果都无法解析，跳到下一个字符继续查找
        current_ptr = ipd_ptr + 4; // 跳过 "+IPD"
        remaining_length = data_length - (current_ptr - data_start);
    }
    
    char result_msg[64];
    // snprintf(result_msg, sizeof(result_msg), 
    //          "[多包] 处理完成，共处理 %d 个包\r\n", processed_packets);
    // debug_print(result_msg);
}

void ESP8266_Process(void)
{
    if(esp8266_rx_index > 0) {
        esp8266_rx_buffer[esp8266_rx_index] = '\0';
        
        // // 首先打印原始数据用于调试
        // debug_print("\r\n[WiFi] 收到数据，开始处理...\r\n");
        // ESP8266_PrintRawData((char*)esp8266_rx_buffer, esp8266_rx_index);
        
        char* data_start = (char*)esp8266_rx_buffer;
        uint32_t data_length = esp8266_rx_index;
        
        // 检查是否为多个+IPD包粘连的情况
        int ipd_count = 0;
        char* search_ptr = data_start;
        while((search_ptr = strstr(search_ptr, "+IPD")) != NULL) {
            ipd_count++;
            search_ptr += 4; // 跳过 "+IPD"
        }
        
        if(ipd_count > 1) {
            char count_msg[64];
            // snprintf(count_msg, sizeof(count_msg), 
            //          "[多包] 检测到 %d 个粘连的IPD包\r\n", ipd_count);
            // debug_print(count_msg);
            
            // 使用多包处理函数
            Process_Multiple_IPD_Packets(data_start, data_length);
        } else {
            // 原有的单包处理逻辑
            char* ipd_ptr = strstr(data_start, "+IPD");
            if(ipd_ptr != NULL) {
                debug_print("[WiFi] 检测到+IPD格式数据\r\n");
                
                int link_id = 0;
                int data_len = 0;
                
                // 解析格式: +IPD,<link_id>,<data_len>:<data>
                if(sscanf(ipd_ptr, "+IPD,%d,%d:", &link_id, &data_len) == 2) {
                    char link_msg[64];
                    // snprintf(link_msg, sizeof(link_msg), 
                    //          "[WiFi] 解析成功: 连接ID=%d, 数据长度=%d\r\n", link_id, data_len);
                    // debug_print(link_msg);
                    
                    // 找到冒号位置
                    char* colon_ptr = strchr(ipd_ptr, ':');
                    if(colon_ptr != NULL) {
                        data_start = colon_ptr + 1; // 跳过冒号
                        
                        // 确保数据长度不超过缓冲区
                        if(data_len > 0 && data_len < 1024) {
                            // 根据连接ID判断数据类型
                            if(link_id == 0) {
                                // 连接0：单播数据（控制指令、编队指令等）
                                // debug_print("[WiFi] 处理单播数据\r\n");
                                // Process_Unicast_Data(data_start);
                            } else if(link_id == 1) {
                                // 连接1：广播数据（其他小车状态）
                                // debug_print("[WiFi] 处理广播数据\r\n");
                                // Process_Broadcast_Data(data_start);
                            } else {
                                char unknown_msg[32];
                                // snprintf(unknown_msg, sizeof(unknown_msg), 
                                //          "[WiFi] 未知连接ID: %d\r\n", link_id);
                                // debug_print(unknown_msg);
                            }
                        } else {
                            debug_print("[WiFi] 数据长度无效\r\n");
                        }
                    } else {
                        debug_print("[WiFi] 未找到冒号分隔符\r\n");
                    }
                } else {
                    debug_print("[WiFi] 解析+IPD格式失败，尝试备用解析\r\n");
                    
                    // 备用解析：手动解析+IPD格式
                    char* comma1 = strchr(ipd_ptr, ',');
                    if(comma1 != NULL) {
                        char* comma2 = strchr(comma1 + 1, ',');
                        if(comma2 != NULL) {
                            char* colon = strchr(comma2 + 1, ':');
                            if(colon != NULL) {
                                // 手动解析连接ID和数据长度
                                link_id = atoi(comma1 + 1);
                                data_len = atoi(comma2 + 1);
                                data_start = colon + 1;
                                
                                char manual_msg[64];
                                // snprintf(manual_msg, sizeof(manual_msg), 
                                //          "[WiFi] 手动解析: ID=%d, Len=%d\r\n", link_id, data_len);
                                // debug_print(manual_msg);
                                
                                // 根据连接ID处理数据
                                if(link_id == 0) {
                                    Process_Unicast_Data(data_start);
                                } else if(link_id == 1) {
                                    Process_Broadcast_Data(data_start);
                                }
                            }
                        }
                    }
                }
            }
            // 检查是否为直接的编队指令（没有+IPD前缀）
            else if(strstr(data_start, "FORMATION:") != NULL) {
                // debug_print("[WiFi] 检测到直接编队指令\r\n");
                Process_Formation_Command(data_start);
            }
            // 检查是否为直接的精简合并广播格式 (以'['开头)
            else if(data_start[0] == '[') {
                // debug_print("[WiFi] 检测到直接的精简合并广播格式\r\n");
                Process_Compact_Broadcast(data_start);
            }
            // 检查是否为JSON格式 (以'{'开头)
            else if(data_start[0] == '{') {
                // debug_print("[WiFi] 检测到JSON格式数据\r\n");
                Process_Broadcast_Data(data_start);
            }
            // 检查控制命令
            else if(strstr(data_start, "CTRL:") != NULL) {
                // debug_print("[WiFi] 检测到控制命令\r\n");
                Process_Control_Command(data_start);
            }
            // 检查拓扑指令
            else if(strstr(data_start, "TOPOLOGY") != NULL) {
                // debug_print("[WiFi] 检测到拓扑指令\r\n");
                Process_Topology_Command(data_start);
            }
            else {
                debug_print("[WiFi] 未知数据格式\r\n");
                
                // 对于未知数据，尝试多种解析方式
                debug_print("[WiFi] 尝试多种解析方式...\r\n");
                
                // 1. 尝试查找+IPD（可能被其他字符干扰）
                char* search_ptr = data_start;
                while((search_ptr = strstr(search_ptr, "IPD")) != NULL) {
                    if(search_ptr > data_start && *(search_ptr - 1) == '+') {
                        debug_print("[WiFi] 找到被干扰的+IPD格式\r\n");
                        // 这里可以添加处理逻辑
                        break;
                    }
                    search_ptr++;
                }
                
                // 2. 尝试作为广播数据处理
                Process_Broadcast_Data(data_start);
            }
        }
        
        // 清理缓冲区
        esp8266_rx_index = 0;
        memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
    } 
    
    // 定期清理过时信息
    static uint32_t last_cleanup_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_cleanup_time > 10000) {
        Cleanup_Old_Car_Info();
        last_cleanup_time = current_time;
    }
}

// 添加调试打印函数
void ESP8266_PrintRawData(const char* data, uint32_t length)
{
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 限制打印频率，避免刷屏
    if(current_time - last_print_time < 100) {
        return;
    }
    last_print_time = current_time;
    
    debug_print("\r\n=== 原始数据开始 ===\r\n");
    
    // 打印数据长度
    char len_msg[32];
    snprintf(len_msg, sizeof(len_msg), "数据长度: %lu 字节\r\n", length);
    debug_print(len_msg);
    
    // 打印ASCII字符
    debug_print("ASCII: ");
    for(uint32_t i = 0; i < length && i < 128; i++) {
        if(data[i] >= 32 && data[i] <= 126) {  // 可打印字符
            char ch[2] = {data[i], '\0'};
            debug_print(ch);
        } else {
            debug_print(".");  // 非打印字符用点表示
        }
    }
    debug_print("\r\n");
    
    // 打印十六进制
    debug_print("HEX:   ");
    for(uint32_t i = 0; i < length && i < 64; i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X ", (uint8_t)data[i]);
        debug_print(hex);
        
        // 每16字节换行
        if((i + 1) % 16 == 0 && i + 1 < length) {
            debug_print("\r\n       ");
        }
    }
    debug_print("\r\n");
    
    // 如果有特殊字符，特别标注
    for(uint32_t i = 0; i < length; i++) {
        if(data[i] == '\r') {
            debug_print("发现回车符(\\r)\r\n");
        } else if(data[i] == '\n') {
            debug_print("发现换行符(\\n)\r\n");
        } else if(data[i] == '\0') {
            debug_print("发现空字符(\\0)\r\n");
        }
    }
    
    debug_print("=== 原始数据结束 ===\r\n\r\n");
}

// 打印接收缓冲区内容
void ESP8266_Debug_PrintBuffer(void)
{
    debug_print("\r\n=== 接收缓冲区状态 ===\r\n");
    
    char buf_info[64];
    snprintf(buf_info, sizeof(buf_info), 
             "缓冲区索引: %d, 缓冲区大小: %lu\r\n", 
             esp8266_rx_index, sizeof(esp8266_rx_buffer));
    debug_print(buf_info);
    
    if(esp8266_rx_index > 0) {
        ESP8266_PrintRawData((char*)esp8266_rx_buffer, esp8266_rx_index);
    } else {
        debug_print("接收缓冲区为空\r\n");
    }
    
    debug_print("=== 缓冲区状态结束 ===\r\n\r\n");
}