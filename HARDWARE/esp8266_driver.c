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

// 初始化通信拓扑
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

// 判断是否应该处理来自指定小车的信息
uint8_t Should_Process_Car_Info(const char* car_id)
{
    // 如果拓扑功能未启用，处理所有小车信息
    if(!topology_enabled) {
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
    
    return result;
}

// 处理拓扑指令
void Process_Topology_Command(const char* command)
{
    char debug_msg[128];
    // 检查新格式 [T,...]
    if(command[0] == '[' && strstr(command, "T,") != NULL) {
        char cmd_type[2];
        int enable;
        int topology_matrix[16];
        int parsed_count;
        
        // 解析指令类型
        if(sscanf(command, "[T,%1s", cmd_type) == 1) {
            switch(cmd_type[0]) {
                case 'E': // 切换拓扑功能 [T,E,1]
                    if(sscanf(command, "[T,E,%d", &enable) == 1) {
                        topology_enabled = enable ? 1 : 0;
                        snprintf(debug_msg, sizeof(debug_msg), 
                                 "[拓扑] 拓扑功能已%s\r\n", 
                                 topology_enabled ? "启用" : "禁用");
                        debug_print(debug_msg);
                    }
                    break;
                    
                case 'M': // 设置拓扑矩阵 [T,M,1,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0]
                    parsed_count = sscanf(command, "[T,M,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                         &topology_matrix[0], &topology_matrix[1], &topology_matrix[2], &topology_matrix[3],
                                         &topology_matrix[4], &topology_matrix[5], &topology_matrix[6], &topology_matrix[7],
                                         &topology_matrix[8], &topology_matrix[9], &topology_matrix[10], &topology_matrix[11],
                                         &topology_matrix[12], &topology_matrix[13], &topology_matrix[14], &topology_matrix[15]);
                    
                    if(parsed_count == 16) {
                        // 自动启用拓扑功能
                        topology_enabled = 1;
                        
                        // 更新拓扑矩阵
                        for(int i = 0; i < 4; i++) {
                            for(int j = 0; j < 4; j++) {
                                communication_topology[i][j] = topology_matrix[i*4 + j];
                            }
                        }
                        
                        debug_print("[拓扑] 拓扑矩阵更新完成\r\n");
                    }
                    break;
                    
                default:
                    debug_print("[拓扑] 未知拓扑指令类型\r\n");
                    break;
            }
        }
        return;
    }
    
}

// 更新拓扑矩阵
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

// 发送状态信息（可靠UDP）
// 优化发送状态函数，增加容错
ESP8266_Status_t ESP8266_SendStatus_UDP_Reliable(float x, float y, float yaw, float voltage, float vx, float vy, float vz)
{
    static char status_msg[128];
    static uint8_t retry_count = 0;
    
    snprintf(status_msg, sizeof(status_msg), 
             "%s:%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.3f", 
             CAR_ID, x, y, yaw, voltage, vx, vy, vz);
    
    // // 如果UDP未初始化，先初始化
    // if(!esp8266_udp_initialized) {
    //     if(ESP8266_InitUDP() != ESP8266_OK) {
    //         return ESP8266_ERROR;
    //     }
    // }
     
    char send_cmd[32];
    int len = sprintf(send_cmd, "AT+CIPSEND=0,%d\r\n", strlen(status_msg));
    
    // 发送命令，最多重试1次（减少重试次数）
    for(retry_count = 0; retry_count < 1; retry_count++) {
        // 清空接收缓冲区
        esp8266_rx_index = 0;
        memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
        
        HAL_UART_Transmit(&huart6, (uint8_t*)send_cmd, len, 100);
        HAL_Delay(5);  // 适当增加延迟
        
        // 检查是否收到">"提示
        uint32_t wait_start = HAL_GetTick();
        while(HAL_GetTick() - wait_start < 500) {  // 缩短超时时间
            if(esp8266_rx_index > 0 && strstr((char*)esp8266_rx_buffer, ">") != NULL) {
                break;
            }
            HAL_Delay(2);
        }
        
        // 发送数据
        HAL_UART_Transmit(&huart6, (uint8_t*)status_msg, strlen(status_msg), 100);
        HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 100);
        
        // 检查发送是否成功
        HAL_Delay(5);  // 等待响应
        if(esp8266_rx_index > 0) {
            if(strstr((char*)esp8266_rx_buffer, "SEND OK") != NULL) {
                last_successful_communication = HAL_GetTick();
                esp8266_rx_index = 0;
                memset(esp8266_rx_buffer, 0, sizeof(esp8266_rx_buffer));
                return ESP8266_OK;
            }
        }   
        
        HAL_Delay(5);
    }
    
    // 发送失败，但不立即标记UDP需要重新初始化
    // 只有当连续失败很多次时才会触发重连
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

// 更新其他小车信息
void Update_Other_Car_Info(const char* car_id, float x, float y, float vx, float vy, float vz, float yaw)
{
    static uint32_t last_update_time = 0;
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

    // 打印更新间隔
    if (last_update_time > 0) {
        uint32_t interval = current_time - last_update_time;
        char interval_msg[32];
        snprintf(interval_msg, sizeof(interval_msg), "[间隔] %lums\n", interval);
        debug_print(interval_msg);
    }
    last_update_time = current_time;
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

// 处理ESP8266接收数据
// 处理ESP8266接收数据 - 最终简化版本
void ESP8266_Process(void)
{
    if(esp8266_rx_index == 0) return;
    
    // 添加结束符
    esp8266_rx_buffer[esp8266_rx_index] = '\0';
    
    char* data_start = (char*)esp8266_rx_buffer;
    uint32_t data_length = esp8266_rx_index;
    
    // 统一处理所有+IPD包
    char* current_ptr = data_start;
    int processed_count = 0;
    
    while((current_ptr = strstr(current_ptr, "+IPD")) != NULL && processed_count < 10) {
        // 快速解析+IPD格式
        char* comma1 = strchr(current_ptr, ',');
        if(!comma1) break;
        
        char* comma2 = strchr(comma1 + 1, ',');
        if(!comma2) break;
        
        char* colon = strchr(comma2 + 1, ':');
        if(!colon) break;
        
        // 手动解析连接ID和数据长度
        int link_id = atoi(comma1 + 1);
        int data_len = atoi(comma2 + 1);
        char* packet_data = colon + 1;
        
        // 确保数据长度有效
        if(data_len > 0 && data_len < 500 && 
           (packet_data + data_len) <= (data_start + data_length)) {
            
            // 统一处理所有数据类型
            Process_All_Data_Types(packet_data, data_len, link_id);
        }
        
        current_ptr = packet_data + data_len;
        processed_count++;
    }
    
    // 立即清空缓冲区
    esp8266_rx_index = 0;
    
    // 定期清理过时信息
    static uint32_t last_cleanup_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_cleanup_time > 10000) {
        Cleanup_Old_Car_Info();
        last_cleanup_time = current_time;
    }
}

// 处理广播小车数据格式数据（新格式：去掉电压数据）
void Process_Compact_Broadcast(const char* data)
{    
    static uint32_t last_update_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 检查数据起始和结束标记
    if(data[0] != '[') {
        return;
    }
    
    char* end_bracket = strchr(data, ']');
    if(end_bracket == NULL) {
        return;
    }
    
    // 跳过开头的'['
    const char* ptr = data + 1;
    
    // 解析小车数量
    int car_count = 0;
    if (sscanf(ptr, "%d", &car_count) != 1) {
        return;
    }
    
    if (car_count <= 0 || car_count > MAX_OTHER_CARS) {
        return;
    }
    
    // 移动到数量后的空格
    ptr = strchr(ptr, ' ');
    if (ptr == NULL) {
        return;
    }
    ptr++; // 跳过空格
    
    // 解析每辆小车的数据
    int processed_cars = 0;
    for (int i = 0; i < car_count; i++) {
        char short_id[4] = {0};  // C1, C2等
        float pos_x = 0, pos_y = 0;
        float heading = 0;
        float vx = 0, vy = 0, vz = 0;
        
        // 解析单个小车数据
        int parsed = sscanf(ptr, "%3s %f %f %f %f %f %f", 
                           short_id, &pos_x, &pos_y, &heading, 
                           &vx, &vy, &vz);
        
        if (parsed == 7) {
            // 转换短ID为完整ID
            char car_id[16];
            snprintf(car_id, sizeof(car_id), "CAR%c", short_id[1]);
            
            // 跳过自己的信息
            if (strcmp(car_id, CAR_ID) != 0) {
                // 拓扑过滤
                if(Should_Process_Car_Info(car_id)) {
                    // 更新其他小车信息
                    Update_Other_Car_Info(car_id, pos_x, pos_y, vx, vy, vz, heading);
                    processed_cars++;
                }
            }
            
            // 移动到下一辆小车数据
            for (int j = 0; j < 7; j++) {
                ptr = strchr(ptr, ' ');
                if (ptr == NULL) break;
                ptr++;
            }
            
            if (ptr == NULL || *ptr == '\0' || *ptr == ']') {
                break;
            }
        } else {
            break;
        }
    }
    
    // 只有在成功更新了至少一辆小车信息后才计算时间差
    if (processed_cars > 0 && last_update_time > 0) {
        uint32_t time_diff = current_time - last_update_time;
        
        char time_msg[32];
        snprintf(time_msg, sizeof(time_msg), "[更新间隔] %lums\n", time_diff);
        debug_print(time_msg);
    }
    
    // 更新最后更新时间
    if (processed_cars > 0) {
        last_update_time = current_time;
    }
}

// 处理广播数据
void Process_Broadcast_Data(const char* data)
{
    // 首先检查是否为新的精简格式指令 [字母,...]
    if (data[0] == '[' && strchr(data, ']') != NULL) {
        // 检查是否为指令（控制、编队、拓扑）
        if (data[1] == 'C' || data[1] == 'F' || data[1] == 'T') {
            // 这是精简格式的指令
            char cmd_type = data[1];
            
            switch(cmd_type) {
                case 'C': // 控制指令 [C,...]
                    debug_print("[广播] 检测到控制指令，转发给控制处理函数\r\n");
                    Process_Control_Command(data);
                    break;
                case 'F': // 编队指令 [F,...]
                    debug_print("[广播] 检测到编队指令，转发给编队处理函数\r\n");
                    Process_Formation_Command(data);
                    break;
                case 'T': // 拓扑指令 [T,...]
                    debug_print("[广播] 检测到拓扑指令，转发给拓扑处理函数\r\n");
                    Process_Topology_Command(data);
                    break;
                default:
                    debug_print("[广播] 未知精简指令格式\r\n");
                    break;
            }
            return;
        } else {
            // 这是广播小车数据格式 [数字 ...]
            Process_Compact_Broadcast(data);
            return;
        }
    }
    
    // 兼容旧格式的指令检查
    // 检查是否为拓扑指令
    if (strstr(data, "TOPOLOGY") != NULL) {
        debug_print("[广播] 检测到拓扑指令(旧格式)，转发给拓扑处理函数\r\n");
        Process_Topology_Command(data);
        return;
    }
    
    // 检查是否为编队指令
    if (strstr(data, "FORMATION") != NULL) {
        debug_print("[广播] 检测到编队指令(旧格式)，转发给编队处理函数\r\n");
        Process_Formation_Command(data);
        return;
    }
    
    // 检查是否为控制指令
    if (strstr(data, "CTRL:") != NULL) {
        debug_print("[广播] 检测到控制指令(旧格式)，转发给控制处理函数\r\n");
        Process_Control_Command(data);
        return;
    }
    
    // 如果都不是，尝试作为未知格式处理
    debug_print("[广播] 未知广播数据格式\r\n");
    
    // 最后尝试通用处理
    if (data[0] == '[') {
        // 可能是广播数据但格式不标准
        Process_Compact_Broadcast(data);
    }
}

// 处理单播数据（控制指令、编队指令、拓扑指令等）
void Process_Unicast_Data(const char* data)
{
    // 检查是否为精简格式 [字母,...]
    if(data[0] == '[') {
        char cmd_type = data[1]; // 第一个字母代表指令类型
        
        switch(cmd_type) {
            case 'C': // 控制指令 [C,...]
                Process_Control_Command(data);
                break;
            case 'F': // 编队指令 [F,...]
                Process_Formation_Command(data);
                break;
            case 'T': // 拓扑指令 [T,...]
                Process_Topology_Command(data);
                break;
            default:
                // 未知指令类型
                break;
        }
        return;
    }
}

// 控制命令处理函数 - 确保与wifi_task.c中的定义一致
void Process_Control_Command(const char* command)
{
    if(command[0] == '[' && strstr(command, "C,") != NULL) {
        char target_car[16];
        float target_x, target_y, target_yaw;
        
        // 解析格式: [C,CAR1,1.5,2.3,45.0]
        if(sscanf(command, "[C,%[^,],%f,%f,%f", 
                  target_car, &target_x, &target_y, &target_yaw) == 4) {
            
            // 检查是否是发给本车的命令
            if(strcmp(target_car, CAR_ID) == 0) {
                Target_position[0] = target_x;
                Target_position[1] = target_y;
                Target_Yaw = target_yaw;
                newCoordinateReceived = 1;
                Auto_mode = 1;
                position_reached = 0;
            }
        }
        return;
    }
}

// 处理多包数据（+IPD 格式）
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
                    
                    // 在多包处理函数中，修改数据处理部分：
                    if (link_id == 0) {
                        // 单播数据：可能是控制指令或编队指令
                        Process_Unicast_Data(temp_packet);
                    } else if (link_id == 1) {
                        // 广播数据：可能是广播数据或广播指令                        
                        Process_Broadcast_Data(temp_packet);
                    }
                    
                    processed_packets++;
                    
                    // 移动到下一个包
                    current_ptr = packet_data_start + data_len;
                    remaining_length = data_length - (current_ptr - data_start);
                    continue;
                } 
            } 
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
    
}

//  处理编队指令
void Process_Formation_Command(const char* command)
{
    char debug_msg[256];
    
    // // 首先打印接收到的原始指令
    // snprintf(debug_msg, sizeof(debug_msg), 
    //          "[编队] 收到指令: %s\r\n", command);
    // debug_print(debug_msg);
    
    // 检查新格式 [F,...]
    if(command[0] == '[' && strstr(command, "F,") != NULL) {
        char cmd_type[2];
        char leader_id[16];
        char formation_type[20];
        float offset_x, offset_y, offset_yaw;
        
        // 解析指令类型
        if(sscanf(command, "[F,%1s", cmd_type) == 1) {
            switch(cmd_type[0]) {
                case 'T': // 停止编队 [F,T]
                    Formation_mode = FORMATION_MODE_NONE;
                    Formation_leader[0] = '\0';
                    debug_print("[编队] 停止编队控制\r\n");
                    break;
                    
                case 'S': // 开始编队 [F,S,CAR1,line]
                    if(sscanf(command, "[F,S,%[^,],%s", leader_id, formation_type) == 2) {
                        debug_print("[编队] 收到开始编队指令\r\n");
                        // 这里可以记录队形类型，等待后续角色指令
                    }
                    break;
                    
                case 'L': // 设置领航者 [F,L,CAR1]
                    if(sscanf(command, "[F,L,%s", leader_id) == 1) {
                        Formation_mode = FORMATION_MODE_LEADER;
                        strcpy(Formation_leader, CAR_ID);
                        Auto_mode = 1;
                        snprintf(debug_msg, sizeof(debug_msg), 
                                 "[编队] 设置为领航者\r\n");
                        debug_print(debug_msg);
                    }
                    break;
                    
                case 'F': // 设置跟随者 [F,F,CAR1,0.5,0.0,0.0]
                    if(sscanf(command, "[F,F,%[^,],%f,%f,%f", 
                               leader_id, &offset_x, &offset_y, &offset_yaw) == 4) {
                        Formation_mode = FORMATION_MODE_FOLLOWER;
                        strcpy(Formation_leader, leader_id);
                        Formation_offset_x = offset_x;
                        Formation_offset_y = offset_y;
                        Formation_offset_yaw = offset_yaw;
                        Auto_mode = 1;
                        
                        snprintf(debug_msg, sizeof(debug_msg), 
                                 "[编队] 设置为跟随者，领航者:%s 偏移(%.2f,%.2f,%.1f)\r\n", 
                                 leader_id, offset_x, offset_y, offset_yaw);
                        debug_print(debug_msg);
                    }
                    break;
                    
                case 'U': // 更新偏移 [F,U,CAR1,0.3,0.2,0.0]
                    if(sscanf(command, "[F,U,%[^,],%f,%f,%f", 
                               leader_id, &offset_x, &offset_y, &offset_yaw) == 4) {
                        // 检查是否是本车跟随的领航者
                        if(Formation_mode == FORMATION_MODE_FOLLOWER && 
                           strcmp(leader_id, Formation_leader) == 0) {
                            Formation_offset_x = offset_x;
                            Formation_offset_y = offset_y;
                            Formation_offset_yaw = offset_yaw;
                            
                            snprintf(debug_msg, sizeof(debug_msg), 
                                     "[编队] 更新偏移量 (%.2f,%.2f,%.1f)\r\n", 
                                     offset_x, offset_y, offset_yaw);
                            debug_print(debug_msg);
                        }
                    }
                    break;
                    
                default:
                    snprintf(debug_msg, sizeof(debug_msg), 
                             "[编队] 未知编队指令类型: %c\r\n", cmd_type[0]);
                    debug_print(debug_msg);
                    break;
            }
        }
        return;
    }
    
}

// 综合数据处理函数 - 统一处理所有类型的数据
void Process_All_Data_Types(const char* data, int data_len, int link_id)
{
    // 快速检测数据格式并分发处理
    if (data_len < 2) return;
    
    // 检测是否为精简格式指令 [字母,...]
    if (data[0] == '[' && data[2] == ',') {
        switch(data[1]) {
            case 'C': // 控制指令 [C,...]
                Process_Control_Command(data);
                break;
            case 'F': // 编队指令 [F,...]
                Process_Formation_Command(data);
                break;
            case 'T': // 拓扑指令 [T,...]
                Process_Topology_Command(data);
                break;
            default:
                // 未知指令类型
                break;
        }
        return;
    }
    
    // 检测是否为广播小车数据格式 [数字 ...]
    if (data[0] == '[' && data[1] >= '0' && data[1] <= '9') {
        Process_Compact_Broadcast_Optimized(data);
        return;
    }
    
    // 如果是广播连接，尝试其他格式
    if (link_id == 1) {
        // 尝试旧格式指令
        if (strstr(data, "TOPOLOGY") != NULL) {
            Process_Topology_Command(data);
        } else if (strstr(data, "FORMATION") != NULL) {
            Process_Formation_Command(data);
        } else if (strstr(data, "CTRL:") != NULL) {
            Process_Control_Command(data);
        }
    }
}

// 优化后的广播数据处理 - 添加间隔打印
void Process_Compact_Broadcast_Optimized(const char* data)
{    
    // 检查数据起始和结束标记
    if(data[0] != '[') return;
    
    char* end_bracket = strchr(data, ']');
    if(end_bracket == NULL) return;
    
    // 跳过开头的'['
    const char* ptr = data + 1;
    
    // 解析小车数量
    int car_count = 0;
    if (sscanf(ptr, "%d", &car_count) != 1) return;
    
    if (car_count <= 0 || car_count > MAX_OTHER_CARS) return;
    
    // 移动到数量后的空格
    ptr = strchr(ptr, ' ');
    if (ptr == NULL) return;
    ptr++; // 跳过空格
    
    // 解析每辆小车的数据
    int processed_cars = 0;
    for (int i = 0; i < car_count; i++) {
        char short_id[4] = {0};
        float values[6]; // pos_x, pos_y, heading, vx, vy, vz
        
        // 一次性解析所有字段
        int parsed = sscanf(ptr, "%3s %f %f %f %f %f %f", 
                           short_id, &values[0], &values[1], &values[2],
                           &values[3], &values[4], &values[5]);
        
        if (parsed == 7) {
            // 转换短ID为完整ID
            char car_id[16];
            snprintf(car_id, sizeof(car_id), "CAR%c", short_id[1]);
            
            // 跳过自己的信息
            if (strcmp(car_id, CAR_ID) != 0) {
                // 拓扑过滤
                if(Should_Process_Car_Info(car_id)) {
                    // 更新其他小车信息（内部会打印间隔）
                    Update_Other_Car_Info(car_id, values[0], values[1], 
                                        values[3], values[4], values[5], values[2]);
                    processed_cars++;
                }
            }
            
            // 快速移动到下一组数据
            for (int j = 0; j < 7; j++) {
                ptr = strchr(ptr, ' ');
                if (ptr == NULL) break;
                ptr++;
            }
            
            if (ptr == NULL || *ptr == '\0' || *ptr == ']') {
                break;
            }
        } else {
            break;
        }
    }
}
