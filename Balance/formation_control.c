#include "formation_control.h"
#include "balance.h"
#include <math.h>
#include <string.h>

// 编队控制变量
uint8_t Formation_mode = 0;           // 编队模式：0-无编队，1-领航者，2-跟随者
char Formation_leader[10] = "";       // 领航者ID
float Formation_offset_x = 0.0f;      // X方向偏移
float Formation_offset_y = 0.0f;      // Y方向偏移  
float Formation_offset_yaw = 0.0f;    // 航向偏移

// 编队控制参数 - 基于你的速度数据调整
float Formation_KP = 0.5f;            // 降低比例系数，减少超调 0.5
float Formation_KD = 1.8f;            // 增加微分系数，抑制超调 1.2
float Formation_max_speed = 0.3f;     // 降低最大速度
float Formation_tolerance = 0.15f;    // 增加容差，减少振荡 0.15
float Formation_min_speed = 0.05f;    // 降低最小速度

// 新增运动状态检测参数
#define VELOCITY_THRESHOLD 0.05f          // 速度阈值，小于此值认为静止
#define ZERO_VELOCITY_COUNT_THRESHOLD 4  // 连续零速度计数阈值
#define MOVING_AVERAGE_SIZE 1             // 移动平均窗口大小

// 运动状态检测变量
static uint8_t zero_velocity_count = 0;
static float velocity_history_vx[MOVING_AVERAGE_SIZE] = {0};
static float velocity_history_vy[MOVING_AVERAGE_SIZE] = {0};
static uint8_t velocity_history_index = 0;
// 控制状态变量
static float prev_error_x = 0.0f, prev_error_y = 0.0f;

// 新增混合控制参数
float Formation_speed_follow_threshold = 0.3f;  // 速度跟随阈值
float Formation_velocity_gain = 1.05f;            // 速度跟随增益

// 新增控制状态
static uint8_t speed_following_mode = 0;  // 速度跟随模式标志


/**************************************************************************
Function: 编队控制处理
Input   : 无
Output  : 无
函数功能：处理编队控制逻辑
**************************************************************************/
void Formation_Control(void)
{
    if (Formation_mode == FORMATION_MODE_NONE) {
        return;
    }
    
    if (Formation_mode == FORMATION_MODE_LEADER) {
        if(Auto_mode && newCoordinateReceived) {
            Auto_Adjust_Position_And_Yaw();
        }
        else if(APP_ON_Flag) {
            Get_RC();
        }
        else {
            Drive_Motor(0, 0, 0);
        }
        return;
    }
    
    if (Formation_mode == FORMATION_MODE_FOLLOWER) {
        Formation_Follower_Control();
    }
}



/**************************************************************************
Function: 检测领航者运动状态
Input   : 领航者速度vx, vy
Output  : 运动状态 (1=移动, 0=静止)
函数功能：通过连续速度数据判断领航者是否在移动
**************************************************************************/
uint8_t Detect_Leader_Motion_State(float vx, float vy)
{
    static uint8_t initialized = 0;
    
    // 初始化速度历史
    if (!initialized) {
        for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
            velocity_history_vx[i] = vx;
            velocity_history_vy[i] = vy;
        }
        initialized = 1;
    }
    
    // 更新速度历史
    velocity_history_vx[velocity_history_index] = vx;
    velocity_history_vy[velocity_history_index] = vy;
    velocity_history_index = (velocity_history_index + 1) % MOVING_AVERAGE_SIZE;
    
    // 计算移动平均速度
    float avg_vx = 0, avg_vy = 0;
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        avg_vx += velocity_history_vx[i];
        avg_vy += velocity_history_vy[i];
    }
    avg_vx /= MOVING_AVERAGE_SIZE;
    avg_vy /= MOVING_AVERAGE_SIZE;
    
    // 计算速度幅值
    float speed_magnitude = sqrtf(avg_vx * avg_vx + avg_vy * avg_vy);
    
    // 判断运动状态
    if (speed_magnitude > VELOCITY_THRESHOLD) {
        zero_velocity_count = 0;  // 重置零速度计数
        return 1;  // 移动状态
    } else {
        zero_velocity_count++;    // 增加零速度计数
        if (zero_velocity_count >= ZERO_VELOCITY_COUNT_THRESHOLD) {
            return 0;  // 确认静止状态
        } else {
            return 1;  // 暂时还认为是移动状态（防止误判）
        }
    }
}

/**************************************************************************
Function: 自适应编队控制
Input   : 无
Output  : 无
函数功能：根据领航者运动状态自适应切换控制模式
**************************************************************************/
void Formation_Follower_Control(void)
{
    // 查找领航者信息
    OtherCarInfo* leader_info = NULL;
    for (int i = 0; i < MAX_OTHER_CARS; i++) {
        if (other_cars[i].valid && strcmp(other_cars[i].car_id, Formation_leader) == 0) {
            leader_info = &other_cars[i];
            break;
        }
    }
    
    if (leader_info == NULL) {
        Drive_Motor(0, 0, 0);
        prev_error_x = 0;
        prev_error_y = 0;
        zero_velocity_count = 0;
        return;
    }
    
    // 检查数据是否超时（2秒超时）
    if (HAL_GetTick() - leader_info->last_update > 2000) {
        Drive_Motor(0, 0, 0);
        prev_error_x = 0;
        prev_error_y = 0;
        zero_velocity_count = 0;
        return;
    }
    
    // 简单的死区处理
    float leader_vx = leader_info->velocity_vx;
    float leader_vy = leader_info->velocity_vy;
    
    if (fabsf(leader_vx) < 0.04f) leader_vx = 0.0f;
    if (fabsf(leader_vy) < 0.04f) leader_vy = 0.0f;

    // 检测领航者运动状态
    uint8_t leader_moving = Detect_Leader_Motion_State(leader_vx, leader_vy);
    
    // 计算在领航者坐标系下的目标位置
    float leader_rad = leader_info->yaw * PI / 180.0f;
    
    float target_x_world = leader_info->position_x + 
                          Formation_offset_x * cosf(leader_rad) - 
                          Formation_offset_y * sinf(leader_rad);
    
    float target_y_world = leader_info->position_y + 
                          Formation_offset_x * sinf(leader_rad) + 
                          Formation_offset_y * cosf(leader_rad);
    
    float target_yaw_world = leader_info->yaw + Formation_offset_yaw;
    while (target_yaw_world >= 360.0f) target_yaw_world -= 360.0f;
    while (target_yaw_world < 0.0f)    target_yaw_world += 360.0f;
    
    // 计算位置误差
    float error_x = target_x_world - position[0];
    float error_y = target_y_world - position[1];
    float error_yaw = target_yaw_world - Yaw;
    
    // 规范化航向误差
    while (error_yaw > 180.0f) error_yaw -= 360.0f;
    while (error_yaw < -180.0f) error_yaw += 360.0f;
    
    // 计算距离
    float distance = sqrtf(error_x * error_x + error_y * error_y);
    
    float control_vx, control_vy; 
    
    // ========== 改进的自适应控制策略 ==========
    
    // 1. 改进的模式切换逻辑：综合考虑领航者状态和位置误差
    uint8_t should_use_speed_follow = 0;
    
    if (leader_moving) {
        // 领航者在移动时，根据位置误差决定模式
        if (distance < Formation_tolerance * 2.0f) {
            // 位置误差较小：使用速度跟随模式
            should_use_speed_follow = 1;
        } else if (distance < Formation_tolerance * 3.0f) {
            // 中等误差：根据误差方向决定
            // 如果误差方向与领航者运动方向一致，使用速度跟随
            float leader_speed_angle = atan2f(leader_vy, leader_vx);
            float error_angle = atan2f(error_y, error_x);
            float angle_diff = fabsf(leader_speed_angle - error_angle);
            if (angle_diff < PI/4.0f || angle_diff > 7.0f*PI/4.0f) {
                should_use_speed_follow = 1;
            }
        }
        // 大误差情况下保持位置控制模式
    }
    // 领航者静止时始终使用位置控制模式
    
    if (should_use_speed_follow) {
        // ========== 速度跟随模式 ==========
        // 领航者在移动且位置误差可控：主要执行速度跟随
        
        control_vx = leader_vx * Formation_velocity_gain;
        control_vy = leader_vy * Formation_velocity_gain;
        
        // 朝向差异补偿
        float yaw_difference = leader_info->yaw - Yaw;
        while (yaw_difference > 180.0f) yaw_difference -= 360.0f;
        while (yaw_difference < -180.0f) yaw_difference += 360.0f;
        
        float yaw_diff_rad = yaw_difference * PI / 180.0f;
        
        if (fabsf(yaw_difference) > 10.0f) {
            float rotated_vx = control_vx * cosf(yaw_diff_rad) - control_vy * sinf(yaw_diff_rad);
            float rotated_vy = control_vx * sinf(yaw_diff_rad) + control_vy * cosf(yaw_diff_rad);
            control_vx = rotated_vx;
            control_vy = rotated_vy;
        }
        
        // 2. 改进的位置修正策略：渐进式动态修正
        if (distance > Formation_tolerance) {
            // 动态计算修正增益：误差越大，修正越强
            float base_gain = 0.05f;  // 基础修正增益
            float dynamic_gain = 0.0f;
            
            // 根据距离误差动态调整增益
            if (distance > Formation_tolerance && distance < 1.0f) {
                dynamic_gain = (distance - Formation_tolerance) * 0.1f;
            } else if (distance >= 1.0f) {
                dynamic_gain = 0.15f;  // 最大修正增益
            }
            
            float total_position_gain = base_gain + dynamic_gain;
            
            // 限制最大修正速度，避免与速度跟随冲突
            float max_correction_speed = Formation_max_speed * 0.4f;
            
            // 计算修正向量
            float error_rad = atan2f(error_y, error_x);
            float error_angle_relative = error_rad - (Yaw * PI / 180.0f);
            
            float correction_vx = total_position_gain * distance * cosf(error_angle_relative);
            float correction_vy = total_position_gain * distance * sinf(error_angle_relative);
            
            // 限制修正速度
            float correction_speed = sqrtf(correction_vx * correction_vx + correction_vy * correction_vy);
            if (correction_speed > max_correction_speed) {
                correction_vx = correction_vx * max_correction_speed / correction_speed;
                correction_vy = correction_vy * max_correction_speed / correction_speed;
            }
            
            // 应用位置修正
            control_vx += correction_vx;
            control_vy += correction_vy;
        }
        
    } else {
        // ========== 位置控制模式 ==========
        // 领航者静止或位置误差较大：执行位置校正
        
        // 计算误差变化率
        float error_derivative_x = error_x - prev_error_x;
        float error_derivative_y = error_y - prev_error_y;
        
        // PD控制
        control_vx = Formation_KP * error_x + Formation_KD * error_derivative_x;
        control_vy = Formation_KP * error_y + Formation_KD * error_derivative_y;
        
        // 接近目标时的减速 - 改进的减速曲线
        float distance_factor = 1.0f;
        if (distance < 0.5f) {
            // 更平滑的减速
            distance_factor = 0.2f + 0.8f * (distance / 0.5f);
        }
        
        control_vx *= distance_factor;
        control_vy *= distance_factor;
        
        // 坐标系转换
        float control_angle_global = atan2f(control_vy, control_vx);
        float control_angle_relative = control_angle_global - (Yaw * PI / 180.0f);
        float control_magnitude = sqrtf(control_vx * control_vx + control_vy * control_vy);
        
        control_vx = control_magnitude * cosf(control_angle_relative);
        control_vy = control_magnitude * sinf(control_angle_relative);
    }
    
    // ========== 公共控制处理 ==========
    
    // 速度限制
    float control_speed = sqrtf(control_vx * control_vx + control_vy * control_vy);
    if (control_speed > Formation_max_speed) {
        control_vx = control_vx * Formation_max_speed / control_speed;
        control_vy = control_vy * Formation_max_speed / control_speed;
    }
    
    // 航向控制 - 根据模式调整增益
    float yaw_gain = should_use_speed_follow ? 0.8f : 1.2f;
    float yaw_control = Yaw_PID_Control(Yaw, target_yaw_world) * yaw_gain;
    
    // 输出控制
    Drive_Motor(control_vx, control_vy, yaw_control);
    
    // 保存误差
    prev_error_x = error_x;
    prev_error_y = error_y;
}

/**************************************************************************
Function: 处理编队指令
Input   : 指令字符串
Output  : 无
函数功能：解析和执行编队控制指令
**************************************************************************/
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

/**************************************************************************
Function: 处理编队更新指令
Input   : 指令字符串
Output  : 无
函数功能：解析和执行编队更新指令
**************************************************************************/
void Process_Formation_Update(const char* command)
{
    char leader_id[10];
    float offset_x, offset_y, offset_yaw;
    
    if (sscanf(command, "FORMATION:UPDATE,%[^,],%f,%f,%f", 
               leader_id, &offset_x, &offset_y, &offset_yaw) == 4) {
        
        // 检查是否是本车跟随的领航者
        if (Formation_mode == FORMATION_MODE_FOLLOWER && strcmp(leader_id, Formation_leader) == 0) {
            Formation_offset_x = offset_x;
            Formation_offset_y = offset_y;
            Formation_offset_yaw = offset_yaw;
            
            char debug_msg[128];
            snprintf(debug_msg, sizeof(debug_msg), 
                     "[编队] 更新偏移量 (%.2f,%.2f,%.1f)\r\n", 
                     offset_x, offset_y, offset_yaw);
            usart1_send_cstring(debug_msg);
        }
    }
}