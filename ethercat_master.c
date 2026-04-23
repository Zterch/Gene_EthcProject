/*
 * EtherCAT 驱动器控制程序 (修正版)
 * 根据实际编码器参数修正单位转换
 * 电机编码器每圈: 2^20 = 1048576 码值
 * 减速比: 101:1
 * 关节每圈: 1048576 × 101 = 105,906,176 码值
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <ecrt.h>

// ===================== 配置参数 =====================
#define VENDOR_ID        0x00010203
#define PRODUCT_ID       0x00000402
#define SLAVE_POSITION   0
#define FREQUENCY        500
#define PERIOD_NS        2000000
#define TEST_DURATION    120

// 编码器参数 - 根据您的实际情况调整
#define MOTOR_ENCODER_COUNTS_PER_REV   1048576  // 电机每转码值 (2^20)
#define GEAR_RATIO                     101.0f   // 减速比
#define JOINT_ENCODER_COUNTS_PER_REV   (MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO)  // 关节每转码值

// 速度模式参数
#define TARGET_VELOCITY_DEG_PER_SEC    -10.0f    // 目标角速度 10度/秒

// DC时钟同步参数
#define DC_SYNC0_CYCLE_TIME  2000000
#define DC_ASSIGN_ACTIVATE   0x0300

// 运动控制参数
#define MOTION_TEST_WAIT_TIME    1000
#define MOTION_RAMP_UP_TIME      2000
#define MOTION_RUN_TIME          5000
#define MOTION_RAMP_DOWN_TIME    2000
#define MOTION_STABILIZE_TIME    1000

// ===================== PDO条目结构定义 =====================
typedef struct {
    unsigned int byte_offset;
    unsigned int bit_position;
    int size_bits;
    const char *name;
} pdo_entry_t;

// ===================== 状态定义 =====================
typedef enum {
    DRIVE_STATE_NOT_READY = 0x0000,
    DRIVE_STATE_SWITCH_ON_DISABLED = 0x0040,
    DRIVE_STATE_READY_TO_SWITCH_ON = 0x0021,
    DRIVE_STATE_SWITCHED_ON = 0x0023,
    DRIVE_STATE_OPERATION_ENABLED = 0x0027,
    DRIVE_STATE_QUICK_STOP_ACTIVE = 0x0007,
    DRIVE_STATE_FAULT = 0x0008,
    DRIVE_STATE_KDE_READY = 0x0A31,
    DRIVE_STATE_KDE_RUNNING = 0x0A37,
    DRIVE_STATE_KDE_READY_FOR_MOTION = 0x0A33,
    DRIVE_STATE_KDE_FAULT = 0x0A18,
    DRIVE_STATE_KDE_SWITCH_DISABLED = 0x0A50,
    DRIVE_STATE_UNKNOWN = 0xFFFF
} drive_state_t;

typedef enum {
    CMD_SHUTDOWN = 0x0006,
    CMD_SWITCH_ON = 0x0007,
    CMD_ENABLE_OPERATION = 0x000F,
    CMD_ENABLE_OPERATION_NEW_SETPOINT = 0x001F,
    CMD_DISABLE_VOLTAGE = 0x0000,
    CMD_FAULT_RESET = 0x0080
} control_word_t;

// ===================== 全局变量 =====================
static volatile int running = 1;
static int operational = 0;
static int motion_started = 0;
static unsigned long cycle_counter = 0;

static ec_master_t *master = NULL;
static ec_domain_t *domain = NULL;
static ec_slave_config_t *slave_config = NULL;
static uint8_t *domain_data = NULL;

// PDO 条目
static pdo_entry_t pdo_control_word = {0, 0, 16, "控制字(0x6040)"};
static pdo_entry_t pdo_target_position = {0, 0, 32, "目标位置(0x607a)"};
static pdo_entry_t pdo_target_velocity = {0, 0, 32, "目标速度(0x60ff)"};
static pdo_entry_t pdo_target_torque = {0, 0, 16, "目标扭矩(0x6071)"};
static pdo_entry_t pdo_status_word = {0, 0, 16, "状态字(0x6041)"};
static pdo_entry_t pdo_actual_position = {0, 0, 32, "实际位置(0x6064)"};
static pdo_entry_t pdo_actual_velocity = {0, 0, 32, "实际速度(0x606c)"};
static pdo_entry_t pdo_actual_torque = {0, 0, 16, "实际扭矩(0x6077)"};
static pdo_entry_t pdo_following_error = {0, 0, 32, "跟随误差(0x60f4)"};
static pdo_entry_t pdo_error_code = {0, 0, 16, "错误代码(0x603f)"};

// 状态机
static int target_state = 0;
static uint32_t state_wait_start = 0;
static int state_timeout = 0;
static int retry_count = 0;
static const int MAX_RETRIES = 5;

// 运动控制
static float current_velocity_deg_per_sec = 0.0f;  // 当前角速度（度/秒）
static int32_t current_target_velocity = 0;        // 当前目标速度（码值/秒）
static uint16_t current_control_word = 0x0000;

// 位置监控
static int32_t last_actual_position = 0;
static int32_t position_change = 0;
static int position_monitoring_started = 0;
static int32_t initial_position = 0;

// 控制字历史记录
static uint16_t last_control_word = 0x0000;
static int32_t last_target_velocity = 0;

// 主站状态
static int master_in_op = 0;
static int slave_in_op = 0;

// DC时钟同步
static uint64_t app_time = 0;
static int dc_configured = 0;

// PDO初始化标志
static int pdo_initialized = 0;

// 运动控制状态
static enum {
    MOTION_IDLE = 0,
    MOTION_WAITING,
    MOTION_ACCELERATING,
    MOTION_RUNNING,
    MOTION_DECELERATING,
    MOTION_STOPPING,
    MOTION_COMPLETED
} motion_state = MOTION_IDLE;

static uint32_t motion_start_time = 0;
static float motion_elapsed_time = 0.0f;
static int motion_test_completed = 0;

// 调试标志
static int debug_enabled = 1;

// ===================== 单位转换函数 =====================
// 码值转换为关节角度（度）
float counts_to_degrees(int32_t counts) {
    return (float)counts * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
}

// 关节角度（度）转换为码值
int32_t degrees_to_counts(float degrees) {
    return (int32_t)(degrees * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
}

// 角度/秒转换为编码器码值/秒（关节端）
int32_t deg_per_sec_to_internal_velocity(float deg_per_sec) {
    // 角度/秒 -> 码值/秒
    // 1. 角度/秒 -> 转/秒: deg_per_sec / 360.0
    // 2. 转/秒 -> 码值/秒: (deg_per_sec / 360.0) * JOINT_ENCODER_COUNTS_PER_REV
    
    float revs_per_sec = deg_per_sec / 360.0f;
    float counts_per_sec = revs_per_sec * JOINT_ENCODER_COUNTS_PER_REV;
    
    if (debug_enabled && fabs(deg_per_sec) > 0.001f) {
        printf("  单位转换: %.3f 度/秒 -> %.6f 转/秒 -> %.0f 码值/秒\n", 
               deg_per_sec, revs_per_sec, counts_per_sec);
    }
    
    return (int32_t)counts_per_sec;
}

// RPM转换为编码器码值/秒（电机端）
int32_t rpm_to_internal_velocity(float rpm) {
    // RPM（电机转/分）-> 码值/秒（电机端）
    // 1. 转/分 -> 转/秒: rpm / 60.0
    // 2. 转/秒 -> 码值/秒: (rpm / 60.0) * MOTOR_ENCODER_COUNTS_PER_REV
    
    float revs_per_sec = rpm / 60.0f;
    float counts_per_sec = revs_per_sec * MOTOR_ENCODER_COUNTS_PER_REV;
    
    if (debug_enabled && fabs(rpm) > 0.001f) {
        printf("  单位转换: %.3f RPM -> %.6f 转/秒 -> %.0f 码值/秒\n", 
               rpm, revs_per_sec, counts_per_sec);
    }
    
    return (int32_t)counts_per_sec;
}

// 码值/秒转换为关节角度/秒
float internal_velocity_to_deg_per_sec(int32_t internal_vel) {
    // 码值/秒 -> 关节角度/秒
    // 1. 码值/秒 -> 转/秒: internal_vel / JOINT_ENCODER_COUNTS_PER_REV
    // 2. 转/秒 -> 角度/秒: (internal_vel / JOINT_ENCODER_COUNTS_PER_REV) * 360.0
    
    float revs_per_sec = (float)internal_vel / JOINT_ENCODER_COUNTS_PER_REV;
    float deg_per_sec = revs_per_sec * 360.0f;
    return deg_per_sec;
}

// 码值/秒转换为RPM（电机端）
float internal_velocity_to_rpm(int32_t internal_vel) {
    // 码值/秒 -> RPM（电机）
    // 1. 码值/秒 -> 转/秒（电机）: internal_vel / MOTOR_ENCODER_COUNTS_PER_REV
    // 2. 转/秒 -> 转/分: (internal_vel / MOTOR_ENCODER_COUNTS_PER_REV) * 60.0
    
    float revs_per_sec = (float)internal_vel / MOTOR_ENCODER_COUNTS_PER_REV;
    float rpm = revs_per_sec * 60.0f;
    return rpm;
}

// ===================== 信号处理 =====================
void signal_handler(int sig) {
    printf("\n收到信号 %d, 正在停止运动...\n", sig);
    running = 0;
}

// ===================== PDO 配置 =====================
static ec_pdo_entry_info_t slave_output_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60ff, 0x00, 32},
    {0x6071, 0x00, 16},
};

static ec_pdo_entry_info_t slave_input_pdo_entries[] = {
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x60f4, 0x00, 32},
    {0x603f, 0x00, 16},
};

static ec_pdo_info_t slave_pdos[] = {
    {0x1601, 4, slave_output_pdo_entries},
    {0x1A01, 6, slave_input_pdo_entries},
};

static ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DISABLE}
};

// ===================== 读写函数 =====================
uint16_t read_status_word() {
    if (!domain_data) return 0;
    uint16_t status;
    memcpy(&status, domain_data + pdo_status_word.byte_offset, 2);
    return status;
}

int32_t read_actual_velocity() {
    if (!domain_data) return 0;
    int32_t velocity;
    memcpy(&velocity, domain_data + pdo_actual_velocity.byte_offset, 4);
    return velocity;
}

int32_t read_actual_position() {
    if (!domain_data) return 0;
    int32_t position;
    memcpy(&position, domain_data + pdo_actual_position.byte_offset, 4);
    return position;
}

uint16_t read_error_code() {
    if (!domain_data) return 0;
    uint16_t error;
    memcpy(&error, domain_data + pdo_error_code.byte_offset, 2);
    return error;
}

void write_control_word(uint16_t value) {
    if (!domain_data) return;
    
    if (value != last_control_word) {
        printf("   [控制字] 0x%04X\n", value);
        last_control_word = value;
    }
    
    memcpy(domain_data + pdo_control_word.byte_offset, &value, 2);
    current_control_word = value;
}

void write_target_velocity(int32_t velocity) {
    if (!domain_data) return;
    
    if (velocity != last_target_velocity) {
        float deg_per_sec = internal_velocity_to_deg_per_sec(velocity);
        float rpm = internal_velocity_to_rpm(velocity);
        printf("   [目标速度] %d 码值/秒 (%.6f 度/秒, %.3f RPM)\n", 
               velocity, deg_per_sec, rpm);
        last_target_velocity = velocity;
    }
    
    memcpy(domain_data + pdo_target_velocity.byte_offset, &velocity, 4);
    current_target_velocity = velocity;
}

void write_target_position(int32_t position) {
    if (!domain_data) return;
    memcpy(domain_data + pdo_target_position.byte_offset, &position, 4);
}

void write_target_torque(int16_t torque) {
    if (!domain_data) return;
    memcpy(domain_data + pdo_target_torque.byte_offset, &torque, 2);
}

// ===================== 位置监控函数 =====================
void monitor_position_changes() {
    if (!domain_data) return;
    
    int32_t current_position = read_actual_position();
    float current_angle = counts_to_degrees(current_position);
    
    if (!position_monitoring_started) {
        last_actual_position = current_position;
        initial_position = current_position;
        position_monitoring_started = 1;
        printf("  开始位置监控，初始位置: %d 码值 (%.6f 度)\n", 
               initial_position, counts_to_degrees(initial_position));
    } else {
        position_change = current_position - last_actual_position;
        float angle_change = counts_to_degrees(position_change);
        
        // 每0.5秒报告一次位置变化
        if (cycle_counter % (FREQUENCY / 2) == 0) {
            int32_t total_change = current_position - initial_position;
            float total_angle_change = counts_to_degrees(total_change);
            
            printf("  位置变化: 本次 %d 码值 (%.6f 度), 累计 %d 码值 (%.6f 度)\n", 
                   position_change, angle_change, total_change, total_angle_change);
        }
        
        last_actual_position = current_position;
    }
}

// ===================== 调试函数 =====================
void debug_all_pdo_values() {
    if (!domain_data) {
        printf("域数据为空！\n");
        return;
    }
    
    printf("\n=== 所有PDO值详细调试 ===\n");
    
    uint16_t status = read_status_word();
    int32_t actual_vel = read_actual_velocity();
    int32_t actual_pos = read_actual_position();
    uint16_t error = read_error_code();
    
    printf("状态字 (0x6041): 0x%04X\n", status);
    
    float actual_vel_deg_per_sec = internal_velocity_to_deg_per_sec(actual_vel);
    float actual_vel_rpm = internal_velocity_to_rpm(actual_vel);
    printf("实际速度 (0x606c): %d 码值/秒 (%.6f 度/秒, %.3f RPM)\n", 
           actual_vel, actual_vel_deg_per_sec, actual_vel_rpm);
    
    printf("实际位置 (0x6064): %d 码值 (%.6f 度)\n", 
           actual_pos, counts_to_degrees(actual_pos));
    printf("错误代码 (0x603f): 0x%04X\n", error);
    
    // 读取其他PDO值
    int32_t actual_torque = 0;
    memcpy(&actual_torque, domain_data + pdo_actual_torque.byte_offset, 2);
    printf("实际扭矩 (0x6077): %d\n", actual_torque);
    
    int32_t following_error = 0;
    memcpy(&following_error, domain_data + pdo_following_error.byte_offset, 4);
    printf("跟随误差 (0x60f4): %d 码值 (%.6f 度)\n", 
           following_error, counts_to_degrees(following_error));
    
    printf("\n目标PDO值:\n");
    // 读取当前发送的目标值
    uint16_t control_word = 0;
    memcpy(&control_word, domain_data + pdo_control_word.byte_offset, 2);
    printf("控制字 (0x6040): 0x%04X\n", control_word);
    
    int32_t target_pos = 0;
    memcpy(&target_pos, domain_data + pdo_target_position.byte_offset, 4);
    printf("目标位置 (0x607a): %d 码值 (%.6f 度)\n", 
           target_pos, counts_to_degrees(target_pos));
    
    int32_t target_vel = 0;
    memcpy(&target_vel, domain_data + pdo_target_velocity.byte_offset, 4);
    float target_vel_deg_per_sec = internal_velocity_to_deg_per_sec(target_vel);
    float target_vel_rpm = internal_velocity_to_rpm(target_vel);
    printf("目标速度 (0x60ff): %d 码值/秒 (%.6f 度/秒, %.3f RPM)\n", 
           target_vel, target_vel_deg_per_sec, target_vel_rpm);
    
    int16_t target_torque = 0;
    memcpy(&target_torque, domain_data + pdo_target_torque.byte_offset, 2);
    printf("目标扭矩 (0x6071): %d\n", target_torque);
    
    // 打印前48字节的十六进制数据
    printf("\n前48字节原始数据:\n");
    for (int i = 0; i < 48; i += 16) {
        printf("  [0x%02X]: ", i);
        for (int j = 0; j < 16 && i+j < 48; j++) {
            printf("%02X ", domain_data[i+j]);
        }
        printf("\n");
    }
}

void debug_pdo_mapping_details() {
    printf("\n=== PDO映射详细信息 ===\n");
    
    // 输出PDO映射
    printf("输出PDO条目 (0x1601):\n");
    printf("  控制字(0x6040): 偏移=%u, 大小=%d位\n", 
           pdo_control_word.byte_offset, pdo_control_word.size_bits);
    printf("  目标位置(0x607a): 偏移=%u, 大小=%d位\n", 
           pdo_target_position.byte_offset, pdo_target_position.size_bits);
    printf("  目标速度(0x60ff): 偏移=%u, 大小=%d位\n", 
           pdo_target_velocity.byte_offset, pdo_target_velocity.size_bits);
    printf("  目标扭矩(0x6071): 偏移=%u, 大小=%d位\n", 
           pdo_target_torque.byte_offset, pdo_target_torque.size_bits);
    
    printf("\n输入PDO条目 (0x1A01):\n");
    printf("  状态字(0x6041): 偏移=%u, 大小=%d位\n", 
           pdo_status_word.byte_offset, pdo_status_word.size_bits);
    printf("  实际位置(0x6064): 偏移=%u, 大小=%d位\n", 
           pdo_actual_position.byte_offset, pdo_actual_position.size_bits);
    printf("  实际速度(0x606c): 偏移=%u, 大小=%d位\n", 
           pdo_actual_velocity.byte_offset, pdo_actual_velocity.size_bits);
    printf("  实际扭矩(0x6077): 偏移=%u, 大小=%d位\n", 
           pdo_actual_torque.byte_offset, pdo_actual_torque.size_bits);
    printf("  跟随误差(0x60f4): 偏移=%u, 大小=%d位\n", 
           pdo_following_error.byte_offset, pdo_following_error.size_bits);
    printf("  错误代码(0x603f): 偏移=%u, 大小=%d位\n", 
           pdo_error_code.byte_offset, pdo_error_code.size_bits);
    
    // 计算PDO总大小
    int output_pdo_size = (pdo_control_word.size_bits + pdo_target_position.size_bits + 
                          pdo_target_velocity.size_bits + pdo_target_torque.size_bits + 7) / 8;
    int input_pdo_size = (pdo_status_word.size_bits + pdo_actual_position.size_bits + 
                         pdo_actual_velocity.size_bits + pdo_actual_torque.size_bits + 
                         pdo_following_error.size_bits + pdo_error_code.size_bits + 7) / 8;
    
    printf("\nPDO总大小:\n");
    printf("  输出PDO: %d字节\n", output_pdo_size);
    printf("  输入PDO: %d字节\n", input_pdo_size);
}

// ===================== 分析当前问题 =====================
void analyze_current_issue() {
    printf("\n=== 问题分析 ===\n");
    
    int32_t actual_vel = read_actual_velocity();
    int32_t actual_pos = read_actual_position();
    float actual_deg_per_sec = internal_velocity_to_deg_per_sec(actual_vel);
    float actual_rpm = internal_velocity_to_rpm(actual_vel);
    
    printf("当前状态:\n");
    printf("  实际速度: %d 码值/秒 (%.6f 度/秒, %.3f RPM)\n", 
           actual_vel, actual_deg_per_sec, actual_rpm);
    printf("  实际位置: %d 码值 (%.6f 度)\n", 
           actual_pos, counts_to_degrees(actual_pos));
    
    float target_deg_per_sec = internal_velocity_to_deg_per_sec(current_target_velocity);
    float target_rpm = internal_velocity_to_rpm(current_target_velocity);
    printf("  目标速度: %d 码值/秒 (%.6f 度/秒, %.3f RPM)\n", 
           current_target_velocity, target_deg_per_sec, target_rpm);
    
    // 检查编码器参数
    printf("\n编码器参数验证:\n");
    printf("  电机编码器每圈: %d 码值 (2^20)\n", MOTOR_ENCODER_COUNTS_PER_REV);
    printf("  减速比: %.0f:1\n", GEAR_RATIO);
    printf("  关节编码器每圈: %.0f 码值\n", JOINT_ENCODER_COUNTS_PER_REV);
    
    // 检查跟随误差
    int32_t following_error = 0;
    if (domain_data) {
        memcpy(&following_error, domain_data + pdo_following_error.byte_offset, 4);
    }
    printf("  跟随误差: %d 码值 (%.6f 度)\n", 
           following_error, counts_to_degrees(following_error));
    
    printf("\n可能的问题:\n");
    printf("  1. 驱动器未正确使能\n");
    printf("  2. 速度指令太小\n");
    printf("  3. 电机抱闸未打开\n");
    printf("  4. 驱动器模式设置错误\n");
    printf("  5. 跟随误差限制太小\n");
}

// ===================== 状态机函数 =====================
drive_state_t get_drive_state(uint16_t status_word) {
    // 检查KDE特定状态（先检查KDE状态）
    switch(status_word) {
        case 0x0A31: return DRIVE_STATE_KDE_READY;
        case 0x0A33: return DRIVE_STATE_KDE_READY_FOR_MOTION;
        case 0x0A37: return DRIVE_STATE_KDE_RUNNING;
        case 0x0A18: return DRIVE_STATE_KDE_FAULT;
        case 0x0A50: return DRIVE_STATE_KDE_SWITCH_DISABLED;
        case 0x0000: return DRIVE_STATE_NOT_READY;
        default: break;
    }
    
    // 处理标准CiA 402状态（只取低8位的状态位）
    uint8_t state_bits = status_word & 0x006F;
    
    // 注意：可能还包含0x0010（内部限制激活）或0x0200（远程控制）等标志位
    // 但状态转换只需要关注低8位的0x6F
    
    if (status_word & 0x0008) {  // 故障位
        return DRIVE_STATE_FAULT;
    }
    
    switch(state_bits) {
        case 0x0040: return DRIVE_STATE_SWITCH_ON_DISABLED;
        case 0x0021: return DRIVE_STATE_READY_TO_SWITCH_ON;
        case 0x0023: return DRIVE_STATE_SWITCHED_ON;
        case 0x0027: return DRIVE_STATE_OPERATION_ENABLED;
        case 0x0007: return DRIVE_STATE_QUICK_STOP_ACTIVE;
        default: 
            // 如果是带标志位的状态，尝试匹配常见组合
            if (status_word == 0x0231) {  // 0x0021 + 0x0200
                return DRIVE_STATE_READY_TO_SWITCH_ON;
            }
            if (status_word == 0x0233) {  // 0x0023 + 0x0200
                return DRIVE_STATE_SWITCHED_ON;
            }
            if (status_word == 0x0237) {  // 0x0027 + 0x0200
                return DRIVE_STATE_OPERATION_ENABLED;
            }
            if (status_word == 0x0250) {  // 0x0040 + 0x0200
                return DRIVE_STATE_SWITCH_ON_DISABLED;
            }
            if (status_word == 0x0210) {  // 0x0010 + 0x0200 (内部限制激活 + 远程控制)
                // 可能是已使能状态的特殊情况
                return DRIVE_STATE_OPERATION_ENABLED;
            }
            return DRIVE_STATE_UNKNOWN;
    }
}

const char* drive_state_string(drive_state_t state) {
    switch(state) {
        case DRIVE_STATE_NOT_READY: return "未就绪";
        case DRIVE_STATE_SWITCH_ON_DISABLED: return "开关禁用";
        case DRIVE_STATE_READY_TO_SWITCH_ON: return "准备开关";
        case DRIVE_STATE_SWITCHED_ON: return "已开关";
        case DRIVE_STATE_OPERATION_ENABLED: return "运行使能";
        case DRIVE_STATE_QUICK_STOP_ACTIVE: return "快速停止激活";
        case DRIVE_STATE_FAULT: return "故障";
        case DRIVE_STATE_KDE_READY: return "KDE准备就绪(0x0A31)";
        case DRIVE_STATE_KDE_RUNNING: return "KDE运行中(0x0A37)";
        case DRIVE_STATE_KDE_READY_FOR_MOTION: return "KDE准备运动(0x0A33)";
        case DRIVE_STATE_KDE_FAULT: return "KDE故障(0x0A18)";
        case DRIVE_STATE_KDE_SWITCH_DISABLED: return "KDE开关禁用(0x0A50)";
        case DRIVE_STATE_UNKNOWN: return "未知";
        default: return "未定义";
    }
}

void set_state_transition(drive_state_t new_target_state, int timeout_cycles) {
    target_state = (int)new_target_state;
    state_wait_start = cycle_counter;
    state_timeout = timeout_cycles;
    retry_count = 0;
    
    printf("   设置状态转换: 目标=0x%04X, 超时=%d周期\n", 
           new_target_state, timeout_cycles);
}

int check_state_transition() {
    if (target_state == 0) return 1;
    
    uint16_t status = read_status_word();
    drive_state_t state = get_drive_state(status);
    
    if ((int)state == target_state) {
        printf("   ✓ 达到目标状态: 0x%04X [%s]\n", status, drive_state_string(state));
        target_state = 0;
        state_timeout = 0;
        retry_count = 0;
        return 1;
    }
    
    if (state_timeout > 0 && cycle_counter - state_wait_start > (unsigned long)state_timeout) {
        printf("   ⚠️ 状态转换超时，当前状态: 0x%04X [%s]\n", status, drive_state_string(state));
        
        retry_count++;
        if (retry_count >= MAX_RETRIES) {
            printf("   ❌ 达到最大重试次数，放弃状态转换\n");
            target_state = 0;
            state_timeout = 0;
            return 0;
        }
        
        state_wait_start = cycle_counter;
        printf("   ↻ 第%d次重试\n", retry_count);
        return -1;
    }
    
    return 0;
}

// ===================== DC时钟同步 =====================
void configure_dc_sync() {
    printf("配置DC时钟同步...\n");
    
    uint16_t assign_activate = DC_ASSIGN_ACTIVATE;
    uint32_t sync0_cycle_time = DC_SYNC0_CYCLE_TIME;
    int32_t sync0_shift_time = 0;
    uint32_t sync1_cycle_time = 0;
    int32_t sync1_shift_time = 0;
    
    ecrt_slave_config_dc(slave_config, assign_activate, 
                         sync0_cycle_time, sync0_shift_time,
                         sync1_cycle_time, sync1_shift_time);
    
    printf("   DC配置完成: assign_activate=0x%04X, sync0_cycle=%dns\n",
           assign_activate, sync0_cycle_time);
    dc_configured = 1;
}

void sync_dc_clock() {
    if (!dc_configured) return;
    
    if (app_time == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        app_time = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    } else {
        app_time += PERIOD_NS;
    }
    
    ecrt_master_application_time(master, app_time);
    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);
}

// ===================== 检查主站和从站状态 =====================
void check_master_slave_states() {
    static uint32_t last_check_time = 0;
    
    if (cycle_counter - last_check_time < FREQUENCY / 2) {
        return;
    }
    last_check_time = cycle_counter;
    
    ec_master_state_t mstate;
    ecrt_master_state(master, &mstate);
    
    if (mstate.al_states != EC_AL_STATE_OP) {
        if (!master_in_op) {
            printf("主站状态: 0x%02X ", mstate.al_states);
            switch(mstate.al_states) {
                case 0x01: printf("(INIT)\n"); break;
                case 0x02: printf("(PREOP)\n"); break;
                case 0x04: printf("(SAFEOP)\n"); break;
                case 0x08: printf("(OP)\n"); break;
                default: printf("(未知)\n"); break;
            }
            
            if (mstate.al_states == EC_AL_STATE_OP) {
                master_in_op = 1;
                printf("  主站已进入OP状态\n");
            } else {
                master_in_op = 0;
            }
        }
    } else {
        if (!master_in_op) {
            printf("✅ 主站已进入OP状态\n");
            master_in_op = 1;
        }
    }
    
    ec_slave_config_state_t config_state;
    ecrt_slave_config_state(slave_config, &config_state);
    
    if (config_state.al_state != EC_AL_STATE_OP) {
        if (!slave_in_op) {
            printf("从站状态: 0x%02X ", config_state.al_state);
            switch(config_state.al_state) {
                case 0x01: printf("(INIT)\n"); break;
                case 0x02: printf("(PREOP)\n"); break;
                case 0x04: printf("(SAFEOP)\n"); break;
                case 0x08: printf("(OP)\n"); break;
                default: printf("(未知)\n"); break;
            }
            
            if (config_state.al_state == EC_AL_STATE_OP) {
                slave_in_op = 1;
                printf("  从站已进入OP状态\n");
            } else {
                slave_in_op = 0;
            }
        }
    } else {
        if (!slave_in_op) {
            printf("✅ 从站已进入OP状态\n");
            slave_in_op = 1;
        }
    }
}

// ===================== 初始化非控制PDO值为0 =====================
void initialize_non_control_pdo_values() {
    if (!domain_data) return;
    
    write_target_position(0);
    write_target_velocity(0);
    write_target_torque(0);
    
    printf("   非控制PDO值已初始化为0\n");
    pdo_initialized = 1;
}

// ===================== 驱动器使能序列 =====================
void handle_drive_state() {
    uint16_t status = read_status_word();
    drive_state_t state = get_drive_state(status);
    uint16_t error_code = read_error_code();
    
    int transition_result = check_state_transition();
    if (transition_result == -1) {
        return;
    }
    
    if (!slave_in_op) {
        printf("等待从站进入OP状态...\n");
        return;
    }
    
    if (state == DRIVE_STATE_FAULT) {
        printf("  处理故障状态 (状态字: 0x%04X)...\n", status);
        printf("  错误代码: 0x%04X\n", error_code);
        
        printf("  发送故障复位命令 (0x0080)\n");
        write_control_word(CMD_FAULT_RESET);
        
        set_state_transition(DRIVE_STATE_SWITCH_ON_DISABLED, 3 * FREQUENCY);
        return;
    }
    
    switch(state) {
        case DRIVE_STATE_NOT_READY:
            printf("  驱动器未就绪，等待...\n");
            break;
            
        case DRIVE_STATE_SWITCH_ON_DISABLED:
            if (!pdo_initialized) {
                initialize_non_control_pdo_values();
            }
            
            printf("  开关禁用状态 (0x%04X)，发送关机命令 (0x0006)\n", status);
            write_control_word(CMD_SHUTDOWN);
            
            set_state_transition(DRIVE_STATE_READY_TO_SWITCH_ON, 10 * FREQUENCY);
            break;
            
        case DRIVE_STATE_READY_TO_SWITCH_ON:
            printf("  准备开关状态 (0x%04X)，发送开关命令 (0x0007)\n", status);
            write_control_word(CMD_SWITCH_ON);
            
            set_state_transition(DRIVE_STATE_SWITCHED_ON, 10 * FREQUENCY);
            break;
            
        case DRIVE_STATE_SWITCHED_ON:
            printf("  已开关状态 (0x%04X)，发送使能命令 (0x000F)\n", status);
            write_control_word(CMD_ENABLE_OPERATION);
            
            set_state_transition(DRIVE_STATE_OPERATION_ENABLED, 10 * FREQUENCY);
            break;
            
        case DRIVE_STATE_OPERATION_ENABLED:
            if (!operational) {
                printf("  ✅ 驱动器已使能！状态: 0x%04X\n", status);
                
                motion_start_time = cycle_counter;
                motion_state = MOTION_WAITING;
                float wait_seconds = MOTION_TEST_WAIT_TIME / 1000.0f;
                printf("  ⏱️ 等待%.1f秒后开始运动测试...\n", wait_seconds);
                
                operational = 1;
                // 持续发送使能命令保持状态
                write_control_word(CMD_ENABLE_OPERATION);
                
                // 开始位置监控
                position_monitoring_started = 0;
                
                // 打印详细调试信息
                if (debug_enabled) {
                    debug_all_pdo_values();
                    debug_pdo_mapping_details();
                    analyze_current_issue();
                }
            }
            break;
            
        // KDE驱动器状态（保留，兼容KDE驱动器）
        case DRIVE_STATE_KDE_SWITCH_DISABLED:
            if (!pdo_initialized) {
                initialize_non_control_pdo_values();
            }
            
            printf("  KDE开关禁用状态 (0x%04X)，发送关机命令 (0x0006)\n", status);
            write_control_word(CMD_SHUTDOWN);
            
            set_state_transition(DRIVE_STATE_KDE_READY, 10 * FREQUENCY);
            break;
            
        case DRIVE_STATE_KDE_READY:
            printf("  KDE准备就绪 (0x%04X)，发送开关命令 (0x0007)\n", status);
            write_control_word(CMD_SWITCH_ON);
            
            set_state_transition(DRIVE_STATE_KDE_READY_FOR_MOTION, 10 * FREQUENCY);
            break;
            
        case DRIVE_STATE_KDE_READY_FOR_MOTION:
            printf("  KDE准备运动 (0x%04X)，发送使能命令 (0x000F)\n", status);
            write_control_word(CMD_ENABLE_OPERATION);
            
            set_state_transition(DRIVE_STATE_KDE_RUNNING, 10 * FREQUENCY);
            break;
            
        case DRIVE_STATE_KDE_RUNNING:
            if (!operational) {
                printf("  ✅ KDE驱动器已使能！状态: 0x%04X\n", status);
                
                motion_start_time = cycle_counter;
                motion_state = MOTION_WAITING;
                float wait_seconds = MOTION_TEST_WAIT_TIME / 1000.0f;
                printf("  ⏱️ 等待%.1f秒后开始运动测试...\n", wait_seconds);
                
                operational = 1;
                write_control_word(CMD_ENABLE_OPERATION);
                
                // 开始位置监控
                position_monitoring_started = 0;
                
                // 打印详细调试信息
                if (debug_enabled) {
                    debug_all_pdo_values();
                    debug_pdo_mapping_details();
                    analyze_current_issue();
                }
            }
            break;
            
        default:
            // 不再发送关机命令，而是根据实际状态处理
            if (cycle_counter % (5 * FREQUENCY) == 0) {
                printf("  未知状态 (0x%04X)，状态位: 0x%02X\n", 
                       status, status & 0x006F);
            }
            break;
    }
}

// ===================== 运行状态维护 =====================
void maintain_operation_state() {
    if (!operational) return;
    
    uint16_t status = read_status_word();
    drive_state_t state = get_drive_state(status);
    
    if (state == DRIVE_STATE_KDE_RUNNING || state == DRIVE_STATE_OPERATION_ENABLED) {
        write_control_word(current_control_word);
        
        static uint32_t last_pdo_update = 0;
        if (cycle_counter - last_pdo_update > FREQUENCY) {
            write_target_position(0);
            write_target_torque(0);
            last_pdo_update = cycle_counter;
        }
    }
    
    if (state == DRIVE_STATE_KDE_FAULT || state == DRIVE_STATE_FAULT) {
        uint16_t error_code = read_error_code();
        printf("  ⚠️ 驱动器进入故障状态: 0x%04X, 错误代码: 0x%04X\n", status, error_code);
        
        operational = 0;
        motion_started = 0;
        motion_state = MOTION_IDLE;
        printf("  发送故障复位命令 (0x0080)\n");
        write_control_word(CMD_FAULT_RESET);
        set_state_transition(DRIVE_STATE_SWITCH_ON_DISABLED, 5 * FREQUENCY);
    }
}

// ===================== 运动控制测试 =====================
void test_motion_control() {
    if (!operational) return;
    
    uint16_t status = read_status_word();
    drive_state_t state = get_drive_state(status);
    
    if (state != DRIVE_STATE_KDE_RUNNING && state != DRIVE_STATE_OPERATION_ENABLED) {
        printf("  等待驱动器进入运行状态...\n");
        return;
    }
    
    motion_elapsed_time = (float)(cycle_counter - motion_start_time) / FREQUENCY * 1000.0f;
    
    switch(motion_state) {
        case MOTION_IDLE:
            break;
            
        case MOTION_WAITING:
            if (motion_elapsed_time >= MOTION_TEST_WAIT_TIME) {
                printf("\n🚀 开始运动控制测试！\n");
                printf("  当前状态: 0x%04X [%s]\n", status, drive_state_string(state));
                motion_state = MOTION_ACCELERATING;
                motion_start_time = cycle_counter;
                motion_elapsed_time = 0.0f;
                
                // 重置位置监控
                position_monitoring_started = 0;
            }
            break;
            
        case MOTION_ACCELERATING:
            if (motion_elapsed_time <= MOTION_RAMP_UP_TIME) {
                float progress = motion_elapsed_time / MOTION_RAMP_UP_TIME;
                current_velocity_deg_per_sec = progress * TARGET_VELOCITY_DEG_PER_SEC;
                
                if (cycle_counter % (FREQUENCY / 10) == 0) {
                    printf("  加速中... %.3f/%.3f 度/秒 (%.0f%%)\n", 
                           current_velocity_deg_per_sec, TARGET_VELOCITY_DEG_PER_SEC, progress * 100);
                    
                    // 监控位置变化
                    monitor_position_changes();
                }
            } else {
                current_velocity_deg_per_sec = TARGET_VELOCITY_DEG_PER_SEC;
                motion_state = MOTION_RUNNING;
                motion_start_time = cycle_counter;
                motion_elapsed_time = 0.0f;
                printf("  ✅ 加速完成，达到目标速度 %.3f 度/秒\n", TARGET_VELOCITY_DEG_PER_SEC);
                printf("  持续发送使能命令 (0x000F)\n");
                write_control_word(CMD_ENABLE_OPERATION);
                motion_started = 1;
                
                // 打印详细调试信息
                if (debug_enabled) {
                    printf("  当前目标速度: %d 码值/秒\n", current_target_velocity);
                }
            }
            
            // 转换为内部速度单位并发送
            int32_t internal_vel = deg_per_sec_to_internal_velocity(current_velocity_deg_per_sec);
            write_target_velocity(internal_vel);
            break;
            
        case MOTION_RUNNING:
            if (motion_elapsed_time <= MOTION_RUN_TIME) {
                current_velocity_deg_per_sec = TARGET_VELOCITY_DEG_PER_SEC;
                
                // 每秒显示一次状态
                if (cycle_counter % FREQUENCY == 0) {
                    int32_t actual_vel = read_actual_velocity();
                    float actual_vel_deg_per_sec = internal_velocity_to_deg_per_sec(actual_vel);
                    float remaining_time = (MOTION_RUN_TIME - motion_elapsed_time) / 1000.0f;
                    
                    printf("  运行中... 目标: %.3f 度/秒, 实际: %.6f 度/秒, 剩余: %.1f秒\n", 
                           current_velocity_deg_per_sec, actual_vel_deg_per_sec, remaining_time);
                    
                    // 监控位置变化
                    monitor_position_changes();
                }
                
                // 每2.5秒打印详细调试信息
                if (debug_enabled && cycle_counter % (int)(FREQUENCY * 2.5) == 0) {
                    printf("  详细状态:\n");
                    int32_t actual_pos = read_actual_position();
                    int32_t actual_vel = read_actual_velocity();
                    printf("    实际位置: %d 码值 (%.6f 度)\n", 
                           actual_pos, counts_to_degrees(actual_pos));
                    printf("    实际速度: %d 码值/秒\n", actual_vel);
                    
                    // 计算位置变化
                    static int32_t last_reported_pos = 0;
                    if (last_reported_pos == 0) {
                        last_reported_pos = actual_pos;
                    } else {
                        int32_t pos_change = actual_pos - last_reported_pos;
                        printf("    位置变化: %d 码值 (%.6f 度)\n", 
                               pos_change, counts_to_degrees(pos_change));
                        last_reported_pos = actual_pos;
                    }
                }
            } else {
                motion_state = MOTION_DECELERATING;
                motion_start_time = cycle_counter;
                motion_elapsed_time = 0.0f;
                printf("  ⬇️ 开始减速...\n");
            }
            
            // 持续发送目标速度
            write_target_velocity(deg_per_sec_to_internal_velocity(current_velocity_deg_per_sec));
            break;
            
        case MOTION_DECELERATING:
            if (motion_elapsed_time <= MOTION_RAMP_DOWN_TIME) {
                float progress = motion_elapsed_time / MOTION_RAMP_DOWN_TIME;
                current_velocity_deg_per_sec = TARGET_VELOCITY_DEG_PER_SEC * (1.0f - progress);
                
                if (cycle_counter % (FREQUENCY / 10) == 0) {
                    printf("  减速中... %.3f/%.3f 度/秒 (%.0f%%)\n", 
                           current_velocity_deg_per_sec, TARGET_VELOCITY_DEG_PER_SEC, progress * 100);
                }
            } else {
                current_velocity_deg_per_sec = 0.0f;
                motion_state = MOTION_STOPPING;
                motion_start_time = cycle_counter;
                motion_elapsed_time = 0.0f;
                printf("  ✅ 减速完成，速度已降为0 度/秒\n");
                printf("  发送普通使能命令 (0x000F)\n");
                write_control_word(CMD_ENABLE_OPERATION);
            }
            
            // 转换为内部速度单位并发送
            internal_vel = deg_per_sec_to_internal_velocity(current_velocity_deg_per_sec);
            write_target_velocity(internal_vel);
            break;
            
        case MOTION_STOPPING:
            if (motion_elapsed_time <= MOTION_STABILIZE_TIME) {
                current_velocity_deg_per_sec = 0.0f;
                
                if (cycle_counter % FREQUENCY == 0) {
                    float remaining_time = (MOTION_STABILIZE_TIME - motion_elapsed_time) / 1000.0f;
                    printf("  稳定中... 剩余: %.1f秒\n", remaining_time);
                    
                    // 最终位置报告
                    int32_t final_position = read_actual_position();
                    int32_t position_change_total = final_position - initial_position;
                    float angle_change_total = counts_to_degrees(position_change_total);
                    printf("  最终位置: %d 码值 (%.6f 度), 总位置变化: %d 码值 (%.6f 度)\n", 
                           final_position, counts_to_degrees(final_position), 
                           position_change_total, angle_change_total);
                }
            } else {
                motion_state = MOTION_COMPLETED;
                motion_test_completed = 1;
                printf("\n✅ 运动控制测试完成！\n");
                
                // 最终调试信息
                int32_t final_position = read_actual_position();
                int32_t position_change_total = final_position - initial_position;
                float angle_change_total = counts_to_degrees(position_change_total);
                printf("  测试总结:\n");
                printf("    - 初始位置: %d 码值 (%.6f 度)\n", 
                       initial_position, counts_to_degrees(initial_position));
                printf("    - 最终位置: %d 码值 (%.6f 度)\n", 
                       final_position, counts_to_degrees(final_position));
                printf("    - 总位置变化: %d 码值 (%.6f 度)\n", 
                       position_change_total, angle_change_total);
                printf("    - 总运行时间: %.1f秒\n", (float)cycle_counter/FREQUENCY);
                
                if (position_change_total == 0) {
                    printf("  ⚠️ 警告: 电机位置没有变化！\n");
                    printf("  可能的原因:\n");
                    printf("    1. 编码器分辨率设置错误 - 电机: %d 码值/圈, 减速比: %.0f:1\n", 
                           MOTOR_ENCODER_COUNTS_PER_REV, GEAR_RATIO);
                    printf("    2. 目标速度设置太小 - 当前: %.3f 度/秒\n", TARGET_VELOCITY_DEG_PER_SEC);
                    printf("    3. 驱动器未正确使能\n");
                    printf("    4. 电机抱闸未打开\n");
                    printf("    5. 驱动器模式设置错误\n");
                    printf("    6. 跟随误差限制太小\n");
                }
            }
            
            // 保持发送零速度
            write_target_velocity(deg_per_sec_to_internal_velocity(current_velocity_deg_per_sec));
            break;
            
        case MOTION_COMPLETED:
            current_velocity_deg_per_sec = 0.0f;
            break;
    }
    
    if (motion_started && motion_state != MOTION_COMPLETED) {
        write_control_word(current_control_word);
    }
}

// ===================== 初始化函数 =====================
int init_ethercat() {
    printf("1. 初始化 EtherCAT 主站...\n");
    
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "错误: 无法请求主站!\n");
        return -1;
    }
    
    domain = ecrt_master_create_domain(master);
    if (!domain) {
        fprintf(stderr, "错误: 无法创建域!\n");
        return -1;
    }
    
    printf("   创建从站配置 (位置: %d)\n", SLAVE_POSITION);
    slave_config = ecrt_master_slave_config(master, 0, SLAVE_POSITION, 
                                           VENDOR_ID, PRODUCT_ID);
    if (!slave_config) {
        fprintf(stderr, "错误: 无法创建从站配置!\n");
        return -1;
    }
    
    printf("   主站初始化完成\n");
    return 0;
}

int config_sdo() {
    printf("2. 配置 SDO...\n");
    
    // 设置操作模式为速度模式
    uint8_t op_mode = 9; // 速度模式
    if (ecrt_slave_config_sdo8(slave_config, 0x6060, 0x00, op_mode)) {
        fprintf(stderr, "错误: 无法设置速度模式\n");
        return -1;
    }
    printf("   操作模式设置为速度模式 (0x%02X)\n", op_mode);
    
    // 设置最大速度 (使用电机端编码器)
    uint32_t max_speed = rpm_to_internal_velocity(1000.0f); // 1000 RPM 电机端
    if (ecrt_slave_config_sdo32(slave_config, 0x607F, 0x00, max_speed)) {
        printf("   警告: 无法设置最大速度\n");
    } else {
        printf("   最大速度设置为: %u 码值/秒 (%.0f RPM 电机端)\n", max_speed, 1000.0f);
    }
    
    return 0;
}

int config_sync_and_pdo() {
    printf("3. 配置同步管理器和 PDO...\n");
    
    if (ecrt_slave_config_pdos(slave_config, EC_END, slave_syncs)) {
        fprintf(stderr, "错误: 无法配置同步管理器和 PDO!\n");
        return -1;
    }
    
    printf("   同步管理器和 PDO 配置完成\n");
    return 0;
}

int register_pdo_entries() {
    printf("4. 注册 PDO 条目...\n");
    
    unsigned int bit_position;
    
    pdo_control_word.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x6040, 0, domain, &bit_position);
    pdo_control_word.bit_position = bit_position;
    
    pdo_target_position.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x607a, 0, domain, &bit_position);
    pdo_target_position.bit_position = bit_position;
    
    pdo_target_velocity.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x60ff, 0, domain, &bit_position);
    pdo_target_velocity.bit_position = bit_position;
    
    pdo_target_torque.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x6071, 0, domain, &bit_position);
    pdo_target_torque.bit_position = bit_position;
    
    pdo_status_word.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x6041, 0, domain, &bit_position);
    pdo_status_word.bit_position = bit_position;
    
    pdo_actual_position.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x6064, 0, domain, &bit_position);
    pdo_actual_position.bit_position = bit_position;
    
    pdo_actual_velocity.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x606c, 0, domain, &bit_position);
    pdo_actual_velocity.bit_position = bit_position;
    
    pdo_actual_torque.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x6077, 0, domain, &bit_position);
    pdo_actual_torque.bit_position = bit_position;
    
    pdo_following_error.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x60f4, 0, domain, &bit_position);
    pdo_following_error.bit_position = bit_position;
    
    pdo_error_code.byte_offset = ecrt_slave_config_reg_pdo_entry(
        slave_config, 0x603f, 0, domain, &bit_position);
    pdo_error_code.bit_position = bit_position;
    
    printf("   PDO条目注册完成\n");
    printf("   实际位置偏移: %u\n", pdo_actual_position.byte_offset);
    printf("   实际速度偏移: %u\n", pdo_actual_velocity.byte_offset);
    printf("   目标速度偏移: %u\n", pdo_target_velocity.byte_offset);
    
    return 0;
}

int activate_master() {
    printf("5. 激活主站...\n");
    
    ec_master_state_t mstate;
    ecrt_master_state(master, &mstate);
    printf("   当前AL状态: 0x%02X ", mstate.al_states);
    switch(mstate.al_states) {
        case 0x01: printf("(INIT)\n"); break;
        case 0x02: printf("(PREOP)\n"); break;
        case 0x04: printf("(SAFEOP)\n"); break;
        case 0x08: printf("(OP)\n"); break;
        default: printf("(未知)\n"); break;
    }
    
    configure_dc_sync();
    
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "错误: 无法激活主站!\n");
        return -1;
    }
    
    domain_data = ecrt_domain_data(domain);
    if (!domain_data) {
        fprintf(stderr, "错误: 无法获取域数据指针!\n");
        return -1;
    }
    
    printf("   域数据指针: %p\n", domain_data);
    printf("   主站激活完成\n");
    
    initialize_non_control_pdo_values();
    
    return 0;
}

// ===================== 主函数 =====================
int main() {
    printf("===========================================\n");
    printf(" EtherCAT 驱动器控制程序 (编码器修正版)\n");
    printf("===========================================\n");
    printf("编码器参数:\n");
    printf("  电机编码器每圈: %d 码值 (2^20)\n", MOTOR_ENCODER_COUNTS_PER_REV);
    printf("  减速比: %.0f:1\n", GEAR_RATIO);
    printf("  关节编码器每圈: %.0f 码值\n", JOINT_ENCODER_COUNTS_PER_REV);
    printf("  1码值 = %.12f 度\n", 360.0f / JOINT_ENCODER_COUNTS_PER_REV);
    printf("===========================================\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    if (init_ethercat() < 0) return 1;
    if (config_sdo() < 0) return 1;
    if (config_sync_and_pdo() < 0) return 1;
    if (register_pdo_entries() < 0) return 1;
    if (activate_master() < 0) return 1;
    
    printf("\n6. 启动控制循环 (按 Ctrl+C 停止)...\n");
    printf("===========================================\n\n");
    
    printf("运动控制测试计划:\n");
    printf("  目标角速度: %.3f 度/秒\n", TARGET_VELOCITY_DEG_PER_SEC);
    printf("  单位转换验证:\n");
    
    // 验证单位转换
    float test_deg_per_sec = TARGET_VELOCITY_DEG_PER_SEC;
    int32_t test_counts_per_sec = deg_per_sec_to_internal_velocity(test_deg_per_sec);
    float test_rpm = internal_velocity_to_rpm(test_counts_per_sec);
    printf("    - %.3f 度/秒 = %d 码值/秒 = %.3f RPM (电机端)\n", 
           test_deg_per_sec, test_counts_per_sec, test_rpm);
    
    test_deg_per_sec = 10.0f;
    test_counts_per_sec = deg_per_sec_to_internal_velocity(test_deg_per_sec);
    test_rpm = internal_velocity_to_rpm(test_counts_per_sec);
    printf("    - %.3f 度/秒 = %d 码值/秒 = %.3f RPM (电机端)\n", 
           test_deg_per_sec, test_counts_per_sec, test_rpm);
    
    printf("\n等待3秒开始初始化...\n");
    
    for (int i = 3; i > 0; i--) {
        printf("  %d秒...\n", i);
        sleep(1);
        if (!running) break;
    }
    
    struct timespec wake_time;
    clock_gettime(CLOCK_MONOTONIC, &wake_time);
    
    printf("\n开始驱动器使能序列...\n");
    printf("===========================================\n");
    
    uint16_t status = read_status_word();
    drive_state_t state = get_drive_state(status);
    uint16_t error = read_error_code();
    int32_t initial_pos = read_actual_position();
    float initial_angle = counts_to_degrees(initial_pos);
    
    printf("初始状态: 0x%04X [%s]", status, drive_state_string(state));
    if (error != 0) printf(", 错误: 0x%04X", error);
    printf(", 初始位置: %d 码值 (%.6f 度)\n", initial_pos, initial_angle);
    
    if (state == DRIVE_STATE_KDE_RUNNING || state == DRIVE_STATE_OPERATION_ENABLED) {
        printf("驱动器已在运行状态，直接开始运动测试\n");
        operational = 1;
        motion_start_time = cycle_counter;
        motion_state = MOTION_WAITING;
        write_control_word(CMD_ENABLE_OPERATION);
    } else if (state == DRIVE_STATE_FAULT || state == DRIVE_STATE_KDE_FAULT) {
        printf("初始状态为故障 (0x%04X)，发送故障复位\n", status);
        write_control_word(CMD_FAULT_RESET);
        set_state_transition(DRIVE_STATE_SWITCH_ON_DISABLED, 10 * FREQUENCY);
    } else if (state == DRIVE_STATE_SWITCH_ON_DISABLED || state == DRIVE_STATE_KDE_SWITCH_DISABLED) {
        printf("初始状态为开关禁用 (0x%04X)，发送关机命令\n", status);
        write_control_word(CMD_SHUTDOWN);
        set_state_transition(DRIVE_STATE_READY_TO_SWITCH_ON, 10 * FREQUENCY);
    } else if (state == DRIVE_STATE_READY_TO_SWITCH_ON) {
        printf("初始状态为准备开关 (0x%04X)，发送开关命令\n", status);
        write_control_word(CMD_SWITCH_ON);
        set_state_transition(DRIVE_STATE_SWITCHED_ON, 10 * FREQUENCY);
    } else if (state == DRIVE_STATE_SWITCHED_ON) {
        printf("初始状态为已开关 (0x%04X)，发送使能命令\n", status);
        write_control_word(CMD_ENABLE_OPERATION);
        set_state_transition(DRIVE_STATE_OPERATION_ENABLED, 10 * FREQUENCY);
    } else if (state == DRIVE_STATE_NOT_READY) {
        printf("初始状态为未就绪，等待驱动器启动...\n");
    }
    
    while (running && cycle_counter < TEST_DURATION * FREQUENCY) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
        wake_time.tv_nsec += PERIOD_NS;
        if (wake_time.tv_nsec >= 1000000000) {
            wake_time.tv_nsec -= 1000000000;
            wake_time.tv_sec++;
        }
        
        sync_dc_clock();
        
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        
        if (cycle_counter % FREQUENCY == 0) {
            status = read_status_word();
            state = get_drive_state(status);
            error = read_error_code();
            int32_t actual_pos = read_actual_position();
            float actual_angle = counts_to_degrees(actual_pos);
            int32_t actual_vel = read_actual_velocity();
            float actual_vel_deg_per_sec = internal_velocity_to_deg_per_sec(actual_vel);
            float actual_rpm = internal_velocity_to_rpm(actual_vel);
            
            printf("\n[%.1fs] 状态: 0x%04X [%s]", 
                   (float)cycle_counter/FREQUENCY, status, drive_state_string(state));
            if (error != 0) printf(", 错误: 0x%04X", error);
            printf(", 位置: %.6f 度, 速度: %.6f 度/秒 (%.3f RPM)", 
                   actual_angle, actual_vel_deg_per_sec, actual_rpm);
            
            if (operational) {
                switch(motion_state) {
                    case MOTION_WAITING:
                        printf(", 等待运动测试");
                        break;
                    case MOTION_ACCELERATING:
                        printf(", 加速中 (目标: %.3f 度/秒)", current_velocity_deg_per_sec);
                        break;
                    case MOTION_RUNNING:
                        printf(", 运行中 (目标: %.3f 度/秒)", current_velocity_deg_per_sec);
                        break;
                    case MOTION_DECELERATING:
                        printf(", 减速中 (目标: %.3f 度/秒)", current_velocity_deg_per_sec);
                        break;
                    case MOTION_STOPPING:
                        printf(", 稳定中");
                        break;
                    case MOTION_COMPLETED:
                        printf(", 测试完成");
                        break;
                    default:
                        break;
                }
            }
            printf("\n");
        }
        
        check_master_slave_states();
        
        if (!operational) {
            handle_drive_state();
        } else {
            maintain_operation_state();
            test_motion_control();
        }
        
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
        
        cycle_counter++;
        
        if (motion_test_completed && motion_state == MOTION_COMPLETED) {
            printf("\n运动测试已完成，等待5秒后退出...\n");
            for (int i = 5; i > 0; i--) {
                printf("  %d秒...\n", i);
                sleep(1);
                if (!running) break;
            }
            break;
        }
    }
    
    printf("\n正在清理...\n");
    
    if (motion_started) {
        printf("停止运动...\n");
        current_velocity_deg_per_sec = 0.0f;
        int32_t zero_vel = deg_per_sec_to_internal_velocity(current_velocity_deg_per_sec);
        write_target_velocity(zero_vel);
        usleep(200000);
        
        printf("发送禁用电压命令...\n");
        write_control_word(CMD_DISABLE_VOLTAGE);
        usleep(200000);
    } else if (operational) {
        printf("停止驱动器...\n");
        write_control_word(CMD_DISABLE_VOLTAGE);
        usleep(200000);
    }
    
    if (master) {
        ecrt_release_master(master);
    }
    
    printf("\n测试完成! 总运行时间: %.1f秒\n", (float)cycle_counter/FREQUENCY);
    
    if (!operational) {
        printf("\n❌ 驱动器未能使能成功\n");
    } else {
        printf("\n✅ 驱动器使能成功!\n");
        if (motion_test_completed) {
            printf("✅ 运动控制测试完成！\n");
            
            // 最终位置报告
            int32_t final_position = read_actual_position();
            int32_t position_change_total = final_position - initial_position;
            float angle_change_total = counts_to_degrees(position_change_total);
            printf("  位置变化总结:\n");
            printf("    - 初始位置: %d 码值 (%.6f 度)\n", 
                   initial_position, counts_to_degrees(initial_position));
            printf("    - 最终位置: %d 码值 (%.6f 度)\n", 
                   final_position, counts_to_degrees(final_position));
            printf("    - 总位置变化: %d 码值 (%.6f 度)\n", 
                   position_change_total, angle_change_total);
            
            if (position_change_total == 0) {
                printf("\n⚠️ 警告: 电机位置没有变化！\n");
                printf("可能的原因:\n");
                printf("  1. 编码器分辨率设置错误 - 当前设置:\n");
                printf("     - 电机编码器每圈: %d 码值\n", MOTOR_ENCODER_COUNTS_PER_REV);
                printf("     - 减速比: %.0f:1\n", GEAR_RATIO);
                printf("     - 关节编码器每圈: %.0f 码值\n", JOINT_ENCODER_COUNTS_PER_REV);
                printf("  2. 目标速度设置太小 - 当前: %.3f 度/秒\n", TARGET_VELOCITY_DEG_PER_SEC);
                printf("  3. 驱动器未正确使能\n");
                printf("  4. 电机抱闸未打开\n");
                printf("  5. 驱动器模式设置错误\n");
                printf("  6. 跟随误差限制太小\n");
            }
        } else if (motion_started) {
            printf("⚠️  注意: 运动测试已开始但未完成\n");
        } else {
            printf("⚠️  注意: 驱动器已使能但未开始运动测试\n");
        }
    }
    
    return 0;
}