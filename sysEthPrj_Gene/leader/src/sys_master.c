/*
 * sys_master.c - EtherCAT 6轴驱动器主站控制程序 (稳定版)
 * 
 * 本模块实现 EtherCAT 主站的完整生命周期管理，完全兼容原始可工作版本的行为：
 *   - 所有轴默认运行于 CSV（速度模式），不进行模式切换
 *   - 速度命令直接写入目标速度
 *   - 位置命令启动软件位置环（PD控制器，每5个周期更新一次）
 *   - 控制字在 OP 状态下固定为 0x000F
 *   - 支持 PID 参数异步读写（SDO）
 * 
 * 作者：周佳燚 (基于原始逻辑重构)
 * 日期：2026-03-11
 */

#include <signal.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ecrt.h>

#include "sys_master.h"
#include "sys_leader.h"
#include "sys_share_messages.h"
#include "sys_share_conf.h"
#include "sys_share_ipc.h"
#include "sys_ctl.h"
#include "sys_commu_ipc.h"

/*==============================================================================
 * 常量定义
 *============================================================================*/

#define VENDOR_ID               0x00010203
#define PRODUCT_ID              0x00000402

#define YKB_VENDOR_ID           0x00009527
#define YKB_PRODUCT_ID          0x00002000

#define ACTI_VENDOR_ID          0x0000A1
#define ACTI_PRODUCT_ID         0x00008081

#define FREQUENCY_HZ            500
#define PERIOD_NS               2000000

#define MOTOR_ENCODER_COUNTS_PER_REV    1048576
#define GEAR_RATIO                      101.0f
#define JOINT_ENCODER_COUNTS_PER_REV    (MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO)

/* 第七轴(伸缩轴)参数 - YKB200电机+丝杠 */
#define AXIS7_ID                        6           /* 第七轴索引 */
#define AXIS7_GEAR_RATIO                3.705f      /* 减速比 */
#define AXIS7_SCREW_PITCH_MM            10.0f       /* 丝杠导程10mm */
#define AXIS7_STROKE_MM                 300.0f       /* 伸缩行程10mm */
#define AXIS7_ENCODER_BITS              17          /* 17位单圈编码器 */
#define AXIS7_ENCODER_COUNTS_PER_REV    (1 << AXIS7_ENCODER_BITS)  /* 131072 */
#define AXIS7_MAX_SPEED_MM_PER_SEC      5.0f        /* 最大伸缩速度 mm/s */
#define AXIS7_MAX_ACC_MM_PER_SEC2       20.0f       /* 最大加速度 mm/s² */

/* 第八轴(六维力传感器)参数 */
#define AXIS8_ID                        7           /* 第八轴索引 - 六维力传感器 */
#define FORCE_SENSOR_SCALE              1000.0f     /* 力传感器数据缩放因子 */

#define DC_SYNC0_CYCLE_TIME     2000000
#define DC_SYNC0_SHIFT          1000000
#define DC_ASSIGN_ACTIVATE       0x0300

#define MODE_SWITCH_DELAY_MS    500      /* 保留但未使用 */

#define POS_CONTROL_HZ          200
#define POS_CONTROL_PERIOD_MS   5
#define POS_CONTROL_PERIOD_S    0.005f

#define VEL_SMOOTH_WINDOW_MS    100
#define UPPER_POS_SMOOTH_CNT    (VEL_SMOOTH_WINDOW_MS / POS_CONTROL_PERIOD_MS)

#define POS_KP_START            10.0f
#define POS_KP_MAX              80.0f
#define POS_KD                  3.0f
#define POS_KP                  15.0f

#define MAX_VEL_DEG_PER_SEC     10.0f
#define MAX_ACC_DEG_PER_SEC2    30.0f

#define POS_ARRIVAL_THRESHOLD   0.02f
#define VEL_ARRIVAL_THRESHOLD   0.5f
#define ARRIVAL_HOLD_MS         40

#define FORCE_ZERO_VEL_THRESHOLD 0.1f

#define UPPER_STATUS_PORT       33334

#define SDO_OPERATION_TIMEOUT_MS 100
#define PID_SDO_TIMEOUT_MS      5000
#define PID_JOB_GUARD_TIMEOUT_MS 10000

#define MSG_DATA_OFFSET         offsetof(MsgLongMsg, mData)

#define LIMIT_MAX(x, max)       ((x) > (max) ? (max) : (x))
#define LIMIT_MIN(x, min)       ((x) < (min) ? (min) : (x))
#define LIMIT(x, min, max)      LIMIT_MIN(LIMIT_MAX(x, max), min)

/*==============================================================================
 * 类型定义
 *============================================================================*/

/* 从站 CiA402 状态 */
typedef enum {
    SLAVE_NOT_READY = 0,
    SLAVE_SWITCH_ON_DISABLED,
    SLAVE_READY_TO_SWITCH_ON,
    SLAVE_SWITCHED_ON,
    SLAVE_OPERATION_ENABLED,
    SLAVE_QUICK_STOP_ACTIVE,
    SLAVE_FAULT,
    SLAVE_UNKNOWN
} SlaveCiAState;

/* 操作模式 (0x6060) */
typedef enum {
    OP_MODE_NONE = 0,
    OP_MODE_CSP  = 8,
    OP_MODE_CSV  = 9,
} OpMode;

/* 上层控制模式（由命令触发） */
typedef enum {
    CTRL_MODE_IDLE = 0,
    CTRL_MODE_CSV_DIRECT_VEL,
    CTRL_MODE_CSV_SOFT_POS,
    CTRL_MODE_CSP_HARD_POS,
} ControlMode;

/* 软件位置环状态 */
typedef struct {
    float    target_pos_deg;
    float    start_pos_deg;
    float    current_pos_deg;
    int      active;
    int      arrived;
    uint32_t arrival_hold_cnt;
    float    vel_history[UPPER_POS_SMOOTH_CNT];
    int      history_idx;
    float    last_vel_cmd;
    float    max_speed_reached;
    uint32_t cycle_cnt;
} CsvSoftPosState;

/* 硬件位置环状态（未使用，保留定义） */
typedef struct {
    int32_t  start_counts;
    int32_t  target_counts;
    int      step;
    int      total_steps;
    int      active;
} CspHardPosState;

/* PID 参数映射表 */
typedef struct {
    uint16_t paramNo;
    uint16_t index;
    uint8_t  subindex;
    uint8_t  dataType;
    float    minVal;
    float    maxVal;
    float    defaultVal;
    const char* name;
    const char* desc;
} KguParamMapping;

/* PID 响应类型（补充） */
typedef struct {
    uint8_t   axisId;
    uint8_t   successCount;
    uint8_t   totalCount;
    uint8_t   reserved;
} KGU_PID_WRITE_RESP_4BYTE;

typedef struct {
    uint8_t   axisId;
    uint8_t   success;
    uint16_t  errorCode;
} KGU_PID_SAVE_RESP;

/* PID SDO 请求集合 */
typedef struct {
    ec_sdo_request_t* req;
    int               created;
} PidSdoReq;

/* PID 读取作业 */
typedef struct {
    int      active;
    uint8_t  axis;
    uint8_t  map_idx;
    uint8_t  waiting;
    uint32_t start_cycle;
    uint32_t step_cycle;
    KGU_PID_PARAMS_PACKET resp;
} PidReadJob;

/* PID 写入作业 */
typedef struct {
    int      active;
    uint8_t  axis;
    uint8_t  total;
    uint8_t  cur;
    uint8_t  waiting;
    uint32_t start_cycle;
    uint32_t step_cycle;
    struct {
        uint16_t paramNo;
        float    value;
    } items[8];
    KGU_PID_WRITE_RESP_4BYTE resp;
} PidWriteJob;

/* PID 保存作业 */
typedef struct {
    int      active;
    uint8_t  axis;
    uint8_t  waiting;
    uint32_t start_cycle;
    uint32_t step_cycle;
    KGU_PID_SAVE_RESP resp;
} PidSaveJob;

/* 回零状态 */
typedef enum {
    HOMING_STATE_IDLE = 0,
    HOMING_STATE_START,
    HOMING_STATE_MOVING_TO_HOME,
    HOMING_STATE_COMPLETED,
    HOMING_STATE_ERROR
} HomingState;

/* 单轴动态数据 */
typedef struct {
    /* 从站状态 */
    SlaveCiAState      state;
    uint16_t           status_word;
    uint16_t           ctrl_word;
    int                in_op;
    int                enable_op_requested;
    int                op_just_entered;
    uint32_t           op_entered_cycle;

    /* 操作模式 */
    OpMode             op_mode;
    uint32_t           mode_switch_cycle;  /* 未使用，保留 */
    int                mode_switching;      /* 未使用，保留 */
    int                mode_stable;         /* 必须为1才能输出控制 */

    /* 上层控制模式 */
    ControlMode        ctrl_mode;

    /* 目标值 */
    float              target_vel_deg;
    int32_t            target_vel_counts;

    /* 当前实际值（零位校准后） */
    float              actual_pos_deg;
    int32_t            actual_pos_counts;
    
    /* 第七轴特殊参数 */
    float              actual_pos_mm;       /* 伸缩位置(mm) */
    float              target_pos_mm;       /* 目标位置(mm) */
    float              stroke_mm;           /* 行程(mm) */
    int                is_linear_axis;      /* 是否为直线轴 */
    
    /* 回零相关 */
    HomingState        homing_state;
    float              home_position_mm;    /* 零位位置(mm) */
    int                homing_required;     /* 是否需要回零 */

    /* 第八轴(六维力传感器)数据 */
    float              force_fx;            /* X方向力 (N) */
    float              force_fy;            /* Y方向力 (N) */
    float              force_fz;            /* Z方向力 (N) */
    float              torque_mx;           /* X方向力矩 (Nm) */
    float              torque_my;           /* Y方向力矩 (Nm) */
    float              torque_mz;           /* Z方向力矩 (Nm) */
    uint32_t           force_sensor_status; /* 传感器状态码 */
    uint32_t           force_sensor_counter;/* 采样计数器 */
    float              force_sensor_temp;   /* 传感器温度 (°C) */
    int                is_force_sensor;     /* 是否为力传感器 */

    /* 软件位置环状态 */
    CsvSoftPosState    csv_pos;
    CspHardPosState    csp_pos;              /* 未使用 */

    /* PDO 偏移量 */
    unsigned int       ctrl_word_offset;
    unsigned int       target_pos_offset;
    unsigned int       target_vel_offset;
    unsigned int       target_tor_offset;
    unsigned int       status_word_offset;
    unsigned int       actual_pos_offset;
    unsigned int       actual_vel_offset;
    unsigned int       actual_tor_offset;
    unsigned int       follow_err_offset;
    unsigned int       err_code_offset;
    unsigned int       op_mode_offset;
    
    /* 六维力传感器PDO偏移量 */
    unsigned int       force_control_offset;  /* 0x2000 ControlCode */
    unsigned int       force_fx_offset;
    unsigned int       force_fy_offset;
    unsigned int       force_fz_offset;
    unsigned int       torque_mx_offset;
    unsigned int       torque_my_offset;
    unsigned int       torque_mz_offset;
    unsigned int       force_status_offset;
    unsigned int       force_counter_offset;
    unsigned int       force_temp_offset;
} AxisData;

/* 主站生命周期状态 */
typedef enum {
    MY_MASTER_STATE_IDLE = 0,
    MY_MASTER_STATE_INIT,
    MY_MASTER_STATE_CONFIG_PDO,
    MY_MASTER_STATE_CONFIG_DC,
    MY_MASTER_STATE_ACTIVE,
    MY_MASTER_STATE_ERROR
} MasterLifecycleState;

/* 命令处理函数类型 */
typedef void (*cmd_handler_t)(MsgLongMsg* msg, ssize_t n);

/*==============================================================================
 * 全局数据
 *============================================================================*/

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_stop_requested = 0;
static unsigned long g_cycle_counter = 0;

static MasterLifecycleState g_master_state = MY_MASTER_STATE_IDLE;
static ec_master_t*         g_master = NULL;
static ec_domain_t*         g_domain = NULL;
static uint8_t*             g_domain_data = NULL;
static ec_slave_config_t*   g_slave_configs[MASTER_ARMS_MAX_SIZE] = {NULL};

static AxisData g_axes[MASTER_ARMS_MAX_SIZE];

/* 零位偏移（度，第七轴为mm） */
static const float AXIS_ZERO_OFFSETS[MASTER_ARMS_MAX_SIZE] = {
    17.03f, -17.131f, 39.143f, 47.096f, -24.272f, 57.861f, 0.0f  /* 第七轴零位偏移为0mm */
};

/* 软限位（度，第七轴为mm） */
static const float AXIS_SOFT_LIMIT_MIN[MASTER_ARMS_MAX_SIZE] = {
    -360.0f, -360.0f, -360.0f, -360.0f, -360.0f, -360.0f, -50.0f   /* 第七轴最小-50mm，允许收缩 */
};
static const float AXIS_SOFT_LIMIT_MAX[MASTER_ARMS_MAX_SIZE] = {
    360.0f, 360.0f, 360.0f, 360.0f, 360.0f, 360.0f, 50.0f          /* 第七轴最大50mm，允许伸出 */
};

/* PID 参数映射表 */
static const KguParamMapping KGU_PARAM_MAP[] = {
    {31, 0x2031, 0x00, 2, 0,    9999, 2000, "Cur_Kp",  "电流环比例增益"},
    {32, 0x2032, 0x00, 2, 0,    9999, 300,  "Cur_Ki",  "电流环积分增益"},
    {33, 0x2033, 0x00, 2, 0,    9999, 1800, "Vel_Kp",  "速度环比例增益"},
    {34, 0x2034, 0x00, 2, 0,    9999, 300,  "Vel_Ki",  "速度环积分增益"},
    {63, 0x2063, 0x00, 2, 1000, 5000, 2000, "Pos_Kp2", "位置环比例增益2(细调)"},
    {64, 0x2064, 0x00, 2, 0,    100,  0,    "Pos_FF",  "位置前馈百分比"},
    {65, 0x2065, 0x00, 2, 0,    24,   22,   "Pos_Kp1", "位置环比例增益1(粗调)"},
};
#define KGU_PARAM_COUNT (sizeof(KGU_PARAM_MAP) / sizeof(KGU_PARAM_MAP[0]))

static PidSdoReq g_pid_sdo_req[MASTER_ARMS_MAX_SIZE][KGU_PARAM_COUNT];
static ec_sdo_request_t* g_pid_save_sdo_req[MASTER_ARMS_MAX_SIZE] = {NULL};

static PidReadJob  g_pid_read_job  = {0};
static PidWriteJob g_pid_write_job = {0};
static PidSaveJob  g_pid_save_job  = {0};

static int32_t           g_sock_upper = -1;
static struct sockaddr_in g_upper_addr;
static int                g_upper_valid = 0;
static uint32_t           g_frame_id = 0;
static uint32_t           g_last_report_cycle = 0;

static ARMS_MASTER_THREAD_INFO* g_pmodule = NULL;

/*==============================================================================
 * 辅助函数声明
 *============================================================================*/

static inline float counts_to_degrees(int32_t counts);
static inline int32_t degrees_to_counts(float degrees);
static inline int32_t deg_per_sec_to_csv(float deg_per_sec);
static inline float rpm_to_joint_deg_per_sec(int32_t rpm);
static inline float counts_to_actual_degrees(int32_t counts, int axis);
static inline int32_t actual_degrees_to_counts(float actual_deg, int axis);
static inline float get_axis_zero_offset(int axis);
static inline float apply_velocity_limit(int axis, float requested_vel);

static uint16_t read_status_word(int axis);
static int32_t  read_actual_position(int axis);
static int32_t  read_actual_velocity(int axis);
static int16_t  read_actual_torque(int axis);
static uint16_t read_error_code(int axis);
static uint8_t  read_op_mode_display(int axis);
static int32_t  read_following_error(int axis);
static void     write_control_word(int axis, uint16_t word);
static void     write_target_velocity(int axis, int32_t vel);
static void     write_target_position(int axis, int32_t pos);
static void     write_target_torque(int axis, int16_t tor);
static void     write_operation_mode(int axis, uint8_t mode);

static SlaveCiAState parse_slave_state(uint16_t status_word);
static const char*   slave_state_str(SlaveCiAState state);
static const char*   op_mode_str(OpMode mode);

static void axis_state_machine(AxisData* axis);
static int  switch_op_mode(int axis, OpMode new_mode);  /* 保留但未使用 */

static void init_csv_soft_position(int axis, float target_actual_deg);
static void update_csv_soft_position(int axis);

static void init_csp_hard_position(int axis, int32_t target_counts);  /* 未使用 */
static void update_csp_hard_position(int axis);                       /* 未使用 */

static void pid_jobs_tick(void);
static void pid_start_read(uint8_t axis);
static void pid_start_write(uint8_t axis, uint8_t count, const uint8_t* data);
static void pid_start_save(uint8_t axis);
static int  pid_any_job_active(void);
static void send_pid_error_response(uint8_t axis, uint8_t error_code);

/* 命令处理函数 */
static void cmd_link_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_create_masters_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_conf_servo_pdos_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_start_op_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_destroy_master_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_fsa_shutdown_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_fsa_switch_on_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_fsa_enop_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_fsa_halt_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_clear_error_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_hand_vel_mm_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_hand_rela_pos_mm_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_hand_abs_pos_m_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_pid_read_params_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_pid_write_params_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_pid_save_to_flash_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_axis7_homing_handler(MsgLongMsg* msg, ssize_t n);
static void cmd_default_handler(MsgLongMsg* msg, ssize_t n);

static int  master_init(void);
static int  master_config_pdo(void);
static int  master_config_dc(void);
static int  master_activate(void);
static void master_cleanup(void);

static void init_upper_socket(void);
static void send_status_report(void);
static void check_send_report(void);

static void run_master_cycle(struct timespec* wake_time);
static void signal_handler(int sig);

/*==============================================================================
 * 命令分发表
 *============================================================================*/

typedef struct {
    int         cmd_id;
    cmd_handler_t handler;
    const char* name;
} CmdEntry;

static const CmdEntry g_cmd_handlers[] = {
    { CMD_LINK,                  cmd_link_handler,                 "CMD_LINK" },
    { CMD_IGH_CREATE_MASTERS,    cmd_create_masters_handler,       "CMD_IGH_CREATE_MASTERS" },
    { CMD_IGH_CONF_SERVO_PDOS,   cmd_conf_servo_pdos_handler,      "CMD_IGH_CONF_SERVO_PDOS" },
    { CMD_IGH_START_OP,          cmd_start_op_handler,             "CMD_IGH_START_OP" },
    { CMD_DESTROY_MASTER,        cmd_destroy_master_handler,       "CMD_DESTROY_MASTER" },
    { CMD_IGH_FSA_SHUTDOWN,      cmd_fsa_shutdown_handler,         "CMD_IGH_FSA_SHUTDOWN" },
    { CMD_IGH_FSA_SWITCH_ON,     cmd_fsa_switch_on_handler,        "CMD_IGH_FSA_SWITCH_ON" },
    { CMD_IGH_FSA_ENOP,          cmd_fsa_enop_handler,             "CMD_IGH_FSA_ENOP" },
    { CMD_IGH_FSA_HALT,          cmd_fsa_halt_handler,             "CMD_IGH_FSA_HALT" },
    { CMD_IGH_CLEAR_ERROR,       cmd_clear_error_handler,          "CMD_IGH_CLEAR_ERROR" },
    { CMD_HAND_VEL_MM,           cmd_hand_vel_mm_handler,          "CMD_HAND_VEL_MM" },
    { CMD_HAND_RELA_POS_MM,      cmd_hand_rela_pos_mm_handler,     "CMD_HAND_RELA_POS_MM" },
    { CMD_HAND_ABS_POS_M,        cmd_hand_abs_pos_m_handler,       "CMD_HAND_ABS_POS_M" },
    { CMD_PID_READ_PARAMS,       cmd_pid_read_params_handler,      "CMD_PID_READ_PARAMS" },
    { CMD_PID_WRITE_PARAMS,      cmd_pid_write_params_handler,     "CMD_PID_WRITE_PARAMS" },
    { CMD_PID_SAVE_TO_FLASH,     cmd_pid_save_to_flash_handler,    "CMD_PID_SAVE_TO_FLASH" },
    { CMD_AXIS7_HOMING,          cmd_axis7_homing_handler,         "CMD_AXIS7_HOMING" },
    { -1,                        cmd_default_handler,              "DEFAULT" }
};

/*==============================================================================
 * 辅助函数实现
 *============================================================================*/

static inline float counts_to_degrees(int32_t counts)
{
    return (float)counts * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
}

static inline int32_t degrees_to_counts(float degrees)
{
    return (int32_t)(degrees * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
}

static inline int32_t deg_per_sec_to_csv(float deg_per_sec)
{
    return (int32_t)(deg_per_sec * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
}

static inline float rpm_to_joint_deg_per_sec(int32_t rpm)
{
    return (float)rpm * 6.0f / GEAR_RATIO;
}

static inline float counts_to_actual_degrees(int32_t counts, int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0.0f;
    return counts_to_degrees(counts) + AXIS_ZERO_OFFSETS[axis];
}

static inline int32_t actual_degrees_to_counts(float actual_deg, int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    float mech_deg = actual_deg - AXIS_ZERO_OFFSETS[axis];
    return degrees_to_counts(mech_deg);
}

static inline float get_axis_zero_offset(int axis)
{
    return (axis >= 0 && axis < MASTER_ARMS_MAX_SIZE) ? AXIS_ZERO_OFFSETS[axis] : 0.0f;
}

static inline float apply_velocity_limit(int axis, float requested_vel)
{
    /* 第七轴使用mm单位 */
    float pos = (axis == AXIS7_ID) ? g_axes[axis].actual_pos_mm : g_axes[axis].actual_pos_deg;
    const char* unit = (axis == AXIS7_ID) ? "mm" : "°";
    
    if (pos > AXIS_SOFT_LIMIT_MAX[axis] && requested_vel > 0.0f) {
        printf("[master] 轴%d 超出上限(%.2f%s)，阻止正向速度 %.2f\n",
               axis, pos, unit, requested_vel);
        return 0.0f;
    }
    if (pos < AXIS_SOFT_LIMIT_MIN[axis] && requested_vel < 0.0f) {
        printf("[master] 轴%d 超出下限(%.2f%s)，阻止负向速度 %.2f\n",
               axis, pos, unit, requested_vel);
        return 0.0f;
    }
    return requested_vel;
}

/*============================================================================
 * 第七轴(伸缩轴)专用转换函数
 *===========================================================================*/
static inline int is_axis7(int axis)
{
    return (axis == AXIS7_ID);
}

static inline int is_axis8(int axis)
{
    return (axis == AXIS8_ID);
}

/* 第七轴：编码器计数转mm
 * 电机转一圈：AXIS7_ENCODER_COUNTS_PER_REV 计数
 * 经过减速比 AXIS7_GEAR_RATIO 后，丝杠转一圈
 * 丝杠导程 AXIS7_SCREW_PITCH_MM mm
 * 
 * 注意：添加负号反转方向，使伸出显示为正数
 */
static inline float axis7_counts_to_mm(int32_t counts)
{
    return -(float)counts * AXIS7_SCREW_PITCH_MM / 
           (AXIS7_ENCODER_COUNTS_PER_REV * AXIS7_GEAR_RATIO);
}

/* 第七轴：mm转编码器计数 */
static inline int32_t axis7_mm_to_counts(float mm)
{
    return (int32_t)(-mm * AXIS7_ENCODER_COUNTS_PER_REV * AXIS7_GEAR_RATIO / 
                     AXIS7_SCREW_PITCH_MM);
}

/* 第七轴：mm/s转内部速度计数 */
static inline int32_t axis7_mm_per_sec_to_counts(float mm_per_sec)
{
    /* 速度单位：计数/秒 = mm/s * (编码器计数/圈 * 减速比) / 导程(mm/圈)
     * 注意：添加负号保持与位置方向一致
     */
    return (int32_t)(-mm_per_sec * AXIS7_ENCODER_COUNTS_PER_REV * AXIS7_GEAR_RATIO / 
                     AXIS7_SCREW_PITCH_MM);
}

/* 第七轴：获取实际位置(mm)，考虑零位偏移 */
static inline float axis7_get_actual_mm(int32_t counts)
{
    return axis7_counts_to_mm(counts) + AXIS_ZERO_OFFSETS[AXIS7_ID];
}

/* 第七轴：实际位置(mm)转编码器计数 */
static inline int32_t axis7_actual_mm_to_counts(float actual_mm)
{
    float mech_mm = actual_mm - AXIS_ZERO_OFFSETS[AXIS7_ID];
    return axis7_mm_to_counts(mech_mm);
}

/*==============================================================================
 * PDO 读写封装
 *============================================================================*/

static inline uint16_t read_status_word(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    uint16_t val;
    memcpy(&val, g_domain_data + g_axes[axis].status_word_offset, 2);
    g_axes[axis].status_word = val;
    return val;
}

static inline int32_t read_actual_position(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int32_t counts;
    memcpy(&counts, g_domain_data + g_axes[axis].actual_pos_offset, 4);
    g_axes[axis].actual_pos_counts = counts;
    
    /* 第七轴使用mm单位 */
    if (is_axis7(axis)) {
        g_axes[axis].actual_pos_mm = axis7_get_actual_mm(counts);
        /* 保持deg字段也有值，用于兼容 */
        g_axes[axis].actual_pos_deg = g_axes[axis].actual_pos_mm;
    } else {
        g_axes[axis].actual_pos_deg = counts_to_actual_degrees(counts, axis);
    }
    return counts;
}

static inline int32_t read_actual_velocity(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int32_t val;
    memcpy(&val, g_domain_data + g_axes[axis].actual_vel_offset, 4);
    return val;
}

static inline int16_t read_actual_torque(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int16_t val;
    memcpy(&val, g_domain_data + g_axes[axis].actual_tor_offset, 2);
    return val;
}

static inline uint16_t read_error_code(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    uint16_t val;
    memcpy(&val, g_domain_data + g_axes[axis].err_code_offset, 2);
    return val;
}

static inline uint8_t read_op_mode_display(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    uint8_t val;
    memcpy(&val, g_domain_data + g_axes[axis].op_mode_offset, 1);
    return val;
}

/* 读取六维力传感器数据 */
static inline void read_force_sensor_data(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    if (!g_axes[axis].is_force_sensor) return;
    
    int32_t fx, fy, fz, mx, my, mz;
    uint32_t status, counter, temp;
    
    /* 读取六维力数据 (32位有符号整数，需要转换为物理单位) */
    memcpy(&fx, g_domain_data + g_axes[axis].force_fx_offset, 4);
    memcpy(&fy, g_domain_data + g_axes[axis].force_fy_offset, 4);
    memcpy(&fz, g_domain_data + g_axes[axis].force_fz_offset, 4);
    memcpy(&mx, g_domain_data + g_axes[axis].torque_mx_offset, 4);
    memcpy(&my, g_domain_data + g_axes[axis].torque_my_offset, 4);
    memcpy(&mz, g_domain_data + g_axes[axis].torque_mz_offset, 4);
    memcpy(&status, g_domain_data + g_axes[axis].force_status_offset, 4);
    memcpy(&counter, g_domain_data + g_axes[axis].force_counter_offset, 4);
    memcpy(&temp, g_domain_data + g_axes[axis].force_temp_offset, 4);
    
    /* 转换为物理单位 (假设数据以0.001N或0.001Nm为单位) */
    g_axes[axis].force_fx = fx / FORCE_SENSOR_SCALE;
    g_axes[axis].force_fy = fy / FORCE_SENSOR_SCALE;
    g_axes[axis].force_fz = fz / FORCE_SENSOR_SCALE;
    g_axes[axis].torque_mx = mx / FORCE_SENSOR_SCALE;
    g_axes[axis].torque_my = my / FORCE_SENSOR_SCALE;
    g_axes[axis].torque_mz = mz / FORCE_SENSOR_SCALE;
    g_axes[axis].force_sensor_status = status;
    g_axes[axis].force_sensor_counter = counter;
    g_axes[axis].force_sensor_temp = temp / FORCE_SENSOR_SCALE;
}

static inline int32_t read_following_error(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    
    if (is_axis7(axis)) {
        /* YKB200没有跟随误差(0x60f4)在PDO中，返回0 */
        return 0;
    } else {
        /* 前6轴使用32位跟随误差 */
        int32_t val;
        memcpy(&val, g_domain_data + g_axes[axis].follow_err_offset, 4);
        return val;
    }
}

static inline void write_control_word(int axis, uint16_t word)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_axes[axis].ctrl_word_offset, &word, 2);
    g_axes[axis].ctrl_word = word;
}

static inline void write_target_velocity(int axis, int32_t vel)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_axes[axis].target_vel_offset, &vel, 4);
}

static inline void write_target_position(int axis, int32_t pos)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_axes[axis].target_pos_offset, &pos, 4);
}

static inline void write_target_torque(int axis, int16_t tor)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_axes[axis].target_tor_offset, &tor, 2);
}

static inline void write_operation_mode(int axis, uint8_t mode)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    if (g_axes[axis].op_mode_offset == 0) return;  /* 未注册该PDO条目 */
    memcpy(g_domain_data + g_axes[axis].op_mode_offset, &mode, 1);
}

/*==============================================================================
 * CiA402 状态解析
 *============================================================================*/

static SlaveCiAState parse_slave_state(uint16_t status_word)
{
    if (status_word & 0x0008) return SLAVE_FAULT;
    if ((status_word & 0x006F) == 0x0007) return SLAVE_QUICK_STOP_ACTIVE;

    switch (status_word & 0x006F) {
        case 0x0000: return SLAVE_NOT_READY;
        case 0x0040: return SLAVE_SWITCH_ON_DISABLED;
        case 0x0021: return SLAVE_READY_TO_SWITCH_ON;
        case 0x0023: return SLAVE_SWITCHED_ON;
        case 0x0027: return SLAVE_OPERATION_ENABLED;
        default: break;
    }

    switch (status_word) {
        case 0x0250: return SLAVE_SWITCH_ON_DISABLED;
        case 0x0231: return SLAVE_READY_TO_SWITCH_ON;
        case 0x0233: return SLAVE_SWITCHED_ON;
        case 0x0237: return SLAVE_OPERATION_ENABLED;
        case 0x0210: return SLAVE_OPERATION_ENABLED;
        case 0x0218: return SLAVE_FAULT;
        default: break;
    }
    return SLAVE_UNKNOWN;
}

static const char* slave_state_str(SlaveCiAState state)
{
    switch (state) {
        case SLAVE_NOT_READY:           return "Not Ready";
        case SLAVE_SWITCH_ON_DISABLED:  return "Switch On Disabled";
        case SLAVE_READY_TO_SWITCH_ON:  return "Ready to Switch On";
        case SLAVE_SWITCHED_ON:         return "Switched On";
        case SLAVE_OPERATION_ENABLED:   return "Operation Enabled";
        case SLAVE_QUICK_STOP_ACTIVE:   return "Quick Stop";
        case SLAVE_FAULT:                return "Fault";
        default:                         return "Unknown";
    }
}

static const char* op_mode_str(OpMode mode)
{
    switch (mode) {
        case OP_MODE_CSP: return "CSP";
        case OP_MODE_CSV: return "CSV";
        default:          return "NONE";
    }
}

/*==============================================================================
 * 从站状态机
 *============================================================================*/

static void axis_state_machine(AxisData* axis)
{
    uint16_t status = axis->status_word;
    SlaveCiAState new_state = parse_slave_state(status);
    uint16_t err_code = read_error_code(axis - g_axes);

    if (new_state != axis->state) {
        printf("[master] 轴%ld 状态: %s -> %s (0x%04X, 错误码: 0x%04X)\n",
               axis - g_axes,
               slave_state_str(axis->state), slave_state_str(new_state),
               status, err_code);

        if (new_state == SLAVE_FAULT) {
            printf("[master] 轴%ld 故障: 0x%04X\n", axis - g_axes, err_code);
            axis->in_op = 0;
            axis->enable_op_requested = 0;
            axis->csv_pos.active = 0;
            axis->ctrl_mode = CTRL_MODE_IDLE;
        }
        else if (new_state == SLAVE_OPERATION_ENABLED) {
            if (!axis->in_op) {
                printf("[master] 轴%ld 进入 Operation Enabled\n", axis - g_axes);
                axis->in_op = 1;
                axis->enable_op_requested = 0;
                axis->op_just_entered = 1;
                axis->op_entered_cycle = (uint32_t)g_cycle_counter;
            }
        } else {
            axis->in_op = 0;
        }
        axis->state = new_state;
    }

    static uint32_t last_fault_clear[MASTER_ARMS_MAX_SIZE] = {0};
    int idx = axis - g_axes;
    if (new_state == SLAVE_FAULT && g_cycle_counter - last_fault_clear[idx] > 500) {
        last_fault_clear[idx] = (uint32_t)g_cycle_counter;
        printf("[master] 轴%d 自动尝试故障恢复...\n", idx);
        write_control_word(idx, 0x0080);
    }
}

/*==============================================================================
 * 模式切换（未使用，保留定义）
 *============================================================================*/

static int switch_op_mode(int axis, OpMode new_mode)
{
    /* 本函数在原始代码中未被调用，保留空实现以避免链接错误 */
    (void)axis; (void)new_mode;
    return 0;
}

/*==============================================================================
 * 软件位置环
 *============================================================================*/

static void init_csv_soft_position(int axis, float target_actual)
{
    AxisData* ad = &g_axes[axis];
    CsvSoftPosState* pos = &ad->csv_pos;

    memset(pos, 0, sizeof(CsvSoftPosState));

    /* 第七轴使用mm单位 */
    if (is_axis7(axis)) {
        pos->target_pos_deg = target_actual;  /* 复用字段，存储mm值 */
        pos->start_pos_deg = ad->actual_pos_mm;
        pos->current_pos_deg = ad->actual_pos_mm;
        pos->active = 1;
        pos->arrived = 0;

        /* 计算当前速度(mm/s) */
        int32_t vel_counts = read_actual_velocity(axis);
        float current_vel_mm_per_sec = axis7_counts_to_mm(vel_counts);
        for (int i = 0; i < UPPER_POS_SMOOTH_CNT; i++) {
            pos->vel_history[i] = current_vel_mm_per_sec;
        }
        pos->last_vel_cmd = current_vel_mm_per_sec;

        printf("[master] 轴%d(伸缩轴) 启动 PD 位置环: 当前 %.2fmm -> 目标 %.2fmm\n",
               axis, ad->actual_pos_mm, target_actual);
    } else {
        pos->target_pos_deg = target_actual;
        pos->start_pos_deg = ad->actual_pos_deg;
        pos->current_pos_deg = ad->actual_pos_deg;
        pos->active = 1;
        pos->arrived = 0;

        float current_vel = rpm_to_joint_deg_per_sec(read_actual_velocity(axis));
        for (int i = 0; i < UPPER_POS_SMOOTH_CNT; i++) {
            pos->vel_history[i] = current_vel;
        }
        pos->last_vel_cmd = current_vel;

        printf("[master] 轴%d 启动 PD 位置环: 实际 %.2f° -> 实际 %.2f°\n",
               axis, ad->actual_pos_deg, target_actual);
    }
}

static void update_csv_soft_position(int axis)
{
    AxisData* ad = &g_axes[axis];
    CsvSoftPosState* pos = &ad->csv_pos;
    if (!pos->active) return;

    pos->cycle_cnt++;

    /* 第七轴使用mm单位，需要不同的控制参数 */
    if (is_axis7(axis)) {
        float pos_actual_mm = ad->actual_pos_mm;
        int32_t vel_counts = read_actual_velocity(axis);
        float vel_actual_mm_per_sec = axis7_counts_to_mm(vel_counts);
        pos->current_pos_deg = pos_actual_mm;  /* 复用字段 */

        float pos_error = pos->target_pos_deg - pos_actual_mm;  /* target_pos_deg复用为mm */

        if (pos->arrived) {
            write_target_velocity(axis, 0);
            if (fabsf(pos_error) > 0.2f) {  /* 第七轴使用更大的容差 */
                printf("[master] 轴%d(伸缩轴) 位置偏离 %.3fmm，重新启动\n", axis, pos_error);
                init_csv_soft_position(axis, pos->target_pos_deg);
            }
            return;
        }

        /* 第七轴PD参数（针对直线运动优化） */
        float axis7_pos_kp = 20.0f;
        float axis7_pos_kd = 2.0f;
        float vel_cmd = axis7_pos_kp * pos_error - axis7_pos_kd * vel_actual_mm_per_sec;

        float abs_error = fabsf(pos_error);
        if (abs_error < 0.5f) {
            float max_vel_near = AXIS7_MAX_SPEED_MM_PER_SEC * abs_error * 2.0f;
            if (fabsf(vel_cmd) > max_vel_near) {
                vel_cmd = (vel_cmd > 0) ? max_vel_near : -max_vel_near;
            }
        }

        if (abs_error < 0.05f && fabsf(vel_actual_mm_per_sec) < 0.2f) {
            vel_cmd = 0;
        }

        float acc = (vel_cmd - pos->last_vel_cmd) / POS_CONTROL_PERIOD_S;
        if (acc > AXIS7_MAX_ACC_MM_PER_SEC2) {
            vel_cmd = pos->last_vel_cmd + AXIS7_MAX_ACC_MM_PER_SEC2 * POS_CONTROL_PERIOD_S;
        } else if (acc < -AXIS7_MAX_ACC_MM_PER_SEC2) {
            vel_cmd = pos->last_vel_cmd - AXIS7_MAX_ACC_MM_PER_SEC2 * POS_CONTROL_PERIOD_S;
        }

        vel_cmd = LIMIT(vel_cmd, -AXIS7_MAX_SPEED_MM_PER_SEC, AXIS7_MAX_SPEED_MM_PER_SEC);
        pos->last_vel_cmd = vel_cmd;

        if (fabsf(vel_cmd) > pos->max_speed_reached) {
            pos->max_speed_reached = fabsf(vel_cmd);
        }

        pos->vel_history[pos->history_idx] = vel_cmd;
        pos->history_idx = (pos->history_idx + 1) % UPPER_POS_SMOOTH_CNT;
        float vel_smooth = 0;
        for (int i = 0; i < UPPER_POS_SMOOTH_CNT; i++) {
            vel_smooth += pos->vel_history[i];
        }
        vel_smooth /= UPPER_POS_SMOOTH_CNT;
        vel_smooth = apply_velocity_limit(axis, vel_smooth);

        /* 第七轴到位判断 */
        if (abs_error < 0.02f && fabsf(vel_actual_mm_per_sec) < 0.1f) {
            pos->arrival_hold_cnt++;
            if (pos->arrival_hold_cnt >= (ARRIVAL_HOLD_MS / POS_CONTROL_PERIOD_MS)) {
                pos->arrived = 1;
                vel_smooth = 0;
                printf("[master] 轴%d(伸缩轴) 到位稳定: 目标 %.3fmm\n", axis, pos->target_pos_deg);
            }
        } else {
            pos->arrival_hold_cnt = 0;
        }

        ad->target_vel_counts = axis7_mm_per_sec_to_counts(vel_smooth);
        write_target_velocity(axis, ad->target_vel_counts);
    } else {
        /* 普通旋转轴的控制逻辑 */
        float pos_actual = ad->actual_pos_deg;
        float vel_actual = rpm_to_joint_deg_per_sec(read_actual_velocity(axis));
        pos->current_pos_deg = pos_actual;

        float pos_error = pos->target_pos_deg - pos_actual;

        if (pos->arrived) {
            write_target_velocity(axis, 0);
            if (fabsf(pos_error) > 0.5f) {
                printf("[master] 轴%d 位置偏离 %.3f°，重新启动\n", axis, pos_error);
                init_csv_soft_position(axis, pos->target_pos_deg);
            }
            return;
        }

        float vel_cmd = POS_KP * pos_error - POS_KD * vel_actual;

        float abs_error = fabsf(pos_error);
        if (abs_error < 1.0f) {
            float max_vel_near = MAX_VEL_DEG_PER_SEC * abs_error;
            if (fabsf(vel_cmd) > max_vel_near) {
                vel_cmd = (vel_cmd > 0) ? max_vel_near : -max_vel_near;
            }
        }

        if (abs_error < FORCE_ZERO_VEL_THRESHOLD && fabsf(vel_actual) < 2.0f) {
            vel_cmd = 0;
        }

        float acc = (vel_cmd - pos->last_vel_cmd) / POS_CONTROL_PERIOD_S;
        if (acc > MAX_ACC_DEG_PER_SEC2) {
            vel_cmd = pos->last_vel_cmd + MAX_ACC_DEG_PER_SEC2 * POS_CONTROL_PERIOD_S;
        } else if (acc < -MAX_ACC_DEG_PER_SEC2) {
            vel_cmd = pos->last_vel_cmd - MAX_ACC_DEG_PER_SEC2 * POS_CONTROL_PERIOD_S;
        }

        vel_cmd = LIMIT(vel_cmd, -MAX_VEL_DEG_PER_SEC, MAX_VEL_DEG_PER_SEC);
        pos->last_vel_cmd = vel_cmd;

        if (fabsf(vel_cmd) > pos->max_speed_reached) {
            pos->max_speed_reached = fabsf(vel_cmd);
        }

        pos->vel_history[pos->history_idx] = vel_cmd;
        pos->history_idx = (pos->history_idx + 1) % UPPER_POS_SMOOTH_CNT;
        float vel_smooth = 0;
        for (int i = 0; i < UPPER_POS_SMOOTH_CNT; i++) {
            vel_smooth += pos->vel_history[i];
        }
        vel_smooth /= UPPER_POS_SMOOTH_CNT;
        vel_smooth = apply_velocity_limit(axis, vel_smooth);

        if (abs_error < POS_ARRIVAL_THRESHOLD && fabsf(vel_actual) < VEL_ARRIVAL_THRESHOLD) {
            pos->arrival_hold_cnt++;
            if (pos->arrival_hold_cnt >= (ARRIVAL_HOLD_MS / POS_CONTROL_PERIOD_MS)) {
                pos->arrived = 1;
                vel_smooth = 0;
                printf("[master] 轴%d 到位稳定: 目标 %.3f°\n", axis, pos->target_pos_deg);
            }
        } else {
            pos->arrival_hold_cnt = 0;
        }

        ad->target_vel_counts = deg_per_sec_to_csv(vel_smooth);
        write_target_velocity(axis, ad->target_vel_counts);
    }
}

/*==============================================================================
 * 硬件位置环（未使用，保留定义）
 *============================================================================*/

static void init_csp_hard_position(int axis, int32_t target_counts)
{
    (void)axis; (void)target_counts;
}

static void update_csp_hard_position(int axis)
{
    (void)axis;
}

/*==============================================================================
 * PID 作业处理
 *============================================================================*/

static int pid_any_job_active(void)
{
    return g_pid_read_job.active || g_pid_write_job.active || g_pid_save_job.active;
}

static void send_pid_error_response(uint8_t axis, uint8_t error_code)
{
    uint8_t resp[4] = {axis, 0, 0, error_code};
    if (g_sock_upper >= 0 && g_upper_valid) {
        sendto(g_sock_upper, resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
    }
}

static inline float pid_decode_value(const uint8_t* data, size_t size)
{
    if (!data || size == 0) return 0.0f;
    if (size >= 4) return (float)EC_READ_S32(data);
    if (size >= 2) return (float)EC_READ_S16(data);
    return (float)data[0];
}

static inline void pid_encode_int16(uint8_t* data, float val)
{
    EC_WRITE_S16(data, (int16_t)val);
}

static void pid_jobs_tick(void)
{
    const uint32_t guard_cycles = PID_JOB_GUARD_TIMEOUT_MS / 2;
    const uint32_t step_guard_cycles = (PID_SDO_TIMEOUT_MS / 2) + 50;

    /* ----- 读取作业 ----- */
    if (g_pid_read_job.active) {
        PidReadJob* job = &g_pid_read_job;
        if (g_cycle_counter - job->start_cycle > guard_cycles) {
            printf("[master] PID 读任务总超时，放弃 axis=%d\n", job->axis);
            if (job->resp.paramCount > 0) {
                int pkt_size = 4 + job->resp.paramCount * sizeof(KGU_PID_PARAM_ITEM);
                sendto(g_sock_upper, &job->resp, pkt_size, 0,
                       (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
            } else {
                send_pid_error_response(job->axis, 6);
            }
            job->active = 0;
            return;
        }

        uint8_t axis = job->axis;
        uint8_t mi = job->map_idx;

        if (axis >= MASTER_ARMS_MAX_SIZE || mi >= KGU_PARAM_COUNT) {
            int pkt_size = 4 + job->resp.paramCount * sizeof(KGU_PID_PARAM_ITEM);
            sendto(g_sock_upper, &job->resp, pkt_size, 0,
                   (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
            job->active = 0;
            return;
        }

        ec_sdo_request_t* req = g_pid_sdo_req[axis][mi].req;
        if (!req || !g_pid_sdo_req[axis][mi].created) {
            printf("[master] 警告: 轴%d Pn%u 的 SDO 请求未创建，使用默认值\n",
                   axis, KGU_PARAM_MAP[mi].paramNo);
            uint8_t out_i = job->resp.paramCount;
            if (out_i < 8) {
                KGU_PID_PARAM_ITEM* item = &job->resp.params[out_i];
                memset(item, 0, sizeof(*item));
                item->paramNo = KGU_PARAM_MAP[mi].paramNo;
                item->value = KGU_PARAM_MAP[mi].defaultVal;
                item->minValue = KGU_PARAM_MAP[mi].minVal;
                item->maxValue = KGU_PARAM_MAP[mi].maxVal;
                item->isValid = 0;
                strncpy(item->name, KGU_PARAM_MAP[mi].name, 15);
                item->name[15] = '\0';
                strncpy(item->description, KGU_PARAM_MAP[mi].desc, 31);
                item->description[31] = '\0';
                job->resp.paramCount++;
            }
            job->map_idx++;
            job->waiting = 0;
            return;
        }

        if (!job->waiting) {
            ec_request_state_t st = ecrt_sdo_request_state(req);
            if (st != EC_REQUEST_BUSY) {
                uint8_t out_i = job->resp.paramCount;
                if (out_i >= 8) {
                    int pkt_size = 4 + job->resp.paramCount * sizeof(KGU_PID_PARAM_ITEM);
                    sendto(g_sock_upper, &job->resp, pkt_size, 0,
                           (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
                    job->active = 0;
                    return;
                }

                KGU_PID_PARAM_ITEM* item = &job->resp.params[out_i];
                memset(item, 0, sizeof(*item));
                item->paramNo = KGU_PARAM_MAP[mi].paramNo;
                item->minValue = KGU_PARAM_MAP[mi].minVal;
                item->maxValue = KGU_PARAM_MAP[mi].maxVal;
                strncpy(item->name, KGU_PARAM_MAP[mi].name, 15);
                item->name[15] = '\0';
                strncpy(item->description, KGU_PARAM_MAP[mi].desc, 31);
                item->description[31] = '\0';

                ecrt_sdo_request_read(req);
                job->waiting = 1;
                job->step_cycle = (uint32_t)g_cycle_counter;
            }
        } else {
            ec_request_state_t st = ecrt_sdo_request_state(req);
            uint8_t out_i = job->resp.paramCount;
            if (st == EC_REQUEST_BUSY) {
                if (g_cycle_counter - job->step_cycle > step_guard_cycles) {
                    job->resp.params[out_i].value = KGU_PARAM_MAP[mi].defaultVal;
                    job->resp.params[out_i].isValid = 0;
                    printf("[master] PID 读取超时 axis=%d Pn%u\n",
                           axis, KGU_PARAM_MAP[mi].paramNo);
                    job->resp.paramCount++;
                    job->map_idx++;
                    job->waiting = 0;
                }
            } else if (st == EC_REQUEST_SUCCESS) {
                const uint8_t* data = ecrt_sdo_request_data(req);
                size_t sz = ecrt_sdo_request_data_size(req);
                job->resp.params[out_i].value = pid_decode_value(data, sz);
                job->resp.params[out_i].isValid = 1;
                printf("[master] PID 读取成功 axis=%d Pn%u = %.1f\n",
                       axis, KGU_PARAM_MAP[mi].paramNo, job->resp.params[out_i].value);
                job->resp.paramCount++;
                job->map_idx++;
                job->waiting = 0;
            } else {
                job->resp.params[out_i].value = KGU_PARAM_MAP[mi].defaultVal;
                job->resp.params[out_i].isValid = 0;
                printf("[master] PID 读取失败 axis=%d Pn%u state=%d\n",
                       axis, KGU_PARAM_MAP[mi].paramNo, (int)st);
                job->resp.paramCount++;
                job->map_idx++;
                job->waiting = 0;
            }
        }
    }

    /* ----- 写入作业 ----- */
    if (g_pid_write_job.active) {
        PidWriteJob* job = &g_pid_write_job;
        if (g_cycle_counter - job->start_cycle > guard_cycles) {
            printf("[master] PID 写任务总超时，放弃 axis=%d\n", job->axis);
            job->active = 0;
            return;
        }

        uint8_t axis = job->axis;
        if (axis >= MASTER_ARMS_MAX_SIZE) {
            job->active = 0;
            return;
        }

        if (job->cur >= job->total) {
            sendto(g_sock_upper, &job->resp, sizeof(job->resp), 0,
                   (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
            job->active = 0;
            return;
        }

        uint16_t paramNo = job->items[job->cur].paramNo;
        float target = job->items[job->cur].value;

        int mi = -1;
        for (int k = 0; k < (int)KGU_PARAM_COUNT; k++) {
            if (KGU_PARAM_MAP[k].paramNo == paramNo) {
                mi = k;
                break;
            }
        }
        if (mi < 0) {
            job->cur++;
            job->waiting = 0;
            return;
        }

        if (target < KGU_PARAM_MAP[mi].minVal || target > KGU_PARAM_MAP[mi].maxVal) {
            job->cur++;
            job->waiting = 0;
            return;
        }

        ec_sdo_request_t* req = g_pid_sdo_req[axis][mi].req;
        if (!req || !g_pid_sdo_req[axis][mi].created) {
            job->cur++;
            job->waiting = 0;
            return;
        }

        if (!job->waiting) {
            ec_request_state_t st = ecrt_sdo_request_state(req);
            if (st != EC_REQUEST_BUSY) {
                pid_encode_int16(ecrt_sdo_request_data(req), target);
                ecrt_sdo_request_write(req);
                job->waiting = 1;
                job->step_cycle = (uint32_t)g_cycle_counter;
            }
        } else {
            ec_request_state_t st = ecrt_sdo_request_state(req);
            if (st == EC_REQUEST_BUSY) {
                if (g_cycle_counter - job->step_cycle > step_guard_cycles) {
                    printf("[master] PID 写入超时 axis=%d Pn%u\n", axis, paramNo);
                    job->cur++;
                    job->waiting = 0;
                }
            } else {
                if (st == EC_REQUEST_SUCCESS) {
                    job->resp.successCount++;
                } else {
                    printf("[master] PID 写入失败 axis=%d Pn%u state=%d\n",
                           axis, paramNo, (int)st);
                }
                job->cur++;
                job->waiting = 0;
            }
        }
    }

    /* PID 保存作业轮询 */
    if (g_pid_save_job.active) {
        PidSaveJob* job = &g_pid_save_job;
        uint8_t axis = job->axis;
        ec_sdo_request_t* req = g_pid_save_sdo_req[axis];

        /* 日志：当前作业状态 */
        printf("[master] 保存作业 tick: axis=%d, active=%d, waiting=%d, cycle=%lu, start_cycle=%u\n",
            axis, job->active, job->waiting, g_cycle_counter, job->start_cycle);

        /* 总超时检查 */
        if (g_cycle_counter - job->start_cycle > guard_cycles) {
            printf("[master] PID 保存任务总超时 (guard_cycles=%u)，放弃 axis=%d\n",
                guard_cycles, axis);
            job->resp.success = 0;
            job->resp.errorCode = 6;  /* 超时错误 */
            sendto(g_sock_upper, &job->resp, sizeof(job->resp), 0,
                (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
            job->active = 0;
            return;
        }

        /* 检查 SDO 请求对象是否存在 */
        if (!req) {
            printf("[master] 错误: 轴%d 保存 SDO 未创建\n", axis);
            job->resp.success = 0;
            job->resp.errorCode = 7;  /* 内部错误 */
            sendto(g_sock_upper, &job->resp, sizeof(job->resp), 0,
                (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
            job->active = 0;
            return;
        }

        if (!job->waiting) {
            /* 未处于等待状态：检查 SDO 是否空闲，若空闲则触发写操作 */
            ec_request_state_t st = ecrt_sdo_request_state(req);
            printf("[master] 轴%d 保存 SDO 状态（未等待）: %d\n", axis, (int)st);

            if (st != EC_REQUEST_BUSY) {
                uint32_t save_cmd = 0x65766173;  /* "save" 签名 */
                memcpy(ecrt_sdo_request_data(req), &save_cmd, 4);
                ecrt_sdo_request_write(req);
                job->waiting = 1;
                job->step_cycle = (uint32_t)g_cycle_counter;
                printf("[master] 轴%d 触发参数保存 (0x1010:01)，step_cycle=%u\n",
                    axis, job->step_cycle);
            } else {
                printf("[master] 轴%d 保存 SDO 正忙，等待下周期\n", axis);
            }
        } else {
            /* 等待 SDO 完成 */
            ec_request_state_t st = ecrt_sdo_request_state(req);
            printf("[master] 轴%d 保存等待中，SDO 状态: %d, step_cycle=%u, 已过周期=%lu\n",
                axis, (int)st, job->step_cycle, g_cycle_counter - job->step_cycle);

            if (st == EC_REQUEST_BUSY) {
                /* 检查步骤超时 */
                if (g_cycle_counter - job->step_cycle > step_guard_cycles) {
                    printf("[master] 轴%d 保存 SDO 超时 (step_guard_cycles=%u)\n",
                        axis, step_guard_cycles);
                    job->resp.success = 0;
                    job->resp.errorCode = 8;  /* 步骤超时 */
                    sendto(g_sock_upper, &job->resp, sizeof(job->resp), 0,
                        (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
                    job->active = 0;
                }
            } else {
                /* SDO 完成（成功或失败） */
                if (st == EC_REQUEST_SUCCESS) {
                    printf("[master] 轴%d 参数保存成功\n", axis);
                    job->resp.success = 1;
                    job->resp.errorCode = 0;
                } else {
                    printf("[master] 轴%d 参数保存失败 (SDO状态=%d)\n", axis, (int)st);
                    job->resp.success = 0;
                    job->resp.errorCode = 9;  /* 通用失败 */
                }
                sendto(g_sock_upper, &job->resp, sizeof(job->resp), 0,
                    (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
                job->active = 0;
            }
        }
    }
}

static void pid_start_read(uint8_t axis)
{
    memset(&g_pid_read_job, 0, sizeof(g_pid_read_job));
    g_pid_read_job.active = 1;
    g_pid_read_job.axis = axis;
    g_pid_read_job.map_idx = 0;
    g_pid_read_job.waiting = 0;
    g_pid_read_job.start_cycle = (uint32_t)g_cycle_counter;
    g_pid_read_job.resp.axisId = axis;
    g_pid_read_job.resp.paramCount = 0;
    g_pid_read_job.resp.resultCode = 0;
}

static void pid_start_write(uint8_t axis, uint8_t count, const uint8_t* data)
{
    memset(&g_pid_write_job, 0, sizeof(g_pid_write_job));
    g_pid_write_job.active = 1;
    g_pid_write_job.axis = axis;
    g_pid_write_job.total = (count > 8) ? 8 : count;
    g_pid_write_job.cur = 0;
    g_pid_write_job.waiting = 0;
    g_pid_write_job.start_cycle = (uint32_t)g_cycle_counter;
    g_pid_write_job.resp.axisId = axis;
    g_pid_write_job.resp.successCount = 0;
    g_pid_write_job.resp.totalCount = g_pid_write_job.total;
    g_pid_write_job.resp.reserved = 0;

    for (int i = 0; i < g_pid_write_job.total; i++) {
        int offset = 4 + i * 6;
        g_pid_write_job.items[i].paramNo = *(uint16_t*)(data + offset);
        g_pid_write_job.items[i].value = *(float*)(data + offset + 2);
    }
}

static void pid_start_save(uint8_t axis)
{
    memset(&g_pid_save_job, 0, sizeof(g_pid_save_job));
    g_pid_save_job.active = 1;
    g_pid_save_job.axis = axis;
    g_pid_save_job.waiting = 0;
    g_pid_save_job.start_cycle = (uint32_t)g_cycle_counter;
    g_pid_save_job.resp.axisId = axis;
    g_pid_save_job.resp.success = 0;
    g_pid_save_job.resp.errorCode = 0;
}

/*==============================================================================
 * 命令处理函数实现
 *============================================================================*/

static void cmd_link_handler(MsgLongMsg* msg, ssize_t n)
{
    printf("[master] 收到 LINK 命令\n");
    if (n >= (ssize_t)(MSG_DATA_OFFSET + 6)) {
        memcpy(&g_upper_addr.sin_port, msg->mData, 2);
        memcpy(&g_upper_addr.sin_addr.s_addr, msg->mData + 2, 4);
        g_upper_addr.sin_family = AF_INET;
        g_upper_valid = 1;
        printf("[master] 上位机地址: %s:%d\n",
               inet_ntoa(g_upper_addr.sin_addr), ntohs(g_upper_addr.sin_port));
    } else if (n > (ssize_t)MSG_DATA_OFFSET) {
        uint16_t port = *(uint16_t*)msg->mData;
        g_upper_addr.sin_port = htons(port);
        g_upper_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        g_upper_addr.sin_family = AF_INET;
        g_upper_valid = 1;
        printf("[master] 上位机端口: %d (IP 默认 127.0.0.1)\n", port);
    }
    int32_t resp = 1;
    sendto(g_sock_upper, &resp, sizeof(resp), 0,
           (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
}

static void cmd_create_masters_handler(MsgLongMsg* msg, ssize_t n)
{
    (void)msg; (void)n;
    printf("[master] 收到命令: 创建主站\n");
    if (g_master_state == MY_MASTER_STATE_IDLE) {
        if (master_init() == 0) {
            g_master_state = MY_MASTER_STATE_INIT;
        }
    }
}

static void cmd_conf_servo_pdos_handler(MsgLongMsg* msg, ssize_t n)
{
    (void)msg; (void)n;
    printf("[master] 收到命令: 配置 PDO\n");
    if (g_master_state == MY_MASTER_STATE_INIT) {
        if (master_config_pdo() == 0) {
            g_master_state = MY_MASTER_STATE_CONFIG_PDO;
        }
    }
}

static void cmd_start_op_handler(MsgLongMsg* msg, ssize_t n)
{
    (void)msg; (void)n;
    printf("[master] 收到命令: 激活主站\n");
    if (g_master_state == MY_MASTER_STATE_CONFIG_PDO ||
        g_master_state == MY_MASTER_STATE_INIT) {
        if (g_master_state == MY_MASTER_STATE_INIT) {
            if (master_config_pdo() != 0) return;
            g_master_state = MY_MASTER_STATE_CONFIG_PDO;
        }
        if (master_config_dc() != 0) return;
        if (master_activate() == 0) {
            g_master_state = MY_MASTER_STATE_ACTIVE;
            for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                g_axes[i].enable_op_requested = 0;
                g_axes[i].in_op = 0;
                g_axes[i].ctrl_word = 0x0000;
                /* 关键：确保模式稳定标志为1 */
                g_axes[i].mode_stable = 1;
            }
            send_status_report();
            printf("[master] 主站激活完成\n");
        }
    }
}

static void cmd_destroy_master_handler(MsgLongMsg* msg, ssize_t n)
{
    (void)msg; (void)n;
    printf("[master] 收到命令: 销毁主站\n");
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        write_control_word(i, 0x0000);
    }
    if (g_domain_data) {
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
        usleep(100000);
    }
    master_cleanup();
}

static void handle_fsa_command(const char* cmd_name, MsgLongMsg* msg, ssize_t n,
                               void (*action)(int axis))
{
    uint8_t mask = 0x3F;
    if (n > (ssize_t)MSG_DATA_OFFSET) {
        mask = (uint8_t)msg->mData[0];
    }
    printf("[master] 收到 %s, 轴掩码: 0x%02X\n", cmd_name, mask);

    if (g_master_state != MY_MASTER_STATE_ACTIVE) {
        printf("[master] 主站未激活，忽略命令\n");
        return;
    }

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (mask & (1 << i)) {
            action(i);
        }
    }
}

static void fsa_shutdown_action(int axis)
{
    printf("[master]  -> 控制轴%d Shutdown\n", axis);
    g_axes[axis].enable_op_requested = 0;
    g_axes[axis].in_op = 0;
    g_axes[axis].csv_pos.active = 0;
    g_axes[axis].ctrl_mode = CTRL_MODE_IDLE;
    write_control_word(axis, 0x0006);
}

static void fsa_switch_on_action(int axis)
{
    write_control_word(axis, 0x0007);
}

static void fsa_enop_action(int axis)
{
    uint16_t status = g_axes[axis].status_word;
    SlaveCiAState state = parse_slave_state(status);
    if (state != SLAVE_SWITCHED_ON) {
        printf("[master] 轴%d 错误: 当前状态不是 Switched On\n", axis);
        return;
    }
    printf("[master] 轴%d 准备进入 OP 状态...\n", axis);
    g_axes[axis].enable_op_requested = 1;
    g_axes[axis].op_just_entered = 1;
    g_axes[axis].op_entered_cycle = (uint32_t)g_cycle_counter;
}

static void fsa_halt_action(int axis)
{
    g_axes[axis].enable_op_requested = 0;
    g_axes[axis].in_op = 0;
    g_axes[axis].csv_pos.active = 0;
    g_axes[axis].ctrl_mode = CTRL_MODE_IDLE;
    write_control_word(axis, 0x0000);
    g_axes[axis].target_vel_deg = 0.0f;
    g_axes[axis].target_vel_counts = 0;
}

static void fsa_clear_error_action(int axis)
{
    g_axes[axis].enable_op_requested = 0;
    g_axes[axis].in_op = 0;
    g_axes[axis].csv_pos.active = 0;
    g_axes[axis].ctrl_mode = CTRL_MODE_IDLE;
    write_control_word(axis, 0x0080);
}

static void cmd_fsa_shutdown_handler(MsgLongMsg* msg, ssize_t n)
{
    handle_fsa_command("CMD_IGH_FSA_SHUTDOWN", msg, n, fsa_shutdown_action);
}

static void cmd_fsa_switch_on_handler(MsgLongMsg* msg, ssize_t n)
{
    handle_fsa_command("CMD_IGH_FSA_SWITCH_ON", msg, n, fsa_switch_on_action);
}

static void cmd_fsa_enop_handler(MsgLongMsg* msg, ssize_t n)
{
    handle_fsa_command("CMD_IGH_FSA_ENOP", msg, n, fsa_enop_action);
}

static void cmd_fsa_halt_handler(MsgLongMsg* msg, ssize_t n)
{
    handle_fsa_command("CMD_IGH_FSA_HALT", msg, n, fsa_halt_action);
}

static void cmd_clear_error_handler(MsgLongMsg* msg, ssize_t n)
{
    handle_fsa_command("CMD_IGH_CLEAR_ERROR", msg, n, fsa_clear_error_action);
}

static void cmd_hand_vel_mm_handler(MsgLongMsg* msg, ssize_t n)
{
    if (g_master_state != MY_MASTER_STATE_ACTIVE) return;

    UPPER_VEL_MSG* vel = (UPPER_VEL_MSG*)msg->mData;

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (vel->mModuleFlag[i]) {
            AxisData* ad = &g_axes[i];
            if (!ad->in_op) {
                printf("[master] 轴%d 错误: 从站未进入 OP 状态\n", i);
                continue;
            }

            /* 直接设置速度，不进行模式切换 */
            float new_vel = vel->mVel[i];
            
            /* 第七轴使用mm/s单位 */
            if (is_axis7(i)) {
                /* 限制第七轴速度范围 */
                if (new_vel > AXIS7_MAX_SPEED_MM_PER_SEC) 
                    new_vel = AXIS7_MAX_SPEED_MM_PER_SEC;
                if (new_vel < -AXIS7_MAX_SPEED_MM_PER_SEC) 
                    new_vel = -AXIS7_MAX_SPEED_MM_PER_SEC;
                
                ad->target_vel_deg = new_vel;  /* 复用字段，单位mm/s */
                ad->target_vel_counts = axis7_mm_per_sec_to_counts(new_vel);
                ad->ctrl_mode = CTRL_MODE_CSV_DIRECT_VEL;
                printf("[master] 轴%d(伸缩轴) CSV 速度: %.3f mm/s\n", i, new_vel);
            } else {
                /* 普通旋转轴使用deg/s */
                if (new_vel > 90.0f) new_vel = 90.0f;
                if (new_vel < -90.0f) new_vel = -90.0f;

                ad->target_vel_deg = new_vel;
                ad->target_vel_counts = deg_per_sec_to_csv(new_vel);
                ad->ctrl_mode = CTRL_MODE_CSV_DIRECT_VEL;
                printf("[master] 轴%d CSV 速度: %.3f deg/s\n", i, ad->target_vel_deg);
            }
        }
    }
}

static void cmd_hand_rela_pos_mm_handler(MsgLongMsg* msg, ssize_t n)
{
    if (g_master_state != MY_MASTER_STATE_ACTIVE) return;

    uint8_t* flags = (uint8_t*)msg->mData;
    float*   pos   = (float*)(msg->mData + 8);

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (flags[i]) {
            AxisData* ad = &g_axes[i];
            if (!ad->in_op) {
                printf("[master] 轴%d 错误: 从站未在 OP 状态\n", i);
                continue;
            }

            /* 第七轴使用mm单位 */
            if (is_axis7(i)) {
                float current_mm = ad->actual_pos_mm;
                float target_mm = current_mm + pos[i];
                
                /* 软限位检查 */
                if (target_mm > AXIS_SOFT_LIMIT_MAX[i]) {
                    printf("[master] 轴%d(伸缩轴) 目标位置 %.3fmm 超出上限，限制为 %.3fmm\n",
                           i, target_mm, AXIS_SOFT_LIMIT_MAX[i]);
                    target_mm = AXIS_SOFT_LIMIT_MAX[i];
                }
                if (target_mm < AXIS_SOFT_LIMIT_MIN[i]) {
                    printf("[master] 轴%d(伸缩轴) 目标位置 %.3fmm 超出下限，限制为 %.3fmm\n",
                           i, target_mm, AXIS_SOFT_LIMIT_MIN[i]);
                    target_mm = AXIS_SOFT_LIMIT_MIN[i];
                }
                
                printf("[master] 轴%d(伸缩轴) 相对位置: 当前 %.3fmm + %.3fmm = 目标 %.3fmm\n",
                       i, current_mm, pos[i], target_mm);

                ad->target_pos_mm = target_mm;
                ad->csp_pos.active = 0;
                init_csv_soft_position(i, target_mm);  /* 使用mm作为目标 */
                ad->ctrl_mode = CTRL_MODE_CSV_SOFT_POS;
            } else {
                float target_actual = ad->actual_pos_deg + pos[i];
                printf("[master] 轴%d 相对位置: 当前实际 %.3f° + %.3f° = 目标实际 %.3f°\n",
                       i, ad->actual_pos_deg, pos[i], target_actual);

                /* 直接启动软件位置环，无模式切换 */
                ad->csp_pos.active = 0;
                init_csv_soft_position(i, target_actual);
                ad->ctrl_mode = CTRL_MODE_CSV_SOFT_POS;
            }
        }
    }
}

static void cmd_hand_abs_pos_m_handler(MsgLongMsg* msg, ssize_t n)
{
    if (g_master_state != MY_MASTER_STATE_ACTIVE) return;

    uint8_t* flags = (uint8_t*)msg->mData;
    float*   pos   = (float*)(msg->mData + 8);

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (flags[i]) {
            AxisData* ad = &g_axes[i];
            if (!ad->in_op) continue;

            /* 第七轴使用mm单位 */
            if (is_axis7(i)) {
                float target_mm = pos[i];
                
                /* 软限位检查 */
                if (target_mm > AXIS_SOFT_LIMIT_MAX[i]) {
                    printf("[master] 轴%d(伸缩轴) 绝对位置 %.3fmm 超出上限，限制为 %.3fmm\n",
                           i, target_mm, AXIS_SOFT_LIMIT_MAX[i]);
                    target_mm = AXIS_SOFT_LIMIT_MAX[i];
                }
                if (target_mm < AXIS_SOFT_LIMIT_MIN[i]) {
                    printf("[master] 轴%d(伸缩轴) 绝对位置 %.3fmm 超出下限，限制为 %.3fmm\n",
                           i, target_mm, AXIS_SOFT_LIMIT_MIN[i]);
                    target_mm = AXIS_SOFT_LIMIT_MIN[i];
                }
                
                printf("[master] 轴%d(伸缩轴) 绝对位置: 当前 %.3fmm -> 目标 %.3fmm\n",
                       i, ad->actual_pos_mm, target_mm);

                ad->target_pos_mm = target_mm;
                ad->csp_pos.active = 0;
                init_csv_soft_position(i, target_mm);
                ad->ctrl_mode = CTRL_MODE_CSV_SOFT_POS;
            } else {
                float target_actual = pos[i];
                printf("[master] 轴%d 绝对位置: 当前实际 %.3f° -> 目标实际 %.3f°\n",
                       i, ad->actual_pos_deg, target_actual);

                ad->csp_pos.active = 0;
                init_csv_soft_position(i, target_actual);
                ad->ctrl_mode = CTRL_MODE_CSV_SOFT_POS;
            }
        }
    }
}

static void cmd_pid_read_params_handler(MsgLongMsg* msg, ssize_t n)
{
    uint8_t axis = *(uint8_t*)msg->mData;
    if (axis >= MASTER_ARMS_MAX_SIZE) {
        send_pid_error_response(axis, 2);
        return;
    }

    if (g_master_state != MY_MASTER_STATE_ACTIVE) {
        send_pid_error_response(axis, 1);
        return;
    }

    ec_master_state_t ms;
    ecrt_master_state(g_master, &ms);
    if (ms.al_states < 0x02) {
        send_pid_error_response(axis, 3);
        return;
    }

    if (pid_any_job_active()) {
        send_pid_error_response(axis, 6);
        return;
    }

    printf("[master] PID 读取请求(异步): axis=%d\n", axis);
    pid_start_read(axis);
}

static void cmd_pid_write_params_handler(MsgLongMsg* msg, ssize_t n)
{
    if (g_master_state != MY_MASTER_STATE_ACTIVE) {
        KGU_PID_WRITE_RESP_4BYTE resp = {0xFF, 0, 0, 0};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        return;
    }

    const uint8_t* data = (const uint8_t*)msg->mData;
    uint8_t axis = data[0];
    uint8_t count = data[1];

    if (axis >= MASTER_ARMS_MAX_SIZE) {
        KGU_PID_WRITE_RESP_4BYTE resp = {axis, 0, count, 0};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        return;
    }

    if (pid_any_job_active()) {
        KGU_PID_WRITE_RESP_4BYTE resp = {axis, 0, count, 2};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        return;
    }

    pid_start_write(axis, count, data);
}



static void cmd_pid_save_to_flash_handler(MsgLongMsg* msg, ssize_t n)
{

    if (g_master_state != MY_MASTER_STATE_ACTIVE) {
        KGU_PID_SAVE_RESP resp = {0xFF, 0, 0xFFFF};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        return;
    }

    uint8_t axis = *(uint8_t*)msg->mData;
    if (axis >= MASTER_ARMS_MAX_SIZE) {
        KGU_PID_SAVE_RESP resp = {axis, 0, 1};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        return;
    }

    if (pid_any_job_active()) {
        KGU_PID_SAVE_RESP resp = {axis, 0, 2};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        return;
    }

    pid_start_save(axis);
}

/*==============================================================================
 * 第七轴(伸缩轴)回零功能
 * 由于使用单圈编码器，断电后位置会丢失，需要回零
 *===========================================================================*/

/* 回零响应结构 */
typedef struct {
    uint8_t  axisId;
    uint8_t  status;      /* 0=进行中, 1=完成, 2=错误 */
    uint16_t errorCode;
    float    currentPos;  /* 当前位置(mm) */
} AXIS7_HOMING_RESP;

static void cmd_axis7_homing_handler(MsgLongMsg* msg, ssize_t n)
{
    if (g_master_state != MY_MASTER_STATE_ACTIVE) {
        AXIS7_HOMING_RESP resp = {AXIS7_ID, 2, 0xFFFF, 0.0f};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        printf("[master] 回零失败: 主站未激活\n");
        return;
    }

    AxisData* ad = &g_axes[AXIS7_ID];
    
    if (!ad->in_op) {
        AXIS7_HOMING_RESP resp = {AXIS7_ID, 2, 1, ad->actual_pos_mm};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
        printf("[master] 轴%d(伸缩轴) 回零失败: 未在OP状态\n", AXIS7_ID);
        return;
    }

    /* 解析回零参数 */
    float target_home_mm = 0.0f;  /* 默认回到0mm位置 */
    float homing_speed_mm_per_sec = 2.0f;  /* 默认回零速度 */
    
    if (n >= (ssize_t)(MSG_DATA_OFFSET + sizeof(float))) {
        target_home_mm = *(float*)(msg->mData);
    }
    if (n >= (ssize_t)(MSG_DATA_OFFSET + 2 * sizeof(float))) {
        homing_speed_mm_per_sec = *(float*)(msg->mData + sizeof(float));
        /* 限制回零速度 */
        if (homing_speed_mm_per_sec > AXIS7_MAX_SPEED_MM_PER_SEC) {
            homing_speed_mm_per_sec = AXIS7_MAX_SPEED_MM_PER_SEC;
        }
        if (homing_speed_mm_per_sec < 0.5f) {
            homing_speed_mm_per_sec = 0.5f;
        }
    }

    printf("[master] 轴%d(伸缩轴) 开始回零: 目标 %.2fmm, 速度 %.2fmm/s\n",
           AXIS7_ID, target_home_mm, homing_speed_mm_per_sec);

    /* 设置回零状态 */
    ad->homing_state = HOMING_STATE_START;
    ad->home_position_mm = target_home_mm;
    ad->homing_required = 1;

    /* 启动位置控制移动到零点 */
    ad->target_pos_mm = target_home_mm;
    ad->csp_pos.active = 0;
    init_csv_soft_position(AXIS7_ID, target_home_mm);
    ad->ctrl_mode = CTRL_MODE_CSV_SOFT_POS;

    /* 发送开始回零的响应 */
    AXIS7_HOMING_RESP resp = {AXIS7_ID, 0, 0, ad->actual_pos_mm};
    sendto(g_sock_upper, &resp, sizeof(resp), 0,
           (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
}

/* 检查回零状态并更新 */
static void check_axis7_homing_status(void)
{
    AxisData* ad = &g_axes[AXIS7_ID];
    
    if (ad->homing_state == HOMING_STATE_IDLE || 
        ad->homing_state == HOMING_STATE_COMPLETED ||
        ad->homing_state == HOMING_STATE_ERROR) {
        return;
    }

    /* 检查位置控制是否到位 */
    if (ad->csv_pos.arrived && ad->homing_state == HOMING_STATE_START) {
        ad->homing_state = HOMING_STATE_COMPLETED;
        ad->homing_required = 0;
        printf("[master] 轴%d(伸缩轴) 回零完成: 当前位置 %.3fmm\n", 
               AXIS7_ID, ad->actual_pos_mm);
        
        /* 发送完成响应 */
        AXIS7_HOMING_RESP resp = {AXIS7_ID, 1, 0, ad->actual_pos_mm};
        sendto(g_sock_upper, &resp, sizeof(resp), 0,
               (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
    }
}

static void cmd_default_handler(MsgLongMsg* msg, ssize_t n)
{
    printf("[master] 未知命令: %d\n", msg->mMsgId);
}

/*==============================================================================
 * 主站生命周期函数
 *============================================================================*/

static int master_init(void)
{
    printf("[master] 初始化 EtherCAT 主站 (轴数 %d)...\n", MASTER_ARMS_MAX_SIZE);

    memset(&g_axes, 0, sizeof(g_axes));
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        g_axes[i].state = SLAVE_NOT_READY;
        g_axes[i].op_mode = OP_MODE_NONE;
        g_axes[i].mode_stable = 1;   /* 默认稳定 */
        
        /* 第七轴特殊初始化 */
        if (is_axis7(i)) {
            g_axes[i].is_linear_axis = 1;
            g_axes[i].stroke_mm = AXIS7_STROKE_MM;
            g_axes[i].homing_required = 1;  /* 单圈编码器需要回零 */
            g_axes[i].homing_state = HOMING_STATE_IDLE;
            printf("[master] 轴%d 初始化为伸缩轴(直线运动), 行程 %.1fmm\n", 
                   i, AXIS7_STROKE_MM);
        } else if (is_axis8(i)) {
            g_axes[i].is_force_sensor = 1;
            g_axes[i].force_fx = 0.0f;
            g_axes[i].force_fy = 0.0f;
            g_axes[i].force_fz = 0.0f;
            g_axes[i].torque_mx = 0.0f;
            g_axes[i].torque_my = 0.0f;
            g_axes[i].torque_mz = 0.0f;
            printf("[master] 轴%d 初始化为六维力传感器\n", i);
        }
    }

    g_master = ecrt_request_master(0);
    if (!g_master) {
        fprintf(stderr, "[master] 错误: 无法请求主站 0\n");
        return -1;
    }

    g_domain = ecrt_master_create_domain(g_master);
    if (!g_domain) {
        fprintf(stderr, "[master] 错误: 无法创建域\n");
        ecrt_release_master(g_master);
        g_master = NULL;
        return -1;
    }

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (i <= 5) {
            g_slave_configs[i] = ecrt_master_slave_config(g_master, 0, i, VENDOR_ID, PRODUCT_ID);
        } else if ( i == 6) {
            g_slave_configs[i] = ecrt_master_slave_config(g_master, 0, i, YKB_VENDOR_ID, YKB_PRODUCT_ID);
        } else if ( i == 7) {
            g_slave_configs[i] = ecrt_master_slave_config(g_master, 0, i, ACTI_VENDOR_ID, ACTI_PRODUCT_ID);
        }
        if (!g_slave_configs[i]) {
            fprintf(stderr, "[master] 错误: 无法配置从站 %d\n", i);
            ecrt_release_master(g_master);
            g_master = NULL;
            return -1;
        }
    }

    printf("[master] 主站初始化完成\n");
    return 0;
}

static int master_config_pdo(void)
{
    printf("[master] 配置 PDO...\n");

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (!is_axis7(i) && !is_axis8(i)) {
            /* 前6轴(KDE电机): 设置默认操作模式为 CSV (9)
             * 注意：第七轴(YKB200)和第八轴(力传感器)不设置0x6060
             */
            uint8_t op_mode = 9;
            int ret_sdo = ecrt_slave_config_sdo8(g_slave_configs[i], 0x6060, 0x00, op_mode);
            if (ret_sdo != 0) {
                fprintf(stderr, "[master] 错误: 无法设置轴%d 默认 CSV 模式, ret=%d\n", i, ret_sdo);
                return -1;
            }
            /* 可选设置 */
            ecrt_slave_config_sdo32(g_slave_configs[i], 0x607F, 0x00, 3000);
            ecrt_slave_config_sdo32(g_slave_configs[i], 0x60C5, 0x00, 10000);
            ecrt_slave_config_sdo32(g_slave_configs[i], 0x60C6, 0x00, 10000);
        }
        /* 第七轴(YKB200)和第八轴(力传感器)不通过SDO预设置操作模式 */
        g_axes[i].op_mode = OP_MODE_CSV;

        if (is_axis7(i)) {
            /* 第七轴(YKB200)使用预配置的PDO，不进行重新配置
             * YKB200已经预配置了正确的PDO (0x1600/0x1A00)
             * 跳过ecrt_slave_config_pdos()，直接注册PDO条目
             */
            printf("[master] 轴%d(YKB200) 跳过PDO重新配置，使用预配置\n", i);
        } else if (is_axis8(i)) {
            /* 第八轴(六维力传感器)使用预配置的PDO，不进行重新配置
             * 传感器已经预配置了正确的PDO (0x1600/0x1A00)
             * 跳过ecrt_slave_config_pdos()，直接注册PDO条目
             */
            printf("[master] 轴%d(六维力传感器) 跳过PDO重新配置，使用预配置\n", i);
        } else {
            /* 前6轴(KDE电机)使用原有PDO配置
             * RxPDO (0x1601): 控制字、目标位置、目标速度、目标力矩
             * TxPDO (0x1A01): 状态字、实际位置、实际速度、实际力矩、跟随误差、错误码、操作模式显示
             */
            static ec_pdo_entry_info_t output_entries[] = {
                {0x6040, 0x00, 16},
                {0x607a, 0x00, 32},
                {0x60ff, 0x00, 32},
                {0x6071, 0x00, 16},
            };
            static ec_pdo_entry_info_t input_entries[] = {
                {0x6041, 0x00, 16},
                {0x6064, 0x00, 32},
                {0x606c, 0x00, 32},
                {0x6077, 0x00, 16},
                {0x60f4, 0x00, 32},
                {0x603f, 0x00, 16},
                {0x6061, 0x00, 8},
            };
            static ec_pdo_info_t pdos[] = {
                {0x1601, 4, output_entries},
                {0x1A01, 7, input_entries},
            };
            static ec_sync_info_t syncs[] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, 1, pdos + 0, EC_WD_DISABLE},
                {3, EC_DIR_INPUT,  1, pdos + 1, EC_WD_DISABLE},
                {0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DISABLE}
            };

            int ret_pdo = ecrt_slave_config_pdos(g_slave_configs[i], EC_END, syncs);
            if (ret_pdo != 0) {
                fprintf(stderr, "[master] 错误: 配置轴%d PDO 失败, ret=%d\n", i, ret_pdo);
                return -1;
            }
        }
    }

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        unsigned int bit_pos;
        if (is_axis7(i)) {
            /* 第七轴(YKB200)的PDO注册 - 使用0x1600/0x1A00
              * RxPDO (0x1600): 0x6040, 0x6060, 0x607A, 0x6071, 0x607F, 0x60FF (6个条目)
              * TxPDO (0x1A00): 0x6041, 0x6061, 0x603F, 0x6064, 0x6077, 0x606C (6个条目)
              * 注意: YKB200没有0x60f4(跟随误差)在PDO中
              */
             /* RxPDO entries */
             g_axes[i].ctrl_word_offset   = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6040, 0, g_domain, &bit_pos);
             g_axes[i].op_mode_offset     = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6060, 0, g_domain, &bit_pos);
             g_axes[i].target_pos_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x607a, 0, g_domain, &bit_pos);
             g_axes[i].target_tor_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6071, 0, g_domain, &bit_pos);
             /* 0x607F (Max Profile Velocity) - 不存储偏移 */
             ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x607f, 0, g_domain, &bit_pos);
             g_axes[i].target_vel_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x60ff, 0, g_domain, &bit_pos);
             /* TxPDO entries */
             g_axes[i].status_word_offset = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6041, 0, g_domain, &bit_pos);
             /* 0x6061 (Mode of Operation Display) - 不存储偏移 */
             ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6061, 0, g_domain, &bit_pos);
             g_axes[i].err_code_offset    = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x603f, 0, g_domain, &bit_pos);
             g_axes[i].actual_pos_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6064, 0, g_domain, &bit_pos);
             g_axes[i].actual_tor_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6077, 0, g_domain, &bit_pos);
             g_axes[i].actual_vel_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x606c, 0, g_domain, &bit_pos);
             /* YKB200没有跟随误差(0x60f4)在PDO中 */
             g_axes[i].follow_err_offset  = 0;  /* 标记为无效 */
             printf("[master] 轴%d(YKB200) PDO注册: ctrl=0x%x, mode=0x%x, status=0x%x, pos=0x%x\n", 
                    i, g_axes[i].ctrl_word_offset, g_axes[i].op_mode_offset, 
                    g_axes[i].status_word_offset, g_axes[i].actual_pos_offset);
        } else if (is_axis8(i)) {
            /* 第八轴(六维力传感器)的PDO注册 - 使用0x1600/0x1A00
             * TxPDO (0x1A00): Fx, Fy, Fz, Mx, My, Mz, StatusCode, SampleCounter, Temper (9个条目)
             */
            /* RxPDO entries (控制码和参数) */
            g_axes[i].force_control_offset = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2000, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2001, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2002, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2003, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2004, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2005, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2006, 0, g_domain, &bit_pos);
            ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x2007, 0, g_domain, &bit_pos);
            
            /* TxPDO entries (六维力数据) */
            g_axes[i].force_fx_offset     = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4000, 0, g_domain, &bit_pos);
            g_axes[i].force_fy_offset     = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4001, 0, g_domain, &bit_pos);
            g_axes[i].force_fz_offset     = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4002, 0, g_domain, &bit_pos);
            g_axes[i].torque_mx_offset    = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4003, 0, g_domain, &bit_pos);
            g_axes[i].torque_my_offset    = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4004, 0, g_domain, &bit_pos);
            g_axes[i].torque_mz_offset    = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4005, 0, g_domain, &bit_pos);
            g_axes[i].force_status_offset = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4006, 0, g_domain, &bit_pos);
            g_axes[i].force_counter_offset= ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4007, 0, g_domain, &bit_pos);
            g_axes[i].force_temp_offset   = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x4008, 0, g_domain, &bit_pos);
            
            /* 为传感器注册兼容字段(使用StatusCode作为状态字) */
            g_axes[i].status_word_offset  = g_axes[i].force_status_offset;  /* 使用StatusCode作为状态指示 */
            g_axes[i].actual_pos_offset   = g_axes[i].force_fx_offset;      /* 占位，实际不使用 */
            
            printf("[master] 轴%d(六维力传感器) PDO注册: Ctrl=0x%x, Fx=0x%x, Fy=0x%x, Fz=0x%x, Mx=0x%x, My=0x%x, Mz=0x%x, Status=0x%x\n", 
                   i, g_axes[i].force_control_offset, g_axes[i].force_fx_offset, g_axes[i].force_fy_offset, 
                   g_axes[i].force_fz_offset, g_axes[i].torque_mx_offset, g_axes[i].torque_my_offset,
                   g_axes[i].torque_mz_offset, g_axes[i].force_status_offset);
        } else {
            /* 前6轴的PDO注册 - 使用0x1601/0x1A01 */
            g_axes[i].ctrl_word_offset   = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6040, 0, g_domain, &bit_pos);
            g_axes[i].target_pos_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x607a, 0, g_domain, &bit_pos);
            g_axes[i].target_vel_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x60ff, 0, g_domain, &bit_pos);
            g_axes[i].target_tor_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6071, 0, g_domain, &bit_pos);
            g_axes[i].status_word_offset = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6041, 0, g_domain, &bit_pos);
            g_axes[i].actual_pos_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6064, 0, g_domain, &bit_pos);
            g_axes[i].actual_vel_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x606c, 0, g_domain, &bit_pos);
            g_axes[i].actual_tor_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6077, 0, g_domain, &bit_pos);
            g_axes[i].follow_err_offset  = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x60f4, 0, g_domain, &bit_pos);
            g_axes[i].err_code_offset    = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x603f, 0, g_domain, &bit_pos);
            g_axes[i].op_mode_offset     = ecrt_slave_config_reg_pdo_entry(g_slave_configs[i], 0x6061, 0, g_domain, &bit_pos);
        }
    }

    /* 创建 PID 参数 SDO 请求 */
    for (int axis = 0; axis < MASTER_ARMS_MAX_SIZE; axis++) {
        for (int pi = 0; pi < (int)KGU_PARAM_COUNT; pi++) {
            if (!g_slave_configs[axis]) {
                printf("[master] 错误: 轴%d 从站配置为空，无法创建 SDO 请求\n", axis);
                continue;
            }
            ec_sdo_request_t* req = ecrt_slave_config_create_sdo_request(
                g_slave_configs[axis],
                KGU_PARAM_MAP[pi].index,
                KGU_PARAM_MAP[pi].subindex,
                KGU_PARAM_MAP[pi].dataType
            );
            g_pid_sdo_req[axis][pi].req = req;
            if (req) {
                ecrt_sdo_request_timeout(req, PID_SDO_TIMEOUT_MS);
                g_pid_sdo_req[axis][pi].created = 1;
                printf("[master] 轴%d Pn%u SDO 请求创建成功 (0x%04X:%u)\n",
                       axis, KGU_PARAM_MAP[pi].paramNo,
                       KGU_PARAM_MAP[pi].index, KGU_PARAM_MAP[pi].subindex);
            } else {
                g_pid_sdo_req[axis][pi].created = 0;
                printf("[master] 警告: 轴%d 创建 PID SDO 请求失败 Pn%u\n",
                       axis, KGU_PARAM_MAP[pi].paramNo);
            }
        }

        if (g_slave_configs[axis]) {
            g_pid_save_sdo_req[axis] = ecrt_slave_config_create_sdo_request(
                g_slave_configs[axis], 0x1010, 0x01, 4);
            if (g_pid_save_sdo_req[axis]) {
                ecrt_sdo_request_timeout(g_pid_save_sdo_req[axis], PID_SDO_TIMEOUT_MS);
                printf("[master] 轴%d 保存 SDO 请求创建成功\n", axis);
            } else {
                printf("[master] 错误: 轴%d 保存 SDO 请求创建失败\n", axis);
            }
        }
    }

    printf("[master] PDO 配置完成\n");
    return 0;
}

static int master_config_dc(void)
{
    printf("[master] 配置 DC 时钟同步...\n");
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (!g_slave_configs[i]) {
            printf("[master] 轴%d 从站配置为空，跳过DC配置\n", i);
            continue;
        }
        if (is_axis7(i)) {
            /* 第七轴(YKB200)不使用DC同步，避免进入OP失败 */
            printf("[master] 轴%d(YKB200) 跳过DC同步配置\n", i);
            continue;
        }
        /* 为支持DC的从站配置DC同步（包括第八轴力传感器） */
        printf("[master] 轴%d 配置DC同步\n", i);
        ecrt_slave_config_dc(g_slave_configs[i], DC_ASSIGN_ACTIVATE,
                             DC_SYNC0_CYCLE_TIME, DC_SYNC0_SHIFT, 0, 0);
    }
    printf("[master] DC 配置完成\n");
    return 0;
}

static int master_activate(void)
{
    printf("[master] 激活主站...\n");

    if (ecrt_master_activate(g_master) != 0) {
        fprintf(stderr, "[master] 错误: 激活主站失败\n");
        return -1;
    }

    g_domain_data = ecrt_domain_data(g_domain);
    if (!g_domain_data) {
        fprintf(stderr, "[master] 错误: 获取域数据失败\n");
        return -1;
    }

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        if (is_axis8(i)) {
            /* 第八轴(六维力传感器): 写入ControlCode = 1 启动传感器 */
            if (g_axes[i].force_control_offset > 0) {
                int32_t control_code = 1;
                memcpy(g_domain_data + g_axes[i].force_control_offset, &control_code, 4);
                printf("[master] 轴%d(力传感器) 写入ControlCode = 1 启动传感器 (偏移=0x%x)\n", 
                       i, g_axes[i].force_control_offset);
            } else {
                printf("[master] 警告: 轴%d(力传感器) ControlCode偏移无效!\n", i);
            }
        } else {
            /* 普通电机轴: 初始化控制字和速度 */
            write_control_word(i, 0x0000);
            write_target_velocity(i, 0);
        }
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_domain);
        int32_t pos = read_actual_position(i);
        if (!is_axis8(i)) {
            write_target_position(i, pos);
        }
        printf("[master] 轴%d 初始位置: %d counts (%.2f°)\n",
               i, pos, counts_to_degrees(pos));
    }
    ecrt_domain_queue(g_domain);
    ecrt_master_send(g_master);
    
    /* 额外发送几次数据确保力传感器收到ControlCode */
    for (int j = 0; j < 5; j++) {
        usleep(10000);  /* 10ms */
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_domain);
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
            if (is_axis8(i) && g_axes[i].force_control_offset > 0) {
                int32_t control_code = 1;
                memcpy(g_domain_data + g_axes[i].force_control_offset, &control_code, 4);
            }
        }
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
    }

    printf("[master] 等待 %d 个从站就绪...\n", MASTER_ARMS_MAX_SIZE);
    int retries = 0;
    const int max_retries = 500;
    int all_ready = 0;
    while (retries < max_retries && !all_ready) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t dc_time = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        ecrt_master_application_time(g_master, dc_time);
        ecrt_master_sync_slave_clocks(g_master);

        ecrt_master_receive(g_master);
        ecrt_domain_process(g_domain);

        all_ready = 1;
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
            uint16_t status = read_status_word(i);
            SlaveCiAState state = parse_slave_state(status);
            ec_master_state_t ms;
            ecrt_master_state(g_master, &ms);

            if (retries % 50 == 0) {
                printf("[master] 轴%d: AL=0x%02X, 状态=%s, 状态字=0x%04X\n",
                       i, ms.al_states, slave_state_str(state), status);
            }

            /* 对于第七轴(YKB200)和第八轴(六维力传感器)，允许更长的启动时间，暂时不阻止主站激活 */
            if (is_axis7(i) || is_axis8(i)) {
                if (state == SLAVE_FAULT && !is_axis8(i)) {
                    all_ready = 0;
                    write_control_word(i, 0x0080);
                }
                /* 第七轴和第八轴不检查AL状态，允许主站先激活其他轴 */
            } else {
                if (ms.al_states != 0x08 || state == SLAVE_FAULT) {
                    all_ready = 0;
                    if (state == SLAVE_FAULT) {
                        write_control_word(i, 0x0080);
                    }
                }
            }

            /* 第八轴(六维力传感器)不需要写入控制字 */
            if (!is_axis8(i)) {
                write_control_word(i, 0x0000);
                write_target_velocity(i, 0);
                write_target_position(i, g_axes[i].actual_pos_counts);
            }
        }

        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);

        usleep(10000);
        retries++;
    }

    /* 确保模式稳定标志为1 */
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        g_axes[i].mode_stable = 1;
    }

    printf("[master] 主站激活完成\n");
    return 0;
}

static void master_cleanup(void)
{
    printf("[master] 清理主站资源...\n");

    if (g_master) {
        ecrt_release_master(g_master);
        g_master = NULL;
    }
    g_domain = NULL;
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        g_slave_configs[i] = NULL;
    }
    g_domain_data = NULL;
    g_master_state = MY_MASTER_STATE_IDLE;

    printf("[master] 清理完成\n");
}

/*==============================================================================
 * 状态上报
 *============================================================================*/

static void init_upper_socket(void)
{
    g_sock_upper = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock_upper < 0) {
        printf("[master] 创建状态上报 socket 失败\n");
        return;
    }

    memset(&g_upper_addr, 0, sizeof(g_upper_addr));
    g_upper_addr.sin_family = AF_INET;
    g_upper_addr.sin_port = htons(UPPER_STATUS_PORT);
    g_upper_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    g_upper_valid = 1;

    printf("[master] 状态上报 socket 已创建\n");
}

static void send_status_report(void)
{
    if (g_sock_upper < 0 || !g_upper_valid) return;

    MASTER_STATUS_REPORT report;
    memset(&report, 0, sizeof(report));

    report.mFrameId = g_frame_id++;
    report.mTimestamp = (uint32_t)(g_cycle_counter * 2);
    report.masterState = (uint8_t)g_master_state;
    report.slaveCount = MASTER_ARMS_MAX_SIZE;
    report.cycleCount = (uint32_t)g_cycle_counter;

    if (g_master) {
        ec_master_state_t ms;
        ecrt_master_state(g_master, &ms);
        report.masterALState = ms.al_states;
    }

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        SLAVE_PDO_DATA* s = &report.slaves[i];
        s->statusWord = g_axes[i].status_word;
        /* 第七轴上报mm单位的位置，其他轴上报deg单位 */
        if (is_axis7(i)) {
            s->actualPosition = (int32_t)(g_axes[i].actual_pos_mm * 1000.0f);  /* mm * 1000 */
        } else {
            s->actualPosition = (int32_t)(g_axes[i].actual_pos_deg * 1000.0f); /* deg * 1000 */
        }
        s->actualVelocity = read_actual_velocity(i);
        s->actualTorque = read_actual_torque(i);
        s->followingError = read_following_error(i);
        s->errorCode = read_error_code(i);
        s->controlWord = g_axes[i].ctrl_word;
        s->targetPosition = 0;
        s->targetVelocity = g_axes[i].target_vel_counts;
        s->targetTorque = 0;
    }

    /* 填充第八轴(六维力传感器)数据 */
    int axis8_idx = AXIS8_ID;
    report.forceFx = g_axes[axis8_idx].force_fx;
    report.forceFy = g_axes[axis8_idx].force_fy;
    report.forceFz = g_axes[axis8_idx].force_fz;
    report.torqueMx = g_axes[axis8_idx].torque_mx;
    report.torqueMy = g_axes[axis8_idx].torque_my;
    report.torqueMz = g_axes[axis8_idx].torque_mz;
    report.forceSensorStatus = g_axes[axis8_idx].force_sensor_status;
    report.forceSensorCounter = g_axes[axis8_idx].force_sensor_counter;
    report.forceSensorTemp = g_axes[axis8_idx].force_sensor_temp;

    int flags = fcntl(g_sock_upper, F_GETFL, 0);
    fcntl(g_sock_upper, F_SETFL, flags | O_NONBLOCK);

    ssize_t sent = sendto(g_sock_upper, &report, sizeof(report), 0,
                           (struct sockaddr*)&g_upper_addr, sizeof(g_upper_addr));
    if (sent != sizeof(report)) {
        printf("[master] 上报失败: %zd/%zu\n", sent, sizeof(report));
    }
}

static void check_send_report(void)
{
    if (g_cycle_counter - g_last_report_cycle >= 50) {
        g_last_report_cycle = (uint32_t)g_cycle_counter;
        send_status_report();
    }
}

/*==============================================================================
 * 实时循环
 *============================================================================*/

static void run_master_cycle(struct timespec* wake_time)
{
    struct timespec rt_time;
    uint64_t dc_time;
    ec_master_state_t ms;

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wake_time, NULL);

    wake_time->tv_nsec += PERIOD_NS;
    while (wake_time->tv_nsec >= 1000000000) {
        wake_time->tv_nsec -= 1000000000;
        wake_time->tv_sec++;
    }

    if (!g_master || g_master_state != MY_MASTER_STATE_ACTIVE) {
        return;
    }

    ecrt_master_receive(g_master);
    ecrt_domain_process(g_domain);

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        read_actual_position(i);
        /* 六维力传感器: 周期性写入ControlCode保持运行 */
        if (is_axis8(i)) {
            if (g_axes[i].force_control_offset > 0) {
                int32_t control_code = 1;  /* 保持传感器运行 */
                memcpy(g_domain_data + g_axes[i].force_control_offset, &control_code, 4);
            }
            read_force_sensor_data(i);
        }
    }

    pid_jobs_tick();
    
    /* 检查第七轴回零状态 */
    check_axis7_homing_status();

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        AxisData* ad = &g_axes[i];

        /* 第八轴(六维力传感器)不需要CiA402状态机控制 */
        if (is_axis8(i)) {
            /* 传感器只需要读取数据，不需要控制字写入 */
            continue;
        }

        read_status_word(i);
        axis_state_machine(ad);

        /* 对于第七轴(YKB200)，通过PDO写入操作模式(CSV=9)
         * 因为0x6060包含在PDO映射中，必须经由PDO写入
         */
        if (is_axis7(i)) {
            write_operation_mode(i, OP_MODE_CSV);
        }

        /* 如果从站已进入 OP 且模式稳定，根据上层控制模式输出 */
        if (ad->in_op && ad->mode_stable) {
            switch (ad->ctrl_mode) {
                case CTRL_MODE_CSV_DIRECT_VEL: {
                    float vel = apply_velocity_limit(i, ad->target_vel_deg);
                    /* 第七轴(伸缩轴)直接使用mm/s单位，方向由axis7_mm_per_sec_to_counts处理 */
                    if (is_axis7(i)) {
                        write_target_velocity(i, axis7_mm_per_sec_to_counts(vel));
                    } else {
                        write_target_velocity(i, deg_per_sec_to_csv(vel));
                    }
                    write_target_position(i, 0);
                    write_control_word(i, 0x000F);
                    break;
                }
                case CTRL_MODE_CSV_SOFT_POS: {
                    /* 每5个周期更新一次位置环（原始行为） */
                    if (g_cycle_counter % 5 == 0) {
                        update_csv_soft_position(i);
                    } else {
                        write_target_velocity(i, ad->target_vel_counts);
                    }
                    write_target_position(i, 0);
                    write_control_word(i, 0x000F);
                    break;
                }
                default:
                    /* 空闲模式：输出零 */
                    write_target_velocity(i, 0);
                    write_target_position(i, ad->actual_pos_counts);
                    write_control_word(i, 0x000F);
                    break;
            }
        } else if (ad->enable_op_requested) {
            write_target_position(i, ad->actual_pos_counts);
            write_target_velocity(i, 0);
            write_control_word(i, 0x000F);
            if (ad->state == SLAVE_OPERATION_ENABLED) {
                ad->in_op = 1;
                ad->enable_op_requested = 0;
                ad->op_just_entered = 1;
                ad->op_entered_cycle = (uint32_t)g_cycle_counter;
            }
        } else if (ad->state == SLAVE_SWITCHED_ON) {
            write_target_position(i, ad->actual_pos_counts);
            write_target_velocity(i, 0);
            write_control_word(i, 0x0007);
        } else {
            write_control_word(i, ad->ctrl_word);
            write_target_position(i, ad->actual_pos_counts);
            write_target_velocity(i, 0);
        }

        if (ad->op_just_entered && ad->in_op) {
            if (g_cycle_counter - ad->op_entered_cycle < 10) {
                write_target_position(i, ad->actual_pos_counts);
                write_target_velocity(i, 0);
            } else {
                ad->op_just_entered = 0;
            }
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &rt_time);
    dc_time = (uint64_t)rt_time.tv_sec * 1000000000ULL + rt_time.tv_nsec;
    ecrt_master_application_time(g_master, dc_time);
    ecrt_master_sync_reference_clock(g_master);
    ecrt_master_sync_slave_clocks(g_master);

    ecrt_domain_queue(g_domain);
    ecrt_master_send(g_master);

    check_send_report();

    if (g_cycle_counter % 5000 == 0) {
        ecrt_master_state(g_master, &ms);
        printf("[master] 状态: AL=0x%02X, 周期=%lu\n", ms.al_states, g_cycle_counter);
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
            if (is_axis8(i)) {
                /* 六维力传感器数据显示 */
                printf("  轴%d(力传感器): Fx=%.3fN, Fy=%.3fN, Fz=%.3fN, Mx=%.3fNm, My=%.3fNm, Mz=%.3fNm\n",
                       i, g_axes[i].force_fx, g_axes[i].force_fy, g_axes[i].force_fz,
                       g_axes[i].torque_mx, g_axes[i].torque_my, g_axes[i].torque_mz);
            } else if (is_axis7(i)) {
                printf("  轴%d(伸缩轴): %s, 实际=%.2fmm\n",
                       i, slave_state_str(g_axes[i].state), g_axes[i].actual_pos_mm);
            } else {
                printf("  轴%d: %s, 实际=%.2f°\n",
                       i, slave_state_str(g_axes[i].state), g_axes[i].actual_pos_deg);
            }
        }
    }

    g_cycle_counter++;
}

/*==============================================================================
 * 信号处理
 *============================================================================*/

static void signal_handler(int sig)
{
    printf("\n[master] 收到信号 %d, 正在停止...\n", sig);
    g_running = 0;
    g_stop_requested = 1;
}

/*==============================================================================
 * 线程入口函数
 *============================================================================*/

void* master_arms_threadEntry(void* pModule)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall");
    }

    struct sched_param param = { .sched_priority = 80 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("pthread_setschedparam");
    }

    ARMS_MASTER_THREAD_INFO* pmodule = (ARMS_MASTER_THREAD_INFO*)pModule;
    if (!pmodule) return NULL;
    g_pmodule = pmodule;

    printf("========================================\n");
    printf(" EtherCAT 6轴主站控制程序 (稳定版)\n");
    printf(" 支持轴数: %d\n", MASTER_ARMS_MAX_SIZE);
    printf("========================================\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    pmodule->mSoket = createSoket(get_mn_master_port(0), MN_LocalIpV4Str, 0, 0);
    if (pmodule->mSoket < 0) {
        printf("[master] 创建 socket 失败\n");
        return NULL;
    }

    init_upper_socket();

    printf("[master] 等待上位机命令...\n");

    struct timespec wake_time;
    clock_gettime(CLOCK_MONOTONIC, &wake_time);

    while (g_running) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        char buf[2048];
        ssize_t n = recvfrom(pmodule->mSoket, buf, sizeof(buf), MSG_DONTWAIT,
                              (struct sockaddr*)&peer, &peer_len);
        if (n > 0) {
            MsgLongMsg* msg = (MsgLongMsg*)buf;
            int cmd = msg->mMsgId;
            if (cmd != CMD_HEARTBEAT) {
                printf("[master] 收到命令: %d, sender=%d\n", cmd, msg->mSender);
            }

            const CmdEntry* entry = g_cmd_handlers;
            int found = 0;
            while (entry->cmd_id != -1) {
                if (entry->cmd_id == cmd) {
                    entry->handler(msg, n);
                    found = 1;
                    break;
                }
                entry++;
            }
            if (!found) {
                cmd_default_handler(msg, n);
            }
        }

        if (g_master_state == MY_MASTER_STATE_ACTIVE) {
            run_master_cycle(&wake_time);
        } else {
            usleep(1000);
        }
    }

    printf("[master] 正在退出...\n");

    if (g_domain_data) {
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
            write_control_word(i, 0x0000);
        }
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
        usleep(100000);
    }

    master_cleanup();

    if (pmodule->mSoket >= 0) {
        close(pmodule->mSoket);
        pmodule->mSoket = -1;
    }
    if (g_sock_upper >= 0) {
        close(g_sock_upper);
        g_sock_upper = -1;
    }

    printf("[master] 已退出\n");
    return NULL;
}