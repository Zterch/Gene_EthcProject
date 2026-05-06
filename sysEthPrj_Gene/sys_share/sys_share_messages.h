/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_share_messages.h - Message types and CMD for supr/leader/master IPC
********************************************************************************/
#ifndef SYS_SHARE_MESSAGES_H
#define SYS_SHARE_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

#pragma pack(1)

#define Msg_T_S (2048)

// 消息头
typedef struct {
    int32_t mSender;
    int32_t mRecer;
    int32_t mMsgId;
    int32_t mNotice;
    int32_t mValue1;
    int32_t mValue2;
    int32_t mCrc;
} MsgHeader;

#define SHORT_MSG_SIZE 256
typedef struct {
    int32_t mSender;
    int32_t mRecer;
    int32_t mMsgId;
    int32_t mNotice;
    int32_t mValue1;
    int32_t mValue2;
    int32_t mCrc;
    char mShortData[SHORT_MSG_SIZE];
} MsgShortMsg;

#define LONG_MSG_SIZE 2048
typedef struct {
    int32_t mSender;
    int32_t mRecer;
    int32_t mMsgId;
    int32_t mNotice;
    int32_t mValue1;
    int32_t mValue2;
    int32_t mCrc;
    char mData[LONG_MSG_SIZE];
} MsgLongMsg;

// 上位机/主控命令 (与 sysEthcProject 对齐)
#define CMD_BASE 100
#define CMD_HEARTBEAT           (CMD_BASE + 1)
#define CMD_LINK                (CMD_BASE + 2)
#define CMD_UNLINK              (CMD_BASE + 3)
#define CMD_IGH_CREATE_MASTERS   (CMD_BASE + 5)
#define CMD_IGH_SCAN_SLAVES      (CMD_BASE + 6)
#define CMD_IGH_CONF_SERVO_PDOS  (CMD_BASE + 9)
#define CMD_IGH_START_OP         (CMD_BASE + 10)
#define CMD_HAND_VEL_MM          (CMD_BASE + 11)
#define CMD_HAND_RELA_POS_MM     (CMD_BASE + 12)
#define CMD_HAND_ABS_POS_M       (CMD_BASE + 13)
#define CMD_IGH_FSA_SHUTDOWN     (CMD_BASE + 14)
#define CMD_IGH_FSA_SWITCH_ON     (CMD_BASE + 15)
#define CMD_IGH_FSA_ENOP         (CMD_BASE + 16)
#define CMD_IGH_FSA_HALT         (CMD_BASE + 17)
#define CMD_IGH_CLEAR_ERROR      (CMD_BASE + 18)
#define CMD_DESTROY_MASTER       (CMD_BASE + 19)
#define CMD_MASTER_OP_KEYDOWN    (CMD_BASE + 20)
#define CMD_AXIS7_HOMING         (CMD_BASE + 21)   /* 第七轴回零命令 */



// 内部消息 ID
#define MSG_ID_BASE_FEATURE 0
#define MSG_ID_MODULE_REGISTER_MSG_Q_ACK (MSG_ID_BASE_FEATURE + 10)
#define MSG_ID_THREAD_START_ACK           (MSG_ID_BASE_FEATURE + 34)
#define MSG_ID_THREAD_REQUEST_EXIT_ALL_ACK (MSG_ID_BASE_FEATURE + 36)
#define MSG_ID_FIRE_IN_THE_HOLE           (MSG_ID_BASE_FEATURE + 5)

// 上位机下发数据结构（速度/位置等）
typedef struct {
    int32_t   mCmd;
    int32_t   mFrameId;
    uint8_t   mModuleFlag[8];
    uint16_t  mDataLength;
    uint16_t  mCRC;
    char      mData[];  /* flexible array member (C99) */
} SUPR_R_MSG;

// 速度控制数据结构（单轴或少量轴）
typedef struct {
    uint8_t  mModuleFlag[8];
    float    mVel[8];  // 各轴目标速度（用户单位，如 deg/s）
} UPPER_VEL_MSG;

// 位置命令结构（相对/绝对共用）
typedef struct {
    uint8_t  mModuleFlag[8];
    float    mPosition;  // 目标位置（度）
    uint8_t  mIsAbsolute; // 0=相对, 1=绝对
} UPPER_POS_MSG;

// ============================================
// 状态上报相关定义 (上位机 ↔ 主站)
// ============================================

// 主站状态定义
#define MASTER_STATE_IDLE           0   // 空闲
#define MASTER_STATE_INIT           1   // 初始化中
#define MASTER_STATE_PREOP          2   // 预操作状态
#define MASTER_STATE_SAFEOP         3   // 安全操作状态
#define MASTER_STATE_OP             4   // 操作状态(OP)
#define MASTER_STATE_ERROR          5   // 错误状态
#define MASTER_STATE_DISABLED       6   // 禁用状态

// 从站状态定义 (CiA 402状态)
#define SLAVE_STATE_NOT_READY       0x0000
#define SLAVE_STATE_SWITCH_DISABLED 0x0040
#define SLAVE_STATE_READY_SWITCH    0x0021
#define SLAVE_STATE_SWITCHED_ON     0x0023
#define SLAVE_STATE_OP_ENABLED      0x0027
#define SLAVE_STATE_QUICK_STOP      0x0007
#define SLAVE_STATE_FAULT           0x0008
#define SLAVE_STATE_UNKNOWN         0xFFFF

// 状态上报命令 (上位机 ← 主站)
#define CMD_STATUS_REPORT           200  // 周期性状态上报
#define CMD_NOTIFY_EVENT            201  // 事件通知
#define CMD_REPLY_OK                202  // 命令执行成功
#define CMD_REPLY_ERROR             203  // 命令执行失败

// 单个从站状态数据 (单个轴的数据)
typedef struct {
    uint16_t  statusWord;       // 状态字 (0x6041)
    int32_t   actualPosition;   // 实际位置 (0x6064)
    int32_t   actualVelocity;   // 实际速度 (0x606c)
    int16_t   actualTorque;     // 实际扭矩 (0x6077)
    int32_t   followingError;   // 跟随误差 (0x60f4)
    uint16_t  errorCode;        // 错误代码 (0x603f)
    uint16_t  controlWord;      // 控制字 (0x6040)
    int32_t   targetPosition;   // 目标位置 (0x607a)
    int32_t   targetVelocity;   // 目标速度 (0x60ff)
    int16_t   targetTorque;     // 目标扭矩 (0x6071)
} SLAVE_PDO_DATA;

#pragma pack(1)  // 关键：确保1字节对齐
// 主站完整状态上报数据结构
typedef struct {
    // 消息头信息
    int32_t   mFrameId;         // 帧序号
    uint32_t  mTimestamp;       // 时间戳(ms)
    
    // 主站整体状态
    uint8_t   masterState;      // 主站状态
    uint8_t   masterALState;    // 主站AL状态
    uint8_t   slaveCount;       // 从站数量
    uint8_t   reserved;
    
    // 各从站PDO数据 (最多支持8个从站)
    SLAVE_PDO_DATA slaves[8];
    
    // 预留扩展
    uint32_t  cycleCount;       // 周期计数
    uint32_t  domainWC;         // 域工作计数
    
    // 第八轴(六维力传感器)数据 - 单独字段
    float     forceFx;          // X方向力 (N)
    float     forceFy;          // Y方向力 (N)
    float     forceFz;          // Z方向力 (N)
    float     torqueMx;         // X方向力矩 (Nm)
    float     torqueMy;         // Y方向力矩 (Nm)
    float     torqueMz;         // Z方向力矩 (Nm)
    uint32_t  forceSensorStatus; // 力传感器状态码
    uint32_t  forceSensorCounter; // 采样计数器
    float     forceSensorTemp;  // 传感器温度 (°C)
} MASTER_STATUS_REPORT;
#pragma pack()  // 恢复默认对齐
// 在文件末尾，#pragma pack() 之前添加:

// ============================================
// PID参数配置相关定义 (KGU系列驱动器)
// ============================================

// PID参数相关命令
#define CMD_PID_READ_PARAMS     (CMD_BASE + 30)   // 读取PID参数
#define CMD_PID_WRITE_PARAMS    (CMD_BASE + 31)   // 写入PID参数
#define CMD_PID_SAVE_TO_FLASH   (CMD_BASE + 32)   // 保存到驱动器Flash

// KGU驱动器PID参数索引 (对应Pn参数号)
typedef enum {
    KGU_PID_NONE = 0,
    KGU_PID_CURRENT_KP = 31,     // Pn031: 电流环比例增益
    KGU_PID_CURRENT_KI = 32,     // Pn032: 电流环积分增益
    KGU_PID_VELOCITY_KP = 33,    // Pn033: 速度环比例增益
    KGU_PID_VELOCITY_KI = 34,    // Pn034: 速度环积分增益
    KGU_PID_POSITION_KP1 = 65,   // Pn065: 位置环比例增益1 (粗调)
    KGU_PID_POSITION_KP2 = 63,   // Pn063: 位置环比例增益2 (细调)
    KGU_PID_POSITION_FF = 64,    // Pn064: 位置前馈
    KGU_PID_MAX_INDEX = 8        // 参数总数
} KGU_PID_INDEX;

// 单个PID参数结构
typedef struct {
    uint16_t  paramNo;           // 参数号 (如31, 32, 33等)
    uint16_t  reserved;
    float     value;             // 参数值
    float     minValue;          // 最小值
    float     maxValue;          // 最大值
    uint16_t  isValid;           // 是否有效
    char      name[16];          // 参数名称
    char      description[32];   // 参数描述
} KGU_PID_PARAM_ITEM;

// PID参数请求/响应包
typedef struct {
    uint8_t   axisId;            // 轴ID
    uint8_t   paramCount;        // 参数数量
    uint16_t  resultCode;        // 结果码 (0=成功)
    KGU_PID_PARAM_ITEM params[8]; // 参数数组 (最多8个)
} KGU_PID_PARAMS_PACKET;

// 写入PID参数请求
typedef struct {
    uint8_t   axisId;            // 轴ID
    uint8_t   paramCount;        // 要写入的参数数量
    uint16_t  reserved;
    struct {
        uint16_t paramNo;        // 参数号
        float    value;          // 目标值
    } items[8];                  // 最多同时写入8个参数
} KGU_PID_WRITE_REQ;

#pragma pack()

#endif
