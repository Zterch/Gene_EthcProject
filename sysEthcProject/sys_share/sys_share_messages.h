/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
#ifndef SYS_ARMS_MESSAGES_HPP
#define SYS_ARMS_MESSAGES_HPP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <sys/types.h>
#include <pthread.h>

//////////////////////////////////////////////////////////////////////////////

#pragma pack(1)
///////////////////////////////////////////////////////////////////////////////
//const int32_t MSG_BASE = 0;
//long msg size
#define Msg_T_S (2048)

// 出错数据包数量等配置参数
#define SAMPLE_RATE_HZ     50
#define WINDOW_SIZE_SEC    2
#define QUEUE_CAPACITY     (SAMPLE_RATE_HZ * WINDOW_SIZE_SEC)  // 100

//增加系统是否出错的标志    6.06  -zjy
extern bool IsSystemError;  

//增加Z轴限位标志          6.16  -zjy
extern bool Is_Zlimit_Open;

extern bool Is_Zout_limit[7][2];


typedef struct {

  char time_str[16];    //出错前数据包记录的北京时间

  int16_t Bat_Remaining[2];   //电池剩余电量
       
  float gas_press_high[2];    //高压气压

  //倾角仪数据 单位：m/s2
  float incln_acc_XY[7][2];

  //绝压传感器值
  float  abs_stress[7];

  //底盘的8个电机数据
  struct {
      float  Position_value;
      float  velocity_value;
  } nimotion_inf[2][8]; /**< somanet information. */

  //机械臂四个电机数据
  struct {
      float  Int_vel_value;
      float  Int_Pos_value;
  } somanet_inf[7][4];

  
} Info_Show_In_Err; 

extern Info_Show_In_Err info_show_in_err;

// 线程安全的循环队列
typedef struct {
    Info_Show_In_Err data[QUEUE_CAPACITY];
    int              head;            // 指向最旧数据
    int              tail;            // 指向下一个插入位置
    int              size;            // 当前元素数量
    pthread_mutex_t  lock;       // 互斥锁
    pthread_cond_t   cond;       // 条件变量（用于等待数据）
} SafeCircularQueue;


//DEFINE COMMON MESSAGE
typedef enum {
  SYS_MSG_V1 = 1,
  SYS_MSG_V2 = 2,
} SYS_MSG_VERSION;


//msg header
typedef struct {
  int32_t mSender;
  int32_t mRecer;
  int32_t mMsgId;

  //可以作为msgId的子消息ID,或者通知
  int32_t mNotice;

  int32_t mValue1;
  int32_t mValue2;
  int32_t mCrc;
}MsgHeader;

//module send to supr
typedef struct  {
  int32_t mSender;
  int32_t mRecer;
  int32_t mMsgId;

  //可以作为msgId的子消息ID,或者通知
  int32_t mNotice;

  int32_t mValue1;
  int32_t mValue2;
  int32_t mCrc;

  pid_t mPid;
}MsgRegisterMsgQ;

//short msg
#define SHORT_MSG_SIZE  256
typedef struct {
  int32_t mSender;
  int32_t mRecer;
  int32_t mMsgId;

  //可以作为msgId的子消息ID,或者通知
  int32_t mNotice;

  int32_t mValue1;
  int32_t mValue2;
  int32_t mCrc;

  char mShortData[SHORT_MSG_SIZE];

}MsgShortMsg;

//long msg.(***MsgLongMsg + MsgHeader < Msg_T_S***)
#define LONG_MSG_SIZE  2048
typedef struct  {
   int32_t mSender;
  int32_t mRecer;
  int32_t mMsgId;

  //可以作为msgId的子消息ID,或者通知
  int32_t mNotice;

  int32_t mValue1;
  int32_t mValue2;
  int32_t mCrc;
  
  char mData[LONG_MSG_SIZE];
}MsgLongMsg;


#define LOGER_MSG_SIZE  256
typedef struct {
   int32_t mSender;
  int32_t mRecer;
  int32_t mMsgId;

  //可以作为msgId的子消息ID,或者通知
  int32_t mNotice;

  int32_t mValue1;
  int32_t mValue2;
  int32_t mCrc;
  
  char mPriStr[LOGER_MSG_SIZE];
}MsgLogMsg;

#define POINTS_VEC_SIZE 4
typedef struct 
{
  float m[POINTS_VEC_SIZE];
}POINTS_F;

///////////////////////////////////////////////////////////////////////////////
//DEFINE COMMON MESSAGE
#define MSG_ID_BASE_FEATURE  0
#define MSG_ID_MODULE_INIT1              (MSG_ID_BASE_FEATURE+1)
#define MSG_ID_MODULE_INIT1_ACK          (MSG_ID_BASE_FEATURE + 2)
#define MSG_ID_MODULE_INIT2              (MSG_ID_BASE_FEATURE+3)
#define MSG_ID_MODULE_INIT2_ACK          (MSG_ID_BASE_FEATURE + 4)
#define MSG_ID_FIRE_IN_THE_HOLE          (MSG_ID_BASE_FEATURE + 5)

#define MSG_ID_MODULE_HEARTBEAT          (MSG_ID_BASE_FEATURE + 6)
#define MSG_ID_MODULE_HEARTBEAT_ACK      (MSG_ID_BASE_FEATURE + 7)

#define MSG_ID_MODULE_REQUEST_WAIT       (MSG_ID_BASE_FEATURE + 8)
#define MSG_ID_MODULE_REQUEST_WAIT_ACK   (MSG_ID_BASE_FEATURE + 9)

#define MSG_ID_MODULE_REGISTER_MSG_Q_ACK  (MSG_ID_BASE_FEATURE+10)
#define MSG_ID_MODULE_TIMEOUT             (MSG_ID_BASE_FEATURE + 11)

#define MSG_ID_MODULE_REQUEST_EXIT_ALL    (MSG_ID_BASE_FEATURE + 12)
#define MSG_ID_MODULE_EXIT_ALL            (MSG_ID_BASE_FEATURE + 13)
#define MSG_ID_MODULE_EXIT_ALL_ACK        (MSG_ID_BASE_FEATURE + 14)

#define MSG_ID_MODULE_REQUEST_STOP_ALL    (MSG_ID_BASE_FEATURE + 15)
#define MSG_ID_MODULE_STOP_ALL            (MSG_ID_BASE_FEATURE + 16)
#define MSG_ID_MODULE_STOP_ALL_ACK        (MSG_ID_BASE_FEATURE + 17)

//thread
#define MSG_ID_THREAD_START_ACK           (MSG_ID_BASE_FEATURE + 34)
#define MSG_ID_THREAD_STOP_ALL_ACK        (MSG_ID_BASE_FEATURE + 35)
#define MSG_ID_THREAD_REQUEST_EXIT_ALL_ACK (MSG_ID_BASE_FEATURE + 36)

//upper
//#define MSG_ID_UPPER_BASE                   100
#if 0
//#define MSG_ID_UPPER_CREATE_MASTERS         (MSG_ID_UPPER_BASE + 5)
#define MSG_ID_UPPER_SCAN_SLAVES            (MSG_ID_UPPER_BASE + 6)
#define MSG_ID_UPPER_CONF_SERVO_PDOS        (MSG_ID_UPPER_BASE + 7)
#define MSG_ID_UPPER_START_OP               (MSG_ID_UPPER_BASE + 8)
#define MSG_ID_UPPER_ANGULAR_VEL            (MSG_ID_UPPER_BASE + 9)
#define MSG_ID_UPPER_FSA_SHUTDOWN           (MSG_ID_UPPER_BASE + 10)
#define MSG_ID_UPPER_FSA_SWITH_ON           (MSG_ID_UPPER_BASE + 11)
#define MSG_ID_UPPER_FSA_EN                 (MSG_ID_UPPER_BASE + 12)
#define MSG_ID_UPPER_FSA_HALT               (MSG_ID_UPPER_BASE + 13)
#define MSG_ID_UPPER_CLEAR_ERROR            (MSG_ID_UPPER_BASE + 14)
//#define MSG_ID_UPPER_DESTROY_MASTERS        (MSG_ID_UPPER_BASE + 15)
#define MSG_ID_UPPER_OP_KEYDOWN             (MSG_ID_UPPER_BASE + 16)
//#define MSG_ID_UPPER_EC3045_PROPORTION      (MSG_ID_UPPER_BASE + 17)
#define MSG_ID_UPPER_MAGIC_VERTICAL_WEIGHT  (MSG_ID_UPPER_BASE + 18)
//#define MSG_ID_UPPER_MAGIC_FOLLOW           (MSG_ID_UPPER_BASE + 19)
#define MSG_ID_UPPER_MAGIC_VERTICAL         (MSG_ID_UPPER_BASE + 20)
#define MSG_ID_UPPER_MAGIC_FOLLOW_VERTICAL  (MSG_ID_UPPER_BASE + 21)
#define MSG_ID_UPPER_MAGIC_STOP             (MSG_ID_UPPER_BASE + 22)
#define MSG_ID_UPPER_R_PARAM                (MSG_ID_UPPER_BASE + 23)
#define MSG_ID_UPPER_W_PARAM                (MSG_ID_UPPER_BASE + 24)
#define MSG_ID_UPPER_CHASSIS_VEL            (MSG_ID_UPPER_BASE + 25)
#endif

///////////////////////////////////////////////////////////////////////////////
//DEFINE leader FEATURE MESSAGE
#define MSG_ID_LEADER_FEATURE  0x100
#define MSG_ID_REQUEST_ETHR_STOP           (MSG_ID_LEADER_FEATURE + 1)
#define MSG_ID_RECER_MDATA                 (MSG_ID_LEADER_FEATURE + 2)
#define MSG_ID_MASTER_GRAPH                (MSG_ID_LEADER_FEATURE + 3)
#define MSG_ID_MASTER_SHOW_INF             (MSG_ID_LEADER_FEATURE + 4)
#define MSG_ID_MASTER_SHOW_PARAMS          (MSG_ID_LEADER_FEATURE + 5)

#define MSG_ID_MASTER_ALL_STOP             (MSG_ID_LEADER_FEATURE + 6)

#define MSG_ID_MASTER_SHOW_SAMPLE_INF      (MSG_ID_LEADER_FEATURE + 7)
///////////////////////////////////////////////////////////////////////////////
//DEFINE LOGER FEATURE MESSAGE
#define MSG_ID_LOGER_FEATURE    0x200
#define MSG_ID_LOGER_SAVE_LOG   (MSG_ID_LOGER_FEATURE + 1)
#define MSG_ID_LOGER_SAVE_DATA  (MSG_ID_LOGER_FEATURE + 2)
#define MSG_ID_LOGER_R_PARAM    (MSG_ID_LOGER_FEATURE + 3)
#define MSG_ID_LOGER_W_PARAM    (MSG_ID_LOGER_FEATURE + 4)

//DEFINE DBWORKER FEATURE MESSAGE
#define MSG_ID_DB_FEATURE    0x250
#define MSG_ID_DB_R_PARAM   (MSG_ID_DB_FEATURE + 1)
#define MSG_ID_DB_W_PARAM   (MSG_ID_DB_FEATURE + 2)

///////////////////////////////////////////////////////////////////////////////
//DEFINE SENDER FEATURE MESSAGE
#define MSG_ID_SENDER_FEATURE    0x300
#define MSG_ID_SENDER_SEND_MSG   (MSG_ID_SENDER_FEATURE + 1)

///////////////////////////////////////////////////////////////////////////////
//DEFINE UPPER FEATURE MESSAGE
#define MSG_ID_UPPER_FEATURE     0x400
#define MSG_ID_UPPER_ACTIONG_MSG      (MSG_ID_UPPER_FEATURE + 1)
#define MSG_ID_UPPER_LOSE_LINK_MSG    (MSG_ID_UPPER_FEATURE + 2)
#define MSG_ID_UPPER_NOTICE           (MSG_ID_UPPER_FEATURE + 3)

///////////////////////////////////////////////////////////////////////////////
//DEFINE ARMS TO CHASSIS FEATURE MESSAGE
#define MSG_ID_ARMS_TO_CHASSIS_FEATURE     0x500
#define MSG_ID_ARMS_MAGIC_CTRL_MSG         (MSG_ID_ARMS_TO_CHASSIS_FEATURE + 1)


//上位機請求主控機消息ID
#define UDP_ID_REQ_FEATURE    0xa00
#define UDP_ID_REQUEST_LINK   (UDP_ID_REQ_FEATURE + 1)


//主控機反饋上位機消息I
#define UDP_ID_ACK_FEATURE    0xb00
#define UDP_ID_ACK_LINK_OK       (UDP_ID_ACK_FEATURE + 1)
#define UDP_ID_ACK_LINK_FAILED   (UDP_ID_ACK_FEATURE + 2)


////////////////////////////////////////////////////////////////////////////////

//定义主站的拓扑结构
typedef struct
{
  //主站的
  int32_t   master_id;

  //机械臂个数
  int32_t   arms_size;
  //EK1100下的从站个数,0代表不存在,==2 代表只有EK1100。 >2 代表机械臂存在
  int32_t   EK1100_slaves[8];

} MASTER_GRAPH;

//DEFINE recer msg
//定义机械臂、底盘控制板接收消息
typedef struct
{
  //cmd 控制命令
  int32_t   mCmd;

  //frame unique dev 0-3
  int32_t   mFrameId;

  //module id,0-6
  uint8_t   mModuleFlag[8];

  //数据长度
  uint16_t   mDataLength;

  //16位的crc累加校验
  uint16_t   mCRC;

  //可变长的数据,最短都是8位
  char       mData[0];

} SUPR_R_MSG;

//上位机控制底盘比例调压阀
typedef struct proportional_value
{
  uint32_t pvid;
  float value;
}proportional_value;

//定义机械臂控制板发送消息
typedef struct
{
  //拉力计
  float 	  mTension;
  //倾角仪X Y
  float     mIncli[2];
  //X Y Z向磁栅尺
  float     mSiko[3];
  int32_t   mVisionFps;
  //SSI 磁栅尺
  float     mSikoSSI[3];
  //绝压传感器
  float     mAbsPressure;
  //比例阀值
  float     mProportions[4];
  //限位开关
  uint16_t   mSwitchs;

  //四个电机数据
  struct {
      uint16_t ctrl_word;
      uint8_t  operation_mode;
      float  Target_position;
      float  target_velocity;
      int16_t  torque_offset;

      uint16_t status_word;
      uint8_t  mode_display;
      float  Ext_Pos_value;
      float  Int_vel_value;
      float  Int_Pos_value;
      float  torque_act_value;
      float  torque_demand;

      //存储命令和速度的误差及均方差
      float  cmd_vel_error;
      float  cmd_vel_mean_error_n;

  } somanet_inf[4]; /**< somanet information. */

  //备用的数据，最大20个size
  uint32_t   mOtherD[20];
}Arm_inf;  /*基础的协议，后期会增加字段*/

//定义机械臂控制板发送消息
typedef struct
{
  //拉力计
  float 	  mTension;
  //倾角仪X Y
  float     mIncli[2];
  //X Y Z向磁栅尺
  float     mSiko[3];
  int32_t   mVisionFps;
  //SSI 磁栅尺
  float     mSikoSSI[3];
  //绝压传感器
  float     mAbsPressure;

  //限位开关
  uint16_t   mSwitchs;

  uint32_t Bat_Remaining;              //增加两个字段用于计算剩余工作时间 -zjy 6.30
  
  float gas_press_high;   
              
}Arm_sample_inf;  /*基础的协议，后期会增加字段*/

//定义主站运行信息发送
typedef struct
{
  uint32_t masterid;
  uint32_t al_states;
  uint32_t st_slaves;
  uint16_t master_link_up;
  uint16_t domain_wc;
}Masters_inf;  /*主站运行信息显示*/

//定义底盘控制板发送消息
#define NIMOTION_SIZE 9
typedef struct
{
  //8个电机数据
  struct {
      uint16_t ctrl_word;
      uint8_t  operation_mode;
      int32_t  Target_position;
      int32_t  target_velocity;

      uint16_t status_word;
      uint8_t  mode_display;
      float  Position_value;
      float  velocity_value;
  } nimotion_inf[NIMOTION_SIZE]; /**< somanet information. */

}Chassis_inf;  /*基础的协议，后期会增加字段*/


//定义底盘器件采集信息
typedef struct
{
  //13个比例调压阀命令和采集
  float    cmd_presss[13];
  float    proportion_presss[13];

  //两个气压计
  //float    gas_weight[2];
  float gas_press_low;
  float gas_press_high;

  //11个称重计
  /*
    1号变送器
      Ch1：左前气足
      Ch2：左前舵轮
      Ch3：左边过缝气足
    2号变送器
      Ch1：左后气足
      Ch2：左后舵轮
    3号变送器
      Ch1：右前气足
      Ch2：右前舵轮
      Ch3：右边过缝气足
    4号变送器
      Ch1：右后气足
      Ch2：右后舵轮
    5号变送器
      Ch1：中间气足
  */
  float    chassis_weight[11];

  //BMS信息
  float Bat_Voltage; 
  float Bat_Current;
  int16_t Bat_Temperature;
  int16_t Bat_Remaining;

  //继电器状态信息
  /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
  */
  uint8_t relays_24V[8];
  uint8_t relays_48V[8];
  uint8_t relays_brk_48V[8];
  uint8_t relays_gas[8];

}Chassis_sensor_inf;  /*基础的协议，后期会增加字段*/

typedef struct
{
  uint16_t   mIdentifier;
  //msg type
  uint16_t   mMsgType;
  //随机码，上位机，自加一
  uint16_t   mRandomCode;
  //states
  uint16_t   mStateCode;

  //时间戳，妙、微妙
  uint32_t   mSysTimeS;
  uint32_t   mSysTimeUs;

  //module id
  uint8_t     mModuleFlag[8];
  //机械臂个数
  uint8_t     mUserArmsSize;
  //ctrl motors data,只传输有somanet的机械臂，下电的机械臂不拷贝到Arm_inf
  char  mData[0];
} ARMS_S_MSG;

//*************************************************//
//主站非实时信息
typedef struct
{
  uint16_t   mIdentifier;
  //msg type
  uint16_t   mMsgType;

  //主站id
  uint8_t    masterId;

  //寄存器地址和子地址
  char   addr[16];
  //igh配置信息数据
  uint16_t   mDataSize;

  uint8_t    mData[0];

} ARMS_S_IGH_MSG;

typedef struct
{
    int32_t rowIndex;
    int colCount;
    //最多30列数据
    char entry[30][32];
} TABLE_ENTRY;


//listener线程结构体信息 thread info
typedef struct
{
  bool              mWorking;
  char              mThreadName[15];

  bool              mInitOk;

  pthread_t         mthreadPid;

  //cpu mask
  uint8_t           mCpuAffinity;

} LISTENER_THREAD_INFO,LOG_THREAD_INFO,DB_WORKER_INFO;

//发送到底盘主站的控制命令消息
typedef struct
{
  //表示吊杆是否存在
  uint8_t   mModuleFlag[8];
  //吊杆X向控制命令
  float     mXMv[8];
  float     mYv[8];    
}ARMS_MAGIC_CTRL_V;
/******************************************upper udp msg start****************************************/


////////////////////////////////upper cmd msg//////////////////////////////
#define MSG_UPPER_CMD_DATA_SIZE 80
#define MSG_SERVER_ACK_DATA_SIZE 10240

//upper cmd mCmd
#define CMD_BASE    100
#define CMD_HEARTBEAT            (CMD_BASE + 1)
#define CMD_LINK                 (CMD_BASE + 2)
#define CMD_UNLINK               (CMD_BASE + 3)
#define CMD_SERVER_QUIT          (CMD_BASE + 4)
#define CMD_IGH_CREATE_MASTERS   (CMD_BASE + 5)
#define CMD_IGH_SCAN_SLAVES      (CMD_BASE + 6)
#define CMD_IGH_SDO_DOWNLOAD     (CMD_BASE + 7)
#define CMD_IGH_SDO_UPLOAD       (CMD_BASE + 8)
#define CMD_IGH_CONF_SERVO_PDOS  (CMD_BASE + 9)
#define CMD_IGH_START_OP         (CMD_BASE + 10)
#define CMD_HAND_VEL_MM          (CMD_BASE + 11)
#define CMD_HAND_RELA_POS_MM     (CMD_BASE + 12)
#define CMD_HAND_ABS_POS_M       (CMD_BASE + 13)
#define CMD_IGH_FSA_SHUTDOWN     (CMD_BASE + 14)
#define CMD_IGH_FSA_SWITH_ON     (CMD_BASE + 15)
#define CMD_IGH_FSA_ENOP         (CMD_BASE + 16)
#define CMD_IGH_FSA_HALT         (CMD_BASE + 17)
#define CMD_IGH_CLEAR_ERROR      (CMD_BASE + 18)
#define CMD_DESTROY_MASTER       (CMD_BASE + 19)
#define CMD_MASTER_OP_KEYDOWN    (CMD_BASE + 20)
#define CMD_EC3045_PROPORTION    (CMD_BASE + 21)
#define CMD_AUTO_VERTICAL_WEIGHT (CMD_BASE + 22)
#define CMD_AUTO_FOLLOW          (CMD_BASE + 23)
#define CMD_AUTO_FOLLOW_CHASSIS  (CMD_BASE + 24)
#define CMD_AUTO_VERTICAL        (CMD_BASE + 25)
#define CMD_AUTO_FOLLOW_VERTICAL (CMD_BASE + 26)
#define CMD_ALL_STOP             (CMD_BASE + 27)
#define CMD_IGH_VERTICAL_LIMIT   (CMD_BASE + 28)

#define CMD_IGH_CHASSIS_VEL      (CMD_BASE + 30)
#define CMD_CHASSIS_WHEEL_AGNGLE (CMD_BASE + 31)
#define CMD_CHASSIS_MOVE         (CMD_BASE + 32)
#define CMD_CHASSIS_STOP         (CMD_BASE + 33)
#define CMD_IGH_SIKO_ZERO        (CMD_BASE + 34)
#define CMD_IGH_Z_ZERO           (CMD_BASE + 35)
#define CMD_IGH_X_EXT_ABS_POS    (CMD_BASE + 36)
#define CMD_IGH_Y_ZERO           (CMD_BASE + 37)
#define CMD_IGH_XS_ZERO          (CMD_BASE + 38)
#define CMD_IGH_XM_ZERO          (CMD_BASE + 39)
#define CMD_IGH_CTRL_WORD        (CMD_BASE + 40)


#define CMD_CONF_R_DB_TABLES_NAME      (CMD_BASE + 43)
#define CMD_CONF_R_TABLE_ENTYR         (CMD_BASE + 44)
#define CMD_CONF_D_TABLE_ENTYR         (CMD_BASE + 45)
#define CMD_CONF_UPDATE_TABLE_ENTYR    (CMD_BASE + 46)


//igh读写相关
#define CMD_IGH_RW_SLAVES_INF       (CMD_BASE + 56)
#define CMD_IGH_RW_SLAVE_SDO        (CMD_BASE + 57)
#define CMD_IGH_RW_SLAVE_UPLOAD     (CMD_BASE + 58)
#define CMD_IGH_RW_SLAVE_DOWNLOAD   (CMD_BASE + 59)
#define CMD_IGH_RW_MASTER_GRAPH     (CMD_BASE + 60)
#define CMD_IGH_RW_SLAVE_ALIAS      (CMD_BASE + 61)
//电磁阀相关
#define CMD_VALVE_ARMS24_ON         (CMD_BASE + 81)
#define CMD_VALVE_ARMS24_OFF        (CMD_BASE + 82)
#define CMD_VALVE_CHASSIS24_ON      (CMD_BASE + 83)
#define CMD_VALVE_CHASSIS24_OFF     (CMD_BASE + 84)
#define CMD_VALVE_ARMS48_ON         (CMD_BASE + 85)
#define CMD_VALVE_ARMS48_OFF        (CMD_BASE + 86)
#define CMD_VALVE_CHASSIS48_ON      (CMD_BASE + 87)
#define CMD_VALVE_CHASSIS48_OFF     (CMD_BASE + 88)
#define CMD_VALVE_ARM_GAS_ON        (CMD_BASE + 89)
#define CMD_VALVE_ARM_GAS_OFF       (CMD_BASE + 90)
#define CMD_VALVE_CHASSIS_GAS_ON    (CMD_BASE + 91)
#define CMD_VALVE_CHASSIS_GAS_OFF   (CMD_BASE + 92)
#define CMD_VALVE_CHASSIS_BRK_ON    (CMD_BASE + 93)
#define CMD_VALVE_CHASSIS_BRK_OFF   (CMD_BASE + 94)
#define CMD_PROPORTIONAL_VALVE      (CMD_BASE + 95)
#define CMD_PROPORTIONAL_VALVES     (CMD_BASE + 96)
//针对机械臂和底盘都适用的
#define CMD_CHASSIS_MAGIC_ENABLE    (CMD_BASE + 150)
#define CMD_INTERPN_START_COLLECT   (CMD_BASE + 151)
#define CMD_INTERPN_COLLECT         (CMD_BASE + 152)
#define CMD_INTERPN_COLLECT_OVER    (CMD_BASE + 153)


//serverAckMsg mAckNotice
#define ACK_NOTICE_BASE  0
#define ACK_NOTICE_LINK_OK                  (ACK_NOTICE_BASE + 1)
#define ACK_NOTICE_LINK_FAILED               (ACK_NOTICE_BASE + 2)
#define ACK_NOTICE_UNLINK_OK                 (ACK_NOTICE_BASE + 3)
#define ACK_NOTICE_SHOW_INF                  (ACK_NOTICE_BASE + 4)
#define ACK_NOTICE_ARM_R_PARAMS              (ACK_NOTICE_BASE + 5)
#define ACK_NOTICE_SHOW_CHASSIS_INF          (ACK_NOTICE_BASE + 6)
#define ACK_NOTICE_SHOW_CHASSIS_SENSOR_INF   (ACK_NOTICE_BASE + 7)
#define ACK_NOTICE_SHOW_MASTER_RUN_INF       (ACK_NOTICE_BASE + 8)
#define ACK_NOTICE_HAND_POS_ARRIVE           (ACK_NOTICE_BASE + 9)
#define ACK_NOTICE_FRAME_R_PARAMS            (ACK_NOTICE_BASE + 10)
#define ACK_NOTICE_DB_R_TABLES_NAME          (ACK_NOTICE_BASE + 11)
#define ACK_NOTICE_DB_R_TABLES_ENTRY         (ACK_NOTICE_BASE + 12)
#define ACK_NOTICE_UPDATE_TABLE_ENTYR_OK     (ACK_NOTICE_BASE + 13)

#define ACK_NOTICE_SHOW_INFO_IN_ERR (ACK_NOTICE_BASE + 14) // 6.16-zjy

#define ACK_NOTICE_IGH_RW_SLAVES (ACK_NOTICE_BASE + 54)
#define ACK_NOTICE_IGH_RW_SDO    (ACK_NOTICE_BASE + 55)
#define ACK_NOTICE_IGH_RW_UPLOAD (ACK_NOTICE_BASE + 56)
#define ACK_NOTICE_IGH_RW_GRAPH  (ACK_NOTICE_BASE + 57)



typedef struct
{
    int32_t mCmd;

    int32_t mNotice;

    int32_t mValue;

    int32_t mDatas[MSG_UPPER_CMD_DATA_SIZE];

    uint32_t mRandomCode;

} UpperCmdMsg;

typedef struct
{
    int32_t mAckMsgType;

    int32_t mAckNotice;

    int32_t mAckValue;

    int32_t mAckDatas[MSG_SERVER_ACK_DATA_SIZE];

    uint32_t mRandomCode;

} ServerAckMsg;
#pragma pack()

/******************************************upper udp msg end****************************************/
// ================ 队列操作函数 ================

// 初始化队列
void queue_init(SafeCircularQueue *q);

// 销毁队列（释放资源）
void queue_destroy(SafeCircularQueue *q);

// 获取当前队列大小
int queue_size(SafeCircularQueue *q);

// 入队操作（线程安全）
void queue_enqueue(SafeCircularQueue *q, Info_Show_In_Err item);

// 出队操作（线程安全）
bool queue_dequeue(SafeCircularQueue *q, Info_Show_In_Err *result);

#endif // SYS_ARMS_MESSAGES_HPP
