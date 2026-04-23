/********************************************************************************
* Copyright (c) 2026-2026 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_defs.hpp
* Module :
* Create on: 2026/2/5
* Author: 周佳燚
* Email: 2312947778@qq.com
* Description about this header file: 主站以及电机的一些参数信息
*
********************************************************************************/

#ifndef SYS_ARMS_DEFS_H
#define SYS_ARMS_DEFS_H

#include <arpa/inet.h>
#include <pthread.h>
#include <ecrt.h>

#include "sys_share_messages.h"
#include "sys_share_defs.h"
#include "sys_share_conf.h"
#include "list.h"

#pragma pack(1)
//定义伺服电机pdo交换的结构体,此结构体是已经转换成可以使用的数据
typedef struct{
    //读取pdo伺服电机的数据
    uint16_t ctrl_word;
    uint8_t  operation_mode;
    int32_t  Target_position;
    int32_t  target_velocity;
    int16_t  torque_offset;

    uint16_t status_word;
    uint8_t  mode_display;
    int32_t  Ext_Pos_value;
    int32_t  velocity_value;
    int32_t  Int_Pos_value;
    int16_t  torque_act_value;
    int16_t  torque_demand;
    //uint16_t  setting_time_now;
    //uint16_t  damping_ratio_now;
}somanet_motors;

typedef enum
{
  S_SHUTDOWN   = 0,
  S_SWITCH_ON,
  S_ENABLE_OP,
  S_HAIT,
  S_RELA_POS,
  S_CLEAR_ERROR,
  S_NULL,
  S_OP_OK,
} S_ETHR_STATE;

//pp模式运行状态机
typedef enum
{
  PP_NULL = 0,
  PP_NEW_POS,
  PP_NEW_BIT4_5,
  PP_NEW_POS_ACK ,
} SOMANET_PP_RUN_STATE;

//ETHERCAT线程结构体信息ETHERCAT thread info
typedef struct
{
  bool         mWorking;
  char         mThreadName[15];

  int  mthreadMasterId;
  pthread_t  mPid;
  //cpu mask
  uint8_t         mCpuAffinity;

  //控制电磁阀接收到上位机的消息
  bool  newUpperMsg;

  float    cmd_proportion[13];

  //指向master主线程的结构体
  void *pTModule;



  
} ETHERCAT_THREAD_INFO;


//主站下的从站信息
typedef struct EK1100_slaves_inf
{
  ec_slave_info_t    slave_in;
  ec_slave_config_t *slave_conf;

  //从站所属的主站id
  uint16_t masteri;

  int childSlaveSize;
  struct list_head list;
} EK1100_slaves_inf;



//定义伺服电机pdo交换的结构体,此结构体是IGH内部使用的，需要再定义使用的结构体
typedef struct{
    //伺服电机使用到的
    unsigned int ctrl_word;
    unsigned int operation_mode;
    unsigned int Target_position;
    unsigned int target_velocity;
    unsigned int torque_offset;

    unsigned int status_word;
    unsigned int mode_display;
    unsigned int Ext_Pos_value;
    unsigned int velocity_value;
    unsigned int Int_Pos_value;
    unsigned int torque_act_value;
    unsigned int torque_demand;


}kaiser_pdos;


typedef struct
{
  //随动的KP KD
    float follow_kp;
    float follow_kd;
    float follow_ki;
    //积分项
    double follow_ins[2];
    //XYZ轴的算法限速m/s
    float magic_max_speed;

    //master使用的参数,
    //轴0位点 编码器值
    int32_t z_encoder_zore;

    //master使用的参数,
    //轴软限位.单位m
    float soft_left_limi,x_soft_right_limi;


} ARM_PARAM;





//master线程结构体信息master thread info
typedef struct
{
  bool         mWorking;
  char         mThreadName[15];

  M_STATE         mState;
  int  mthreadMasterId;

  pthread_t  mPid;
  //cmd upper end
  //创建soket
  int32_t mSoket;
  //cpu mask
  uint8_t      mCpuAffinity;


  void *pTChassis;
  void *mSerialWorker;
  int32_t mIdentifier;

/****************************以下是igh主站使用*****************************/
  KAISER_MODE kaiser_mode; //主站配置伺服电机运行模式
  SOMANET_PP_RUN_STATE pp_run_state[MASTER_ARMS_MAX_SIZE]; //吊杆中所有机械臂PP模式下，每个轴运行状态
  uint32_t al_states;
  uint32_t st_slaves;
  uint16_t master_link_up;
  uint16_t domain_wc;

  //ethercat线程，主要负责1000hz交换数据
  ETHERCAT_THREAD_INFO mEtherWorker;
  //主站
  int masterId;
  //ecrt_
  //主站和域使用
  ec_master_t *master;
  ec_master_state_t master_state;
  ec_domain_t *domain1;
  uint8_t *domain1_pd;
  ec_domain_state_t domain1_state;

  //激活主站标志，主站激活后可以使用PDO进行伺服操作
  bool masterActivated;

  //从站信息,一个主站上最多7个臂，从站不超过56个，因此设置100个从站
  EK1100_slaves_inf slaves_inf[MASTER_ARMS_MAX_SIZE];

  //在CiA 402中，规定电力驱动系统（PDS）的行为应由有限状态自动机（FSA）控制
  S_ETHR_STATE  arm_fsa[MASTER_ARMS_MAX_SIZE];

  int slaves_size;
  //机械臂的个数，也就是EK1100的个数
  int armsSize;
  
  //交换的pdo数据,一个arm下有6个电机
  kaiser_pdos   somanet_pdo[MASTER_ARMS_MAX_SIZE];


  somanet_motors motors[MASTER_ARMS_MAX_SIZE];
/****************************以下是igh主站使用 end*****************************/

/****************************************上位机速度控制、位置控制*************************************/
  //上位机缓坡加速使用,上位机速度控制，采用固定加速度。单位mm
  float  upper_target_velocity[MASTER_ARMS_MAX_SIZE][4];
  float  upper_velocity_v0[MASTER_ARMS_MAX_SIZE][4];
  uint8_t upper_target_v_Flag[MASTER_ARMS_MAX_SIZE];
  float  upper_acc[MASTER_ARMS_MAX_SIZE][4];
  float  upper_last_v_cmd[MASTER_ARMS_MAX_SIZE][4];
 
  //上位机位置控制使用,上位机速度控制，采用固定加速度。单位mm
  float  upper_target_pos[MASTER_ARMS_MAX_SIZE];
  #define    upper_pos_smooth_cnt 10 
  float  upper_pos_vel_smooth[MASTER_ARMS_MAX_SIZE][upper_pos_smooth_cnt];
  uint8_t upper_target_p_Flag[MASTER_ARMS_MAX_SIZE];
  int32_t upper_target_p_Kp_cnt[MASTER_ARMS_MAX_SIZE];

  //位置控制，最大加速度限制做平滑处理
  float  target_p_last_cmdvel[MASTER_ARMS_MAX_SIZE];

/*************end*/

/********************************************机械臂转换成国际单位的器件数据*****************************************/
  //六个轴的方向正负号
  int32_t  AXIS_DIR[MASTER_ARMS_MAX_SIZE];


/**********************************伺服电机PDO*********************************/
  //速度指令mm
  float  cmd_velocity[MASTER_ARMS_MAX_SIZE];
  //状态字
  uint16_t somanet_status[MASTER_ARMS_MAX_SIZE];
  
  //轴速度，单位：mm/s
  float Speed[MASTER_ARMS_MAX_SIZE];

  //轴位置 单位m
  double Pos[MASTER_ARMS_MAX_SIZE];

  //单元内轴当前力矩值
  double torqueActual[MASTER_ARMS_MAX_SIZE];
  
  //速度环生成的力矩
  double torqueDemand[MASTER_ARMS_MAX_SIZE];


/********************************************转换成国际单位的器件数据end*****************************************/



/********************************************配置参数*****************************************/
  //采集传感器及电机 滤波器的数据结构体
  ARM_PARAM mAparams[MASTER_ARMS_MAX_SIZE];
  //主站上传数据时间间隔
  uint32_t PACK_MSG_MS ;

/********************************************配置参数end*****************************************/




} ARMS_MASTER_THREAD_INFO;




#endif // SYS_ARMS_DEFS_H
