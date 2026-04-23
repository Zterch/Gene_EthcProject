/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-18 13:05:10
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_daemon.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:创建线程模块（后期修改为多进程），修改线程优先级，线程绑定核模块
*
********************************************************************************/
#ifndef SYS_ARMS_DEFS_H
#define SYS_ARMS_DEFS_H

#include <arpa/inet.h>
#include <pthread.h>
#include "ecrt.h"
#include "sys_share_messages.h"
#include "sys_share_defs.h"
#include "sys_share_conf.h"
#include "sys_ctl.h"
#include "list.h"
#include "sys_db.h"

#define USEC_PER_SEC		1000000
#define NSEC_PER_SEC		1000000000

#define SEMANET_VID_PID           0x000022d2,0x00000201   /*Vendor ID, product code*/

//X Y Z 轴转换成m单位系数。
/*
Xm Xs： 螺距是50  减速比是10 电机旋转一圈2^20 脉冲
Y： 螺距是40  减速比是10 电机旋转一圈2^20 脉冲
*/
#define EXT_X_PULSE_TO_M  (1/1000000.0)
#define INT_X_PULSE_TO_M  (50.0/10.0/1048576.0/1000.0)
#define Y_PULSE_TO_M  (40.0/10.0/1048576.0/1000.0)
#define INT_Z_PULSE_TO_M  (3.1415*120/70.0/1048576.0/1000.0)
#define EXT_Z_PULSE_TO_M  (1/1000000.0)
/*
Xm Xs： 螺距是50  减速比是10 电机旋转一圈2^20 脉冲
Y： 螺距是40  减速比是10 电机旋转一圈2^20 脉冲
RPM转成速度  单位mm/s

Z： 周长2*pi*60  减速比是70 电机旋转一圈2^20 脉冲
RPM转成速度  单位mm/s
*/
#define X_RPM_TO_V_MM  (50.0/10.0/60.0)
#define Y_RPM_TO_V_MM  (40.0/10.0/60.0)
#define Z_RPM_TO_V_MM  (2*3.14159*60/70.0/60.0)

/*
底盘行走电机： 直径200mm  减速比是42 电机旋转一圈10000 脉冲
底盘旋转电机： 一圈360度  减速比是70, 6.25 电机旋转一圈10000 脉冲
RPM转成速度  单位mm/s
*/
#define W_POS_TO_M  (3.1415*200.0/42.0/10000.0/1000.0)
#define R_POS_TO_MM  (360.0/6.25/70.0/131072.0)

//命令速度是用户单位，采集的速度是RPM，需要转换成mm单位
#define W_VEL_MM_TO_USER  (3.1415*200.0/42.0/10000.0)
#define W_VEL_RPM_TO_MM   (3.1415*200.0/42.0/60)

//命令速度是用户单位，采集的速度是RPM，需要转换成度单位
#define R_VEL_RPM_TO_DEG  (360.0/6.25/70.0/60.0)
#define R_VEL_DEG_TO_USER  (360.0/6.25/70.0/131072.0)

//行走和旋转轮子方向
static int32_t CHASSIS_DIR[] = {1, -1, 1, -1, -1, -1, -1, -1, 1};

//////////////////////////////////// System internal structure  /////////////////////////////////////

////////////////////////////////////

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

typedef enum
{
  AL_NULL   = 0,
  AL_INIT,
  AL_PREOP,
  AL_SAFEOP,
  AL_OP,
} MASTER_AL_STATE;
///end of udp msg ctrl part

//pp模式运行状态机
typedef enum
{
  PP_NULL = 0,
  PP_NEW_POS,
  PP_NEW_BIT4_5,
  PP_NEW_POS_ACK ,
} SOMANET_PP_RUN_STATE;

///for udp msg state part/////////////


///end of udp msg ctrl part

///end of udp msg state part


////////////////////////////////////printf queue /////////////////////////////////////


/////////////////////////////////整个系统中线程定义 //////////////////////////////////////////////
//一个主站下，最多挂载从站个数
#define MASTER_SLAVES_MAX_SIZE 100
//一个主站下，domain域zudomain域最多的条目,
/*  

*/
#define MASTER_DOMAIN_MAX_SIZE 109


#define ARMS_CONF_PARAM_SIZE 20

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


}somanet_pdos;

typedef struct{
    //力迈胜伺服电机使用到的
    #if 0
    {0x6040, 0x00, 16}, /* ControlWord */
    {0x6060, 0x00, 8}, /* ModeOperation */
    {0x607a, 0x00, 32}, /* Position target value */
    {0x6042, 0x00, 16}, /* VITargVelocity */
    {0x6071, 0x00, 16}, /* Target torque  */
    {0x60c1, 0x01, 32}, /* s00_NumberEntries     */
    {0x60ff, 0x00, 32}, /* Target velocity  */
    {0x2031, 0x02, 16}, /* VDI_VirtualLevelCommSet       */

    {0x603f, 0x00, 16}, /* ErrorCode */
    {0x6041, 0x00, 16}, /* StatusWord */
    {0x6061, 0x00, 8}, /* Modes of operation display */
    {0x0000, 0x00, 8}, /* Gap */
    {0x6064, 0x00, 32}, /* Position actual user value  */
    {0x6063, 0x00, 32}, /* Position actual encoder value */
    {0x6069, 0x00, 32}, /* Velocity sensor actual value  */
    {0x606c, 0x00, 32}, /* Velocity actual value  */
    {0x6077, 0x00, 16}, /* Torque actual value  */
    {0x6078, 0x00, 16}, /* Current actual value  */
    {0x200b, 0x05, 16}, /* InterTorqueRef        */
    #endif
    unsigned int ctrl_word;
    unsigned int operation_mode;
    unsigned int Target_position;
    unsigned int target_velocity;
   
    unsigned int status_word;
    unsigned int mode_display;
    unsigned int Position_value;
    unsigned int velocity_value;
}NiMotion_pdos;

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
}somanet_motors,NiMotion_motors;

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


//serial线程运行状态切换
typedef enum
{
  SR_NULL = 0,
  SR_OPEN,
  SR_CLOSE,
  SR_RUN,
} S_STATE;

//ETHERCAT线程结构体信息ETHERCAT thread info
typedef struct
{
  bool         mWorking;
  char         mThreadName[15];

  int  mthreadMasterId;
  pthread_t  mPid;
  //cpu mask
  uint8_t         mCpuAffinity;

  //串口、网线程运行状态切换
  S_STATE mState;
  float serialTention[8];

  //控制电磁阀接收到上位机的消息
  bool  newUpperMsg;
  SUPR_R_MSG upperMsg;

  //底盘传感器信息
  Chassis_sensor_inf chserinf;

  float    cmd_proportion[13];

  //指向master主线程的结构体
  void *pTModule;

  //指向chassis 的master主线程的结构体
  void *pTChassisModule;

  int32_t recVisionframesCnt[8];

  //倾角仪数据
  float inclinxy[8][2];

  //拉力机数据
  float tension[8];

  //继电器开关状态
  /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
  */
  //uint8_t relays_24V[8];
  //uint8_t relays_48V[8];
  //uint8_t relays_brk_48V[8];
  //uint8_t relays_gas[8];
  
} ETHERCAT_THREAD_INFO,SERIAL_THREAD_INFO,VISION_THREAD_INFO;


//magic线程结构体信息thread info
typedef enum
{
  R_NULL = 0,
  R_STOP,
  R_VERTICAL_WEIGHT ,
  R_FOLLOW ,
  R_VERTICAL ,
  R_FOLLOW_VERTICAL ,
} MAGIC_RUN_STATE;
typedef struct
{
  bool         mWorking;
  char         mThreadName[15];
  
  MAGIC_RUN_STATE mRunState;

  void *pMaster;

  //cmd upper

  int  mthreadMasterId;
  pthread_t  mPid;
  //cpu mask
  uint8_t         mCpuAffinity;

  //标识机械臂的ID号。
  int16_t     mArmsId;

  /******************************算法使用到的数据结构*************************/
  //磁栅尺转换到末端值
  float sikoX_end,sikoY_end;
  
  //last周期绳索末端值偏差
  float rope_end_last_x,rope_end_last_y;

  //XYZ轴当前的速度m/s
  //float speed_x,speed_y,speed_z;

  //X主副轴算法起始偏差
  float x_intpos_start_error;
  float x_extpos_start_error;

  float last_synchron_dert;

  /*******************************是否启用算法******************************/
  uint8_t enable_flag;

  /*********************************速度命令插值到每个刷新周期************************************/
  float   target_vel_x_inters[100];
  float   target_vel_y_inters[100];
  int     target_vel_i;

  //随动算法运行周期数
  uint32_t magic_follow_runcnt;

  /********************************************以下是可修改的参数********************************************/
 
  float floating_last_p1;
  //微重力模式下绝压值
  //float cmd_target_abs;

  //微重力模式下记录上个周期卷扬同命令值
  float cmd_z_Last_speed;

  //称重模式下记录上周期数据
  float last_dert_z,last_cmd_medium,last_last_dert_z;

  /*******************************输入整形算法****************************************/
  //上个周期的随动命令值
  float cmd_last_xv,cmd_last_yv;
  float cmd_fir_last_xv;

  //称重模式下KI存储的10周期数据
  float ki_err[100];
  
  //算法测试使用的临时
  float temps[10];


  //记录额外的计算信息,Xm Xs Y Z
  #define CMD_VEL_MEAN_ERROR_SIZE 50
  float cmd_vel_error[4];
  float cmd_vel_mean_error_n[4];
  float mean_errors[4][CMD_VEL_MEAN_ERROR_SIZE];

  float xStartPos,yStartPos;
  float xSStartPos;

  /*
  //仿真运行次数
  uint32_t magic_sim_cnt;


  uint32_t auto_follow_sim_d_size;
  POINTS_F* auto_follow_sim_data;
  float tests[10];
*/
} MAGIC_THREAD_INFO;

//主站采集的传感器滤波结构体,2ms采集一次数据
#define F_ARR_SIZE 500
typedef struct
{
  //磁栅尺滤波次数
  int F_siko_n;
  //相机采集倾角滤波次数
  int F_cam_n;
  //拉力计数据滤波次数
  int F_tension_n;
  //倾角仪数据滤波
  int F_incln_n;
  //电机速度、位置滤波次数
  int F_vel_n;
  //绝压传感器滤波次数
  int F_abs_n;
  //垂直算法滤波次数
  int F_z_cmd_n;
  //随动算法滤波次数
  int F_xy_cmd_n;

  //磁栅尺数据滤波,存储数据
  float  sikoX_arr[F_ARR_SIZE];
  float  sikoY_arr[F_ARR_SIZE];
  float  sikoZ_arr[F_ARR_SIZE];
  float  sikoZ_V_arr[F_ARR_SIZE];
  float  sikoZ_A_arr[F_ARR_SIZE];
  //相机倾角数据滤波,存储数据
  float  camX_arr[F_ARR_SIZE];
  float  camY_arr[F_ARR_SIZE];

  //拉力计数据滤波,存储数据
  float  tension_arr[F_ARR_SIZE];
  //倾角仪XY数据滤波
  float  incln_X_vec[F_ARR_SIZE];
  float  incln_Y_vec[F_ARR_SIZE];

  //电机速度、位置滤波数据存储
  float  xM_vel_arr[F_ARR_SIZE];
  float  xS_vel_arr[F_ARR_SIZE];
  float  y_vel_arr[F_ARR_SIZE];
  float  z_vel_arr[F_ARR_SIZE];
  
  //绝压传感器值滤波
  float  abs_press_arr[F_ARR_SIZE];

  //垂直算法电机控制命令滤波
  float  m_z_cmd_arr[F_ARR_SIZE];

  //随动算法X电机控制命令滤波
  float  m_x_cmd_arr[F_ARR_SIZE];
  //随动算法y电机控制命令滤波
  float  m_y_cmd_arr[F_ARR_SIZE];
  
  float  d_rope_x_arr[F_ARR_SIZE];
  float  d_rope_y_arr[F_ARR_SIZE];
  
} FILTERING_PARAM;

typedef struct
{
  //随动的KP KD
    float follow_kp;
    float follow_kd;
    float follow_ki;
    //积分项
    double follow_ins[2];
    //XYZ轴的算法限速m/s
    float magic_max_x_speed,magic_max_y_speed,magic_max_z_speed;

    //气缸中位值
    float cylinder_median_pos;
    //称重模式下ebsl wn
    float floating_weight_ebsl,floating_weight_wn;
    
    //微重力模式下ebsl wn
    float floating_air_ebsl,floating_air_wn,floating_air_target_press;
    float floating_windlass_ebsl,floating_windlass_wn;
    //称重及气浮模式下中阀的最大电压值
    float median_ratio_max_u;

    //master使用的参数,
    //Z轴0位点 编码器值
    int32_t z_encoder_zore;
    //Y轴0位点 编码器值
    int32_t y_encoder_zore;
    //x副轴0位点 编码器值
    int32_t xs_encoder_zore;
    //x主轴0位点 编码器值
    int32_t xm_encoder_zore;

    //master使用的参数,
    //XYZ 轴软限位.单位m
    float x_soft_left_limi,x_soft_right_limi;
    float y_soft_front_limit,y_soft_back_limit;
    float z_soft_up_limit,z_soft_down_limit;

    //上层或者下层
    int8_t down_up;

    //相机XY向系数值。转换到底部
    float vision_coefficient_x, vision_coefficient_y;

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
  uint8_t         mCpuAffinity;

  //框架ID
  uint16_t  mIdentifier;

  void *pTChassis;
/****************************以下是视觉线程使用*****************************/
  //ethercat线程，主要负责1000hz交换数据
  VISION_THREAD_INFO mVisionWorker; 
/****************************以下是视觉线程 end*****************************/

/****************************以下是igh主站使用*****************************/
  SOMANETN_MODE somanetn_mode; //主站配置伺服电机运行模式
  SOMANET_PP_RUN_STATE pp_run_state[MASTER_ARMS_MAX_SIZE][4]; //吊杆中所有机械臂PP模式下，每个轴运行状态
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
  
  //交换的pdo数据,一个arm下有4个电机
  somanet_pdos   somanet_pdo[MASTER_ARMS_MAX_SIZE][4];

  //EC-3045.0:开关量输出 1 2 3 4模拟量输出，5 开关量输入 6 7 8 9模拟量输入
  unsigned int   EC_3045[MASTER_ARMS_MAX_SIZE][10];

  uint8_t   EC_3045_switch[MASTER_ARMS_MAX_SIZE];
  uint16_t  EC_3045_abs_stress[MASTER_ARMS_MAX_SIZE];
  //EC-3045.0:开关量输出 1 2 3 4模拟量输出，5 开关量输入 6 7 8 9模拟量输入
  uint16_t       EC_3045_vari[MASTER_ARMS_MAX_SIZE][10];

  somanet_motors motors[MASTER_ARMS_MAX_SIZE][4];
/****************************以下是igh主站使用 end*****************************/

/****************************************上位机速度控制、位置控制*************************************/
  //上位机缓坡加速使用,上位机速度控制，采用固定加速度。单位mm
  float  upper_target_velocity[MASTER_ARMS_MAX_SIZE][4];
  float  upper_velocity_v0[MASTER_ARMS_MAX_SIZE][4];
  uint8_t upper_target_v_Flag[MASTER_ARMS_MAX_SIZE];
  float  upper_acc[MASTER_ARMS_MAX_SIZE][4];
  float  upper_last_v_cmd[MASTER_ARMS_MAX_SIZE][4];
 
  //上位机位置控制使用,上位机速度控制，采用固定加速度。单位mm
  float  upper_target_pos[MASTER_ARMS_MAX_SIZE][4];
  #define    upper_pos_smooth_cnt 10 
  float  upper_pos_vel_smooth[MASTER_ARMS_MAX_SIZE][4][upper_pos_smooth_cnt];
  uint8_t upper_target_p_Flag[MASTER_ARMS_MAX_SIZE];
  int32_t upper_target_p_Kp_cnt[MASTER_ARMS_MAX_SIZE];

  //位置控制，最大加速度限制做平滑处理
  float  target_p_last_cmdvel[MASTER_ARMS_MAX_SIZE][4];

/*************end*/

/********************************************机械臂转换成国际单位的器件数据*****************************************/
  //四个轴的方向正负号
  int32_t  AXIS_DIR[MASTER_ARMS_MAX_SIZE][4];

  //磁栅尺数据 单位:m
  float sikoXYZ[MASTER_ARMS_MAX_SIZE][3];
  float sikoXYZ_ZERO[MASTER_ARMS_MAX_SIZE][3];

  float sikoZ_Last[MASTER_ARMS_MAX_SIZE];
  float sikoZ_V[MASTER_ARMS_MAX_SIZE];
  float sikoZ_V_Last[MASTER_ARMS_MAX_SIZE];
  float sikoZ_A[MASTER_ARMS_MAX_SIZE];

  float camXY[MASTER_ARMS_MAX_SIZE][2];

  int32_t camFps[MASTER_ARMS_MAX_SIZE];

  //拉力计数据 单位：kg
  float tension[MASTER_ARMS_MAX_SIZE];

  //倾角仪数据 单位：m/s2
  float incln_acc_XY[MASTER_ARMS_MAX_SIZE][2];

/**********************************伺服电机PDO*********************************/
  //速度指令mm
  float  cmd_velocity[MASTER_ARMS_MAX_SIZE][4];
  //X Y Z状态字
  uint16_t somanet_status[MASTER_ARMS_MAX_SIZE][4];
  
  //x轴主从轴速度，y z轴速度，单位：mm/s
  float xMExtSpeed[MASTER_ARMS_MAX_SIZE];
  float xSExtSpeed[MASTER_ARMS_MAX_SIZE];
  float xMIntSpeed[MASTER_ARMS_MAX_SIZE];
  float xSIntSpeed[MASTER_ARMS_MAX_SIZE];
  float ySpeed[MASTER_ARMS_MAX_SIZE];
  float zSpeed[MASTER_ARMS_MAX_SIZE];
  //x轴主从轴位置，y z轴位置 单位m
  double xMExtPos[MASTER_ARMS_MAX_SIZE];
  double xMIntPos[MASTER_ARMS_MAX_SIZE];
  double xSIntPos[MASTER_ARMS_MAX_SIZE];
  double xSExtPos[MASTER_ARMS_MAX_SIZE];

  double yPos[MASTER_ARMS_MAX_SIZE];
  double zPos[MASTER_ARMS_MAX_SIZE];

  //单元内轴当前力矩值
  double torqueActual[MASTER_ARMS_MAX_SIZE][4];

  //速度环生成的力矩
  double torqueDemand[MASTER_ARMS_MAX_SIZE][4];

  //磁栅尺Z
  double zExtPos[MASTER_ARMS_MAX_SIZE];
  //X轴主轴 副轴的绝对位置起始点
  double xStartMExtPos[MASTER_ARMS_MAX_SIZE];
  double xStartSExtPos[MASTER_ARMS_MAX_SIZE];

  //绝压传感器值
  float  abs_stress[MASTER_ARMS_MAX_SIZE];

  //比例阀控制命令  单位：电压V
  float proportion_cmd[MASTER_ARMS_MAX_SIZE][4];
/********************************************转换成国际单位的器件数据end*****************************************/

/********************************************设置上下层单元id*****************************************/
  //id<0时候单元不存在
  int16_t first_id[2],last_id[2];

/********************************************配置参数*****************************************/
  //采集传感器及电机 滤波器的数据结构体
  FILTERING_PARAM mF_Inf[MASTER_ARMS_MAX_SIZE];
  ARM_PARAM mAparams[MASTER_ARMS_MAX_SIZE];
  //主站上传数据时间间隔
  uint32_t PACK_MSG_MS ;

/********************************************配置参数end*****************************************/

/********************************************仿真测试*******************************************/
  //uint32_t auto_follow_sim_d_size[MASTER_ARMS_MAX_SIZE];
  //POINTS_F* auto_follow_sim_data[MASTER_ARMS_MAX_SIZE];

/******************end*/

  //算法线程信息,最多有7个臂
  MAGIC_THREAD_INFO mMagicWorker[MASTER_ARMS_MAX_SIZE];

  //参数列表.一套机械臂有一套参数
  //param_list p_params[MASTER_ARMS_MAX_SIZE];

/**********************************************插值数据******************************************/
  quads_unit_head mQuads[MASTER_ARMS_MAX_SIZE];

/***********************************************串口线程指针*****************************************/
  SERIAL_THREAD_INFO *mSerialWorker;

} ARMS_MASTER_THREAD_INFO;


//master线程结构体信息
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
  uint8_t         mCpuAffinity;

  uint16_t mIdentifier;
/****************************以下是igh主站使用*****************************/
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

  //从站信息,一个主站上最多
  EK1100_slaves_inf slaves_inf;

  //在CiA 402中，规定电力驱动系统（PDS）的行为应由有限状态自动机（FSA）控制
  S_ETHR_STATE  arm_fsa;
 
/****************************以下是igh主站使用 end*****************************/
  //记录domain偏移量。
  NiMotion_pdos nimotion_pdos[NIMOTION_SIZE];
  int           motor_size;

  //真实传输的PDO
  NiMotion_motors motors[NIMOTION_SIZE];
/********************************************底盘转换成国际单位的器件数据*****************************************/
  //8个轴的减速比,m/s转RPM
  float  CH_SPEED_TO_RPM[NIMOTION_SIZE];

  //8个轴的方向正负号
  int32_t  CH_AXIS_DIR[NIMOTION_SIZE];

  //状态字
  uint16_t ni_stword[NIMOTION_SIZE];

  //单位：mm/s
  float w_speed[4];
  float r_speed[4];

  //x轴主从轴位置，y z轴位置 单位m
  double w_pos[4];
  double r_pos[4];

  //速度指令滤波.
  #define SMOOTH_SIZE 12
  int32_t  tar_vel_cmd[NIMOTION_SIZE];
  int32_t  velocity_smooth[NIMOTION_SIZE][SMOOTH_SIZE];
/********************************************转换成国际单位的器件数据end*****************************************/
/****************************以下是串口使用*****************************/
  //串口线程，
  SERIAL_THREAD_INFO *mSerialWorker;

/********************************底盘配置参数************************************/
  //参数列表.一套机械臂有一套参数
  //param_list p_params;

  //参数列表，行走、旋转最大速度
  float max_w_v;
  float max_r_v;

  //四个舵轮的零点位置
  float r_pos_zore[4];

/**********************************底盘算法运行状态********************************************/
  uint8_t enable_flag;
  MAGIC_RUN_STATE mRunState;

/***********************************机械臂运行信息**************************************/
  ARMS_MAGIC_CTRL_V  mArmsMagicCtrl;

/***********************************缓坡加减速************************************************/
  //行走电机的缓坡.速度控制
  int32_t upper_target_v_Flag[5];
  //舵轮的行走和旋转目标速度
  float   upper_target_velocity[5][2]; 
  float   upper_velocity_v0[5][2];
  float   upper_acc[5][2];
  float   upper_last_v_cmd[5][2];

  //行走电机的缓坡.位置控制.四个舵轮
  int32_t upper_target_p_Flag[4];
  //舵轮的行走和旋转目标速度
  float   upper_target_pos[4]; 
  float   upper_vel_tag[4];
  float   upper_last_p_cmd[4];

//吊杆主站信息
  ARMS_MASTER_THREAD_INFO *mArmsModule;
  int16_t  ArmsReferId;

  //基座随动算法
  float fai_R_1,d_fai_R1,fai_R_2;
  float last_d_xo1_R,last_d_yo1_R,last_d_fai_R1;
  float last_dd_fai_R1;

  //缓冲圆区域启用算法标志,r1是小圆，r2是大圆。大于r2时候需要控制，此时记录标志位
  bool ideal_r_signal_flag;

  //随动算法，指定吊杆理想位置
  float L_R_ideal;
  float D_R_ideal;

  //全局变量
  float v_d_now[4];
  float d_sita_d_base_now[4];
  float sita_base_now[4];

  //基座XY向加速度期望值
  float dd_x01_real;
  float dd_y01_real;

} CHASSIS_MASTER_THREAD_INFO;

#pragma pack()
#endif // SYS_ARMS_DEFS_H
