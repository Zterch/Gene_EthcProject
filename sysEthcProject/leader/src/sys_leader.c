/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_supr.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:系统中的超级线程模块，此模块负责创建所有线程，管理其他模块，周期的发送
消息通知其他模块。
*
********************************************************************************/
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <sys/file.h>

#include "sys_ctl.h"
#include "sys_db.h"
#include "sys_leader.h"
#include "sys_master.h"
#include "sys_master_chassis.h"
#include "sys_leader_defs.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"
#include "sys_master_serial.h"
#include "sys_commu_ipc.h"

const char  MN_MASTER_NAME[2][24] = {"MN_MASTER_ARMS","MN_MASTER_CHASSIS"};

//模块结构体，线程名字，入口函数，pid
typedef struct _MODULEINFOS
{
  const char* mName;
  void *(*mEntry)(void* mModule);
  int32_t mPri;
  pthread_t mPid;
} MODULEINFOS;

//socket
int32_t  mRecSoket;

uint16_t mFrameId;

//thread parame
#define MODULE_NUMS 2

ARMS_MASTER_THREAD_INFO mArmsModule;
CHASSIS_MASTER_THREAD_INFO mChassisModule;
//串口线程，主要负责读写底盘器件并交换数据
SERIAL_THREAD_INFO mSerialWorker; 

//文件锁描述符
static int g_single_proc_inst_lock_fd = -1;

static int suprWorking = 1;

//数据库中所有表
strs_list tables_name_list = {0};
int32_t tables_name_list_ack_index;

db_table tables_entry = {0};
int32_t amrs_param_ack_arm_id;
static int32_t tables_entry_ack_index = -1;

static int32_t prepareEnv(void);
static int32_t initSupr(void);
static int32_t startModules(void);
static int32_t init_db_param();
static int32_t init_up_down_id();
static int32_t suprMainLoop();
static void signalHandle(int mSignal);
static void checkTimerWorking();
static void single_proc_inst_lockfile_cleanup(void);
int32_t is_single_proc_inst_running(const char *process_name);

static int32_t initPhase1(const int32_t* mMsgAllQs);
static int32_t initPhase2();
static int32_t sendNoticeToSupr(const int32_t msMsgId, const int32_t mNotice);
static int32_t sendNoticeToMyself(const int32_t msMsgId, const int32_t mNotice);

static int32_t handleSetZZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
static int32_t handleSetYZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
static int32_t handleSetXMZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
static int32_t handleSetXSZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
static int32_t setParamMotorPosZore(int mid,char* key,float value);

static int32_t readTableEntry(const char* table_name);
static int32_t deleteTableEntry(SUPR_R_MSG *mRMsg);
static int32_t updateTableEntry(SUPR_R_MSG *mRMsg);
static void  read_table_vaule_to_arms(ARMS_MASTER_THREAD_INFO *pTModule,int armid,const char* table);
static void  read_table_vaule_to_frame(const char* table);
static void  update_vaule_to_arms(ARMS_MASTER_THREAD_INFO *pTModule,int armid,const char* key,double value);
static void  update_vaule_to_frame(const char* key,double value);

static int32_t armsReadDBTableName(SUPR_R_MSG *mRMsg);

void  update_params_to_frame(CHASSIS_MASTER_THREAD_INFO * pchassis);

static int32_t sendLongMsg(const int mMsgQ, const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue, const char* mMsgData, const int32_t mDSize);
static int32_t deInitSupr(void);

static int32_t sendMsgToUpper(const uint16_t mMsgId, int i, void* data, int32_t dataSize);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//关闭文件锁
static void single_proc_inst_lockfile_cleanup(void)
{
    if (g_single_proc_inst_lock_fd != -1) {
        close(g_single_proc_inst_lock_fd);
        g_single_proc_inst_lock_fd = -1;
    }
}

int32_t is_single_proc_inst_running(const char *process_name)
{
    char lock_file[128];
    snprintf(lock_file, sizeof(lock_file), "/var/tmp/%s.lock", process_name);

    g_single_proc_inst_lock_fd = open(lock_file, O_CREAT|O_RDWR, 0644);
    if (-1 == g_single_proc_inst_lock_fd) {
        fprintf(stderr, "Fail to open lock file(%s). Error: %s\n",
            lock_file, strerror(errno));
        return -1;
    }

    if (0 == flock(g_single_proc_inst_lock_fd, LOCK_EX | LOCK_NB)) {
        atexit(single_proc_inst_lockfile_cleanup);
        return 0;
    }

    close(g_single_proc_inst_lock_fd);
    g_single_proc_inst_lock_fd = -1;
    return -1;
}

static int32_t sendNoticeToSupr(const int32_t msMsgId, const int32_t mNotice)
{
  int iret = 0;
  iret = MsgSendNotice(mRecSoket,get_mn_port(MN_ID_SUPR), MN_ID_LEADER, MN_ID_SUPR, msMsgId, mNotice, 0);
  return iret;
}

static int32_t sendNoticeToMyself(const int32_t msMsgId, const int32_t mNotice)
{
    int iret = 0;
    iret = MsgSendNotice(mRecSoket,get_mn_port(MN_ID_LEADER), MN_ID_LEADER, MN_ID_LEADER, msMsgId, mNotice, 0);
    return iret;
}

static int32_t sendNoticeToThread(const int32_t msMsgId, const int32_t mNotice)
{
    int iret = 0;
    for(int i=0; i<MODULE_NUMS; i++)
    {
        MsgSendNotice(mRecSoket,get_mn_master_port(i), MN_ID_LEADER, MN_ID_LEADER, msMsgId, mNotice, 0);
    }
    return iret;
}

static int32_t sendLongMsg(const int mMsgQ, const int32_t mRecer, const int32_t mMsgId,
                           const int32_t mNotice, const int32_t mValue, const char* mMsgData, const int32_t mDSize)
{
  int iret = 0;
  //iret = BASE::MsgSendLongMsg(mMsgQ, BASE::MN_ID_LEADER, mRecer, mMsgId, mNotice, mValue, mMsgData, mDSize);
  return iret;
}

static int32_t sendMsgToUpper(const uint16_t mMsgId, int i, void* data, int32_t dataSize)
{
    int mSMsgSize = sizeof(ARMS_S_MSG)+dataSize;
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);

    memcpy(mSMsg->mData,data,dataSize);
    //框架1，暂时设置
    mSMsg->mIdentifier = mArmsModule.mIdentifier;
    if(i>=0 && i<7)
      mSMsg->mModuleFlag[i] = 1;
    
    mSMsg->mMsgType = mMsgId;
    sendDataToUpper(mRecSoket,MSG_ID_MASTER_SHOW_PARAMS,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg);
}

static int32_t initPhase1(const int32_t* mMsgAllQs)
{
  int iRet = 0;
  if(mMsgAllQs != 0)
  {
    ;
  }
  iRet = sendNoticeToSupr(MSG_ID_MODULE_INIT1_ACK, 0);
  return iRet;
}

/******************************************************************************
* 功能：初始化整个系统的运行环境
* @return Descriptions
******************************************************************************/
static int32_t prepareEnv(void) {
  int32_t iRet = 0;

  printf("leader APP STARTING\n");
  //TODO:: msg and msg q need refactor after first version.

  if(initSupr() < 0)
    return -1;

  init_db_param();

  init_up_down_id();

  return iRet;
}

/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(void) 
{
  //need handler error case
  int32_t iRet = 0;

  signal(SIGINT, signalHandle);

  //创建soket
  mRecSoket = createSoket(get_mn_port(MN_ID_LEADER), MN_LocalIpV4Str, 0, 50000);

  //supr run in cpu0
  hiSetCpuAffinity(CPU_LEAD);
  hiSetThreadsched(pthread_self(), PRI_LEAD);

  //向supr注册msg队列
  pid_t mpid = getpid();
  iRet = MsgSendRegisterMsgQ(mRecSoket,get_mn_port(MN_ID_SUPR), MN_ID_LEADER, MN_ID_SUPR, MSG_ID_MODULE_REGISTER_MSG_Q_ACK, mpid);

  return iRet;
}

static int32_t init_up_down_id()
{
    //读取下层和上层起始id结束id.1:下层，2：上层。注意：下层id必须连续，上层id也必须连续
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {   
        if(mArmsModule.mAparams[i].down_up==1)
        {
            mArmsModule.first_id[0] = i;
            break;
        }
    }
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mArmsModule.mAparams[i].down_up==1)
        {
            mArmsModule.last_id[0] = i;
        }
    }
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {   
        if(mArmsModule.mAparams[i].down_up==2)
        {
            mArmsModule.first_id[1] = i;
            break;
        }
    }
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mArmsModule.mAparams[i].down_up==2)
        {
            mArmsModule.last_id[1] = i;
        }
    }
}

static int32_t init_db_param()
{
    //创建slaves_inf头
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {    
      //初始化数据库
      create_param_table(DB_FILE,db_arms_tables[i]);
      insert_params_db(DB_FILE,db_arms_tables[i],arms_params_names);

      //初始化参数链表
      read_table_vaule_to_arms(&mArmsModule,i,db_arms_tables[i]);
    }
    //初始化数据库
    create_param_table(DB_FILE,db_frame_table);
    insert_params_db(DB_FILE,db_frame_table,frame_params_names);
    //初始化参数链表
    read_table_vaule_to_frame(db_frame_table);
}

static void  update_vaule_to_frame(const char* key,double value)
{  
    //底盘参数设置，通过串口指针得到
    //主站通信配置参数
    if(strcmp("CHASSIS_MAX_W_V",key) == 0)
    {
        mChassisModule.max_w_v = fabs(value);
    }else if(strcmp("CHASSIS_MAX_R_V",key) == 0)
    {
        mChassisModule.max_r_v = fabs(value);
    }else if(strcmp("CHASSIS1_ZORE",key) == 0)
    {
        mChassisModule.r_pos_zore[0] = value;
    }else if(strcmp("CHASSIS2_ZORE",key) == 0)
    {
        mChassisModule.r_pos_zore[1] = value;
    }else if(strcmp("CHASSIS3_ZORE",key) == 0)
    {
        mChassisModule.r_pos_zore[2] = value;
    }else if(strcmp("CHASSIS4_ZORE",key) == 0)
    {
        mChassisModule.r_pos_zore[3] = value;
    }else if(strcmp("MASTER_UPLOAD_MSG_MS",key) == 0)
    {
        mArmsModule.PACK_MSG_MS  = abs((uint32_t)value);   
    }else{
        ;
    }
}

static void  update_vaule_to_arms(ARMS_MASTER_THREAD_INFO *pTModule,int armid,const char* key,double value)
{
    MAGIC_THREAD_INFO *pMagicWorker = (MAGIC_THREAD_INFO *)&pTModule->mMagicWorker[armid];
    if(strcmp("FOLLOW_KP",key) == 0)
    {
        pTModule->mAparams[armid].follow_kp = value;
    }else if(strcmp("FOLLOW_KD",key) == 0)
    {
        pTModule->mAparams[armid].follow_kd = value;
    }else if(strcmp("FOLLOW_KI",key) == 0)
    {
        pTModule->mAparams[armid].follow_ki = value;
    }else if(strcmp("FOLLOW_MAX_X_SPEED",key) == 0)
    {
        pTModule->mAparams[armid].magic_max_x_speed = value;
    }else if(strcmp("FOLLOW_MAX_Y_SPEED",key) == 0)
    {
        pTModule->mAparams[armid].magic_max_y_speed = value;
    }else if(strcmp("CYLINDER_MEDIAN_POS",key) == 0)
    {
        pTModule->mAparams[armid].cylinder_median_pos = value;
    }else if(strcmp("FLOATING_WEIGHT_EBSL",key) == 0)
    {
        pTModule->mAparams[armid].floating_weight_ebsl = value;
    }else if(strcmp("FLOATING_WEIGHT_WN",key) == 0)
    {
        pTModule->mAparams[armid].floating_weight_wn = value;
    }else if(strcmp("FLOATING_MAX_Z_SPEED",key) == 0)
    {
        pTModule->mAparams[armid].magic_max_z_speed = value;
    }else if(strcmp("MEDIAN_RATIO_MAX_U",key) == 0)
    {
        pTModule->mAparams[armid].median_ratio_max_u = value;
    }else if(strcmp("FLOATING_AIR_EBSL",key) == 0)
    {
        pTModule->mAparams[armid].floating_air_ebsl = value;
    }else if(strcmp("FLOATING_AIR_WN",key) == 0)
    {
        pTModule->mAparams[armid].floating_air_wn = value;
    }else if(strcmp("FLOATING_AIR_TARGET_PRESS",key) == 0)
    {
        pTModule->mAparams[armid].floating_air_target_press = value;
    }else if(strcmp("FLOATING_WINDLASS_EBSL",key) == 0)
    {
        pTModule->mAparams[armid].floating_windlass_ebsl = value;
    }else if(strcmp("FLOATING_WINDLASS_WN",key) == 0)
    {
        pTModule->mAparams[armid].floating_windlass_wn = value;
    }else if(strcmp("MOTOR_Z_ZORE",key) == 0)
    {
        pTModule->mAparams[armid].z_encoder_zore = (int32_t)value;
    }else if(strcmp("MOTOR_Y_ZORE",key) == 0)
    {
        pTModule->mAparams[armid].y_encoder_zore = (int32_t)value;
    }else if(strcmp("MOTOR_XS_ZORE",key) == 0)
    {
        pTModule->mAparams[armid].xs_encoder_zore = (int32_t)value;
    }else if(strcmp("MOTOR_XM_ZORE",key) == 0)
    {
        pTModule->mAparams[armid].xm_encoder_zore = (int32_t)value;
    }else if(strcmp("X_SOFT_LEFT_LIMIT",key) == 0)
    {
        pTModule->mAparams[armid].x_soft_left_limi = value;
    }else if(strcmp("X_SOFT_RIGHT_LIMIT",key) == 0)
    {
        pTModule->mAparams[armid].x_soft_right_limi = value;
    }else if(strcmp("Y_SOFT_FRONT_LIMIT",key) == 0)
    {
        pTModule->mAparams[armid].y_soft_front_limit = value;
    }else if(strcmp("Y_SOFT_BACK_LIMIT",key) == 0)
    {
        pTModule->mAparams[armid].y_soft_back_limit = value;
    }else if(strcmp("Z_SOFT_UP_LIMIT",key) == 0)
    {
        pTModule->mAparams[armid].z_soft_up_limit = value;
    }else if(strcmp("Z_SOFT_DOWN_LIMIT",key) == 0)
    {
        pTModule->mAparams[armid].z_soft_down_limit = value;
    }else if(strcmp("FILTER_SIKO_TIMES",key) == 0)
    {
        pTModule->mF_Inf[armid].F_siko_n = (int32_t)value;
    }else if(strcmp("FILTER_CAM_TIMES",key) == 0)
    {
        pTModule->mF_Inf[armid].F_cam_n = (int32_t)value;
    }else if(strcmp("FILTER_TENSION_TIMNES",key) == 0)
    {
        pTModule->mF_Inf[armid].F_tension_n = (int32_t)value;
    }else if(strcmp("FILTER_INCLN_TIMES",key) == 0)
    {
        pTModule->mF_Inf[armid].F_incln_n = (int32_t)value;
    }else if(strcmp("FILTER_SPEED_TIMES",key) == 0)
    {
        pTModule->mF_Inf[armid].F_vel_n = (int32_t)value;
    }else if(strcmp("FILTER_ABS_PRESS_TIMES",key) == 0)
    {
        pTModule->mF_Inf[armid].F_abs_n = (int32_t)value;
    }else if(strcmp("FILTER_FLOATING_MOTOR_Z_V",key) == 0)
    {
        pTModule->mF_Inf[armid].F_z_cmd_n = (int32_t)value;
    }else if(strcmp("FILTER_FOLLOW_MOTOR_XY_V",key) == 0)
    {
        pTModule->mF_Inf[armid].F_xy_cmd_n = (int32_t)value;
    }else if(strcmp("DOWN_OR_UP",key) == 0)
    {
        pTModule->mAparams[armid].down_up = (int8_t)value;
    }else if(strcmp("VISION_COEFFICIENT_X",key) == 0)
    {
        pTModule->mAparams[armid].vision_coefficient_x = value;
    }else if(strcmp("VISION_COEFFICIENT_Y",key) == 0)
    {
        pTModule->mAparams[armid].vision_coefficient_y = value;
    }
    else{

    }
}

static void  read_table_vaule_to_frame(const char* table)
{
    int pI=0;
    while (strlen(frame_params_names[pI].key)>0)
    {
        //插入数据库
        double vaule = 0;
        get_value_from_dbtable(DB_FILE,table,frame_params_names[pI].key,&vaule);
        update_vaule_to_frame(frame_params_names[pI].key,vaule);
        pI++;
    }
}

static void  read_table_vaule_to_arms(ARMS_MASTER_THREAD_INFO *pTModule,int armid,const char* table)
{
    int pI=0;
    while (strlen(arms_params_names[pI].key)>0)
    {
        //插入数据库
        double vaule = 0;
        get_value_from_dbtable(DB_FILE,table,arms_params_names[pI].key,&vaule);
        update_vaule_to_arms(pTModule,armid,arms_params_names[pI].key,vaule);
        pI++;
    }
}

void  update_params_to_frame(CHASSIS_MASTER_THREAD_INFO * pchassis)
{
    #if 0
    param_list * plist = &pchassis->p_params;    
    //底盘参数设置，通过串口指针得到
    pchassis->max_w_v = fabs(get_list_value(plist,"CHASSIS_MAX_W_V"));
    pchassis->max_r_v = fabs(get_list_value(plist,"CHASSIS_MAX_R_V"));
    pchassis->r_pos_zore[0] = get_list_value(plist,"CHASSIS1_ZORE");
    pchassis->r_pos_zore[1] = get_list_value(plist,"CHASSIS2_ZORE");
    pchassis->r_pos_zore[2] = get_list_value(plist,"CHASSIS3_ZORE");
    pchassis->r_pos_zore[3] = get_list_value(plist,"CHASSIS4_ZORE"); 
    //主站通信配置参数
    mArmsModule.PACK_MSG_MS  = abs((uint32_t)get_list_value(plist,"MASTER_UPLOAD_MSG_MS"));  
    #endif  
}

/******************************************************************************
* 功能：各个线程模块初始化，并且启动个各模块线程
* @return Descriptions
******************************************************************************/
static int32_t startModules(void) {

  // arms thread
  mArmsModule.mWorking = true;
  mArmsModule.mCpuAffinity  = CPU_MASTER;
  strcpy(mArmsModule.mThreadName, MN_MASTER_NAME[0]);
  mArmsModule.mState = M_STATE_INIT;
  mArmsModule.mthreadMasterId = 0;
  mArmsModule.masterId = 0;
  mArmsModule.slaves_size = 0;
  mArmsModule.mSerialWorker = &mSerialWorker;
  mArmsModule.mIdentifier = mFrameId;
  mArmsModule.pTChassis = &mChassisModule;
  usleep(10000);
  //机械臂主站
  mArmsModule.mPid  = hiCreateThread(MN_MASTER_NAME[0], master_arms_threadEntry,PRI_MASTER, &mArmsModule);

  // chassis thread
  mChassisModule.mWorking = true;
  mChassisModule.mCpuAffinity  = CPU_MASTER;
  strcpy(mChassisModule.mThreadName, MN_MASTER_NAME[1]);
  mChassisModule.mState = M_STATE_INIT;
  mChassisModule.mthreadMasterId = 1;
  mChassisModule.masterId = 1;
  //拷贝串口线程指针
  mChassisModule.mSerialWorker = &mSerialWorker;
  mChassisModule.mIdentifier = mFrameId;
  mChassisModule.mArmsModule = &mArmsModule;
  mChassisModule.ArmsReferId = 0;
  usleep(10000);
  //机械臂主站
  mChassisModule.mPid  = hiCreateThread(MN_MASTER_NAME[1], master_chassis_threadEntry,PRI_MASTER, &mChassisModule);

  //seral串口线程创建
  mSerialWorker.mWorking = 1;
  strcpy(mSerialWorker.mThreadName,"serials");
  mSerialWorker.pTModule = &mArmsModule;
  mSerialWorker.pTChassisModule = &mChassisModule;
  mSerialWorker.mPid = hiCreateThread("serials", serialThread,PRI_MASTER, &mSerialWorker);

  return 0;
}


static int32_t initPhase2()
{
  int32_t iRet = 0;
  iRet = startModules();
  return iRet;
}

/******************************************************************************
* 功能：检查leader线程模块是否在运行
* @return Descriptions
******************************************************************************/
static void checkTimerWorking()
{
  if(!mArmsModule.mWorking || !mChassisModule.mWorking)
    suprWorking = 0;
}

static int32_t handleSetXMZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            //设置Z轴0位置偏移,编码器值
            setParamMotorPosZore(i,"MOTOR_XM_ZORE",pTModule->motors[i][1].Int_Pos_value);
        }
    }
}

static int32_t handleSetZZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            //设置Z轴0位置偏移,编码器值
            setParamMotorPosZore(i,"MOTOR_Z_ZORE",pTModule->motors[i][3].Int_Pos_value);
        }
    }
}

static int32_t handleSetYZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            //设置Y轴0位置偏移,编码器值
            //y轴没有配置外部编码器，Ext_Pos_value，读取的值是606c寄存器里的，也就是电机内部编码器
            setParamMotorPosZore(i,"MOTOR_Y_ZORE",pTModule->motors[i][0].Ext_Pos_value);
        }
    }
}

static int32_t handleSetXSZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            //设置x轴0位置偏移,编码器值
            setParamMotorPosZore(i,"MOTOR_XS_ZORE",pTModule->motors[i][2].Int_Pos_value);
        }
    }
}

static int32_t setParamMotorPosZore(int mid,char* key,float value)
{
    int iRet = 0;
    char s_value[100];
    sprintf(s_value,"%f",value);
    iRet = update_table_entry(DB_FILE,db_arms_tables[mid],"name",key,"value",s_value);
    if(iRet == 0)
    {
        update_vaule_to_arms(&mArmsModule,mid,key,value);
    }
}


static int32_t  armsReadDBTableName(SUPR_R_MSG *mRMsg)
{
    deinit_strs_list(&tables_name_list);
    INIT_LIST_HEAD(&tables_name_list.list);
    db_tables_name(DB_FILE, &tables_name_list);

    //定期的上传到upper，模拟定时器
    tables_name_list_ack_index = tables_name_list.str_cnt;
}

static int32_t deleteTableEntry(SUPR_R_MSG *mRMsg)
{
    //是否查询frame参数
    TABLE_ENTRY* table_entry =  (TABLE_ENTRY*)mRMsg->mData;
    delete_table_entry(DB_FILE,table_entry->entry[0],table_entry->entry[1],table_entry->entry[2]);
}

static int32_t updateTableEntry(SUPR_R_MSG *mRMsg)
{
    int iRet = 0;
    //是否查询frame参数
    TABLE_ENTRY* table_entry =  (TABLE_ENTRY*)mRMsg->mData;
    iRet = update_table_entry(DB_FILE,table_entry->entry[0],table_entry->entry[1],table_entry->entry[2],table_entry->entry[3],table_entry->entry[4]);
    if(iRet == 0)
    {
        char* errord;
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
        {
            if(strcmp(db_arms_tables[i],table_entry->entry[0]) == 0)
            {
                update_vaule_to_arms(&mArmsModule,i,table_entry->entry[2], strtod(table_entry->entry[4],&errord));
                //发送修改完毕
                sendMsgToUpper(ACK_NOTICE_UPDATE_TABLE_ENTYR_OK,-1,0,0);
                break;
            }
        }

        if(strcmp(db_frame_table,table_entry->entry[0]) == 0)
        {
            update_vaule_to_frame(table_entry->entry[2], strtod(table_entry->entry[4],&errord));
        }
    }
    return 0;
}

static int32_t readTableEntry(const char* table_name)
{
    //是否查询table参数
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(strcmp(db_arms_tables[i],table_name) == 0)
        {
            deinit_table_params(&tables_entry);
            read_param_db_table(DB_FILE,db_arms_tables[i],&tables_entry);
            //重新排序，按照arms_params_names重新排序
            int pI = 0;
            while (strlen(arms_params_names[pI].key)>0)
            {
                int pK = 0;
                //插入数据库
                for (int ti = 0; ti < tables_entry.rows; ti++)
                {
                    if(strcmp(arms_params_names[pI].key,tables_entry.table_value[ti].colValue[0]) == 0)
                    {
                        pK = ti;
                        break;
                    }
                }

                //拷贝指针
                for (int  k = 0; k < tables_entry.cols; k++)
                {
                    char * temp = tables_entry.table_value[pI].colValue[k];
                    tables_entry.table_value[pI].colValue[k] = tables_entry.table_value[pK].colValue[k];
                    tables_entry.table_value[pK].colValue[k] = temp;
                }
                pI++;
            }

            amrs_param_ack_arm_id = i;
            //定期的上传到upper，模拟定时器
            tables_entry_ack_index = tables_entry.rows;
            return 0;
        }
    }
    //是否查询frame参数
    if(strcmp(db_frame_table,table_name) == 0)
    {
        deinit_table_params(&tables_entry);
        read_param_db_table(DB_FILE,db_frame_table,&tables_entry);
        //定期的上传到upper，模拟定时器
        tables_entry_ack_index = tables_entry.rows;
        return 0;
    }
    //是否查询插值参数
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(strcmp(db_interpn_tables[i],table_name) == 0)
        {
            deinit_table_params(&tables_entry);
            read_param_db_table(DB_FILE,db_interpn_tables[i],&tables_entry);
            amrs_param_ack_arm_id = i;
            //定期的上传到upper，模拟定时器
            tables_entry_ack_index = tables_entry.rows;
            return 0;
        }
    }
}


/******************************************************************************
* 功能：supr线程陷入函数，周期性的控制leader等线程运行
* @return Descriptions
******************************************************************************/
static int32_t suprMainLoop(){
  //TODU
  char mData[Msg_T_S];
  suprWorking = 1;

  struct sockaddr_in  mPeerAddr;
  socklen_t mun = sizeof(mPeerAddr);

  static int recThreadCnt = 0;

  static uint32_t runCnt = 0;

  while(suprWorking)
  {
    if(recvfrom(mRecSoket, mData, Msg_T_S, 0, (struct sockaddr*)&(mPeerAddr), &mun)>0)
    {
      MsgHeader *mMsg = (MsgHeader *)mData;
      switch (mMsg->mMsgId)
      {
        case MSG_ID_MODULE_INIT1:
        {
          MsgShortMsg *mRegiMsg = (MsgShortMsg *)mMsg;
          //注册所有模块的消息队列
          printf("leader MSG_ID_MODULE_INIT1 %d\n",mRegiMsg->mNotice);
          mFrameId = (uint16_t)mRegiMsg->mNotice;
          initPhase1((int32_t*)mRegiMsg->mShortData);
          break;
        }
        case MSG_ID_MODULE_INIT2:
        {
          MsgShortMsg *mRegiMsg = (MsgShortMsg *)mMsg;
          //注册所有模块的消息队列
          printf("MSG_ID_MODULE_INIT2 \n");
          initPhase2();
          recThreadCnt = 0;
          break;
        }
        case MSG_ID_THREAD_START_ACK:
        {
          recThreadCnt++;
          if(recThreadCnt == MODULE_NUMS)
            sendNoticeToSupr(MSG_ID_MODULE_INIT2_ACK, 0);
          break;
        }
        case MSG_ID_FIRE_IN_THE_HOLE:
        {
          printf("leader MSG_ID_FIRE_IN_THE_HOLE\n");
          sendNoticeToThread(MSG_ID_FIRE_IN_THE_HOLE,0);
          break;
        }
        case CMD_IGH_Z_ZERO:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            handleSetZZero(&mArmsModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_IGH_Y_ZERO:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            handleSetYZero(&mArmsModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_IGH_XS_ZERO:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            handleSetXSZero(&mArmsModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_IGH_XM_ZERO:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            handleSetXMZero(&mArmsModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_CONF_R_DB_TABLES_NAME:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            armsReadDBTableName((SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_CONF_R_TABLE_ENTYR:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            readTableEntry(((SUPR_R_MSG *)mLMsg->mData)->mData);
            break;
        }
        case CMD_CONF_D_TABLE_ENTYR:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            deleteTableEntry((SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_CONF_UPDATE_TABLE_ENTYR:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mData;
            //更新数据库大致3ms左右
            updateTableEntry((SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case MSG_ID_REQUEST_ETHR_STOP:
        {
          //TUDO
          printf("MSG_ID_REQUEST_ETHR_STOP:%d\n",mArmsModule.mState);
          break;
        }
        case MSG_ID_MODULE_STOP_ALL:
        {
          //TUDO
          mArmsModule.mState  = M_STATE_STOP;
          mChassisModule.mState  = M_STATE_STOP;
          mSerialWorker.mState = SR_CLOSE;
          break;
        }
        case MSG_ID_THREAD_STOP_ALL_ACK:
        {
          sendNoticeToSupr(MSG_ID_MODULE_STOP_ALL_ACK, 0);
          break;
        }
        case MSG_ID_MODULE_EXIT_ALL:
        {
          recThreadCnt = 0;
          sendNoticeToThread(MSG_ID_MODULE_EXIT_ALL,0);
          break;
        }
        case MSG_ID_THREAD_REQUEST_EXIT_ALL_ACK:
        {
          recThreadCnt++;
          if(recThreadCnt == MODULE_NUMS)
          {
              sendNoticeToSupr(MSG_ID_MODULE_EXIT_ALL_ACK, 0);
              suprWorking = 0;
          }
          break;
        }  
        default:
        {
          break;
        }
      } //end switch
    }//end if

    //发送tables_name_list
    if(tables_name_list_ack_index>0)
    {
        int n = tables_name_list.str_cnt - tables_name_list_ack_index;
        strs_list *li = NULL,*tmp = NULL;
        int index = 0;
        list_for_each_entry_safe(li, tmp, &(tables_name_list.list), list)
        {   
            if(n == index)
            {
                //发送表名字到上位机
                sendMsgToUpper(ACK_NOTICE_DB_R_TABLES_NAME,-1,li->str,strlen(li->str)+1);
                break;
            }
            index++;
        }
        tables_name_list_ack_index--;
    }

    //定期上传arms表
    if(tables_entry_ack_index>=0)
    {
        int n = tables_entry.rows - tables_entry_ack_index;
        //发送表名字到上位机
        TABLE_ENTRY entry = {0};
        entry.rowIndex = n-1;
        entry.colCount = tables_entry.cols;
        if(n == 0)
        {
            //n==0 时候传输的是列名
            for (int i = 0; i < entry.colCount; i++)
            {
                strcpy(entry.entry[i],tables_entry.colName[i]);
            }
        }else
        {   
            //n！=0 时候传输的是第n行数据
            for (int i = 0; i < entry.colCount; i++)
            {
                strcpy(entry.entry[i],tables_entry.table_value[n-1].colValue[i]);
            }
        }
        sendMsgToUpper(ACK_NOTICE_DB_R_TABLES_ENTRY,-1,&entry,sizeof(TABLE_ENTRY));

        tables_entry_ack_index--;
    }

    //checkTimerWorking();
  }// end while

}

/******************************************************************************
* 功能：释放、销毁线程资源
* @return Descriptions
******************************************************************************/
static int32_t deInitSupr(void)
{
  int32_t iRet = 0;

  single_proc_inst_lockfile_cleanup();

  //删除创建的soket
  if(mRecSoket>0)
  {
      close(mRecSoket);
      mRecSoket = -1;
  }
  printf("*********** leader deInitSupr\n");
  deinit_table_params(&tables_entry);
  return iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void signalHandle(int mSignal)
{
  sendNoticeToMyself(MSG_ID_MODULE_EXIT_ALL,0);
  printf("oops! stop!!!\n");
}

/******************************************************************************
* 功能：系统入口函数，main函数调用
* @return Descriptions
******************************************************************************/
int32_t leaderAppStartUp(char** mSuprMsgK) {

  int32_t iRet = 0;
  //创建互斥文件锁，防止多个进程实例运行
  if(is_single_proc_inst_running("leader")<0)
  {
    printf("已经存在\n");
    return -1;
  }

  iRet = prepareEnv();
  if (0 == iRet) {
    printf("leader runing now! ctrl+c to endup\n");
    iRet = suprMainLoop();
  }
  //销毁串口线程
  mSerialWorker.mWorking = false;

  pthread_join(mArmsModule.mPid, NULL);
  pthread_join(mChassisModule.mPid, NULL);
  pthread_join(mSerialWorker.mPid, NULL);
  printf("quit timer module\n");


  deInitSupr();

  printf("quit sysArms all modules\n");
  return iRet;
}
