/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_loger.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:日志模块，系统运行的情况记录在这个模块中。
*
********************************************************************************/
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdarg.h>
#include <pthread.h>
#include <math.h>

#include "sys_master_chassis.h"
#include "sys_leader_defs.h"
#include "sys_master_magic.h"
#include "sys_master_vision.h"
#include "sys_ctl.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"
#include "sys_db.h"

#define CHASSIS_MAX_V_DEG   1.0
#define CHASSIS_MAX_V_XY_M  0.200

//------------四个舵轮的位置坐标-----------
float x_points[]={-2.815,-2.815, 2.815, 2.815};
float y_points[]={0.891, -2.024 ,0.891,-2.024};

/*Config PDOs 只能使用nimotion默认的PDO进行配置*/
static ec_pdo_entry_info_t ni_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* ControlWord */
    {0x6060, 0x00, 8}, /* ModeOperation */
    {0x0000, 0x00, 8}, /* Gap */
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
};

static ec_pdo_info_t ni_pdos[] = {
    {0x1600, 9, ni_pdo_entries + 0}, /* RxPDO0 */
    {0x1a00, 11, ni_pdo_entries + 9}, /* TxPDO0 */
};

static ec_sync_info_t nimotion_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, ni_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, ni_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

static int32_t sendDataToUpper(int32_t mSoket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, int32_t mDSize);
//发送给leader通知
static int32_t sendNoticeToLeader(int32_t mSocket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);
static int  createMaster(int mid,ec_master_t **pms,ec_domain_t **pdi);
static void  destroy_slaves_inf(EK1100_slaves_inf *slaves_inf_list);

/**************************************以下是底盘使用到的函数声明***************************************/
static int     initchassis(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int     destroyChassisMaster(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int     scanChassisSlaves(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int     confChassisSlavesPdos(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int     activeChassisMater(CHASSIS_MASTER_THREAD_INFO *pTModule);
static void    activeChassisMaterKeydown(CHASSIS_MASTER_THREAD_INFO *pTModule);

static void    chassisLoopRecLocalMsg(CHASSIS_MASTER_THREAD_INFO *pTModule);
static void    packChassis_S_MSG(CHASSIS_MASTER_THREAD_INFO *pTModule);
static void    packChassis_sensor_S_MSG(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int32_t chassisSetSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,float m1_w,float m1_v,float m2_w,float m2_v,float m3_w,float m3_v,float m4_w,float m4_v);
static int32_t chassisSetISpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float w_v_mm,float r_v_deg);
static int32_t chassisSetIRotatePos(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float r_deg);

static int32_t chassisIRotatePos(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float r_deg);

static void    cyc_rotate_pos_upper_cmd(CHASSIS_MASTER_THREAD_INFO *pTModule);
static void    filter_smooth(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int32_t chassisMove(CHASSIS_MASTER_THREAD_INFO *pTModule,float x_m,float y_m,float r_v_radian);

static int32_t chassisSetIWSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float w_v_mm);
static int32_t chassisSetIRSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float r_v);
static int32_t chassisSetUpSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,float w_v_mm);
static void    chassis_magic_run(CHASSIS_MASTER_THREAD_INFO *pTModule);
static void    chassis_magic_step1(CHASSIS_MASTER_THREAD_INFO *pTModule,float dd_omi_need, float dd_xo1_need,float dd_yo1_need,float p_d_omi);

static void    chassisMagicStateChange(CHASSIS_MASTER_THREAD_INFO *pTModule,MAGIC_RUN_STATE state);
static void    checkChassisWarning(CHASSIS_MASTER_THREAD_INFO *pTModule);
static int32_t setChassisProportional(CHASSIS_MASTER_THREAD_INFO *pTModule,uint32_t proId,float value);
static int32_t setChassisProportionals(CHASSIS_MASTER_THREAD_INFO *pTModule,proportional_value *values,int pvs);
static void    cyc_vel_upper_cmd(CHASSIS_MASTER_THREAD_INFO *pTModule);

static int  sign(float x);

static void getbest_base_par(float dd_xo1_need,float dd_yo1_need,float dd_omi_need,float sita_now[4],float d_sita_d_now[4],float v_d_now[4],
                            float d_sita_d_max,float dd_sita_d_max,float d_v_d_max,float v_d_max,float d_omi,float dt,
                            //output
                            float*dd_xo1_best,float*dd_yo1_best,float*dd_omi_best,float d_sita_d[4],float d_v_d[4]);

static float f_min(float x[3],float dd_xo1_need,float dd_yo1_need,float dd_omi_need);


static void mycon(
    float x1,
    float x2,
    float x3,
    float sita_now[4],
    float d_sita_d_now[4],
    float v_d_now[4],
    float d_sita_d_max,
    float dd_sita_d_max,
    float d_v_d_max,
    float v_d_max,
    float d_omi,
    float dt,
    
    float *constr,
    float *d_sita_d,
    float *d_v_d
);

static void mycon_1(
    float x1,
    float x2,
    float x3,
    CHASSIS_MASTER_THREAD_INFO *pTModule,
    float d_sita_d_max,
    float dd_sita_d_max,
    float d_v_d_max,
    float v_d_max,
    float d_omi,
    float dt,
    
    float *constr,
    float *d_sita_d,
    float *d_v_d
);

/**************************************************master bus***************************************************/
void* ethercatChassisThread(void* );

/***********************************************************************************************************/
static int32_t sendDataToUpper(int32_t mSoket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, int32_t mDSize)
{
  int iRet = 0;
  if(mSoket<0)
    return -1;
  iRet = MsgSendLongMsg(mSoket, get_port("LISTENER_LOCAL_PORT"), MN_ID_LEADER, MN_ID_SUPR, mMsgId, mNotice, mValue, mData, mDSize);
  return iRet;
}
static int32_t sendNoticeToLeader(int32_t mSocket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
  int iRet = 0;
  if(mSocket<0)
    return -1;
  iRet = MsgSendNotice(mSocket, get_mn_port(MN_ID_LEADER),MN_ID_LEADER, MN_ID_LEADER,mMsgId, mNotice, mValue);
  return iRet;
}

static int  createMaster(int mid,ec_master_t **pms,ec_domain_t **pdi)
{
    int iRet = -1;
    if(*pms == NULL)
    {
        *pms = ecrt_request_master(mid);
        if (*pms==NULL)
        {
            printf("#########:create master%d failed\n\r",mid);
            iRet = -1;
        }else{
            printf("#########:create master%d OK\n\r",mid);
            *pdi = ecrt_master_create_domain(*pms);
            printf("#########:create master%d domain1 OK\n\r",mid);
            iRet = 0;
        }
    }
    return iRet;
}
static int initchassis(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
  //创建soket.*非阻塞模式
  pTModule->mSoket = createSoket(get_mn_master_port(pTModule->mthreadMasterId), MN_LocalIpV4Str, 0, 0);

  pTModule->masterActivated=false;
  pTModule->mEtherWorker.mWorking = 1;
  strcpy(pTModule->mEtherWorker.mThreadName,"ethercatIgh");
  pTModule->mEtherWorker.pTModule = pTModule;
  pTModule->mEtherWorker.mPid = hiCreateThread("EtherIGH", ethercatChassisThread,PRI_MASTER, &pTModule->mEtherWorker);
  memset(pTModule->upper_target_v_Flag,0,sizeof(int32_t)*5);
  memset(pTModule->upper_target_p_Flag,0,sizeof(int32_t)*4);
  memset(pTModule->velocity_smooth,0,sizeof(int32_t)*NIMOTION_SIZE*SMOOTH_SIZE);
  memset(pTModule->tar_vel_cmd,0,sizeof(int32_t)*NIMOTION_SIZE);
}

static void  packChassis_S_MSG(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    int mSMsgSize = sizeof(ARMS_S_MSG)+sizeof(Chassis_inf);
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);

    //框架1，暂时设置
    mSMsg->mIdentifier = pTModule->mIdentifier;;
    mSMsg->mMsgType = ACK_NOTICE_SHOW_CHASSIS_INF;

    struct timespec  nowTime;
    clock_gettime(CLOCK_MONOTONIC, &nowTime);
    mSMsg->mSysTimeS = nowTime.tv_sec;
    mSMsg->mSysTimeUs = nowTime.tv_nsec/1000;

    int armsI = 0;
    Chassis_inf  *mChassis = (Chassis_inf  *)mSMsg->mData;
    for (int i = 0; i < NIMOTION_SIZE; i++)
    {
        //只拷贝伺服电机的机械臂
        mChassis->nimotion_inf[i].status_word = pTModule->ni_stword[i];
        if(i%2 == 0){
            mChassis->nimotion_inf[i].velocity_value = pTModule->w_speed[i/2];
            mChassis->nimotion_inf[i].Position_value = pTModule->w_pos[i/2];
        }
        else{
            mChassis->nimotion_inf[i].velocity_value = pTModule->r_speed[(i-1)/2];
            mChassis->nimotion_inf[i].Position_value = pTModule->r_pos[(i-1)/2];
        }
    }
    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg); 
}

static void   packChassis_sensor_S_MSG(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    int mSMsgSize = sizeof(ARMS_S_MSG)+sizeof(Chassis_sensor_inf);
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);

    //框架1，暂时设置
    mSMsg->mIdentifier = pTModule->mIdentifier;
    mSMsg->mMsgType = ACK_NOTICE_SHOW_CHASSIS_SENSOR_INF;

    struct timespec  nowTime;
    clock_gettime(CLOCK_MONOTONIC, &nowTime);
    mSMsg->mSysTimeS = nowTime.tv_sec;
    mSMsg->mSysTimeUs = nowTime.tv_nsec/1000;

    int armsI = 0;
    Chassis_sensor_inf  *mChsin = (Chassis_sensor_inf  *)mSMsg->mData;
    memcpy(mChsin,&pTModule->mSerialWorker->chserinf,sizeof(Chassis_sensor_inf));

    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg); 
}

static int32_t chassisSetISpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float w_v_mm,float r_v_deg)
{
    chassisSetIWSpeed(pTModule,mid,w_v_mm);
    chassisSetIRSpeed(pTModule,mid,r_v_deg);
}

static int32_t chassisSetIRotatePos(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float r_deg)
{
    //轴安装顺序 Y XM XS Z
    pTModule->upper_target_pos[mid] = r_deg; 
    pTModule->upper_target_p_Flag[mid] = 1;
}

static int32_t chassisIRotatePos(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float r_deg)
{
    //轴安装顺序 Y XM XS Z
    pTModule->upper_target_pos[mid] = r_deg; 
    pTModule->upper_target_p_Flag[mid] = 1;
}

static void  cyc_rotate_pos_upper_cmd(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    /**
     * 200HZ 运行周期
     */
    ARMS_MASTER_THREAD_INFO * pArms = pTModule->mArmsModule;
    for (int i = 0; i < 4; i++)
    {
        if(pTModule->upper_target_p_Flag[i])
        {
            //V_cmd = Kp*(S_obj-S_now) + Kd(^S_obj-^S_now)
            float cmdSpeed;
            float kp = 3,kd = 0.1;
            cmdSpeed=kp*(pTModule->upper_target_pos[i]-pTModule->r_pos[i])-kd*(pTModule->r_speed[i]);

            //限制速度
            if(cmdSpeed>30)
                cmdSpeed = 30;
            if(cmdSpeed<-30)
                cmdSpeed = -30;

            //判断是否都达到目标速度
            if(fabs(pTModule->r_pos[i]-pTModule->upper_target_pos[i]) < 0.1)
                cmdSpeed = 0;
            chassisSetIRSpeed(pTModule,i,cmdSpeed);
            //到达位置
            if(fabs(pTModule->r_pos[i]-pTModule->upper_target_pos[i]) < 0.1)
            {
                pTModule->upper_target_p_Flag[i] = 0;
            }
        }
    }
}

static void  filter_smooth(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    //指令滤波
    for (int  i = 0; i < 8; i++)
    {
        for (int k = 0; k < SMOOTH_SIZE-1; k++)
        {
            pTModule->velocity_smooth[i][k] = pTModule->velocity_smooth[i][k+1];
        } 
        pTModule->velocity_smooth[i][SMOOTH_SIZE-1] = pTModule->tar_vel_cmd[i];

        int32_t smoothV = 0;
        for (int k = 0; k < SMOOTH_SIZE; k++)
        {
            smoothV += pTModule->velocity_smooth[i][k];
        } 
        pTModule->motors[i].target_velocity = (int32_t)(smoothV/SMOOTH_SIZE);
    }
}

static int32_t chassisMove(CHASSIS_MASTER_THREAD_INFO *pTModule,float x_v_m,float y_v_m,float r_v_radian)
{
    //左前，左后，右前，右后四个舵轮的座标
    for (int i = 0; i < 4; i++)
    {
        float cmd_radian,cmd_work_v;
        float vX = -r_v_radian*y_points[i] + x_v_m;
        float vY = r_v_radian*x_points[i] + y_v_m;
        cmd_radian = atan2(vY,vX);
        cmd_work_v = sqrt(vX*vX+vY*vY);  

        chassisSetIRotatePos(pTModule,i,cmd_radian*180.0/3.14159);
        chassisSetIWSpeed(pTModule,i,cmd_work_v*1000.0);
    }
}

static int32_t chassisSetIWSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float w_v_mm)
{
    if(mid>=0 && mid<=3)
    {
        //限速启用,mm/s ,参数列表是mm/s : w*r + v0
        float maxSpeed = 600;//3400*(CHASSIS_MAX_V_DEG/57.0) + CHASSIS_MAX_V_XY_M*1000.0;
		if(w_v_mm > maxSpeed)
			w_v_mm = maxSpeed;
		if(w_v_mm < -maxSpeed)
			w_v_mm = -maxSpeed;
        uint16_t index = mid*2;
        pTModule->tar_vel_cmd[index] = (int32_t)(w_v_mm/W_VEL_MM_TO_USER)*CHASSIS_DIR[index];
    }
}

static int32_t chassisSetIRSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,int mid,float r_v)
{
    if(mid>=0 && mid<=3)
    {
        uint16_t index = mid*2+1;
        float maxSpeed = 30.0;//deg
        LIMIT_MAX(r_v,maxSpeed);
        LIMIT_MIN(r_v,-maxSpeed);
        pTModule->tar_vel_cmd[index] = (int32_t)(r_v/R_VEL_DEG_TO_USER)*CHASSIS_DIR[index];
    }
}

static int32_t chassisSetUpSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,float w_v_mm)
{
    if(w_v_mm > 20)
        w_v_mm = 20;
    if(w_v_mm < -20)
        w_v_mm = -20;
    pTModule->motors[8].target_velocity = w_v_mm*10000/60;
}

static int32_t chassisSetSpeed(CHASSIS_MASTER_THREAD_INFO *pTModule,float m1_w,float m1_v,float m2_w,float m2_v,float m3_w,float m3_v,float m4_w,float m4_v)
{
    /*
    电机轴转速 rpm=驱动轴转速 × 传动比6091h x 60 /编码器分辨率
    */
    //根据实际结构，真实的转速和行走速度 转换成电机单位。
    /*
    pTModule->motors[0].target_velocity = m1_w*W_VEL_TO_MM;
    pTModule->motors[1].target_velocity = m1_v*10000/60;
    pTModule->motors[2].target_velocity = m2_w*W_VEL_TO_MM;
    pTModule->motors[3].target_velocity = m2_v*10000/60;
    pTModule->motors[4].target_velocity = m3_w*W_VEL_TO_MM;
    pTModule->motors[5].target_velocity = m3_v*10000/60;
    pTModule->motors[6].target_velocity = m4_w*W_VEL_TO_MM;
    pTModule->motors[7].target_velocity = m4_v*10000/60;
    */
}

static int32_t setChassisProportional(CHASSIS_MASTER_THREAD_INFO *pTModule,uint32_t proId,float value)
{
    if (proId>=0 && proId <=12)
    {
        pTModule->mSerialWorker->cmd_proportion[proId] = value;
    }    
}

static int32_t setChassisProportionals(CHASSIS_MASTER_THREAD_INFO *pTModule,proportional_value *values,int pvs)
{
    for (int i = 0; i < pvs; i++)
    {
        setChassisProportional(pTModule,values[i].pvid,values[i].value);
    }
}

static int32_t handleStartSlight(CHASSIS_MASTER_THREAD_INFO *pTModule,int wheelId,float w_vmm,float r_vdeg)
{
    pTModule->upper_target_v_Flag[wheelId] = 0;
    //轴安装顺序 Y XM XS Z
    pTModule->upper_target_velocity[wheelId][0] = w_vmm; 
    pTModule->upper_target_velocity[wheelId][1] = r_vdeg;

    //xm xs y z
    pTModule->upper_velocity_v0[wheelId][0] = pTModule->w_speed[wheelId];
    pTModule->upper_velocity_v0[wheelId][1] = pTModule->r_speed[wheelId];

    //200hz 调用周期，加减速1s
    for (int i = 0; i < 2; i++)
    {
        //行走速度
        if(pTModule->upper_target_velocity[wheelId][i] > pTModule->upper_velocity_v0[wheelId][i])
            pTModule->upper_acc[wheelId][i] = 80;
        else if(pTModule->upper_target_velocity[wheelId][i] < pTModule->upper_velocity_v0[wheelId][i])
            pTModule->upper_acc[wheelId][i] = -80;
        else
            pTModule->upper_acc[wheelId][i] = 0;

        pTModule->upper_last_v_cmd[wheelId][i] = pTModule->upper_velocity_v0[wheelId][i];
    }
    pTModule->upper_target_v_Flag[wheelId] = 1;
}

static void cyc_vel_upper_cmd(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    /**
     * 200HZ 运行周期
     */
    for (int i = 0; i < 4; i++)
    {
        if(pTModule->upper_target_v_Flag[i])
        {
            //V_cmd = V_now + a*dt
            float cmdSpeed[2];
            int cmd_target_ok = 0;
            for (int j = 0; j < 2; j++)
            {
                cmdSpeed[j] = pTModule->upper_last_v_cmd[i][j] + pTModule->upper_acc[i][j]*0.005;
                if(pTModule->upper_acc[i][j]>0 && cmdSpeed[j]>=pTModule->upper_target_velocity[i][j])
                {
                    cmdSpeed[j]=pTModule->upper_target_velocity[i][j];
                    pTModule->upper_acc[i][j] = 0;
                    cmd_target_ok++;
                }else if(pTModule->upper_acc[i][j]<0 && cmdSpeed[j]<=pTModule->upper_target_velocity[i][j])
                {
                    cmdSpeed[j]=pTModule->upper_target_velocity[i][j];
                    pTModule->upper_acc[i][j] = 0;
                    cmd_target_ok++;
                }else if(fabs(pTModule->upper_acc[i][j]) < 1e-7)
                {
                    cmd_target_ok++;
                }
            }

            //判断是否都达到目标速度
            if(
                fabs(cmdSpeed[0]-pTModule->upper_target_velocity[i][0]) < 1 &&
                fabs(cmdSpeed[1]-pTModule->upper_target_velocity[i][1]) < 1 &&
                cmd_target_ok >= 2
            )
            {
                pTModule->upper_target_v_Flag[i] = 0;
                cmdSpeed[0] = pTModule->upper_target_velocity[i][0];
                cmdSpeed[1] = pTModule->upper_target_velocity[i][1];
            }

            for (int j = 0; j < 2; j++)
            {
                pTModule->upper_last_v_cmd[i][j] = cmdSpeed[j];
            }
            chassisSetIWSpeed(pTModule,i,cmdSpeed[0]);
            chassisSetIRSpeed(pTModule,i,cmdSpeed[1]);
        }
    }
}

static int sign(float x)
{
    if(x>0)
    {
        return 1;
    }
    else if(x<0)
    {
        return -1;
    }else{
        return 0;
    }
}
static void chassis_magic_step1(CHASSIS_MASTER_THREAD_INFO *pTModule,float dd_omi_need, float dd_xo1_need,float dd_yo1_need,float p_d_omi)
{
    /*
    %此程序用于分析具有4个舵轮的框架，考虑的限制包括：舵轮最大旋转角速度、
    %舵轮前进速度的最大加速度、舵轮最大前进速度
    %程序输入的参数：框架的旋转角速度/角加速度，框架的水平速度/加速度、舵轮的坐标dx、dy

    %2025年3月15日：这个程序目前还有问题。
    %2025年4月6日：修改计算策略，将框架的角加速度、水平加速度这3个参数统一考虑
    %，然后再分解到每个舵轮。
    %本程序是做三维搜索，其目标函数为k1*x加速度误差^2+k2*y加速度误差^2+k3*角加速度误差^2。
    %可以用优化算法，也可以用5*5*5=125次计算来寻找最优值。
    */
    float dt=0.02;
    float pi=3.1415926;

    //%---------------------约束值---------------------
    float d_sita_d_max=5*pi/180;     //舵轮最大旋转角速度
    float dd_sita_d_max=3000*pi/180;    //舵轮最大旋转角加速度
    float d_v_d_max=10;         //舵轮前进速度的最大加速度
    float v_d_max=0.02;                               //舵轮最大前进速度

    ////---------------当前状态值--------------------    
    float d_omi=p_d_omi;        //框架的当前角速度，大程序中用dd_omi积分得到

    //------------四个舵轮的位置坐标-----------

    for(int i=0; i<4 ;i++)
    {
        if(fabs(pTModule->v_d_now[i])<1e-7)
            pTModule->v_d_now[i]=0.0001*sign(pTModule->v_d_now[i]);
    }
   
    //------------下面求合理的框架水平加速度、角加速度，还有舵轮的角速度和加速度---------------
    //%------------下面求合理的框架水平加速度、角加速度，还有舵轮的角速度和加速度---------------
    float dd_xo1_best,dd_yo1_best,dd_omi_best;
    float d_sita_d[4],d_v_d[4];

    float x00[3]={dd_xo1_need,dd_yo1_need,dd_omi_need};
    float x0[3] ={dd_xo1_need,dd_yo1_need,dd_omi_need};

    float x_best[3];
    float constr,b[4],b1[4];

    float x1[3];
    float v_min;

    #if 0
    for(int i=0;i<=10;i++)
    {
        x0[0]=x00[0]-x00[0]/10*i;
        x0[1]=x00[1]-x00[1]/10*i;
        x0[2]=x00[2]-x00[2]/10*i;

        mycon_1(x0[0],x0[1],x0[2],pTModule->sita_base_now,pTModule->d_sita_d_base_now,pTModule->v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt,&constr,b,b1);
        if (constr<1e-7)
        {
            //printf("constr<1e-7: %d \n",i);
            break;     //%遇到第一个满足条件后，停止计算
        }
    }

    //%计算f(x0)
    v_min=f_min(x0,dd_xo1_need,dd_yo1_need,dd_omi_need);
    //%记录x0对应的目标函数值，后面进行最小值更新。
    memcpy(x_best,x0,sizeof(float)*3);
    float scal1,scal2,scal3;
    //%下面开始在初始值附件进行搜索，寻找更优的数据
    if (fabs(x_best[0])<1e-7 && fabs(x_best[1])<1e-7 && fabs(x_best[2])<1e-7)
    {
        scal1=0.001;
        scal2=0.001;
        scal3=0.001*pi/180;
    }
    else
    {
        scal1=x_best[0]*0.5;
        scal2=x_best[1]*0.5;
        scal3=x_best[2]*0.5;
    }

    for(int i=1; i<=5; i++)
    {
        for(int j=1; j<=5; j++)
        {
            for(int k=1; k<=5; k++)
            {
                x1[0] = x0[0] + scal1*((float)i)-scal1*3.0;
                x1[1] = x0[1] + scal2*((float)j)-scal2*3.0;
                x1[2] = x0[2] + scal3*((float)k)-scal3*3.0;

                if (fabs(dd_omi_need) < 1e-7)
                {
                    x1[2] = x0[2];
                }

                float v=f_min(x1,dd_xo1_need,dd_yo1_need,dd_omi_need);

                //[constr,b,b1]=mycon(x1,a2);
                mycon(x1[0],x1[1],x1[2],pTModule->sita_base_now,pTModule->d_sita_d_base_now,pTModule->v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt,&constr,b,b1);

                if ((constr<1e-6) && (v<=v_min))
                {
                    v_min=v;
                    memcpy(x_best,x1,sizeof(float)*3);
                }
            }
        }
    }

    //%至此得到最优的框架加速度参数x_best,在大程序中，这几个参数要提供给吊杆加速度计算的程序
    dd_xo1_best=x_best[0];              //%框架的x向加速度
    dd_yo1_best=x_best[1];              //%框架的y向加速度
    dd_omi_best=x_best[2];              //%框架的旋转角加速度

    //%函数mycon计算出各个舵轮的角速度和前进加速度
    //%注意这里的d_sita_d、d_v_d是相对于惯性系的，不是相对于框架的。
    //[constr,d_sita_d,d_v_d]=mycon(x_best,a2);
    #endif
    //dd_omi_need    //框架的旋转角加速度，大程序中用PID产生该数据;
    //dd_xo1_need           //框架的x向加速度，大程序中用PID产生该数据
    //dd_yo1_need              //框架的y向加速度，大程序中用PID产生该数据

    x_best[0] = dd_xo1_need;              //%框架的x向加速度
    x_best[1] = dd_yo1_need;              //%框架的y向加速度
    x_best[2] = dd_omi_need;              //%框架的旋转角加速度

    mycon_1(x_best[0],x_best[1],x_best[2],pTModule,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt,&constr,d_sita_d,d_v_d);

    //printf("最优解为：x = [%f, %f, %f]  [sita_base_now,d_v_d][%f %f] %f\n", x_best[0], x_best[1],x_best[2]*180/3.14159,pTModule->sita_base_now[0],d_v_d[0],pTModule->v_d_now[0]);
    //printf("期望值：x = [%f, %f, %f] \n",dd_xo1_need,dd_yo1_need,dd_omi_need);

    dd_omi_best = 0;
    d_omi=d_omi+dd_omi_best*dt;

    for (int i = 0; i < 4; i++)
    {
        LIMIT_MAX(d_v_d[i],d_v_d_max);
        LIMIT_MIN(d_v_d[i],-d_v_d_max);
        pTModule->v_d_now[i]=pTModule->v_d_now[i] + d_v_d[i]*dt;
        LIMIT_MAX(pTModule->v_d_now[i],v_d_max);
        LIMIT_MIN(pTModule->v_d_now[i],-v_d_max);

        pTModule->d_sita_d_base_now[i]=d_sita_d[i]-d_omi;                     //  d_sita_d_now 为舵轮相对于框架的角速度，d_sita_d为舵轮相对于惯性系的角速度
        LIMIT_MAX(pTModule->d_sita_d_base_now[i],d_sita_d_max);
        LIMIT_MIN(pTModule->d_sita_d_base_now[i],-d_sita_d_max);
        //暂时不用角度控制，使用d_sita_d_base_now 角速度控制
        pTModule->sita_base_now[i] = pTModule->sita_base_now[i] + pTModule->d_sita_d_base_now[i]*dt;  //sita_base_now为舵轮相对于框架的角度命令值
    }

    //基座真实的加速度值.仅仅在平动时候有效
    pTModule->dd_x01_real = d_v_d[0]*cos(pTModule->sita_base_now[0]) - pTModule->v_d_now[0]*sin(pTModule->sita_base_now[0])*(pTModule->d_sita_d_base_now[0]+d_omi);
    pTModule->dd_y01_real = d_v_d[0]*sin(pTModule->sita_base_now[0]) + pTModule->v_d_now[0]*cos(pTModule->sita_base_now[0])*(pTModule->d_sita_d_base_now[0]+d_omi);

    //printf("\n");
    //下面这个程序用以检验舵轮的速度、角度是否协调
}

static float f_min(float x[3],float dd_xo1_need,float dd_yo1_need,float dd_omi_need)
{
    float iRet = 0;
    iRet = 3*(x[0]-dd_xo1_need)*(x[0]-dd_xo1_need) + (x[1]-dd_yo1_need)*(x[1]-dd_yo1_need) + 10*(x[2]-dd_omi_need)*(x[2]-dd_omi_need);
    return iRet;
}

static void getbest_base_par(     
                            float dd_xo1_need,
                            float dd_yo1_need,
                            float dd_omi_need,
                            float sita_now[4],
                            float d_sita_d_now[4],
                            float v_d_now[4],
                            float d_sita_d_max,
                            float dd_sita_d_max,
                            float d_v_d_max,
                            float v_d_max,
                            float d_omi,
                            float dt,
                            //output
                            float*dd_xo1_best,
                            float*dd_yo1_best,
                            float*dd_omi_best,
                            float d_sita_d[4],
                            float d_v_d[4]
                                )
{
   #if 0
    //function [dd_xo1_best,dd_yo1_best,dd_omi_best,d_sita_d,d_v_d]=getbest_base_par(dd_xo1_need,dd_yo1_need,dd_omi_need,sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt)
    //%------------下面求合理的框架水平加速度和角加速度---------------

    //%准备参数a2，依次为4个舵轮的角度sita_now，4个舵轮的前进速度v_d_last，
    //%舵轮最大旋转角速度d_sita_d_max、舵轮最大前进加速度d_v_d_max，
    //%舵轮最大前进速度v_d_max，框架旋转角速度d_omi
    //a2=[sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt];

    //%定义目标函数，约束函数为mycon
    //f = @(x) 3*(x(1)-dd_xo1_need)^2 + (x(2)-dd_yo1_need)^2+10* (x(3)-dd_omi_need)^2;

    printf("*** %f %f %f\n",sita_now[0],d_sita_d_now[0],v_d_now[0]);
    //% 初始猜测解x00(将PID计算出来的框架水平加速度和角加速度作为初始值)
    float x00[3];
    float v_min;
    float x0[3]={0};
    float scal1,scal2,scal3;
    float x_best[3];
    float constr,b[4],b1[4];
    float pi = 3.1415926;
    float x1[3];

    x00[0] = dd_xo1_need;
    x00[1] = dd_yo1_need;
    x00[2] = dd_omi_need;
    //%计算x0是否满足约束条件

    memcpy(x0,x00,sizeof(float)*3);
    //%[constr,b,b1]=mycon(x0,a2);
    //%找到一个满足约束条件的初始解，第一个试探的是x00，最后一个是[0，0，0],总有一个是满足约束的

    for(int i=0;i<=10;i++)
    {
        x0[0]=x00[0]-x00[0]/10*i;
        x0[1]=x00[1]-x00[1]/10*i;
        x0[2]=x00[2]-x00[2]/10*i;

        //[constr,b,b1]=mycon(x0,a2);
        mycon(x0[0],x0[1],x0[2],sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt,&constr,b,b1);

        if (constr<1e-7)
        {
            printf("constr<1e-7: %d \n",i);
            break;     //%遇到第一个满足条件后，停止计算
        }
    }

    //%计算f(x0)
    v_min=f_min(x0,dd_xo1_need,dd_yo1_need,dd_omi_need);
    //%记录x0对应的目标函数值，后面进行最小值更新。
    memcpy(x_best,x0,sizeof(float)*3);
    //%下面开始在初始值附件进行搜索，寻找更优的数据
    if (fabs(x_best[0])<1e-7 && fabs(x_best[1])<1e-7 && fabs(x_best[2])<1e-7)
    {
        scal1=0.001;
        scal2=0.001;
        scal3=0.001*pi/180;
    }
    else
    {
        scal1=x_best[0]*0.5;
        scal2=x_best[1]*0.5;
        scal3=x_best[2]*0.5;
    }

    for(int i=1; i<=5; i++)
    {
        for(int j=1; j<=5; j++)
        {
            for(int k=1; k<=5; k++)
            {
                x1[0] = x0[0] + scal1*((float)i)-scal1*3.0;
                x1[1] = x0[1] + scal2*((float)j)-scal2*3.0;
                x1[2] = x0[2] + scal3*((float)k)-scal3*3.0;

                if (fabs(dd_omi_need) < 1e-7)
                {
                    x1[2] = x0[2];
                }
                
                float v=f_min(x1,dd_xo1_need,dd_yo1_need,dd_omi_need);

                //[constr,b,b1]=mycon(x1,a2);
                mycon(x1[0],x1[1],x1[2],sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt,&constr,b,b1);

                if ((constr<1e-6) && (v<=v_min))
                {
                    v_min=v;
                    memcpy(x_best,x1,sizeof(float)*3);
                }
            }
        }
    }

    //%至此得到最优的框架加速度参数x_best,在大程序中，这几个参数要提供给吊杆加速度计算的程序
    *dd_xo1_best=x_best[0];              //%框架的x向加速度
    *dd_yo1_best=x_best[1];              //%框架的y向加速度
    *dd_omi_best=x_best[2];              //%框架的旋转角加速度

    //%函数mycon计算出各个舵轮的角速度和前进加速度
    //%注意这里的d_sita_d、d_v_d是相对于惯性系的，不是相对于框架的。
    //[constr,d_sita_d,d_v_d]=mycon(x_best,a2);
    mycon(x_best[0],x_best[1],x_best[2],sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt,&constr,d_sita_d,d_v_d);
    printf("最优解为：x = [%f, %f, %f]\n", x_best[0], x_best[1],x_best[2]*180/3.14159);
    printf("目标函数值为：%f\n", v_min);
    printf("期望为：x = [%f, %f, %f]\n",  dd_xo1_need,  dd_yo1_need, dd_omi_need*180/3.14159);
#endif
}

static void mycon(
                float x1,
                float x2,
                float x3,
                float sita_now[4],
                float d_sita_d_now[4],
                float v_d_now[4],
                float d_sita_d_max,
                float dd_sita_d_max,
                float d_v_d_max,
                float v_d_max,
                float d_omi,
                float dt,
                
                float *constr,
                float *d_sita_d,
                float *d_v_d
            )
{
    //% 功能1：计算非线性约束constr
    //% 功能2：根据输入的框架参数，计算4个舵轮的旋转角速度和前进加速度。
    //function [constr,d_sita_d,d_v_d]=mycon(x,a2)
    //%x 按照次序为x加速度 y加速度和 角加速度
    //%  a2=[sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt];
    float dd_xo1=x1;
    float dd_yo1=x2;
    float dd_omi=x3;

    //------------四个舵轮的位置坐标-----------
    *constr=0;

    for(int i=0;i<4;i++)
    {
        float v_d=v_d_now[i];
        if (fabs(v_d)<0.001)
        {
            v_d=sign(v_d)*0.001;
        }
        float sita=sita_now[i];

        //%------------第一步计算各舵轮的xy轴加速度
        float  ax=-dd_omi*y_points[i]+dd_xo1-(d_omi)*(d_omi)*x_points[i]*1;
        float  ay= dd_omi*x_points[i]+dd_yo1-(d_omi)*(d_omi)*y_points[i]*1;
        //printf("mycon:%f %f %f %f %f %f \n",ay, dd_omi,dx[i],dd_yo1,d_omi,dy[i]);
        //%------------第二步计算舵轮的旋转速度和前进加速度，_d表示为舵轮
        d_sita_d[i]=(ay*cos(sita)-ax*sin(sita))/v_d;
        d_v_d[i]=ax*cos(sita)+ay*sin(sita);

        //%------------第三步对舵轮的旋转速度和前进加速度进行限制
        if (fabs(d_sita_d[i]) > d_sita_d_max)           //%对舵轮角速度进行限制
        {
            *constr= *constr+fabs(d_sita_d[i])- d_sita_d_max;
        }
        if (fabs(d_sita_d[i]-d_sita_d_now[i]) > ((dd_sita_d_max)*(dt)) )           //%对舵轮角加速度进行限制
        {
            *constr= *constr+fabs(d_sita_d[i]-d_sita_d_now[i])- (dd_sita_d_max)*(dt);
        }
        if (fabs(d_v_d[i]) > d_v_d_max)                  //%对舵轮前进轮的最大加速度进行限制
        {
            *constr= *constr+fabs(d_v_d[i])- d_v_d_max;
        }
    }
}

static void mycon_1(
    float x1,
    float x2,
    float x3,
    CHASSIS_MASTER_THREAD_INFO *pTModule,
    float d_sita_d_max,
    float dd_sita_d_max,
    float d_v_d_max,
    float v_d_max,
    float d_omi,
    float dt,
    
    float *constr,
    float *d_sita_d,
    float *d_v_d
)
{
    //% 功能1：计算非线性约束constr
    //% 功能2：根据输入的框架参数，计算4个舵轮的旋转角速度和前进加速度。
    //function [constr,d_sita_d,d_v_d]=mycon(x,a2)
    //%x 按照次序为x加速度 y加速度和 角加速度
    //%  a2=[sita_now,d_sita_d_now,v_d_now,d_sita_d_max,dd_sita_d_max,d_v_d_max,v_d_max,d_omi,dt];
    float dd_xo1=x1;
    float dd_yo1=x2;
    float dd_omi=x3;

    //------------四个舵轮的位置坐标-----------

    *constr=0;

    for(int i=0;i<4;i++)
    {
        float v_d= pTModule->v_d_now[i];
        if (fabs(v_d)<0.001)
        {
            v_d=sign(v_d)*0.001;
        }
        float sita= pTModule->sita_base_now[i];

        //%------------第一步计算各舵轮的xy轴加速度
        float  ax=-dd_omi*y_points[i]+dd_xo1-(d_omi)*(d_omi)*x_points[i]*1;
        float  ay= dd_omi*x_points[i]+dd_yo1-(d_omi)*(d_omi)*y_points[i]*1;

        //printf("mycon:%f %f %f %f %f %f \n",ay, dd_omi,dx[i],dd_yo1,d_omi,dy[i]);
        //%------------第二步计算舵轮的旋转速度和前进加速度，_d表示为舵轮
        d_sita_d[i]=(ay*cos(sita)-ax*sin(sita))/v_d;
        d_v_d[i]=ax*cos(sita)+ay*sin(sita);

        //暂时测试Y向不动
        //d_sita_d[i]= 0;

        //%------------第三步对舵轮的旋转速度和前进加速度进行限制
        if (fabs(d_sita_d[i]) > d_sita_d_max)           //%对舵轮角速度进行限制
        {
            *constr= *constr+fabs(d_sita_d[i])- d_sita_d_max;
        }
        if (fabs(d_sita_d[i]-pTModule->d_sita_d_base_now[i]) > ((dd_sita_d_max)*(dt)) )           //%对舵轮角加速度进行限制
        {
            *constr= *constr+fabs(d_sita_d[i]-pTModule->d_sita_d_base_now[i])- (dd_sita_d_max)*(dt);
        }
        if (fabs(d_v_d[i]) > d_v_d_max)                  //%对舵轮前进轮的最大加速度进行限制
        {
            *constr= *constr+fabs(d_v_d[i]) - d_v_d_max;
        }
       
        //printf("mycon_1_%d:%f %f %f %f %f %f %f %f\n",i,d_sita_d[i],d_v_d[i],sita,ax,ay,dd_xo1,dd_yo1,dd_omi);
    }

}

static void chassis_magic_run(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    ARMS_MAGIC_CTRL_V *pctrl = &pTModule->mArmsMagicCtrl;
    ARMS_MASTER_THREAD_INFO *PArms = pTModule->mArmsModule;
    if((pTModule->mRunState == R_FOLLOW || pTModule->mRunState == R_FOLLOW_VERTICAL) && pTModule->enable_flag == 1)
    {
        //%设置控制参数（基座角度）
        float ebslon=0.8;
        float wn=0.8;
        float KP_fai=wn*wn;
        float KD_fai=2*wn*ebslon;

        //缓冲圈,理想点圆的半径
        float ideal_r1 = 0.01;
        float ideal_r2 = 0.02;
        
        //%设置控制参数（基座平动）
        ebslon=0.5;
        wn=0.1;
        float  KP_b=wn*wn;
        float  KD_b=2*wn*ebslon;

        float dd_fai_max = 0.02;  //角加速度最大限制
        float ddd_fai_max=0.04;   //角加加速度值

        float fai_R_2=pTModule->fai_R_2;  //%机械臂与基座的夹角，本项目中固定不变，当前是90度
        float fai_R_1=pTModule->fai_R_1;   //%基座在地面惯性坐标系中的角度

        float L_R_ideal= pTModule->L_R_ideal;   //%第二个卸载装置距离中心点的理想距离
        float D_R_ideal= pTModule->D_R_ideal;   //%第二个卸载装置机械臂伸出的理想距离

        float d_fai_R1= pTModule->d_fai_R1;   //% 基座旋转速度初值设置
        
        float d_xo1_R=0;     // %基座x向速度初值设置
        float d_yo1_R=0;    // %基座y向速度初值设置

        //吊杆的X向位置和速度
        int mid = pTModule->ArmsReferId;
        float L_R = PArms->xMIntPos[mid];
        float d_L_R = PArms->xMIntSpeed[mid]/1000.0;
        float D_R = PArms->yPos[mid];
        float d_D_R = PArms->ySpeed[mid]/1000.0;

        float alfa_R=0;   //%基座与理想基座位置的差异。先设为0，也就是使得基座先不旋转
        float d_alfa_R=0;

        //判断理想点圆的坐标
        float now_ideal_r = sqrt((L_R_ideal-L_R)*(L_R_ideal-L_R) + (D_R_ideal-D_R)*(D_R_ideal-D_R));

        //%计算基座控制命令
        float dd_fai_R1= KP_fai*alfa_R+KD_fai*d_alfa_R;           // %计算基座在本体惯性坐标系中角加速度命令
        
        float dd_xo1_R = -(KP_b*(L_R_ideal-L_R)-KD_b*d_L_R);     //%计算基座在本体惯性惯性坐标系中x向加速度命令
        float dd_yo1_R = -(KP_b*(D_R_ideal-D_R)*sin(fai_R_2)-KD_b*d_D_R*sin(fai_R_2) ); //%基座在本体惯性坐标系中y向加速度命令
        
/*************************************************************跟踪的目标点在大圆 或者 从大圆进入缓冲区需要控制************************************************************/
        if(
            (now_ideal_r>ideal_r2) || 
            (now_ideal_r>ideal_r1 && now_ideal_r<ideal_r2 && pTModule->ideal_r_signal_flag) 
        )  //启用控制算法
        {
            //启用控制标志
            pTModule->ideal_r_signal_flag = true;
            chassis_magic_step1(pTModule,0,dd_xo1_R,dd_yo1_R,0);
        }else
        {   
            /*
            pTModule->ideal_r_signal_flag = false;
            //设置舵轮角度
            float sita=atan2(dd_yo1_R,dd_xo1_R);                       //舵轮的角度

            pTModule->sita_base_now[0]=sita;
            pTModule->sita_base_now[1]=sita;
            pTModule->sita_base_now[2]=sita;  
            pTModule->sita_base_now[3]=sita;

            memset(pTModule->v_d_now,0,sizeof(float)*4);
            */
        }

        /*
        printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", pTModule->sita_base_now[0]*57.29, pTModule->sita_base_now[1]*57.29, pTModule->sita_base_now[2]*57.29, pTModule->sita_base_now[3]*57.29,
            pTModule->v_d_now[0]*1000,pTModule->v_d_now[1]*1000,pTModule->v_d_now[2]*1000,pTModule->v_d_now[3]*1000,
            dd_xo1_R,dd_yo1_R,L_R_ideal,L_R,D_R_ideal,D_R);
        */
        chassisSetIRSpeed(pTModule,0, pTModule->d_sita_d_base_now[0]*57.29);
        chassisSetIRSpeed(pTModule,1, pTModule->d_sita_d_base_now[0]*57.29);
        chassisSetIRSpeed(pTModule,2, pTModule->d_sita_d_base_now[0]*57.29);
        chassisSetIRSpeed(pTModule,3, pTModule->d_sita_d_base_now[0]*57.29);

        chassisSetIWSpeed(pTModule,0,pTModule->v_d_now[0]*1000);
        chassisSetIWSpeed(pTModule,1,pTModule->v_d_now[0]*1000);
        chassisSetIWSpeed(pTModule,2,pTModule->v_d_now[0]*1000);
        chassisSetIWSpeed(pTModule,3,pTModule->v_d_now[0]*1000);

        checkChassisWarning(pTModule);
    }else
    {
        pTModule->dd_x01_real = 0;
        pTModule->dd_y01_real = 0;
    }
}

static void  checkChassisWarning(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    if((pTModule->mRunState == R_FOLLOW || pTModule->mRunState == R_FOLLOW_VERTICAL) && pTModule->enable_flag == 1)
    {
        /*************************************************判断舵轮是否拉扯**************************************************/
        //左前和左后Y方向速度、右前和右后Y向速度、左前和右前速度X向速度、左后和右后X向速度
        float max_pull_v = 0.002;
        float left_front_xv = (pTModule->w_speed[0]/1000.0)*sin(pTModule->r_pos[0]/57.29);
        float left_front_yv = (pTModule->w_speed[0]/1000.0)*cos(pTModule->r_pos[0]/57.29);
        float left_back_xv = (pTModule->w_speed[1]/1000.0)*sin(pTModule->r_pos[1]/57.29);
        float left_back_yv = (pTModule->w_speed[1]/1000.0)*cos(pTModule->r_pos[1]/57.29);
        float right_front_xv = (pTModule->w_speed[2]/1000.0)*sin(pTModule->r_pos[2]/57.29);
        float right_front_yv = (pTModule->w_speed[2]/1000.0)*cos(pTModule->r_pos[2]/57.29);
        float right_back_xv = (pTModule->w_speed[3]/1000.0)*sin(pTModule->r_pos[3]/57.29);
        float right_back_yv = (pTModule->w_speed[3]/1000.0)*cos(pTModule->r_pos[3]/57.29);

        if( fabs(left_front_xv - right_front_xv) > max_pull_v)
        {
            printf("********左前 和 右前 X向拉扯:%f\n",fabs(left_front_xv - right_front_xv));
            //sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_ALL_STOP,0,0,0,0);
        }
        if( fabs(left_back_xv - right_back_xv) > max_pull_v)
        {
            printf("********左后 和 右后 X向拉扯%f\n",fabs(left_back_xv - right_back_xv));
            //sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_ALL_STOP,0,0,0,0);
        }
        if( fabs(left_front_yv - left_back_yv) > max_pull_v)
        {
            printf("********左前 和 左后 Y向拉扯%f\n",fabs(left_front_yv - left_back_yv));
            //sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_ALL_STOP,0,0,0,0);
        }
        if( fabs(right_front_yv - right_back_yv) > max_pull_v)
        {
            printf("********右前 和 右后 Y向拉扯%f\n",fabs(right_front_yv - right_back_yv));
            //sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_ALL_STOP,0,0,0,0);
        }

        /*************************************************判断舵轮是否到目标角度**************************************************/
        float max_pos_error = 1.0;
        for (int i = 0; i < 4; i++)
        {
            if( fabs(pTModule->sita_base_now[i]*57.29 - pTModule->r_pos[i]) > max_pos_error)
            {
                printf("舵轮 %d por_error:%f \n",i,pTModule->sita_base_now[i]*57.29 - pTModule->r_pos[i]);
            }
        }
    }
}

static void chassisMagicStateChange(CHASSIS_MASTER_THREAD_INFO *pTModule,MAGIC_RUN_STATE state)
{
    switch (state)
    {
        case R_FOLLOW:
        case R_FOLLOW_VERTICAL:
        {
            chassisSetISpeed(pTModule,0,0,0);
            chassisSetISpeed(pTModule,1,0,0);
            chassisSetISpeed(pTModule,2,0,0);
            chassisSetISpeed(pTModule,3,0,0);
            pTModule->mRunState = state;
            //设置初始值
            pTModule->fai_R_1 = 0*3.14159/180;;
            pTModule->d_fai_R1 = 0;
            pTModule->fai_R_2 = 90*3.14159/180;          

            pTModule->last_d_xo1_R = 0;
            pTModule->last_d_yo1_R = 0;
            pTModule->last_d_fai_R1 = 0;
            pTModule->last_dd_fai_R1 = 0;
            pTModule->ideal_r_signal_flag = false;

            //算法使用，弧度
            pTModule->sita_base_now[0] = pTModule->r_pos[0]/57.29;
            pTModule->sita_base_now[1] = pTModule->r_pos[0]/57.29;
            pTModule->sita_base_now[2] = pTModule->r_pos[0]/57.29;
            pTModule->sita_base_now[3] = pTModule->r_pos[0]/57.29;

            pTModule->dd_x01_real = 0;
            pTModule->dd_y01_real = 0;
            
            memset( pTModule->d_sita_d_base_now,0,sizeof(float)*4);
            break;
        }
        case R_STOP:{
            //缓慢减速
            memset(pTModule->upper_target_v_Flag,0,sizeof(int32_t)*5);
            for (int i = 0; i < 4; i++)
            {
                //位置控制取消,启用速度控制
                pTModule->upper_target_p_Flag[i] = 0;
                //减速停止
                handleStartSlight(pTModule,i,0,0);
            }
            
            pTModule->mRunState = R_NULL;
            break;
        }
        default:
        {
            break;
        }
    }
}

static void  chassisLoopRecLocalMsg(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    struct sockaddr_in  mPeerAddr;
    socklen_t mun = sizeof(mPeerAddr);
    char mLocalData[Msg_T_S];

    if(recvfrom(pTModule->mSoket, mLocalData, Msg_T_S, 0, (struct sockaddr*)&(mPeerAddr), &mun)>0)
    {
      MsgHeader *mMsg = (MsgHeader *)mLocalData;
      MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
      switch (mMsg->mMsgId)
      {
        case MSG_ID_FIRE_IN_THE_HOLE:
        {
          pTModule->mState = M_STATE_RUN;
          printf("BASE::MSG_ID_FIRE_IN_THE_HOLE\n");
          break;
        }
        case MSG_ID_MODULE_EXIT_ALL:
        {
          printf("BASE::MSG_ID_MODULE_EXIT_ALL\n");
          pTModule->mState = M_STATE_QUIT;

          pTModule->mWorking = false;

          //ethercat线程销毁
          pTModule->mEtherWorker.mWorking = false;
          //pTModule->mMagicWorker[i].mWorking = false;
          pthread_join(pTModule->mEtherWorker.mPid, NULL);

          //发送到leader 已经退出
          sendNoticeToLeader(pTModule->mSoket,MSG_ID_THREAD_REQUEST_EXIT_ALL_ACK, 0, 0);
          printf("timer endup\n");
          break;
        }
        case CMD_IGH_CREATE_MASTERS:
        {
          createMaster(pTModule->masterId,&pTModule->master,&pTModule->domain1);
          break;
        }
        case CMD_DESTROY_MASTER:
        {
          destroyChassisMaster(pTModule);
          break;
        }
        case CMD_IGH_SCAN_SLAVES:
        {
          scanChassisSlaves(pTModule);
          break;
        }
        case CMD_IGH_CONF_SERVO_PDOS:
        {
          confChassisSlavesPdos(pTModule);
          break;
        }
        case CMD_IGH_START_OP:
        {
          activeChassisMater(pTModule);
          break;
        }
        case CMD_MASTER_OP_KEYDOWN:
        {
          activeChassisMaterKeydown(pTModule);
          break;
        }
        case CMD_IGH_FSA_SHUTDOWN:
        {
            pTModule->arm_fsa = S_SHUTDOWN;
            break;
        }
        case CMD_IGH_FSA_SWITH_ON:
        {
            pTModule->arm_fsa = S_SWITCH_ON;
            break;
        }
        case CMD_IGH_FSA_ENOP:
        {
            pTModule->arm_fsa = S_ENABLE_OP;
            break;
        }
        case CMD_IGH_FSA_HALT:
        {
            pTModule->arm_fsa = S_HAIT;
            break;
        }  
        case CMD_IGH_CLEAR_ERROR:
        {
            pTModule->arm_fsa = S_CLEAR_ERROR;
            break;
        }
        case MSG_ID_ARMS_MAGIC_CTRL_MSG:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            ARMS_MAGIC_CTRL_V *smsg =  (ARMS_MAGIC_CTRL_V *)mLMsg->mData;
            memcpy(&pTModule->mArmsMagicCtrl,mLMsg->mData,sizeof(ARMS_MAGIC_CTRL_V));
            break;
        } 
        case CMD_CHASSIS_STOP:
        {
            memset(pTModule->upper_target_v_Flag,0,sizeof(int32_t)*5);
            for (int i = 0; i < 4; i++)
            {
                //位置控制取消,启用速度控制
                pTModule->upper_target_p_Flag[i] = 0;
                //减速停止
                handleStartSlight(pTModule,i,0,0);
            }
            break; 
        }
        case CMD_IGH_CHASSIS_VEL:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG *smsg =  (SUPR_R_MSG *)mLMsg->mData;
            float *pVel = (float*)smsg->mData;
            memset(pTModule->upper_target_v_Flag,0,sizeof(int32_t)*5);
            for (int i = 0; i < 4; i++)
            {
                if(smsg->mModuleFlag[i])
                {
                    //轴安装顺序 Y XM XS Z
                    handleStartSlight(pTModule,i,pVel[0],pVel[1]);
                }
            }
            //开启顶升气缸。不使用缓坡加减速
            if(smsg->mModuleFlag[4])
            {
                chassisSetUpSpeed(pTModule,pVel[0]);
            }
            break;
        }    
        case CMD_CHASSIS_WHEEL_AGNGLE:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG *smsg =  (SUPR_R_MSG *)mLMsg->mData;
            float *pPos = (float*)smsg->mData;
            for (int i = 0; i < 4; i++)
            {
                if(smsg->mModuleFlag[i])
                {
                    //轴安装顺序 Y XM XS Z
                    chassisSetIRotatePos(pTModule,i,pPos[i]);
                }
            }
            break;
        }   
        case CMD_CHASSIS_MOVE:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG *smsg =  (SUPR_R_MSG *)mLMsg->mData;
            float *pPos = (float*)smsg->mData;
            //横向单位m/s  纵向单位 m/s 旋转单位rad/s 
            chassisMove(pTModule,pPos[0]/1000.0,pPos[1]/1000.0,pPos[2]*3.14159/180.0);
            break;
        } 
        case CMD_VALVE_CHASSIS24_ON:
        case CMD_VALVE_CHASSIS24_OFF:
        case CMD_VALVE_CHASSIS48_ON:
        case CMD_VALVE_CHASSIS48_OFF:
        case CMD_VALVE_CHASSIS_GAS_ON:
        case CMD_VALVE_CHASSIS_GAS_OFF:
        case CMD_VALVE_CHASSIS_BRK_ON:
        case CMD_VALVE_CHASSIS_BRK_OFF:
        {
          MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
          memcpy(&pTModule->mSerialWorker->upperMsg,mLMsg->mData,sizeof(SUPR_R_MSG));
          pTModule->mSerialWorker->newUpperMsg = true;
          break;
        }
        case CMD_PROPORTIONAL_VALVE:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG *pmsg = (SUPR_R_MSG *)mLMsg->mData;
            proportional_value *pv =  (proportional_value *)pmsg->mData;
            setChassisProportional(pTModule,pv->pvid,pv->value);
            break;
        }
        case CMD_PROPORTIONAL_VALVES:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG *pmsg = (SUPR_R_MSG *)mLMsg->mData;
            int pvs = *(int *)pmsg->mData;
            proportional_value *pv =  (proportional_value *)(pmsg->mData+4);
            setChassisProportionals(pTModule,pv,pvs);
            break;
        }
        case CMD_AUTO_FOLLOW:
        case CMD_AUTO_FOLLOW_CHASSIS:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            chassisMagicStateChange(pTModule,R_FOLLOW);
            break;
        }
        case CMD_ALL_STOP:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            chassisMagicStateChange(pTModule,R_STOP);
            break;
        }
        case CMD_CHASSIS_MAGIC_ENABLE:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG *pmsg = (SUPR_R_MSG *)mLMsg->mData;
            uint8_t *penab = (uint8_t*)pmsg->mData;
            pTModule->enable_flag = penab[0];
            break;
        }
        case CMD_AUTO_FOLLOW_VERTICAL:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            chassisMagicStateChange(pTModule,R_FOLLOW_VERTICAL);
            break;
        }
        default:
        {
          break;
        }
      } //end switch

    }
}
static void  destroy_slaves_inf(EK1100_slaves_inf *slaves_inf_list)
{
    if(slaves_inf_list->childSlaveSize >0)
    {   
        EK1100_slaves_inf *i = NULL;
        EK1100_slaves_inf *tmp = NULL;
        list_for_each_entry_safe(i, tmp, &slaves_inf_list->list, list) 
        {
            list_del(&i->list);
            free(i);
        }
        slaves_inf_list->childSlaveSize = 0;
    }
}

static int  destroyChassisMaster(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    //伺服运行过程中，急停电机
    if (pTModule->masterActivated)
    {
        pTModule->arm_fsa = S_HAIT;
        usleep(2000);
        pTModule->masterActivated = false;
    }
    
    //销毁master
    if(pTModule->master != NULL)
    {
        ecrt_release_master(pTModule->master);
        pTModule->master = NULL;
    }
    //销毁从站信息
    destroy_slaves_inf(&pTModule->slaves_inf);
}
static int scanChassisSlaves(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
  int iRet = -1;
  if(!pTModule->master)
    return iRet;

  //创建slaves_inf头
  destroy_slaves_inf(&pTModule->slaves_inf);
  INIT_LIST_HEAD(&pTModule->slaves_inf.list);

  //使用命令行，查询从站个数
  FILE*fp = NULL;
  char comm[64]="";
  sprintf(comm,"sudo ethercat -m %d slave | wc -l",pTModule->masterId);
  fp = popen(comm,"r");

  int slavesize = 0;
  if (fp != NULL)
  {   
      char buffer[100];
      while (fgets(buffer, 100,fp) != NULL)
          slavesize = atoi(buffer);

      fclose(fp);          
  }

  for (int i = 0; i < slavesize; i++)
  {
    ec_slave_info_t slave_in;
    if(ecrt_master_get_slave(pTModule->master,i,&slave_in)<0)
      break;

    EK1100_slaves_inf *childSlave = (EK1100_slaves_inf*)malloc(sizeof(EK1100_slaves_inf));
    memset((char*)childSlave,0,sizeof(EK1100_slaves_inf));
    memcpy(&childSlave->slave_in,&slave_in,sizeof(ec_slave_info_t));

    list_add_tail(&childSlave->list,&pTModule->slaves_inf.list);
    pTModule->slaves_inf.childSlaveSize++;

    printf("********* %s %d %d \n", childSlave->slave_in.name,childSlave->slave_in.alias,childSlave->slave_in.position );
  }
  return 0;
}

static int confChassisSlavesPdos(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    if(pTModule->slaves_inf.childSlaveSize< 1)
    {
      printf("confSlavesPdos:底盘从站不存在\n\r");
      return -1;
    }

    //遍历从站，找到所有伺服电机，并通过sdo配置所使用的pdo
    int motorIndex=0;

    EK1100_slaves_inf *inf_t = NULL ;
    list_for_each_entry(inf_t, &pTModule->slaves_inf.list, list)
    {
        //创建从站时候一定不要使用alias，使用0 ，position的绝对位置测试
        ec_slave_config_t * sc = ecrt_master_slave_config(pTModule->master, 0,inf_t->slave_in.position, inf_t->slave_in.vendor_id,inf_t->slave_in.product_code);

        if(sc==NULL)
        {
            printf("ecrt_master_slave_config ********* NULL\n");
            return -1;
        }
        inf_t->slave_conf = sc;
        //机械臂的somanet结点，
        if( (strstr(inf_t->slave_in.name,"PMM6020") != NULL) || 
            (strstr(inf_t->slave_in.name,"PMM8075") != NULL) ||
            (strstr(inf_t->slave_in.name,"PSM6040") != NULL)
            )
        {
            ecrt_slave_config_sdo16(sc, 0x6040, 0, 0x80 );
            ecrt_slave_config_sdo8(sc, 0x6060, 0, 9);
            if (ecrt_slave_config_pdos(sc, EC_END, nimotion_syncs) == 0)
            {
                //配置域
                pTModule->nimotion_pdos[motorIndex].ctrl_word = ecrt_slave_config_reg_pdo_entry(sc,0x6040, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].operation_mode = ecrt_slave_config_reg_pdo_entry(sc,0x6060, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].Target_position = ecrt_slave_config_reg_pdo_entry(sc,0x607A, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].target_velocity = ecrt_slave_config_reg_pdo_entry(sc,0x60FF, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].status_word = ecrt_slave_config_reg_pdo_entry(sc,0x6041, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].mode_display = ecrt_slave_config_reg_pdo_entry(sc,0x6061, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].Position_value = ecrt_slave_config_reg_pdo_entry(sc,0x6064, 0, pTModule->domain1, NULL);
                pTModule->nimotion_pdos[motorIndex].velocity_value = ecrt_slave_config_reg_pdo_entry(sc,0x606c, 0, pTModule->domain1, NULL);
            }
            motorIndex++;
        }
        //机械臂的EC-3045结点，
        if(strstr(inf_t->slave_in.name,"EC-3400") != NULL)
        {   
            printf("EC-3045 EC-3045\n");
        }
    }
    pTModule->motor_size = motorIndex;
    printf("###########################\n");
    return 0;
}

static int activeChassisMater(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    int iRet = -1;
    if(pTModule->slaves_inf.childSlaveSize< 1)
      return iRet;
    //设置DC时钟

    EK1100_slaves_inf *inf_t = NULL;
    list_for_each_entry(inf_t, &pTModule->slaves_inf.list, list)
    {
        //电机使用dc
        if( (strstr(inf_t->slave_in.name,"PMM6020") != NULL) || 
            (strstr(inf_t->slave_in.name,"PMM8075") != NULL) ||
            (strstr(inf_t->slave_in.name,"PSM6040") != NULL))
        {
            ecrt_slave_config_dc(inf_t->slave_conf, 0x0300, 1000000, 0, 0, 0);
            printf("PMM6020 ecrt_slave_config_dc \n");
        }       
    }
    int ret = ecrt_master_select_reference_clock(pTModule->master, NULL);
    if (ret < 0) {
        printf("Failed to select reference clock\n");
        return iRet;
    }
    
    printf("Activating master...\n");
    if (ecrt_master_activate(pTModule->master)) {
        return iRet;
    }
    else
    {
        printf("*Master activated*\n");
    }
    if (!(pTModule->domain1_pd = ecrt_domain_data(pTModule->domain1))) {
        printf("ecrt_domain_data EXIT_FAILURE\n");
    }
    printf("starting 1000HZ pdo\n");
    //开启1000HZ循环
    pTModule->masterActivated=true;

    return 0;
}

static void activeChassisMaterKeydown(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    if(createMaster(pTModule->masterId,&pTModule->master,&pTModule->domain1) != 0)
        return;

    if(scanChassisSlaves(pTModule) != 0)
        return;

    if(confChassisSlavesPdos(pTModule) != 0)
        return;

    if(activeChassisMater(pTModule) != 0)
        return;
}
/******************************************************************************
* 功能：tsnorm函数将timespec中的纳秒，妙格式化。按照1m = 1000000000ns，
* @param ts : ts是时间结构体，包含：妙 纳秒
* @return Descriptions
******************************************************************************/
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

static void  packM_S_master_run_inf(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    //拷贝master主站下的信息到upper
    int mSMsgSize = sizeof(ARMS_S_MSG)+sizeof(Masters_inf);
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);

    //框架1，暂时设置
    mSMsg->mIdentifier = pTModule->mIdentifier;
    mSMsg->mMsgType = ACK_NOTICE_SHOW_MASTER_RUN_INF;

    struct timespec  nowTime;
    clock_gettime(CLOCK_MONOTONIC, &nowTime);
    mSMsg->mSysTimeS = nowTime.tv_sec;
    mSMsg->mSysTimeUs = nowTime.tv_nsec/1000;
    Masters_inf  *mMasterInf = (Masters_inf  *)mSMsg->mData;

    //主站0
    mMasterInf->masterid = 1;
    
    mMasterInf->al_states = pTModule->al_states;
    mMasterInf->st_slaves = pTModule->st_slaves;
    mMasterInf->master_link_up = pTModule->master_link_up;
    mMasterInf->domain_wc = pTModule->domain_wc;

    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    
    free(mSMsg);
}

void* master_chassis_threadEntry(void* pModule)
{
  CHASSIS_MASTER_THREAD_INFO *pTModule = (CHASSIS_MASTER_THREAD_INFO *) pModule;

  if(NULL == pTModule)
  {
    printf("null timer\n");
    return 0;
  }

  initchassis(pTModule);

  unsigned int nDelay = 1000;        /* usec */
  //timer 定时器
  struct timespec  next, interval;
  interval.tv_sec  = nDelay / USEC_PER_SEC;
  interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
  clock_gettime(CLOCK_MONOTONIC, &next);

  printf("%s running!threadEntry\n",pTModule->mThreadName);
  sendNoticeToLeader(pTModule->mSoket,MSG_ID_THREAD_START_ACK, 0, 0);
  
  memset(pTModule->upper_target_v_Flag,0,sizeof(float)*5);

  int ret = 0;
  uint32_t runCnt = 0;
  pTModule->mRunState = R_NULL;
  while(pTModule->mWorking)
  {
    next.tv_sec  += interval.tv_sec;
    next.tv_nsec += interval.tv_nsec;
    tsnorm(&next);
    if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)))
    {
      if (ret != EINTR)
        printf("clock_nanosleep failed. errno:");
      continue;
    }

    runCnt++;
    //200HZ rec local msg
    if((runCnt) % 5==0)
    {
        chassisLoopRecLocalMsg(pTModule);

        //循环2s的增加速度到目标值
        cyc_vel_upper_cmd(pTModule);

        //位置控制
        cyc_rotate_pos_upper_cmd(pTModule);

        //指令滤波
        filter_smooth(pTModule);
    }

    if((runCnt%50) == 0)
    {
        chassis_magic_run(pTModule);
    }

    //20HZ发送底盘舵轮控制数据
    if((runCnt) % 50==0)
    {
        //主站激活之后才发送消息到上位机
        if(pTModule->masterActivated)
        {
            packChassis_S_MSG(pTModule);

            if (!IsSystemError) {
                for (int i = 0; i < NIMOTION_SIZE; i++)
                {
                    
                    if(i%2 == 0){
                        //mChassis->nimotion_inf[i].velocity_value = pTModule->w_speed[i/2];
                        info_show_in_err.nimotion_inf[1][i].velocity_value = pTModule->w_speed[i/2];
                        //mChassis->nimotion_inf[i].Position_value = pTModule->w_pos[i/2];
                        info_show_in_err.nimotion_inf[1][i].Position_value = pTModule->w_pos[i/2];
                    }
                    else{
                        //mChassis->nimotion_inf[i].velocity_value = pTModule->r_speed[(i-1)/2];
                        info_show_in_err.nimotion_inf[1][i].velocity_value = pTModule->r_speed[(i-1)/2];
                        //mChassis->nimotion_inf[i].Position_value = pTModule->r_pos[(i-1)/2];
                        info_show_in_err.nimotion_inf[1][i].Position_value = pTModule->r_pos[(i-1)/2];
                    }
                }
                
            }

        }
    }

    //2HZ发送底盘采集的传感器数据
    if((runCnt) % 500==0)
    {
        packChassis_sensor_S_MSG(pTModule);
    }

    if((runCnt) % 99==0)
    {
        //主站激活之后才发送消息到上位机
        if(pTModule->masterActivated)
        {
            packM_S_master_run_inf(pTModule);
        }
    }

  } 

  sleep(1);
  //销毁master
  if(pTModule->master != NULL)
  {
      ecrt_release_master(pTModule->master);
      pTModule->master = NULL;
  }
  if(pTModule->mSoket>0)
  {
    close(pTModule->mSoket);
    pTModule->mSoket = -1;
  }

}

/******************************************************************************
* 功能：子线程：master使用ethercat交换pdo线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : args
* @return Descriptions
******************************************************************************/
static void check_chassis_domain1_state(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    static uint32_t cnt = 0;
    ec_domain_state_t ds;
    ecrt_domain_state(pTModule->domain1, &ds);
    if (ds.working_counter != pTModule->domain1_state.working_counter)
    {
        printf("master:%d Domain1: WC %u. cnt:%d\n", pTModule->mthreadMasterId,ds.working_counter,cnt++);
    }
    if (ds.wc_state != pTModule->domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    pTModule->domain1_state = ds;
}
static void check_chassis_master_state(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    ec_master_state_t ms;
    ecrt_master_state(pTModule->master, &ms);
    if (ms.slaves_responding != pTModule->master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
        pTModule->st_slaves = ms.slaves_responding;
    }
    if (ms.al_states != pTModule->master_state.al_states)
    {
        printf("AL states: 0x%02X. \n", ms.al_states);
        if(ms.al_states == 0x01)
            pTModule->al_states = AL_INIT;
        if(ms.al_states == 0x02)
            pTModule->al_states = AL_PREOP;
        if(ms.al_states == 0x04)
            pTModule->al_states = AL_SAFEOP;
        if(ms.al_states == 0x08)
            pTModule->al_states = AL_OP;
    }
    if (ms.link_up != pTModule->master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
        pTModule->master_link_up = ms.link_up;
    }
    pTModule->master_state = ms;
}

static int32_t chaeckChassisError(CHASSIS_MASTER_THREAD_INFO *pTModule)
{

}

static int32_t chassisFsa(CHASSIS_MASTER_THREAD_INFO *pTModule)
{
    int32_t iRet = 0;
  
    //有限状态机切换
    //4个伺服电机、EK1100 Ek1110 ...
    if(pTModule->slaves_inf.childSlaveSize>=3)
    {
        for (int i = 0; i < NIMOTION_SIZE; i++)
        {
            //写入4个电机的数据
            switch (pTModule->arm_fsa){
                case S_SHUTDOWN:{
                    //"开启"禁用状态，此时需要发送控制字
                    pTModule->motors[i].ctrl_word = 0x0006;
                    pTModule->motors[i].operation_mode = 0x09;
                    pTModule->tar_vel_cmd[i] = 0;
                    pTModule->motors[i].target_velocity = 0;
                    break;
                }
                case S_SWITCH_ON:{
                    pTModule->motors[i].ctrl_word = 0x0007;
                    pTModule->motors[i].operation_mode = 0x09;
                    pTModule->tar_vel_cmd[i] = 0;
                    pTModule->motors[i].target_velocity = 0; 
                    //printf("S_SWITCH_ON %d\n",i);                     
                    break;
                }
                case S_ENABLE_OP:{
                    pTModule->motors[i].ctrl_word = 0x000F;
                    pTModule->motors[i].operation_mode = 0x09;
                    break;
                }
                case S_HAIT:{
                    pTModule->motors[i].ctrl_word = 0x02; 
                    pTModule->tar_vel_cmd[i] = 0;
                    pTModule->arm_fsa = S_NULL;
                    break;
                }
                case S_CLEAR_ERROR:
                {
                    pTModule->motors[i].ctrl_word = 0x80; 
                    pTModule->motors[i].target_velocity = 0;
                    pTModule->tar_vel_cmd[i] = 0;
                    pTModule->arm_fsa = S_NULL;
                    break;
                }
                case S_NULL:{
                    //TUDO init some param
                    memset(pTModule->upper_target_v_Flag,0,sizeof(int)*4);
                    memset(pTModule->upper_target_p_Flag,0,sizeof(int)*4);
                    pTModule->tar_vel_cmd[i] = 0;
                    break;
                }
                default:{
                    break;
                }
            }
        }
        //电机再op状态下出现error，进行急停
        if(chaeckChassisError(pTModule)!=0 && pTModule->arm_fsa==S_ENABLE_OP)
        {
            //pTModule->arm_fsa = S_HAIT;
        }   
    }
  return iRet;
}
//获取当前系统时间
#define     TIMESPEC2NS(T)       ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
static uint64_t system_time_ns(void)
{
    struct timespec  rt_time;
    clock_gettime(CLOCK_REALTIME, &rt_time);
    uint64_t time = TIMESPEC2NS(rt_time);
    return time;
}
void* ethercatChassisThread(void* pModule)
{
    ETHERCAT_THREAD_INFO *pEModule = (ETHERCAT_THREAD_INFO *) pModule;
    CHASSIS_MASTER_THREAD_INFO *pTModule = (CHASSIS_MASTER_THREAD_INFO *)pEModule->pTModule;
    uint16_t mIdentifier = pTModule->mIdentifier;
    unsigned int nDelay = 1000;        /* usec */
    //timer 定时器
    struct timespec  next, interval;
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int ret = 0;
    uint32_t  runCnt = 0;
    while (pEModule->mWorking)
    {
        next.tv_sec  += interval.tv_sec;
        next.tv_nsec += interval.tv_nsec;
        tsnorm(&next);
        if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)))
        {
            if (ret != EINTR)
            printf("clock_nanosleep failed. errno:");
            continue;
        }

        if (!pTModule->masterActivated)
          continue;


        struct timespec  igh_s_time,igh_e_time;
        clock_gettime(CLOCK_MONOTONIC, &igh_s_time);

        ecrt_master_receive(pTModule->master);
        ecrt_domain_process(pTModule->domain1);
        /*Check process data state(optional)*/
        check_chassis_domain1_state(pTModule);
        //Check for master state
        check_chassis_master_state(pTModule);

        /******************************************实时传输PDO******************************************/
        //读取交换的数据
        for (int i = 0; i < pTModule->motor_size; i++)
        {
            //nimotion 伺服电机，读取速度单位是RPM  位置单位是：用户单位
            pTModule->ni_stword[i] = EC_READ_U16(pTModule->domain1_pd + pTModule->nimotion_pdos[i].status_word );
            
            float speedU = (float) EC_READ_S32(pTModule->domain1_pd + pTModule->nimotion_pdos[i].velocity_value);
            float posU = (float) EC_READ_S32(pTModule->domain1_pd + pTModule->nimotion_pdos[i].Position_value);
            //转换成电机圈
            if(i==0 || i==2 || i==4 || i==6)
            {
                //行走电机
                pTModule->w_pos[i/2] = posU*(W_POS_TO_M*CHASSIS_DIR[i]);
                pTModule->w_speed[i/2] = speedU * (W_VEL_RPM_TO_MM*CHASSIS_DIR[i]);
            }
            if(i==1 || i==3 || i==5 || i==7)
            {
                //旋转电机.位置转换.mIdentifier是框架id ： 1，2
                pTModule->r_pos[(i-1)/2] = posU*(R_POS_TO_MM*CHASSIS_DIR[i])- pTModule->r_pos_zore[i/2];
                pTModule->r_speed[(i-1)/2] =speedU * (R_VEL_RPM_TO_DEG*CHASSIS_DIR[i]);
            }

        }

        /******************************************有限状态机*******************************************/
        chassisFsa(pTModule);

        /******************************************实时传输PDO******************************************/
        //写入交换somanet的数据，1ms 刷新写入一次
        for (int i = 0; i < pTModule->motor_size; i++)
        {
            //将从站输出的值转换成PDO类型的
            //sensors_write_i_to_f(pTModule,i);
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->nimotion_pdos[i].ctrl_word, pTModule->motors[i].ctrl_word);
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->nimotion_pdos[i].operation_mode, pTModule->motors[i].operation_mode);
            EC_WRITE_S32(pTModule->domain1_pd + pTModule->nimotion_pdos[i].target_velocity, pTModule->motors[i].target_velocity);
        }        

         /**< Application-layer states of all slaves.
                                  The states are coded in the lower 4 bits.
                                  If a bit is set, it means that at least one
                                  slave in the bus is in the corresponding
                                  state:
                                  - Bit 0: \a INIT
                                  - Bit 1: \a PREOP
                                  - Bit 2: \a SAFEOP
                                  - Bit 3: \a OP */
        if(pTModule->master_state.al_states == 0x08)
        {
            ;
        }else
        {
          ;
        }

        // write application time to master
        ecrt_master_application_time(pTModule->master, system_time_ns());
        ecrt_master_sync_reference_clock(pTModule->master);
        ecrt_master_sync_slave_clocks(pTModule->master);
        
        /*Send process data*/
        ecrt_domain_queue(pTModule->domain1);
        ecrt_master_send(pTModule->master);

        clock_gettime(CLOCK_MONOTONIC, &igh_e_time);
    }

    sleep(1);
    printf("ethercatThread exit\n\r");
}
