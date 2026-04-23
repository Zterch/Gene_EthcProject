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

#include "sys_master.h"
#include "sys_leader_defs.h"
#include "sys_master_magic.h"
#include "sys_master_vision.h"
#include "sys_ctl.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"
#include "sys_db.h"
#include "sys_leader.h"
#include "sys_commu_ipc.h"


static uint32_t masCnt = 0;
/*以下是配置的参数,leader线程使用,数据字典名字，每个臂的具体参数在：param_list p_params 中*/

static int32_t initSupr(ARMS_MASTER_THREAD_INFO *pTModule);
static int     moduleEndUp(ARMS_MASTER_THREAD_INFO *pTModule);
static int     handle_module_exit(ARMS_MASTER_THREAD_INFO *pTModule);
static int     fetchRecerMsg(ARMS_MASTER_THREAD_INFO *pTModule);
static void    LoopRecLocalMsg(ARMS_MASTER_THREAD_INFO *pTModule);

static int     createMaster(int mid,ec_master_t **pms,ec_domain_t **pdi);
static void    destroyMaster(ARMS_MASTER_THREAD_INFO *pTModule);
static void    destroy_slaves_inf(EK1100_slaves_inf *slaves_inf_list);

static int     scanSlaves(ARMS_MASTER_THREAD_INFO *pTModule);
static int     confSlavesPdos(ARMS_MASTER_THREAD_INFO *pTModule);
static void    confSomanetCSVPdos(ec_domain_t *domain1,ec_slave_info_t *inf,ec_slave_config_t *sc,somanet_pdos * ppdos);
static void    confEC3045Pdos(ARMS_MASTER_THREAD_INFO *pTModule,ec_slave_info_t *inf,ec_slave_config_t *sc,int EK1100Index);

static int     activeMater(ARMS_MASTER_THREAD_INFO *pTModule);
static void    activeMaterKeydown(ARMS_MASTER_THREAD_INFO *pTModule);
static void    clearErrorSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg);
static void    upperAngularVelSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg);
static void    upperRelaPosSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg);
static void    upperAbsPosSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg);
static void    upperEC3045Proportions(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg);
static void    check_limit_switch(ARMS_MASTER_THREAD_INFO *pTModule);
static void    check_arms_near(ARMS_MASTER_THREAD_INFO *pTModule);
static void    checkFollowMSPosError(ARMS_MASTER_THREAD_INFO *pTModule);
static void    upperFSASomanets(ARMS_MASTER_THREAD_INFO *pTModule,S_ETHR_STATE fsastate);
static void    packM_S_chassisMSG(ARMS_MASTER_THREAD_INFO *pTModule);

static void    packM_S_Sample_MSG(ARMS_MASTER_THREAD_INFO *pTModule);

static void    packM_S_MSG(ARMS_MASTER_THREAD_INFO *pTModule);
static void    packM_S_master_run_inf(ARMS_MASTER_THREAD_INFO *pTModule);
static void    cyc_vel_upper_cmd(ARMS_MASTER_THREAD_INFO *pTModule);
static void    cyc_pos_upper_cmd(ARMS_MASTER_THREAD_INFO *pTModule);

static void    packM_S_err_MSG(ARMS_MASTER_THREAD_INFO *pTModule, Info_Show_In_Err *info_err_bag); //打包出错前信息到上位机

//发送上位机消息封装
static int32_t sendNoticeToUpper(ARMS_MASTER_THREAD_INFO *pTModule,uint16_t Fid,const int32_t mId, const uint16_t mNotice);

static inline void tsnorm(struct timespec *ts);

static int32_t cycCollectSensers(ARMS_MASTER_THREAD_INFO *pTModule);

static int32_t magicStateChange(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg,MAGIC_RUN_STATE state);
static int32_t magicChangeEnableFlag(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg);
static int32_t handleSikoZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);

static int32_t handleStartSlight(ARMS_MASTER_THREAD_INFO *pTModule,int armsid,float xmv,float xsv,float yv,float zv,float acc_mm);
static int32_t handleSetExtXPos(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);

static int32_t armsSetExtXPos(ARMS_MASTER_THREAD_INFO *pTModule,int aid);

static int32_t preRunFollow(ARMS_MASTER_THREAD_INFO *pTModule,int i);
//线性插值
static void handleInterpnStartCollect(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
static void handleInterpnCollecting(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
static void handleInterpnCollectOver(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg);
/*****************************************************************************/


/*****************************日志打印函数********************************/

/*****************************end日志打印函数********************************/

/*****************************ethercat总线相关函数********************************/
#define     TIMESPEC2NS(T)       ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
static void check_domain1_state(ARMS_MASTER_THREAD_INFO *pTModule);
static void check_master_state(ARMS_MASTER_THREAD_INFO *pTModule);
static int32_t chaeckArmIError(ARMS_MASTER_THREAD_INFO *pTModule,int i);
static int32_t chaeckArmIStop(ARMS_MASTER_THREAD_INFO *pTModule,int i);
static void  prop_to_ec3045(ARMS_MASTER_THREAD_INFO* pTModule,int armsid);
static void  somanets_i_to_f(ARMS_MASTER_THREAD_INFO* pTModule,int armsid);
static uint64_t system_time_ns(void);
static int32_t armsFsa(ARMS_MASTER_THREAD_INFO *pTModule);

void* ethercatBusThread(void* );
static void reflash_tpdo_Ebus(ARMS_MASTER_THREAD_INFO *pTModule);
static void reflash_rpdo_Ebus(ARMS_MASTER_THREAD_INFO *pTModule);
/******************************************************************************
* 功能：此函数销毁初始化模块，释放线程指针里的内容
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int moduleEndUp(ARMS_MASTER_THREAD_INFO *pTModule)
{
    destroyMaster(pTModule);
    if(pTModule->mSoket>0)
    {
        close(pTModule->mSoket);
        pTModule->mSoket = -1;
    }
    return 0;
}

static int  handle_module_exit(ARMS_MASTER_THREAD_INFO *pTModule)
{
    printf("BASE::MSG_ID_MODULE_EXIT_ALL\n");
    pTModule->mState = M_STATE_QUIT;

    pTModule->mWorking = false;
    //线程销毁
    pTModule->mEtherWorker.mWorking = false;
    pTModule->mVisionWorker.mWorking = false;

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        pTModule->mMagicWorker[i].mWorking = false;
    }
    pthread_join(pTModule->mEtherWorker.mPid, NULL);
    pthread_join(pTModule->mVisionWorker.mPid, NULL);

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        pthread_join(pTModule->mMagicWorker[i].mPid, NULL);
    }
    //发送到leader 已经退出
    sendNoticeToLeader(pTModule->mSoket,MSG_ID_THREAD_REQUEST_EXIT_ALL_ACK, 0, 0);
    printf("timer endup\n");
    return 0;
}



/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(ARMS_MASTER_THREAD_INFO *pTModule) {

  int32_t iRet = 0;
  //leader run in cpux
  hiSetCpuAffinity(pTModule->mCpuAffinity);

  //创建soket.*非阻塞模式
  pTModule->mSoket = createSoket(get_mn_master_port(pTModule->mthreadMasterId), MN_LocalIpV4Str, 0, 0);

  //创建slaves_inf头
  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {    
    pTModule->slaves_inf[i].childSlaveSize = 0;
    INIT_LIST_HEAD(&pTModule->slaves_inf[i].list);
  }

  //测试创建插值表数据库。
  #if 0
  //delete_table(DB_FILE,db_interpn_tables[3]);
  for (int  i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {
    //delete_table(DB_FILE,db_interpn_tables[i]);
    create_interpn_table(DB_FILE,db_interpn_tables[i]);
  }
  #endif

  //创建插值表
  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {    
    memset(&pTModule->mQuads[i],0,sizeof(quads_unit_head));
    pTModule->mQuads[i].isValid = 0;
    pTModule->mQuads[i].rows = 0;
    pTModule->mQuads[i].cols = 0;
    read_interpn_row_db(DB_FILE,db_interpn_tables[i],&pTModule->mQuads[i].rows);
    read_interpn_col_db(DB_FILE,db_interpn_tables[i],&pTModule->mQuads[i].cols);

    if(pTModule->mQuads[i].rows>0 && pTModule->mQuads[i].cols>0)
    {
        interpn_unit *p = (interpn_unit *)malloc(sizeof(interpn_unit)*pTModule->mQuads[i].rows*pTModule->mQuads[i].cols);
        if(p!=NULL)
        {
            read_interpn_db(DB_FILE,db_interpn_tables[i],p);
            pTModule->mQuads[i].pQus = generate_quads(p,pTModule->mQuads[i].rows,pTModule->mQuads[i].cols);
            pTModule->mQuads[i].isValid = 1;
            /*
            for (int  j = 0; j < pTModule->mQuads[i].rows*pTModule->mQuads[i].cols; j++)
            {
                printf("%d %f %f %f %f \n",i,p[j].xy.x,p[j].xy.y,p[j].vas.x,p[j].vas.y);
            }
            */
            free(p);
        }
    }
 
    pTModule->mQuads[i].mNewInterpns = NULL;
    //printf("interpn_v:(%d %d) \n",pTModule->mQuads[i].rows,pTModule->mQuads[i].cols);
  }

  //interpn_value v = interpn_magic(&pTModule->mQuads[0], -20,-20);  
  //printf("interpn_v:(%f %f) \n", v.x,v.y);

  #if 0
  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {  
    interpn_value v = interpn_magic(&pTModule->mQuads[i],-20,-20);  
    printf("interpn_v:(%f %f) \n", v.x,v.y);
  }
  #endif
  
  /*一号臂的电机的减速比、轴方向*/
  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {
        pTModule->AXIS_DIR[i][0] = 1;
        pTModule->AXIS_DIR[i][1] = -1;
        pTModule->AXIS_DIR[i][2] = 1;
        pTModule->AXIS_DIR[i][3] = -1;
  }

  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {
     pTModule->sikoXYZ_ZERO[i][0] = 0.0;
     pTModule->sikoXYZ_ZERO[i][1] = 0.0;
     pTModule->sikoXYZ_ZERO[i][2] = 0.0;
     //上电时刻默认给中阀 小阀门-5V全部放气
     for (int j = 2; j < 4; j++)
     {
        pTModule->proportion_cmd[i][j] = -5;   
     }
     //外部编码器绝对值起始位置
     pTModule->xStartMExtPos[i] = 0;
     pTModule->xStartSExtPos[i] = 0;
  }
    
  //ethercat线程
  pTModule->mEtherWorker.mWorking = 1;
  strcpy(pTModule->mEtherWorker.mThreadName,"ethercatIgh");
  pTModule->mEtherWorker.pTModule = pTModule;
  pTModule->mEtherWorker.mPid = hiCreateThread("EtherIGH", ethercatBusThread,PRI_MASTER, &pTModule->mEtherWorker);

 //vision网络线程创建
  pTModule->mVisionWorker.mWorking = 1;
  strcpy(pTModule->mVisionWorker.mThreadName,"vision");
  pTModule->mVisionWorker.pTModule = pTModule;
  pTModule->mVisionWorker.mPid = hiCreateThread("vision", visionThread,PRI_MASTER, &pTModule->mVisionWorker);


  //创建算法magic线程。
  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {
    pTModule->mMagicWorker[i].mWorking = 1;
    sprintf(pTModule->mMagicWorker[i].mThreadName,"arm:%d",i);
    pTModule->mMagicWorker[i].mthreadMasterId = pTModule->masterId;
    pTModule->mMagicWorker[i].mArmsId = i;
    pTModule->mMagicWorker[i].pMaster = pTModule;
    pTModule->mMagicWorker[i].mPid = hiCreateThread("armn", magic_threadEntry,PRI_MASTER, &(pTModule->mMagicWorker[i]));
  }

  //初始化主站信息
  pTModule->master=NULL;
  memset(&pTModule->master_state,0,sizeof(ec_master_state_t));
  pTModule->domain1=NULL;
  memset(&pTModule->domain1_state,0,sizeof(ec_domain_state_t));

  //初始化速度、位置控制信息
  memset(pTModule->upper_target_p_Flag,0,sizeof(uint8_t)*MASTER_ARMS_MAX_SIZE);
  memset(pTModule->upper_target_v_Flag,0,sizeof(uint8_t)*MASTER_ARMS_MAX_SIZE);
  memset(pTModule->upper_target_p_Kp_cnt,0,sizeof(int32_t)*MASTER_ARMS_MAX_SIZE);

  return iRet;
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

static int fetchRecerMsg(ARMS_MASTER_THREAD_INFO *pTModule)
{
  //check is error
  int32_t errorCnt = 0;
  //for(int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
  {
    //if(mTMsg[i].mMsgIsValid == 0)
    //  errorCnt++;
  }
  return 0;
}

/******************************************************************************
* 功能：
* @return Descriptions
******************************************************************************/
static int32_t cycCollectSensers(ARMS_MASTER_THREAD_INFO *pTModule)
{

}

static int32_t magicChangeEnableFlag(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg)
{
    uint8_t *enf = (uint8_t*)mSMsg->mData;
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(pTModule->slaves_inf[i].childSlaveSize>4 && mSMsg->mModuleFlag[i])
        {
            MAGIC_THREAD_INFO *pMagic = &pTModule->mMagicWorker[i];
            if(enf[0] == 1)
            {
                pMagic->enable_flag = 1;
            }else{

                pMagic->enable_flag = 0;
            }
        }
    }    
}

static int32_t preRunFollow(ARMS_MASTER_THREAD_INFO *pTModule,int i)
{
    MAGIC_THREAD_INFO *pMagic = &pTModule->mMagicWorker[i];
    armsSetSpeedXY(pTModule,i,0,0,0);
    pMagic->rope_end_last_x = 0;
    pMagic->rope_end_last_y = 0;
    //主轴副轴位置偏差,内部和外部编码器
    pMagic->x_extpos_start_error = pTModule->xMExtPos[i] - pTModule->xSExtPos[i];
    pMagic->x_intpos_start_error = pTModule->xMIntPos[i] - pTModule->xSIntPos[i];

    pMagic->cmd_last_xv = 0;
    pMagic->cmd_last_yv = 0;

    //使用X轴外部编码器，跟绝对值编码器对准
    armsSetExtXPos(pTModule,i);
    //清空X Y命令滤波
    memset(pTModule->mF_Inf[i].m_x_cmd_arr,0,sizeof(float)*F_ARR_SIZE);
    memset(pTModule->mF_Inf[i].m_y_cmd_arr,0,sizeof(float)*F_ARR_SIZE);
    memset(pTModule->mF_Inf[i].d_rope_x_arr,0,sizeof(float)*F_ARR_SIZE);
    memset(pTModule->mF_Inf[i].d_rope_y_arr,0,sizeof(float)*F_ARR_SIZE);

    //力矩偏置使用到
    pMagic->xStartPos = pTModule->xMIntPos[i];
    pMagic->xSStartPos = pTModule->xSIntPos[i];
    pMagic->yStartPos = pTModule->yPos[i];

    CHASSIS_MASTER_THREAD_INFO *pTChassis = pTModule->pTChassis;
    pTChassis->fai_R_1 = 0*3.14159/180;;
    pTChassis->d_fai_R1 = 0;
    pTChassis->fai_R_2 = 90*3.14159/180;          

    pTChassis->last_d_xo1_R = 0;
    pTChassis->last_d_yo1_R = 0;
    pTChassis->last_d_fai_R1 = 0;

    //命令插值初始化
    memset(pMagic->target_vel_x_inters,0,sizeof(float)*100);
    memset(pMagic->target_vel_y_inters,0,sizeof(float)*100);
    pMagic->target_vel_i = 0;

    //速度输出低通滤波器
    pMagic->cmd_fir_last_xv = 0;
    pMagic->magic_follow_runcnt = 0;

    //指定算法吊杆的理想位置
    CHASSIS_MASTER_THREAD_INFO *pchassis = (CHASSIS_MASTER_THREAD_INFO *)pTModule->pTChassis;
    pchassis->L_R_ideal = pTModule->xMIntPos[i];
    pchassis->D_R_ideal = pTModule->yPos[i];
    pchassis->ArmsReferId = i;
}

static int32_t magicStateChange(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg,MAGIC_RUN_STATE state)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(pTModule->slaves_inf[i].childSlaveSize>4 && mSMsg->mModuleFlag[i])
        {
            MAGIC_THREAD_INFO *pMagic = &pTModule->mMagicWorker[i];
            switch (state)
            {
                case R_VERTICAL_WEIGHT:{
                    //初始化称重
                    pMagic->last_dert_z = 0;
                    pMagic->last_cmd_medium = 0;
                    pTModule->proportion_cmd[i][3] = 0;
                    break;
                }
                case R_FOLLOW:
                {
                    preRunFollow(pTModule,i);
                    break;
                }
                case R_VERTICAL:{
                    //发送执行机构停止，然后跳转到NULL 状态
                    //pMagic->cmd_speed_z = 0;
                    pMagic->last_dert_z = 0;
                    pMagic->floating_last_p1 = 0;
                    pMagic->cmd_z_Last_speed = 0;
                    //设置输入整形算法初始值
                    memset(pTModule->mF_Inf[i].m_z_cmd_arr,0,sizeof(float)*500);
                    break;
                }
                case R_FOLLOW_VERTICAL:{

                    //发送执行机构停止，然后跳转到NULL 状态
                    //pMagic->cmd_speed_z = 0;
                    pMagic->last_dert_z = 0;
                    pMagic->floating_last_p1 = 0;
                    pMagic->cmd_z_Last_speed = 0;
                    memset(pTModule->mF_Inf[i].m_z_cmd_arr,0,sizeof(float)*500);

                    preRunFollow(pTModule,i);
                    break;
                }
                case R_STOP:{
                    //缓慢减速,加速度200mm/s2
                    handleStartSlight(pTModule,i,0,0,0,0,300);

                    //比例阀设置-1V，
                    pTModule->proportion_cmd[i][3] = -0.4;
                    break;
                }
                default:
                {
                    break;
                }
            }

            pTModule->mMagicWorker[i].mRunState = state;
        }
    }
}


static int32_t handleSikoZero(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            //磁栅尺置零
            pTModule->sikoXYZ_ZERO[i][0] -= pTModule->sikoXYZ[i][0];
            pTModule->sikoXYZ_ZERO[i][1] -= pTModule->sikoXYZ[i][1];
            //pTModule->sikoXYZ_ZERO[i][2] -= pTModule->sikoXYZ[i][2];
        }
    }
}

static int32_t sendNoticeToUpper(ARMS_MASTER_THREAD_INFO *pTModule,uint16_t Fid,const int32_t mId, const uint16_t mNotice)
{
    int mSMsgSize = sizeof(ARMS_S_MSG);
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);
    //框架1，暂时设置
    mSMsg->mIdentifier = Fid;
    mSMsg->mModuleFlag[mId] = 1;
    mSMsg->mMsgType = mNotice;
    sendDataToUpper(pTModule->mSoket,MSG_ID_UPPER_NOTICE,0,mSMsgSize,(char*)mSMsg,mSMsgSize); 
}

static int32_t armsSetExtXPos(ARMS_MASTER_THREAD_INFO *pTModule,int aid)
{
    if(pTModule->masterActivated && pTModule->slaves_inf[aid].childSlaveSize>4)
    {
        pTModule->xStartMExtPos[aid] += (pTModule->xMIntPos[aid] - pTModule->xMExtPos[aid]);
        pTModule->xStartSExtPos[aid] += (pTModule->xSIntPos[aid] - pTModule->xSExtPos[aid]);
    }
}

static void  destroyMaster(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //伺服运行过程中，急停电机
    if (pTModule->masterActivated)
    {
        for (int i = 0; i < pTModule->armsSize; i++)
        {
            pTModule->arm_fsa[i] = S_HAIT;
        }
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
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {  
        destroy_slaves_inf(&pTModule->slaves_inf[i]);
    }
    //销毁申请内存
    #if 0
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(pTModule->mMagicWorker[i].auto_follow_sim_d_size >0 )
        {
            //设置仿真数据大小，并申请内存
            free(pTModule->mMagicWorker[i].auto_follow_sim_dx);
            free(pTModule->mMagicWorker[i].auto_follow_sim_dy);
            pTModule->mMagicWorker[i].auto_follow_sim_d_size = 0;
        }
    }
    #endif 
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

static int  scanSlaves(ARMS_MASTER_THREAD_INFO *pTModule)
{
  int iRet = -1;
  if(!pTModule->master)
    return iRet;

  //创建slaves_inf头
  for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
  {    
    destroy_slaves_inf(&pTModule->slaves_inf[i]);
    INIT_LIST_HEAD(&pTModule->slaves_inf[i].list);
  }

  int EK1100size = 0;

  //使用命令行，查询从站个数
  FILE *fp = NULL;
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

    if(strstr(slave_in.name,"EK1100") != NULL)
    {
        //list_add_tail(&childSlave->list,&pTModule->slaves_inf[EK1100size].list);
        EK1100size++;  
    }else{
        //list_add_tail(&childSlave->list,&pTModule->slaves_inf[EK1100size-1].list);
    }

    list_add_tail(&childSlave->list,&pTModule->slaves_inf[EK1100size-1].list);
    pTModule->slaves_inf[EK1100size-1].childSlaveSize++;

    printf("&&&&&&& %s %d %d \n", childSlave->slave_in.name,childSlave->slave_in.alias,childSlave->slave_in.position );
  }
  //设置从站个数、arms的个数
  pTModule->slaves_size = slavesize;
  pTModule->armsSize = EK1100size;
  return 0;
}

static void  confSomanetCSVPdos(ec_domain_t *domain1,ec_slave_info_t *inf,ec_slave_config_t *sc,somanet_pdos * ppdos)
{
    /*Config PDOs*/
    uint8_t rpdo_entries_size = 5;
    uint8_t tpdo_entries_size = 7;
    ec_pdo_entry_info_t rpdo_entries[] = {
        /*RxPdo 0x1600*/
        {0x6040, 0x00, 16},
        {0x6060, 0x00, 8},
        {0x607A, 0x00, 32},
        {0x60FF, 0x00, 32},
        {0x60B2, 0x00, 16},
    };
    ec_pdo_entry_info_t tpdo_entries[] = {
        /*TxPdo 0x1A00*/
        {0x6041, 0x00, 16},
        {0x6061, 0x00, 8},
        {0x6064, 0x00, 32},
        {0x606C, 0x00, 32},
        {0x2111, 0x02, 32},
        {0x6077, 0x00, 16},
        {0x6074, 0x00, 16}
    };

    ec_pdo_info_t ec_rpdos[] = {
        //RxPdo
        {0x1600, rpdo_entries_size, rpdo_entries + 0 }, /*recive pdo maping*/
    };
    ec_pdo_info_t ec_tpdos[] = {
        //TxPdo
        {0x1A00, tpdo_entries_size, tpdo_entries + 0 } /*send pdo maping*/
    };

    /*
     通过ethercat命令行可以看到和pdo相关的几个数据结构，主站中也是通过链表这么存储的。如果从站支持邮箱协议，0,1默认给邮箱通信使用。
     SM(Sync manager index):0 邮箱数据
     SM(Sync manager index):1 邮箱数据
     SM(Sync manager index):2 pdos 0x1600 ...
                                          多个pdo条目
     SM(Sync manager index):3 pdos 0x1a00 ...
                                          多个pdo条目
     */
    ec_sync_info_t servo_syncs[] = {
        { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
        { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
        { 2, EC_DIR_OUTPUT, 1, ec_rpdos + 0, EC_WD_DISABLE },
        { 3, EC_DIR_INPUT, 1, ec_tpdos + 0, EC_WD_DISABLE },
        { 0xFF}
    };
    

    //清除伺服错误
    ecrt_slave_config_sdo16( sc, 0x6040, 0, 0x80 );
    ecrt_slave_config_sdo16( sc, 0x2010, 0x0a, 0x07D0);
    ecrt_slave_config_sdo16( sc, 0x2010, 0x0b, 0x07D0);
    #if 0
    //配置RPDO
    /* Clear RxPdo */
    ecrt_slave_config_sdo8( sc, 0x1C12, 0, 0 ); /* clear sm pdo 0x1c12 */
    ecrt_slave_config_sdo8( sc, 0x1600, 0, 0 ); /* clear RxPdo 0x1600 */
    ecrt_slave_config_sdo8( sc, 0x1601, 0, 0 ); /* clear RxPdo 0x1601 */
    ecrt_slave_config_sdo8( sc, 0x1602, 0, 0 ); /* clear RxPdo 0x1602 */
    ecrt_slave_config_sdo8( sc, 0x1603, 0, 0 ); /* clear RxPdo 0x1603 */

    ecrt_slave_config_sdo32( sc, 0x1600, 1, 0x60400010 ); /* 0x6040:0/16bits, control word */
    ecrt_slave_config_sdo32( sc, 0x1600, 2, 0x60600008 ); /* 0x6060:0/8bits  ,Modes of operation */              
    ecrt_slave_config_sdo32( sc, 0x1600, 3, 0x607A0020 ); /* 0x607A:0/32bits ,Target position  */
    ecrt_slave_config_sdo32( sc, 0x1600, 4, 0x60FF0020 ); /* 0x60FF:0/32bits ,target velocity  */
    ecrt_slave_config_sdo32( sc, 0x1600, 5, 0x60B20010 ); /* 0x60B2:0/16bits ,Torque offset  */
    //ecrt_slave_config_sdo32( sc, 0x1600, 6, 0x20100a10 ); /* 0x2010:0a/16bits ,Setting time  */
    //ecrt_slave_config_sdo32( sc, 0x1600, 7, 0x20100b10 ); /* 0x2010:0b/16bits ,Damping radio  */

    ecrt_slave_config_sdo8(  sc, 0x1600, 0, rpdo_entries_size); /* set number of PDO entries for 0x1600 */
    ecrt_slave_config_sdo16( sc, 0x1C12, 1, 0x1600 ); /* list all RxPdo in 0x1C12:1-4 */
    ecrt_slave_config_sdo8(  sc, 0x1C12, 0, 1 ); /* set number of RxPDO */

    //配置TPDO
    ecrt_slave_config_sdo8( sc, 0x1C13, 0, 0 ); /* clear sm pdo 0x1c13 */
    ecrt_slave_config_sdo8( sc, 0x1A00, 0, 0 ); /* clear TxPdo 0x1A00 */
    ecrt_slave_config_sdo8( sc, 0x1A01, 0, 0 ); /* clear TxPdo 0x1A01 */
    ecrt_slave_config_sdo8( sc, 0x1A02, 0, 0 ); /* clear TxPdo 0x1A02 */
    ecrt_slave_config_sdo8( sc, 0x1A03, 0, 0 ); /* clear TxPdo 0x1A03 */
    /* Define TxPdo */
    ecrt_slave_config_sdo32( sc, 0x1A00, 1, 0x60410010 ); /* 0x6041:0/16bits, status word */
    ecrt_slave_config_sdo32( sc, 0x1A00, 2, 0x60610010 ); /* 0x6061:0/8bits,  mode of operation display*/
    ecrt_slave_config_sdo32( sc, 0x1A00, 3, 0x60640020 );  /* 0x6064:0/32bits, act position */
    ecrt_slave_config_sdo32( sc, 0x1A00, 4, 0x606C0020 );  /* 0x606c:0/32bits, act velocity */
    
    //双编码器使用
    ecrt_slave_config_sdo32( sc, 0x1A00, 5, 0x21110220 );  /* 0x606c:0/32bits, encoder1 pos */
    ecrt_slave_config_sdo32( sc, 0x1A00, 6, 0x60770010 );  /* 0x6077:0/16bits, torque act value */
    ecrt_slave_config_sdo32( sc, 0x1A00, 7, 0x20100a10 );  /* 2010:0a/316bits, setting time */
    ecrt_slave_config_sdo32( sc, 0x1A00, 8, 0x20100b10 );  /* 0x2010:0b/16bits, damping ratio */

    ecrt_slave_config_sdo8(  sc, 0x1A00, 0, tpdo_entries_size ); /* set number of PDO entries for 0x1A00 */
    ecrt_slave_config_sdo16( sc, 0x1C13, 1, 0x1A00 ); /* list all TxPdo in 0x1C13:1-4 */
    ecrt_slave_config_sdo8(  sc, 0x1C13, 0, 1 ); /* set number of TxPDO */
   #endif
    //ecrt_slave_config_sdo8( sc, 0x6060, 0, 9);

    if (ecrt_slave_config_pdos(sc, EC_END, servo_syncs))
    {
        printf("Failed to configure servo PDOs!\n");
    }
    else
    {
        printf("*Success to configuring servo PDOs*\n");
    }

    ppdos->ctrl_word = ecrt_slave_config_reg_pdo_entry(sc,0x6040, 0, domain1, NULL);
    ppdos->operation_mode = ecrt_slave_config_reg_pdo_entry(sc,0x6060, 0, domain1, NULL);
    ppdos->Target_position = ecrt_slave_config_reg_pdo_entry(sc,0x607A, 0, domain1, NULL);
    ppdos->target_velocity = ecrt_slave_config_reg_pdo_entry(sc,0x60FF, 0, domain1, NULL);
    ppdos->torque_offset  =  ecrt_slave_config_reg_pdo_entry(sc,0x60B2, 0, domain1, NULL);

    ppdos->status_word = ecrt_slave_config_reg_pdo_entry(sc,0x6041, 0, domain1, NULL);
    ppdos->mode_display = ecrt_slave_config_reg_pdo_entry(sc,0x6061, 0, domain1, NULL);
    ppdos->Ext_Pos_value = ecrt_slave_config_reg_pdo_entry(sc,0x6064, 0, domain1, NULL);
    ppdos->velocity_value = ecrt_slave_config_reg_pdo_entry(sc,0x606c, 0, domain1, NULL);
    ppdos->Int_Pos_value = ecrt_slave_config_reg_pdo_entry(sc,0x2111, 2, domain1, NULL);
    ppdos->torque_act_value = ecrt_slave_config_reg_pdo_entry(sc,0x6077, 0, domain1, NULL);
    ppdos->torque_demand = ecrt_slave_config_reg_pdo_entry(sc,0x6074, 0, domain1, NULL);
    
    //ppdos->setting_time_now   =  ecrt_slave_config_reg_pdo_entry(sc,0x2010, 0x0a, domain1, NULL);
    //ppdos->damping_ratio_now   =  ecrt_slave_config_reg_pdo_entry(sc,0x2010, 0x0b, domain1, NULL);
}

static void   confEC3045Pdos(ARMS_MASTER_THREAD_INFO *pTModule,ec_slave_info_t *inf,ec_slave_config_t *sc,int EK1100Index)
{
    ec_pdo_entry_info_t slave_9_pdo_entries[] = {
        {0x7000, 0x01, 1}, /* Switch_Out 0 */
        {0x7000, 0x02, 1}, /* Switch_Out 1 */
        {0x7000, 0x03, 1}, /* Switch_Out 2 */
        {0x7000, 0x04, 1}, /* Switch_Out 3 */
        {0x7000, 0x05, 1}, /* Switch_Out 4 */
        {0x7000, 0x06, 1}, /* Switch_Out 5 */
        {0x7000, 0x07, 1}, /* Switch_Out 6 */
        {0x7000, 0x08, 1}, /* Switch_Out 7 */
        {0x0000, 0x00, 8}, /* Gap */
        {0x7001, 0x01, 16}, /* Ao_Out 0 */
        {0x7001, 0x02, 16}, /* Ao_Out 1 */
        {0x7001, 0x03, 16}, /* Ao_Out 2 */
        {0x7001, 0x04, 16}, /* Ao_Out 3 */
        {0x6000, 0x01, 1}, /* Switch_In 0 */
        {0x6000, 0x02, 1}, /* Switch_In 1 */
        {0x6000, 0x03, 1}, /* Switch_In 2 */
        {0x6000, 0x04, 1}, /* Switch 3 */
        {0x6000, 0x05, 1}, /* Switch_In 4 */
        {0x6000, 0x06, 1}, /* Switch_In 5 */
        {0x6000, 0x07, 1}, /* Switch_In 6 */
        {0x6000, 0x08, 1}, /* Switch_In 7 */
        {0x0000, 0x00, 8}, /* Gap */
        {0x6001, 0x01, 16}, /* Ain_In 0 */
        {0x6001, 0x02, 16}, /* Ain_In 1 */
        {0x6001, 0x03, 16}, /* Ain_In 2 */
        {0x6001, 0x04, 16}, /* Ain 3 */
    };

    ec_pdo_info_t slave_9_pdos[] = {
        {0x1600, 9, slave_9_pdo_entries + 0}, /* DO RxPDO-Map */
        {0x1601, 4, slave_9_pdo_entries + 9}, /* AO RxPDO-Map */
        {0x1a00, 9, slave_9_pdo_entries + 13}, /* DI TxPDO-Map */
        {0x1a01, 4, slave_9_pdo_entries + 22}, /* AI TxPDO-Map */
    };

    ec_sync_info_t slave_9_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 2, slave_9_pdos + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 2, slave_9_pdos + 2, EC_WD_DISABLE},
        {0xff}
    };

    if (ecrt_slave_config_pdos(sc, EC_END, slave_9_syncs))
    {
        printf("Failed to configure EC-3045 PDOs!\n");
    }
    else
    {
        printf("*Success to configuring EC-3045 PDOs*\n");
    }

    int iRet = -1;
    //开关量输出PDO偏移
    pTModule->EC_3045[EK1100Index][0] = ecrt_slave_config_reg_pdo_entry(sc,0x7000, 0x01, pTModule->domain1, NULL);
   
    //模拟量输出PDO偏移量
    pTModule->EC_3045[EK1100Index][1] = ecrt_slave_config_reg_pdo_entry(sc,0x7001, 0x01, pTModule->domain1, NULL);
    pTModule->EC_3045[EK1100Index][2] = ecrt_slave_config_reg_pdo_entry(sc,0x7001, 0x02, pTModule->domain1, NULL);
    pTModule->EC_3045[EK1100Index][3] = ecrt_slave_config_reg_pdo_entry(sc,0x7001, 0x03, pTModule->domain1, NULL);
    pTModule->EC_3045[EK1100Index][4] = ecrt_slave_config_reg_pdo_entry(sc,0x7001, 0x04, pTModule->domain1, NULL);

    //开关量输入PDO偏移量
    pTModule->EC_3045[EK1100Index][5] = ecrt_slave_config_reg_pdo_entry(sc,0x6000, 0x01, pTModule->domain1, NULL);
  
    //模拟量输入PDO偏移量
    pTModule->EC_3045[EK1100Index][6] = ecrt_slave_config_reg_pdo_entry(sc,0x6001, 0x01, pTModule->domain1, NULL);
    pTModule->EC_3045[EK1100Index][7] = ecrt_slave_config_reg_pdo_entry(sc,0x6001, 0x02, pTModule->domain1, NULL);
    pTModule->EC_3045[EK1100Index][8] = ecrt_slave_config_reg_pdo_entry(sc,0x6001, 0x03, pTModule->domain1, NULL);
    pTModule->EC_3045[EK1100Index][9] = ecrt_slave_config_reg_pdo_entry(sc,0x6001, 0x04, pTModule->domain1, NULL);

}

static int  confSlavesPdos(ARMS_MASTER_THREAD_INFO *pTModule)
{
    if(pTModule->armsSize<1 || pTModule->armsSize > 7)
    {
      printf("confSlavesPdos:机械臂不存在\n\r");
      return -1;
    }

    //遍历从站，找到所有伺服电机，并通过sdo配置所使用的pdo
    int somanetIndex=0;
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        EK1100_slaves_inf *inf_t = NULL ;
        list_for_each_entry(inf_t, &pTModule->slaves_inf[i].list, list)
        {
            //创建从站时候一定不要使用alias，使用0 ，position的绝对位置测试
            ec_slave_config_t * ec_sla_con = ecrt_master_slave_config(
                                                        pTModule->master, 
                                                        0,
                                                        inf_t->slave_in.position, 
                                                        inf_t->slave_in.vendor_id,
                                                        inf_t->slave_in.product_code);

            if(ec_sla_con==NULL)
            {
               printf("ecrt_master_slave_config ********* NULL\n");
               return -1;
            }
            inf_t->slave_conf = ec_sla_con;
            //如果是EK1100节点的话就赋值后continue
            if(strstr(inf_t->slave_in.name,"EK1100") != NULL)
            {
                somanetIndex=0;
            }
  
            //机械臂的somanet结点，
            if(strstr(inf_t->slave_in.name,"SOMANET") != NULL)
            {
                if(pTModule->somanetn_mode == MODE_CSV)
                {
                    confSomanetCSVPdos(pTModule->domain1, &inf_t->slave_in, ec_sla_con, &pTModule->somanet_pdo[i][somanetIndex]);
                }
                somanetIndex++;
            }

            //机械臂的EC-3045结点，
            if(strstr(inf_t->slave_in.name,"EC-3045") != NULL)
            {   
                printf("EC-3045 EC-3045\n");
                confEC3045Pdos(pTModule, &inf_t->slave_in, ec_sla_con, i);
            }
        }
    }
    
    printf("###########################\n");
    return 0;
}

static void  clearErrorSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg)
{
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mSMsg->mModuleFlag[i])
        {
            //速度清零
            pTModule->motors[i][0].target_velocity = 0;
            pTModule->motors[i][1].target_velocity = 0;
            pTModule->motors[i][2].target_velocity = 0;
            pTModule->motors[i][3].target_velocity = 0;
            //控制字设置清除错误
            pTModule->motors[i][0].ctrl_word = 0x80;
            pTModule->motors[i][1].ctrl_word = 0x80;
            pTModule->motors[i][2].ctrl_word = 0x80;
            pTModule->motors[i][3].ctrl_word = 0x80;
        }
    }
}

static int32_t handleSetExtXPos(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
           //设置目标压力
           armsSetExtXPos(pTModule,i);
        }
    } 
}

static int32_t handleStartSlight(ARMS_MASTER_THREAD_INFO *pTModule,int armsid,float xmv,float xsv,float yv,float zv,float acc_mm)
{
    pTModule->upper_target_v_Flag[armsid] = 0;
    //轴安装顺序 Y XM XS Z
    pTModule->upper_target_velocity[armsid][0] = xmv; 
    pTModule->upper_target_velocity[armsid][1] = xsv;
    pTModule->upper_target_velocity[armsid][2] = yv;
    pTModule->upper_target_velocity[armsid][3] = zv;

    //xm xs y z
    pTModule->upper_velocity_v0[armsid][0] = pTModule->xMIntSpeed[armsid];
    pTModule->upper_velocity_v0[armsid][1] = pTModule->xSIntSpeed[armsid];
    pTModule->upper_velocity_v0[armsid][2] = pTModule->ySpeed[armsid];
    pTModule->upper_velocity_v0[armsid][3] = pTModule->zSpeed[armsid];
    //200hz 调用周期，加减速1s
    for (int i = 0; i < 4; i++)
    {
        if(pTModule->upper_target_velocity[armsid][i] > pTModule->upper_velocity_v0[armsid][i])
            pTModule->upper_acc[armsid][i] = acc_mm;
        else if(pTModule->upper_target_velocity[armsid][i] < pTModule->upper_velocity_v0[armsid][i])
            pTModule->upper_acc[armsid][i] = -acc_mm;
        else
            pTModule->upper_acc[armsid][i] = 0;

        pTModule->upper_last_v_cmd[armsid][i] = pTModule->upper_velocity_v0[armsid][i];
    }
    pTModule->upper_target_v_Flag[armsid] = 1;
}

static void  upperAngularVelSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg)
{
    memset(pTModule->upper_target_v_Flag,0,sizeof(uint8_t)*MASTER_ARMS_MAX_SIZE);
    float *pVel = (float*)mSMsg->mData;
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mSMsg->mModuleFlag[i])
        {
            //轴安装顺序 Y XM XS Z,加速度200mm/s2
            handleStartSlight(pTModule,i,pVel[0],pVel[1],pVel[2],pVel[3],200);
        }
    }
}

static void  upperRelaPosSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg)
{
    #if 1
    //使用CSV速度模式 进行位置控制
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mSMsg->mModuleFlag[i])
        {
            //轴安装顺序 Y XM XS Z
            float *pRpos = (float*)mSMsg->mData;
            pTModule->upper_target_pos[i][0] = pTModule->xMIntPos[i] + pRpos[0]/1000.0;
            pTModule->upper_target_pos[i][1] = pTModule->xSIntPos[i] + pRpos[1]/1000.0;
            pTModule->upper_target_pos[i][2] = pTModule->yPos[i] +     pRpos[2]/1000.0;
            pTModule->upper_target_pos[i][3] = pTModule->zPos[i] +     pRpos[3]/1000.0;
            
            //位置控制的pd算法控制命令（速度），进行滤波平滑
            for (int  j = 0; j < 4; j++)
            {
                for (int k = 0; k < upper_pos_smooth_cnt; k++)
                {
                    pTModule->upper_pos_vel_smooth[i][j][k] = 0;
                } 
                pTModule->target_p_last_cmdvel[i][j] = 0;
            }
            pTModule->upper_target_p_Flag[i] = 1;
            pTModule->upper_target_p_Kp_cnt[i] = 0;

        }
    }
    #endif

    #if 0
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mSMsg->mModuleFlag[i])
        {
            //轴安装顺序 Y XM XS Z.pRpos传输顺序：XM XS Y Z
            float * pRpos= (float*)mSMsg->mData;     
            if(fabs(pRpos[0]) > 1e-7)
            {
                pTModule->motors[i][1].Target_position = (int32_t)(((double)pRpos[0]/1000.0)/INT_X_PULSE_TO_M)*pTModule->AXIS_DIR[i][1];
                if(pTModule->pp_run_state[i][1] == PP_NULL)
                {
                    pTModule->pp_run_state[i][1] = PP_NEW_POS;
                }
            }       
            
            if(fabs(pRpos[1]) > 1e-7)
            {
                pTModule->motors[i][2].Target_position = (int32_t)(((double)pRpos[1]/1000.0)/INT_X_PULSE_TO_M)*pTModule->AXIS_DIR[i][2];
                if(pTModule->pp_run_state[i][2] == PP_NULL)
                {
                    pTModule->pp_run_state[i][2] = PP_NEW_POS;
                }
            } 

            if(fabs(pRpos[2]) > 1e-7)
            {
                pTModule->motors[i][0].Target_position = (int32_t)(((double)pRpos[2]/1000.0)/INT_X_PULSE_TO_M)*pTModule->AXIS_DIR[i][0];
                if(pTModule->pp_run_state[i][0] == PP_NULL)
                {
                    pTModule->pp_run_state[i][0] = PP_NEW_POS;
                }
            } 

            if(fabs(pRpos[3]) > 1e-7)
            {
                pTModule->motors[i][3].Target_position = (int32_t)(((double)pRpos[3]/1000.0)/INT_X_PULSE_TO_M)*pTModule->AXIS_DIR[i][3];
                if(pTModule->pp_run_state[i][3] == PP_NULL)
                {
                    pTModule->pp_run_state[i][3] = PP_NEW_POS;

                    pTModule->motors[i][3].profile_velocit = 50;
                    pTModule->motors[i][3].profile_acceleration = 50;
                    pTModule->motors[i][3].profile_deceleration = 50;
                }
            } 
        }
    }
    #endif

}

static void  upperAbsPosSomanets(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg)
{
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mSMsg->mModuleFlag[i])
        {
            //轴安装顺序 Y XM XS Z.ABS绝对位置单位m
            float *pRpos = (float*)mSMsg->mData;           
            //位置控制的pd算法控制命令（速度），进行滤波平滑
            for (int  j = 0; j < 4; j++)
            {
                pTModule->upper_target_pos[i][j] = pRpos[i*4+j];
                for (int k = 0; k < upper_pos_smooth_cnt; k++)
                {
                    pTModule->upper_pos_vel_smooth[i][j][k] = 0;
                } 
                pTModule->target_p_last_cmdvel[i][j] = 0;
            }
            pTModule->upper_target_p_Flag[i] = 1;
            pTModule->upper_target_p_Kp_cnt[i] = 0;
        }
    }
}

static void checkFollowMSPosError(ARMS_MASTER_THREAD_INFO *pTModule)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            MAGIC_THREAD_INFO *pMagic = &pTModule->mMagicWorker[i];
            switch (pMagic->mRunState)
            {
                case R_FOLLOW:
                case R_FOLLOW_VERTICAL:
                {
                    //当前主辅轴位置偏差
                    float x_intpos_now_error = pTModule->xMIntPos[i] - pTModule->xSIntPos[i];
                    if(fabs(x_intpos_now_error-pTModule->mMagicWorker[i].x_intpos_start_error) > 0.0003)
                    {
                        printf("******X主副轴位置偏差过大:(%f %f) error:%f \n",pMagic->x_intpos_start_error,x_intpos_now_error,x_intpos_now_error-pMagic->x_intpos_start_error);
                        pMagic->target_vel_i = 0;
                        memset(pMagic->target_vel_x_inters,0,sizeof(float)*100);
                        memset(pMagic->target_vel_y_inters,0,sizeof(float)*100);
                        sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_ALL_STOP,0,0,0,0);
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
}

static void  check_arms_near(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //OP模式下才有效.判断单元之间突破最小距离时候，立即停止
    if(pTModule->al_states != AL_OP)
        return ;
    //down_up:1 下层，down_up:上层
    float minL = 0.55;  //设置单元之间最小接近距离
    bool isnear = false;
/**************************************************判断下层****************************************************/
    for (int up_d = 0; up_d < 2; up_d++)
    {
        int16_t f_id=pTModule->first_id[up_d];
        int16_t l_id=pTModule->last_id[up_d];
        //判断下层第一个和最后一个是否触碰边界
        float x0_left_limt = pTModule->mAparams[f_id].x_soft_left_limi;
        if((pTModule->xMIntPos[f_id]< x0_left_limt) || (pTModule->xSIntPos[f_id]< x0_left_limt))
        {
            if(pTModule->xMIntSpeed[f_id]<0 || pTModule->xSIntSpeed[f_id]<0)
            {
                isnear = true;
            }
        }
        float xn_right_limt = pTModule->mAparams[l_id].x_soft_right_limi;
        if((pTModule->xMIntPos[l_id] > xn_right_limt) || (pTModule->xSIntPos[l_id] > xn_right_limt))
        {
            if(pTModule->xMIntSpeed[l_id]>0 || pTModule->xSIntSpeed[l_id]>0)
            {
                isnear = true;
            }
        }
        //判断同层之间是否触碰软线
        for (int i = f_id; i < l_id; i++)
        {
            //前一个的右和后一个的左判断是否触碰软线
            if( 
                (fabs(pTModule->xMIntPos[i]-pTModule->xMIntPos[i+1]) < minL) || 
                (fabs(pTModule->xSIntPos[i]-pTModule->xSIntPos[i+1]) < minL)
            )
            {
                if(pTModule->xMIntSpeed[i]>0 || pTModule->xSIntSpeed[i]>0)
                {
                    isnear = true;
                }

                if(pTModule->xMIntSpeed[i+1]<0 || pTModule->xSIntSpeed[i+1]<0)
                {
                    isnear = true;
                }
            }
        }

        //判断同层Y轴是否触碰软线
        for (int i = f_id; i <= l_id; i++)
        {
            float y_front_limt = pTModule->mAparams[i].y_soft_front_limit;
            if(pTModule->yPos[i] > y_front_limt)
            {
                if(pTModule->ySpeed[i]>0)
                {
                    isnear = true;
                }
            }

            float y_back_limt = pTModule->mAparams[i].y_soft_back_limit;
            if(pTModule->yPos[i] < y_back_limt)
            {
                if(pTModule->ySpeed[i]<0)
                {
                    isnear = true;
                }
            }
        }
    }


    /***********************************判断上下层是否干涉***********************************/
    int16_t f_down_id=pTModule->first_id[0];
    int16_t l_down_id=pTModule->last_id[0];
    int16_t f_up_id=pTModule->first_id[1];
    int16_t l_up_id=pTModule->last_id[2];

    for (int i = f_down_id; i <= l_down_id; i++)
    {
        for (int j = f_up_id; j <= l_up_id; j++)
        {

            if( pTModule->yPos[i] > pTModule->yPos[j] && 
                ( fabs(pTModule->xMIntPos[i]-pTModule->xMIntPos[j]) < minL || fabs(pTModule->xSIntPos[i]-pTModule->xSIntPos[j])  < minL)
            )
            {
                if( (pTModule->xMIntSpeed[i]>0 && pTModule->xMIntSpeed[j]<0) || 
                    (pTModule->xMIntSpeed[i]<0 && pTModule->xMIntSpeed[j]>0) ||
                    (pTModule->xSIntSpeed[i]>0 && pTModule->xSIntSpeed[j]<0) || 
                    (pTModule->xSIntSpeed[i]<0 && pTModule->xSIntSpeed[j]>0)
                )
                {
                    //isnear = true;
                    printf("(%f %f) (%f %f)\n",pTModule->yPos[i] , pTModule->yPos[j],pTModule->xMIntPos[i],pTModule->xMIntPos[j]);
                }

            }
        }
    }

    if(isnear)
    {
        sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_ALL_STOP,0,0,0,0);
    }
        
}

static void  check_limit_switch(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //OP模式下才有效
    if(pTModule->al_states == AL_OP)
    {
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
        {
            /**********************************限位开关检测**********************************/
            //第一位 主轴左边 第二位主轴后边 第三位主轴右边
            //第四位 副轴左边 第五位副轴前边 第六位副轴右边
            if(pTModule->slaves_inf[i].childSlaveSize>4)
            {
                //主轴左限位处理
                uint8_t s_code = pTModule->EC_3045_switch[i];
                //主轴左限位处理
                if( ((~s_code)&0b00000001)!=0 && pTModule->xMIntSpeed[i]<0)
                {
                    pTModule->motors[i][1].target_velocity = 0;
                    pTModule->motors[i][2].target_velocity = 0;
                }        
                //副轴左限位处理
                if( ((~s_code)&0b00001000)!=0 && pTModule->xSIntSpeed[i]<0)
                {
                    pTModule->motors[i][1].target_velocity = 0;
                    pTModule->motors[i][2].target_velocity = 0;
                }

                //主轴右限位处理
                if( ((~s_code)&0b00000100)!=0 && pTModule->xMIntSpeed[i]>0)  
                {       
                    pTModule->motors[i][1].target_velocity = 0;
                    pTModule->motors[i][2].target_velocity = 0;
                }
                //副轴右限位处理
                if( ((~s_code)&0b00100000)!=0 && pTModule->xSIntSpeed[i]>0)
                {
                    pTModule->motors[i][1].target_velocity = 0;
                    pTModule->motors[i][2].target_velocity = 0;
                }

                //Y轴前限位处理，主轴后边开关
                if( ((~s_code)&0b00000010)!=0 && pTModule->ySpeed[i]>0)    
                {
                    pTModule->motors[i][0].target_velocity = 0;
                }      
                //Y轴后限位处理，副轴前边开关
                if( ((~s_code)&0b00010000)!=0 && pTModule->ySpeed[i]<0)
                {
                    pTModule->motors[i][0].target_velocity = 0;
                }
            }
            /**********************************限位开关检测 end**********************************/
        }
    }
}
static void  upperEC3045Proportions(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mSMsg)
{
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mSMsg->mModuleFlag[i])
        {
            float *pProCmd = (float*)mSMsg->mData;
            pTModule->proportion_cmd[i][0] = pProCmd[0];
            pTModule->proportion_cmd[i][1] = pProCmd[1];
            pTModule->proportion_cmd[i][2] = pProCmd[2];
            pTModule->proportion_cmd[i][3] = pProCmd[3];
        }
    }
}

static void  upperFSASomanets(ARMS_MASTER_THREAD_INFO *pTModule,S_ETHR_STATE fsastate)
{
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        pTModule->arm_fsa[i] = fsastate;
    }
}

static int  activeMater(ARMS_MASTER_THREAD_INFO *pTModule)
{
    int iRet = -1;
    if(pTModule->slaves_size<1)
      return iRet;
    //设置DC时钟
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        EK1100_slaves_inf *inf_t = NULL;
        list_for_each_entry(inf_t, &pTModule->slaves_inf[i].list, list)
        {
             //电机使用dc
            if(strstr(inf_t->slave_in.name,"SOMANET") != NULL)
            {
                ecrt_slave_config_dc(inf_t->slave_conf, 0x0700, 1000000, 500000, 0, 0);
            }       
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


static void  activeMaterKeydown(ARMS_MASTER_THREAD_INFO *pTModule)
{
    if(createMaster(pTModule->masterId,&pTModule->master,&pTModule->domain1) != 0)
        return;

    if(scanSlaves(pTModule) != 0)
        return;

    if(confSlavesPdos(pTModule) != 0)
        return;

    if(activeMater(pTModule) != 0)
        return;
}

static void  cyc_vel_upper_cmd(ARMS_MASTER_THREAD_INFO *pTModule)
{
    /**
     * 200HZ 运行周期
     */
    
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(pTModule->upper_target_v_Flag[i])
        {
            //V_cmd = V_now + a*dt
            float cmdSpeed[4];
            int cmd_target_ok = 0;
            for (int j = 0; j < 4; j++)
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
                }else if(pTModule->upper_acc[i][j] == 0.0)
                {
                    cmd_target_ok++;
                }
            }

            //判断是否都达到目标速度
            if(
                fabs(cmdSpeed[0]-pTModule->upper_target_velocity[i][0]) < 0.01 &&
                fabs(cmdSpeed[1]-pTModule->upper_target_velocity[i][1]) < 0.01 &&
                fabs(cmdSpeed[2]-pTModule->upper_target_velocity[i][2]) < 0.01 && 
                fabs(cmdSpeed[3]-pTModule->upper_target_velocity[i][3]) < 0.01 &&
                cmd_target_ok >= 4
            )
            {
                pTModule->upper_target_v_Flag[i] = 0;
                cmdSpeed[0] = pTModule->upper_target_velocity[i][0];
                cmdSpeed[1] = pTModule->upper_target_velocity[i][1];
                cmdSpeed[2] = pTModule->upper_target_velocity[i][2];
                cmdSpeed[3] = pTModule->upper_target_velocity[i][3];
            }

            for (int j = 0; j < 4; j++)
            {
                pTModule->upper_last_v_cmd[i][j] = cmdSpeed[j];
            }
            armsSetSpeed(pTModule,i,cmdSpeed[0],cmdSpeed[1],cmdSpeed[2],cmdSpeed[3]);

            
            
        }
        
        
    }
    
}

static void  cyc_pos_upper_cmd(ARMS_MASTER_THREAD_INFO *pTModule)
{
    /**
     * 200HZ 运行周期
     */
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++)
    {
        if(pTModule->upper_target_p_Flag[i])
        {
            //V_cmd = Kp*(S_obj-S_now) + Kd(^S_obj-^S_now)
            float cmdSpeed[4];
            float kp = ((float)pTModule->upper_target_p_Kp_cnt[i])/2.0;
            float kd = ((float)pTModule->upper_target_p_Kp_cnt[i])/20.0;
            if(kp>500)
            {
                kp = 500;
            }
            if(kd>50)
            {
                kd = 50;
            }
            pTModule->upper_target_p_Kp_cnt[i]++;

            float error[4];
            error[0] = pTModule->upper_target_pos[i][0]-pTModule->xMIntPos[i];
            error[1] = pTModule->upper_target_pos[i][1]-pTModule->xSIntPos[i];
            error[2] = pTModule->upper_target_pos[i][2]-pTModule->yPos[i];
            error[3] = pTModule->upper_target_pos[i][3]-pTModule->zPos[i];

            cmdSpeed[0]=kp*(error[0])-kd*(pTModule->xMIntSpeed[i]/1000.0);
            cmdSpeed[1]=kp*(error[1])-kd*(pTModule->xSIntSpeed[i]/1000.0);
            cmdSpeed[2]=kp*(error[2])    -kd*(pTModule->ySpeed[i]/1000.0);
            cmdSpeed[3]=kp*(error[3])    -kd*(pTModule->zSpeed[i]/1000.0);
           
            //printf("%f %f %f %f \n",error[0],cmdSpeed[0],error[1],cmdSpeed[1]);
            /*
            for (int j = 0; j < 4; j++)
            {
                float max_acc = 20;
                //起始阶段限制加速度.
                float new_cmd_acc = (cmdSpeed[j] - pTModule->target_p_last_cmdvel[i][j])/0.005;
                if(new_cmd_acc > max_acc)
                {
                    cmdSpeed[j] = pTModule->target_p_last_cmdvel[i][j] + max_acc*0.005;
                }
                if(new_cmd_acc < -max_acc)
                {
                    cmdSpeed[j] = pTModule->target_p_last_cmdvel[i][j] - max_acc*0.005;
                }
                pTModule->target_p_last_cmdvel[i][j] = cmdSpeed[j];
            }
            */
            #if 1
            //速度命令平滑处理
            for (int  j = 0; j < 4; j++)
            {
                for (int k = 0; k < upper_pos_smooth_cnt-1; k++)
                {
                    pTModule->upper_pos_vel_smooth[i][j][k] = pTModule->upper_pos_vel_smooth[i][j][k+1];
                } 
                pTModule->upper_pos_vel_smooth[i][j][upper_pos_smooth_cnt-1] = cmdSpeed[j];

                float smoothV = 0;
                for (int k = 0; k < upper_pos_smooth_cnt; k++)
                {
                    smoothV += pTModule->upper_pos_vel_smooth[i][j][k];
                } 
                cmdSpeed[j] = smoothV/upper_pos_smooth_cnt;
            }
            #endif
            //判断是否都达到目标速度
            if(fabs(error[0]) < 0.0002)
                cmdSpeed[0] = 0;
            if(fabs(error[1]) < 0.0002)
                cmdSpeed[1] = 0;
            if(fabs(error[2]) < 0.0002)
                cmdSpeed[2] = 0;
            if(fabs(error[3]) < 0.0002)
                cmdSpeed[3] = 0;

            //限制速度
            for (int j = 0; j < 4; j++)
            {
                if(cmdSpeed[j]>50)
                    cmdSpeed[j] = 50;
                if(cmdSpeed[j]<-50)
                    cmdSpeed[j] = -50;
            }
            armsSetSpeed(pTModule,i,cmdSpeed[0],cmdSpeed[1],cmdSpeed[2],cmdSpeed[3]);
            //printf("%f %f %f %f\n",cmdSpeed[0],cmdSpeed[1], cmdSpeed[2],cmdSpeed[3]);
            //到达位置
            if(
                fabs(pTModule->xMIntPos[i]-pTModule->upper_target_pos[i][0]) < 0.0002 &&
                fabs(pTModule->xSIntPos[i]-pTModule->upper_target_pos[i][1]) < 0.0002 &&
                fabs(pTModule->yPos[i]-pTModule->upper_target_pos[i][2]) < 0.0002 &&
                fabs(pTModule->zPos[i]-pTModule->upper_target_pos[i][3]) < 0.0002
            )
            {
                pTModule->upper_target_p_Flag[i] = 0;
                pTModule->upper_target_p_Kp_cnt[i] = 0;
                sendNoticeToUpper(pTModule,pTModule->mIdentifier,i,ACK_NOTICE_HAND_POS_ARRIVE);
            }
        }
    }
}


/*** 
 * @description: 
 * @Author: lyq
 * @Date: 2024-06-04 01:31:37
 * @return {*}
 */
static void LoopRecLocalMsg(ARMS_MASTER_THREAD_INFO *pTModule)
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
            handle_module_exit(pTModule);
            break;
        }
        case CMD_IGH_CREATE_MASTERS:
        {
            createMaster(pTModule->masterId,&pTModule->master,&pTModule->domain1);
            break;
        }
        case CMD_DESTROY_MASTER:
        {
            if(pTModule->master != NULL)
            {
                    destroyMaster(pTModule);
                    //关闭视觉系统
                    pTModule->mVisionWorker.mState = SR_CLOSE;     
            }
            break;
        }
        case CMD_IGH_SCAN_SLAVES:
        {
            scanSlaves(pTModule);
            break;
        }
        case CMD_IGH_CONF_SERVO_PDOS:
        {
            confSlavesPdos(pTModule);
            break;
        }
        case CMD_IGH_START_OP:
        {
            activeMater(pTModule);
            break;
        }
        case CMD_MASTER_OP_KEYDOWN:
        {
            //只能使用速度模式，因为驱动器pos设置的是外部编码器
            pTModule->somanetn_mode = MODE_CSV;
            activeMaterKeydown(pTModule);
            //打开拉力计串口
            pTModule->mVisionWorker.mState = SR_OPEN;
            break;
        }
        case CMD_IGH_CLEAR_ERROR:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            clearErrorSomanets(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_HAND_VEL_MM:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            memset(pTModule->upper_target_p_Flag,0,sizeof(uint8_t)*MASTER_ARMS_MAX_SIZE);
            upperAngularVelSomanets(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            
        
            break;
        }
        case CMD_HAND_RELA_POS_MM:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            upperRelaPosSomanets(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_HAND_ABS_POS_M:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            upperAbsPosSomanets(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_EC3045_PROPORTION:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            upperEC3045Proportions(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_AUTO_VERTICAL_WEIGHT:
        {
            //称重算法
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            magicStateChange(pTModule,(SUPR_R_MSG *)mLMsg->mData,R_VERTICAL_WEIGHT);
            break;
        }
        case CMD_AUTO_FOLLOW:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            magicStateChange(pTModule,(SUPR_R_MSG *)mLMsg->mData,R_FOLLOW);
            break;
        }
        case CMD_IGH_X_EXT_ABS_POS:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            handleSetExtXPos(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_AUTO_VERTICAL:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            magicStateChange(pTModule,(SUPR_R_MSG *)mLMsg->mData,R_VERTICAL);
            break;
        }
        case CMD_AUTO_FOLLOW_VERTICAL:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            magicStateChange(pTModule,(SUPR_R_MSG *)mLMsg->mData,R_FOLLOW_VERTICAL);
            break;
        }
        case CMD_ALL_STOP:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            magicStateChange(pTModule,(SUPR_R_MSG *)mLMsg->mData,R_STOP);
            break;
        }
        case CMD_IGH_SIKO_ZERO:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            handleSikoZero(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_IGH_CTRL_WORD:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            SUPR_R_MSG * pms = (SUPR_R_MSG *)mLMsg->mData;
            pTModule->motors[0][3].ctrl_word = *((uint16_t*)pms->mData);
            break;
        }
        case CMD_IGH_FSA_SHUTDOWN:
        {
            IsSystemError = true;
            upperFSASomanets(pTModule,S_SHUTDOWN);

            break;
        }
        case CMD_IGH_FSA_SWITH_ON:
        {
            if (IsSystemError) {
                //packM_S_err_MSG(pTModule);
            }

            IsSystemError = false;

            upperFSASomanets(pTModule,S_SWITCH_ON);
            break;
        }
        case CMD_IGH_FSA_ENOP:
        {
            IsSystemError = false;
            upperFSASomanets(pTModule,S_ENABLE_OP);
            break;
        }
        case CMD_IGH_FSA_HALT:
        {
            //6.16 -zjy
            IsSystemError = true;
            
            upperFSASomanets(pTModule,S_HAIT);

            /*
            //6.16 -zjy
            printf("出错急停前电池的剩余电量为：%d \n", info_show_in_err.Bat_Remaining[1]);
            printf("出错急停前高压气压为：%f \n", info_show_in_err.gas_press_high[1]);

            for (int i = 0; i < pTModule->armsSize; i++)
            {
               
                if(pTModule->slaves_inf[i].childSlaveSize>4)
                {
                    printf("出错急停前正在运行的 %d 号机械臂 xM位置：%f, xS位置：%f, y位置：%f, z位置：%f, xM速度：%f, xS速度：%f, y速度：%f, z速度：%f\n", i+1, 
                        info_show_in_err.somanet_inf[i][0].Int_Pos_value,
                        info_show_in_err.somanet_inf[i][1].Int_Pos_value,
                        info_show_in_err.somanet_inf[i][2].Int_Pos_value,
                        info_show_in_err.somanet_inf[i][3].Int_Pos_value,
                        info_show_in_err.somanet_inf[i][0].Int_vel_value,
                        info_show_in_err.somanet_inf[i][1].Int_vel_value,
                        info_show_in_err.somanet_inf[i][2].Int_vel_value,
                        info_show_in_err.somanet_inf[i][3].Int_vel_value
                    );
                }
            }*/

            break;
        }   
        case CMD_VALVE_ARMS24_ON:
        case CMD_VALVE_ARMS24_OFF:
        case CMD_VALVE_ARM_GAS_ON:
        case CMD_VALVE_ARM_GAS_OFF:
        case CMD_VALVE_ARMS48_ON:
        case CMD_VALVE_ARMS48_OFF:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            memcpy(&pTModule->mSerialWorker->upperMsg,mLMsg->mData,sizeof(SUPR_R_MSG));
            pTModule->mSerialWorker->newUpperMsg = true;
            break;
        }  
        case CMD_INTERPN_START_COLLECT:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            handleInterpnStartCollect(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        } 
        case CMD_INTERPN_COLLECT:
        {
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            handleInterpnCollecting(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        } 
        case CMD_INTERPN_COLLECT_OVER:
        { 
            MsgLongMsg *mLMsg = (MsgLongMsg *)mLocalData;
            handleInterpnCollectOver(pTModule,(SUPR_R_MSG *)mLMsg->mData);
            break;
        }
        case CMD_IGH_VERTICAL_LIMIT:
        {
            Is_Zlimit_Open = (mLMsg->mData[0] == 'Y'?true : false);
            // printf("data: %c",mLMsg->mData[0]);
            // Is_Zlimit_Open = true;
        }
        
        default:
        {
            break;
        }
      } //end switch

    }

}

////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void handleInterpnStartCollect(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            int32_t *pI = (int32_t*)mRMsg->mData;
            pTModule->mQuads[i].isValid = 0;
            pTModule->mQuads[i].rows = pI[0];
            pTModule->mQuads[i].cols = pI[1];
            pTModule->mQuads[i].mNewInterpns = (interpn_unit*)malloc(sizeof(interpn_unit)*pI[0]*pI[1]);
        }
    }
}

static void handleInterpnCollecting(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            int32_t *pI = (int32_t*)mRMsg->mData;
            int rows = pTModule->mQuads[i].rows;
            int cols = pTModule->mQuads[i].cols;
            pTModule->mQuads[i].mNewInterpns[pI[1]*cols+pI[0]].xi = pI[0];
            pTModule->mQuads[i].mNewInterpns[pI[1]*cols+pI[0]].yj = pI[1];
            //传感器数据拷贝
            pTModule->mQuads[i].mNewInterpns[pI[1]*cols+pI[0]].xy.x = pTModule->xMIntPos[i];
            pTModule->mQuads[i].mNewInterpns[pI[1]*cols+pI[0]].xy.y = pTModule->yPos[i];
            pTModule->mQuads[i].mNewInterpns[pI[1]*cols+pI[0]].vas.x = pTModule->incln_acc_XY[i][0];
            pTModule->mQuads[i].mNewInterpns[pI[1]*cols+pI[0]].vas.y = pTModule->incln_acc_XY[i][1];

            printf("%d %d %f %f %f %f \n",pI[0],pI[1],pTModule->xMIntPos[i],pTModule->yPos[i],pTModule->incln_acc_XY[i][0],pTModule->incln_acc_XY[i][1]);
        }
    }
}

static void handleInterpnCollectOver(ARMS_MASTER_THREAD_INFO *pTModule,SUPR_R_MSG *mRMsg)
{
    for(int i=0; i<MASTER_ARMS_MAX_SIZE; i++)
    {
        if(mRMsg->mModuleFlag[i])
        {
            int32_t *pI = (int32_t*)mRMsg->mData;
            int rows = pTModule->mQuads[i].rows;
            int cols = pTModule->mQuads[i].cols;
            if((rows-1)==pI[1] && (cols-1)==pI[0])
            {
                //插值表创建
                create_interpn_table(DB_FILE,db_interpn_tables[i]);
                insert_interpn_db(DB_FILE,db_interpn_tables[i],
                                  pTModule->mQuads[i].mNewInterpns,
                                  rows,cols);
                pTModule->mQuads[i].pQus = generate_quads(pTModule->mQuads[i].mNewInterpns,rows,cols);
                pTModule->mQuads[i].isValid = 1;
            }
            if(pTModule->mQuads[i].mNewInterpns != NULL)
            {
                free(pTModule->mQuads[i].mNewInterpns);
                pTModule->mQuads[i].mNewInterpns = NULL;
            }
        }
    }
}

int32_t armsSetSpeed(ARMS_MASTER_THREAD_INFO *pTModule,int armsI,float xMspeed,float xSspeed,float yspeed,float zspeed)
{
    //4个伺服电机、float类型转换成整型的，根据减速比等系数，转换成RPM
    armsSetSpeedXY(pTModule,armsI,xMspeed,xSspeed,yspeed);
    armsSetSpeedZ(pTModule,armsI,zspeed);
}

int32_t armsSetSpeedXY(ARMS_MASTER_THREAD_INFO *pTModule,int armsI,float xMspeed,float xSspeed,float yspeed)
{
    pTModule->motors[armsI][0].target_velocity = (int32_t)(yspeed/Y_RPM_TO_V_MM)*pTModule->AXIS_DIR[armsI][0];
    pTModule->motors[armsI][1].target_velocity = (int32_t)(xMspeed/X_RPM_TO_V_MM)*pTModule->AXIS_DIR[armsI][1];
    pTModule->motors[armsI][2].target_velocity = (int32_t)(xSspeed/X_RPM_TO_V_MM)*pTModule->AXIS_DIR[armsI][2];

    //设置最大限速,RPM
    LIMIT_MAX(pTModule->motors[armsI][0].target_velocity,4500);
    LIMIT_MIN(pTModule->motors[armsI][0].target_velocity,-4500);

    LIMIT_MAX(pTModule->motors[armsI][1].target_velocity,4500);
    LIMIT_MIN(pTModule->motors[armsI][1].target_velocity,-4500);

    LIMIT_MAX(pTModule->motors[armsI][2].target_velocity,4500);
    LIMIT_MIN(pTModule->motors[armsI][3].target_velocity,-4500);
}

int32_t armsSetSpeedZ(ARMS_MASTER_THREAD_INFO *pTModule,int armsI,float zspeed)
{
    int32_t zv = (int32_t)(zspeed/Z_RPM_TO_V_MM)*pTModule->AXIS_DIR[armsI][3];
    //设置最大限速,RPM
    LIMIT_MAX(zv,2900);
    LIMIT_MIN(zv,-2900);
    pTModule->motors[armsI][3].target_velocity = zv;
}

//算法模块出现意外，需要停止算法运行
int32_t magic_stop_i(ARMS_MASTER_THREAD_INFO *pTModule,int armsid)
{
    pTModule->mMagicWorker[armsid].mRunState = R_STOP;
    handleStartSlight(pTModule,armsid,0,0,0,0,200);
}

//打包系统出错前信息的函数
static void packM_S_err_MSG(ARMS_MASTER_THREAD_INFO *pTModule, Info_Show_In_Err *info_err_bag)
{
    int mSMsgSize = sizeof(ARMS_S_MSG)+sizeof(Info_Show_In_Err);
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);
    memcpy(mSMsg->mData, info_err_bag, sizeof(Info_Show_In_Err));

    //框架1，暂时设置
    mSMsg->mIdentifier = pTModule->mIdentifier;
    mSMsg->mMsgType = ACK_NOTICE_SHOW_INFO_IN_ERR;

    struct timespec  nowTime;
    clock_gettime(CLOCK_MONOTONIC, &nowTime);
    mSMsg->mSysTimeS = nowTime.tv_sec;
    mSMsg->mSysTimeUs = nowTime.tv_nsec/1000;
    // Info_Show_In_Err *mInfoshowerr = (Info_Show_In_Err *)mSMsg->mData;
    // mInfoshowerr->stop_result[0] = 'S';
    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg);
}


static void  packM_S_master_run_inf(ARMS_MASTER_THREAD_INFO *pTModule)
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
    mMasterInf->masterid = 0;
    mMasterInf->al_states = pTModule->al_states;
    mMasterInf->st_slaves = pTModule->st_slaves;
    mMasterInf->master_link_up = pTModule->master_link_up;
    mMasterInf->domain_wc = pTModule->domain_wc;

    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg);
}

static void  packM_S_chassisMSG(ARMS_MASTER_THREAD_INFO *pTModule)
{
    ARMS_MAGIC_CTRL_V *pmsg = (ARMS_MAGIC_CTRL_V*)malloc(sizeof(ARMS_MAGIC_CTRL_V));
    memset(pmsg,0,sizeof(ARMS_MAGIC_CTRL_V));
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //只拷贝伺服电机的机械臂
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            MAGIC_THREAD_INFO *pTmagic = &pTModule->mMagicWorker[i];
            pmsg->mModuleFlag[i] = 1;
            pmsg->mXMv[i] = pTmagic->cmd_last_xv;
            pmsg->mYv[i]  = pTmagic->cmd_last_yv;
        }
    }
    sendDataToChassis(pTModule->mSoket,MSG_ID_ARMS_MAGIC_CTRL_MSG,0,sizeof(ARMS_MAGIC_CTRL_V),(char*)pmsg,sizeof(ARMS_MAGIC_CTRL_V));
    free(pmsg);
}

static void  packM_S_Sample_MSG(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //拷贝master主站下的信息到upper
    int userArmsSize = 0;
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //只拷贝包含somanet的从站
        if(pTModule->slaves_inf[i].childSlaveSize>6)
            userArmsSize++;
    }
    
    int mSMsgSize = sizeof(ARMS_S_MSG)+sizeof(Arm_sample_inf)*userArmsSize;
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);

    //框架1，暂时设置
    mSMsg->mIdentifier = pTModule->mIdentifier;
    mSMsg->mMsgType = ACK_NOTICE_SHOW_INF;
    mSMsg->mUserArmsSize = userArmsSize;

    struct timespec  nowTime;
    clock_gettime(CLOCK_MONOTONIC, &nowTime);
    mSMsg->mSysTimeS = nowTime.tv_sec;
    mSMsg->mSysTimeUs = nowTime.tv_nsec/1000;

    int armsI = 0;
    Arm_sample_inf  *mArms = (Arm_sample_inf  *)mSMsg->mData;
    
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //只拷贝伺服电机的机械臂
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            mSMsg->mModuleFlag[i] = pTModule->slaves_inf[i].childSlaveSize;
            //memcpy(&mArms[armsI].somanet_inf,&pTModule->motors[i],sizeof(somanet_motors)*4);

           
            //传感器数据拷贝
            mArms[armsI].mIncli[0] = pTModule->incln_acc_XY[i][0];
            mArms[armsI].mIncli[1] = pTModule->incln_acc_XY[i][1];
            mArms[armsI].mTension  = pTModule->tension[i];

            mArms[armsI].mSiko[0] = pTModule->sikoXYZ[i][0];
            mArms[armsI].mSiko[1] = pTModule->sikoXYZ[i][1];
            mArms[armsI].mSiko[2] = pTModule->sikoXYZ[i][2];

            //拷贝相机fps
            mArms[armsI].mVisionFps = pTModule->camFps[i];

            //EC3045开关接触时 相应位为0
            mArms[armsI].mSwitchs = (~pTModule->EC_3045_switch[i]) & (0b00111111);

            //EC-3045采集的
            mArms[armsI].mAbsPressure = pTModule->abs_stress[i];

            mArms[armsI].Bat_Remaining = pTModule->mSerialWorker->chserinf.Bat_Remaining;

            armsI++;
        }
    }
    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_SAMPLE_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg);
}

static void  packM_S_MSG(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //拷贝master主站下的信息到upper
    int userArmsSize = 0;
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //只拷贝包含somanet的从站
        if(pTModule->slaves_inf[i].childSlaveSize>6)
            userArmsSize++;
    }
    
    int mSMsgSize = sizeof(ARMS_S_MSG)+sizeof(Arm_inf)*userArmsSize;
    ARMS_S_MSG* mSMsg = malloc(mSMsgSize);
    memset(mSMsg,0,mSMsgSize);

    //框架1，暂时设置
    mSMsg->mIdentifier = pTModule->mIdentifier;
    mSMsg->mMsgType = ACK_NOTICE_SHOW_INF;
    mSMsg->mUserArmsSize = userArmsSize;

    struct timespec  nowTime;
    clock_gettime(CLOCK_MONOTONIC, &nowTime);
    mSMsg->mSysTimeS = nowTime.tv_sec;
    mSMsg->mSysTimeUs = nowTime.tv_nsec/1000;

    int armsI = 0;
    Arm_inf  *mArms = (Arm_inf  *)mSMsg->mData;
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //只拷贝伺服电机的机械臂
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            mSMsg->mModuleFlag[i] = pTModule->slaves_inf[i].childSlaveSize;
            //memcpy(&mArms[armsI].somanet_inf,&pTModule->motors[i],sizeof(somanet_motors)*4);

            /*四个电机的状态字*/
            mArms[armsI].somanet_inf[0].status_word =  pTModule->somanet_status[i][0];
            mArms[armsI].somanet_inf[1].status_word =  pTModule->somanet_status[i][1];
            mArms[armsI].somanet_inf[2].status_word =  pTModule->somanet_status[i][2];
            mArms[armsI].somanet_inf[3].status_word =  pTModule->somanet_status[i][3];
            
            //XM轴速度 位置
            mArms[armsI].somanet_inf[0].Int_vel_value =  pTModule->xMIntSpeed[i];
            mArms[armsI].somanet_inf[1].Int_vel_value =  pTModule->xSIntSpeed[i];
            mArms[armsI].somanet_inf[2].Int_vel_value =  pTModule->ySpeed[i];
            mArms[armsI].somanet_inf[3].Int_vel_value =  pTModule->zSpeed[i];

            mArms[armsI].somanet_inf[0].Ext_Pos_value =  pTModule->xMExtPos[i];
            mArms[armsI].somanet_inf[1].Ext_Pos_value =  pTModule->xSExtPos[i];

            mArms[armsI].somanet_inf[0].Int_Pos_value =  pTModule->xMIntPos[i];
            mArms[armsI].somanet_inf[1].Int_Pos_value =  pTModule->xSIntPos[i];
            mArms[armsI].somanet_inf[2].Int_Pos_value =  pTModule->yPos[i];
            mArms[armsI].somanet_inf[3].Int_Pos_value =  pTModule->zPos[i];

            //电机速度环中的力矩偏差
            mArms[armsI].somanet_inf[0].torque_offset =  pTModule->motors[i][1].torque_offset;
            mArms[armsI].somanet_inf[1].torque_offset =  pTModule->motors[i][2].torque_offset;
            mArms[armsI].somanet_inf[2].torque_offset =  pTModule->motors[i][0].torque_offset;
            mArms[armsI].somanet_inf[3].torque_offset =  pTModule->motors[i][3].torque_offset;
            
            for (int j = 0; j < 4; j++)
            {
                //电机当前力矩拷贝
                mArms[armsI].somanet_inf[j].torque_act_value =  pTModule->torqueActual[i][j];
                //电机速度环生成的力矩
                mArms[armsI].somanet_inf[j].torque_demand =  pTModule->torqueDemand[i][j];
                //当前速度指令拷贝
                mArms[armsI].somanet_inf[j].target_velocity =  pTModule->cmd_velocity[i][j];
                //拷贝当前速度命令和执行error
                mArms[armsI].somanet_inf[j].cmd_vel_error =  pTModule->mMagicWorker[i].cmd_vel_error[j];
                mArms[armsI].somanet_inf[j].cmd_vel_mean_error_n =  pTModule->mMagicWorker[i].cmd_vel_mean_error_n[j];
            }
           
            //传感器数据拷贝
            mArms[armsI].mIncli[0] = pTModule->incln_acc_XY[i][0];
            mArms[armsI].mIncli[1] = pTModule->incln_acc_XY[i][1];
            mArms[armsI].mTension  = pTModule->tension[i];

            mArms[armsI].mSiko[0] = pTModule->sikoXYZ[i][0];
            mArms[armsI].mSiko[1] = pTModule->sikoXYZ[i][1];
            mArms[armsI].mSiko[2] = pTModule->sikoXYZ[i][2];

            //拷贝相机fps
            mArms[armsI].mVisionFps = pTModule->camFps[i];

            //EC3045开关接触时 相应位为0
            mArms[armsI].mSwitchs = (~pTModule->EC_3045_switch[i]) & (0b00111111);

            //EC-3045采集的
            mArms[armsI].mAbsPressure = pTModule->abs_stress[i];
            for (int k = 0; k < 4; k++)
            {
                mArms[armsI].mProportions[k] = pTModule->proportion_cmd[i][k];
            }   

            //主辅轴偏差
            mArms[armsI].mOtherD[0] = (uint32_t)(pTModule->mMagicWorker[i].last_synchron_dert*1000000);

            //插值测试
            interpn_value v = interpn_magic(&pTModule->mQuads[2], pTModule->xMIntPos[2],pTModule->yPos[2]);  
            mArms[armsI].mOtherD[1] = (uint32_t)(v.x *1000000);
            mArms[armsI].mOtherD[2] = (uint32_t)(v.y *1000000);

            armsI++;
        }
    }
    sendDataToUpper(pTModule->mSoket,MSG_ID_MASTER_SHOW_INF,0,mSMsgSize,(char*)mSMsg,mSMsgSize);
    free(mSMsg);
}

/*****************************************ethercat总线相关**************************************************/
//获取当前系统时间
static uint64_t system_time_ns(void)
{
    struct timespec  rt_time;
    clock_gettime(CLOCK_REALTIME, &rt_time);
    uint64_t time = TIMESPEC2NS(rt_time);
    return time;
}

static int32_t chaeckArmIError(ARMS_MASTER_THREAD_INFO *pTModule, int i)
{
    int iRet = 0;
    //出现error
    if((pTModule->motors[i][0].status_word & 0x004F) == 0x08 ||
       (pTModule->motors[i][1].status_word & 0x004F) == 0x08 ||
       (pTModule->motors[i][2].status_word & 0x004F) == 0x08 ||
       (pTModule->motors[i][3].status_word & 0x004F) == 0x08 )
    {
        //迁移到状态NULL
        return 1;
    } 

    return iRet;
}

static int32_t chaeckArmIStop(ARMS_MASTER_THREAD_INFO *pTModule, int i)
{
    int iRet = 0;
    //出现error
    if((pTModule->motors[i][0].velocity_value) == 0 &&
       (pTModule->motors[i][1].velocity_value) == 0 &&
       (pTModule->motors[i][2].velocity_value) == 0 &&
       (pTModule->motors[i][3].velocity_value) == 0 )
    {
        //迁移到状态NULL
        return 1;
    } 

    return iRet;
}

/******************************************************************************
* 功能：
* @return Descriptions
******************************************************************************/
static int32_t armsFsa(ARMS_MASTER_THREAD_INFO *pTModule)
{
    int32_t iRet = 0;
  
    //有限状态机切换
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //4个伺服电机、EK1100 Ek1110 ...
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            //写入4个电机的数据
            switch (pTModule->arm_fsa[i])
            {
                case S_SHUTDOWN:
                {
                    //"开启"禁用状态，此时需要发送控制字
                    for (int j = 0; j < 4; j++)
                    {
                        pTModule->motors[i][j].ctrl_word = 0x0006;
                        pTModule->motors[i][j].operation_mode = pTModule->somanetn_mode;
                        pTModule->motors[i][j].target_velocity = 0;
                    }
                    break;
                }
                case S_SWITCH_ON:
                {
                    for (int j = 0; j < 4; j++)
                    {
                        pTModule->motors[i][j].ctrl_word = 0x0007;
                        pTModule->motors[i][j].operation_mode = pTModule->somanetn_mode;
                        pTModule->motors[i][j].target_velocity = 0;   
                    }                   
                    break;
                }
                case S_ENABLE_OP:
                {
                    //判断状态字是否op成功
                    if
                    ( (pTModule->motors[i][0].status_word&0x67)==0x27 &&(pTModule->motors[i][1].status_word&0x67)==0x27 && 
                      (pTModule->motors[i][2].status_word&0x67)==0x27 && (pTModule->motors[i][3].status_word&0x67)==0x27
                    )
                    {
                        pTModule->arm_fsa[i] = S_OP_OK;
                    }
                    else
                    {
                        for (int j = 0; j < 4; j++)
                        {
                            pTModule->motors[i][j].ctrl_word = 0x000F;
                            pTModule->motors[i][j].operation_mode = pTModule->somanetn_mode;
                        }
                    }
                    break;
                }
                case S_HAIT:
                {
                    
                    for (int j = 0; j < 4; j++)
                    {
                        pTModule->motors[i][j].ctrl_word = 0x02; 
                        pTModule->motors[i][j].target_velocity = 0;
                    }

                    if(chaeckArmIStop(pTModule,i))
                    {
                        pTModule->arm_fsa[i] = S_NULL;
                    }
                    break;
                }
                case S_NULL:{
                    //TUDO init some param
                    break;
                }
                case S_OP_OK:{
                    
                    //TUDO init some param
                    break;
                }
                default:{
                    break;
                }
            }

            //电机再op状态下出现error，进行急停
            if(chaeckArmIError(pTModule,i)!=0 && (pTModule->arm_fsa[i]==S_ENABLE_OP || pTModule->arm_fsa[i]==S_NULL))
            {
                IsSystemError = true;  //6.16 -zjy

                pTModule->arm_fsa[i] = S_HAIT;

                /*
                // 6.16 -zjy
                printf("出错急停前电池的剩余电量为：%d \n", info_show_in_err.Bat_Remaining[1]);
                printf("出错急停前高压气压为：%f \n", info_show_in_err.gas_press_high[1]);
                
                for (int i = 0; i < pTModule->armsSize; i++)
                {
                
                    if(pTModule->slaves_inf[i].childSlaveSize>4)
                    {
                        printf("出错急停前正在运行的 %d 号机械臂 xM位置：%f, xS位置：%f, y位置：%f, z位置：%f, xM速度：%f, xS速度：%f, y速度：%f, z速度：%f\n", i+1, 
                            info_show_in_err.somanet_inf[i][0].Int_Pos_value,
                            info_show_in_err.somanet_inf[i][1].Int_Pos_value,
                            info_show_in_err.somanet_inf[i][2].Int_Pos_value,
                            info_show_in_err.somanet_inf[i][3].Int_Pos_value,
                            info_show_in_err.somanet_inf[i][0].Int_vel_value,
                            info_show_in_err.somanet_inf[i][1].Int_vel_value,
                            info_show_in_err.somanet_inf[i][2].Int_vel_value,
                            info_show_in_err.somanet_inf[i][3].Int_vel_value
                        );
                    }
                }*/
            } 
            
        }
    }

  return iRet;
}

/******************************************************************************
* 功能：采集的传感器数据转换成float类型真实值
* @return Descriptions
******************************************************************************/
static void  prop_to_ec3045(ARMS_MASTER_THREAD_INFO* pTModule,int armsid)
{
    /*
    EC-3045绑定的变量：
        EC_3045_vari[armsid][1]是比例调压阀1
        EC_3045_vari[armsid][2]是比例调压阀2
        EC_3045_vari[armsid][3]是比例阀-小阀
        EC_3045_vari[armsid][4]是比例阀-中阀
    */
    //算法使用到的中阀的EC-3045模拟量输出.中阀 小阀。将0点移动到5V
    for (int i = 2; i < 4; i++)
    {
        if((pTModule->proportion_cmd[armsid][i] + 5) >= 10)
        {
            pTModule->EC_3045_vari[armsid][1+i] = (uint16_t)(10*409.6);
        }else if((pTModule->proportion_cmd[armsid][i] + 5) <= 0)
        {
            pTModule->EC_3045_vari[armsid][1+i] = (uint16_t)(0*409.6);
        }else{
            pTModule->EC_3045_vari[armsid][1+i] = (uint16_t)((pTModule->proportion_cmd[armsid][i]+5)*409.6);
        }
    }
        
    //两个比例调压阀。
    for (int i = 0; i < 2; i++)
    {
        pTModule->EC_3045_vari[armsid][1+i] = (uint16_t)(pTModule->proportion_cmd[armsid][i]*409.6);
    }    
}

static void  somanets_i_to_f(ARMS_MASTER_THREAD_INFO* pTModule,int armsid)
{
   //x轴主从轴位置，y z轴位置 单位m,外部编码器方向要遵守右手法则,
    double tempPos = -(double)pTModule->motors[armsid][1].Ext_Pos_value;
    pTModule->xMExtPos[armsid] = tempPos*EXT_X_PULSE_TO_M*pTModule->AXIS_DIR[armsid][1]+pTModule->xStartMExtPos[armsid];
    //X主轴内部位置，基于0点的编码器位置
    tempPos = (double)pTModule->motors[armsid][1].Int_Pos_value-pTModule->mAparams[armsid].xm_encoder_zore;
    pTModule->xMIntPos[armsid] = tempPos*INT_X_PULSE_TO_M*pTModule->AXIS_DIR[armsid][1];

    tempPos = -(double)pTModule->motors[armsid][2].Ext_Pos_value;
    pTModule->xSExtPos[armsid] = tempPos*EXT_X_PULSE_TO_M*pTModule->AXIS_DIR[armsid][2]+pTModule->xStartSExtPos[armsid];;
    //X副轴内部位置，基于0点的编码器位置
    tempPos = (double)pTModule->motors[armsid][2].Int_Pos_value-pTModule->mAparams[armsid].xs_encoder_zore;
    pTModule->xSIntPos[armsid] = tempPos*INT_X_PULSE_TO_M*pTModule->AXIS_DIR[armsid][2];

    //y轴没有配置外部编码器，Ext_Pos_value，读取的值是606c寄存器里的，也就是电机内部编码器
    tempPos = (double)pTModule->motors[armsid][0].Ext_Pos_value-pTModule->mAparams[armsid].y_encoder_zore;
    pTModule->yPos[armsid]  = tempPos*Y_PULSE_TO_M*pTModule->AXIS_DIR[armsid][0];
    
    //Z轴位置，基于0点的编码器位置
    tempPos = (double)pTModule->motors[armsid][3].Int_Pos_value - pTModule->mAparams[armsid].z_encoder_zore;
    pTModule->zPos[armsid]  = tempPos*INT_Z_PULSE_TO_M*pTModule->AXIS_DIR[armsid][3];

    //外部编码器接的是磁栅尺Z
    tempPos = (double)pTModule->motors[armsid][3].Ext_Pos_value;
    pTModule->zExtPos[armsid]  = tempPos*EXT_Z_PULSE_TO_M;
   
    /*伺服电机数据转换成真实的单位  具体系数需要核算*/
    pTModule->xMIntSpeed[armsid] = ((float)pTModule->motors[armsid][1].velocity_value)*X_RPM_TO_V_MM*pTModule->AXIS_DIR[armsid][1];
    pTModule->xSIntSpeed[armsid] = ((float)pTModule->motors[armsid][2].velocity_value)*X_RPM_TO_V_MM*pTModule->AXIS_DIR[armsid][2];
    //X轴使用主轴和副轴的位置均值，差分速度
    
    pTModule->ySpeed[armsid]  = ((float)pTModule->motors[armsid][0].velocity_value)*Y_RPM_TO_V_MM*pTModule->AXIS_DIR[armsid][0];
    pTModule->zSpeed[armsid]  = ((float)pTModule->motors[armsid][3].velocity_value)*Z_RPM_TO_V_MM*pTModule->AXIS_DIR[armsid][3];   

    /*四个电机的状态字*/
    pTModule->somanet_status[armsid][0] = pTModule->motors[armsid][1].status_word;
    pTModule->somanet_status[armsid][1] = pTModule->motors[armsid][2].status_word;
    pTModule->somanet_status[armsid][2]  = pTModule->motors[armsid][0].status_word;
    pTModule->somanet_status[armsid][3]  = pTModule->motors[armsid][3].status_word;

    //轴内的四个电机的力矩当前值
    pTModule->torqueActual[armsid][0] = (float)pTModule->motors[armsid][1].torque_act_value;
    pTModule->torqueActual[armsid][1] = (float)pTModule->motors[armsid][2].torque_act_value;
    pTModule->torqueActual[armsid][2] = (float)pTModule->motors[armsid][0].torque_act_value;
    pTModule->torqueActual[armsid][3] = (float)pTModule->motors[armsid][3].torque_act_value;

    //速度环生成的力矩值
    pTModule->torqueDemand[armsid][0] = (float)pTModule->motors[armsid][1].torque_demand;
    pTModule->torqueDemand[armsid][1] = (float)pTModule->motors[armsid][2].torque_demand;
    pTModule->torqueDemand[armsid][2] = (float)pTModule->motors[armsid][0].torque_demand;
    pTModule->torqueDemand[armsid][3] = (float)pTModule->motors[armsid][3].torque_demand;
}

static void check_master_state(ARMS_MASTER_THREAD_INFO *pTModule)
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

static void check_domain1_state(ARMS_MASTER_THREAD_INFO *pTModule)
{
    static uint32_t cnt = 0;
    ec_domain_state_t ds;
    ecrt_domain_state(pTModule->domain1, &ds);
    if (ds.working_counter != pTModule->domain1_state.working_counter)
    {
        printf("master:%d Domain1: WC %u. cnt:%d\n",pTModule->mthreadMasterId, ds.working_counter,cnt++);
        pTModule->domain_wc = ds.working_counter;
    }
    if (ds.wc_state != pTModule->domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    pTModule->domain1_state = ds;
}

/***********************************************************************************************/

////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @return Descriptions
******************************************************************************/
void* master_arms_threadEntry(void* pModule)
{
  ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *) pModule;

  //初始化出错数据包队列
  SafeCircularQueue queue;
  queue_init(&queue);

  if(NULL == pTModule)
  {
    printf("null timer\n");
    return 0;
  }
  initSupr(pTModule);

  unsigned int nDelay = 1000;        /* usec */
  //timer 定时器
  struct timespec  next, interval;
  interval.tv_sec  = nDelay / USEC_PER_SEC;
  interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
  clock_gettime(CLOCK_MONOTONIC, &next);

  printf("\n%s running!threadEntry\n",pTModule->mThreadName);
  sendNoticeToLeader(pTModule->mSoket,MSG_ID_THREAD_START_ACK, 0, 0);

  int ret = 0;
  uint32_t runCnt = 0;
  int maxR =0;
  pTModule->masterActivated=false;
  uint32_t pack_ms = 10;

  //系统出错数据包的记录的北京时间
  time_t rawtime;
  struct tm *timeinfo;
  
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
        LoopRecLocalMsg(pTModule);
        //循环2s的增加速度到目标值
        cyc_vel_upper_cmd(pTModule);
        cyc_pos_upper_cmd(pTModule);

        //检查是否触碰软线
        //check_arms_near(pTModule);
        //检查随动算法主副轴是否误差过大
        //checkFollowMSPosError(pTModule);
    }
    //检查机械臂上限位开关，并作处理
    check_limit_switch(pTModule);

    //50HZ发送到上位机,机械臂信息
    if((runCnt) % pack_ms==0)
    {
        static uint32_t ccc = 0;
        //主站激活之后才发送消息到上位机
        if(pTModule->masterActivated)
        {
            packM_S_MSG(pTModule);
            packM_S_Sample_MSG(pTModule);
            

            if (!IsSystemError) {
                time(&rawtime);
                rawtime += 8 * 3600;
                timeinfo = localtime(&rawtime);
                // strftime(info_show_in_err.time_str,sizeof(info_show_in_err.time_str),"%H:%M:%S", timeinfo);
                // info_show_in_err.Bat_Remaining[1] = pTModule->mSerialWorker->chserinf.Bat_Remaining; //剩余的电量
                // info_show_in_err.gas_press_high[1] = pTModule->mSerialWorker->chserinf.gas_press_high; //高压气压
                
                //插入新数据包到队列
                Info_Show_In_Err new_data;
                strftime(new_data.time_str,sizeof(new_data.time_str),"%H:%M:%S", timeinfo);
                new_data.Bat_Remaining[1] = pTModule->mSerialWorker->chserinf.Bat_Remaining; //剩余的电量
                new_data.Bat_Remaining[1] = pTModule->mSerialWorker->chserinf.gas_press_high; //高压气压

                for (int i = 0; i < 8; i++) {
                    new_data.nimotion_inf[1][i].Position_value = info_show_in_err.nimotion_inf[1][i].Position_value;
                    new_data.nimotion_inf[1][i].velocity_value = info_show_in_err.nimotion_inf[1][i].velocity_value;
                }


                for (int i = 0; i < pTModule->armsSize; i++)
                {
                    //只拷贝伺服电机的机械臂
                    if(pTModule->slaves_inf[i].childSlaveSize>4)
                    {

                        //6.16 随动z轴限位处理 -zjy
                        
                        if (Is_Zlimit_Open) 
                        {
                            if (pTModule->zPos[i] < 1.0) {
            
                                printf("%d 号Z轴下限超出限位\n", i+1);
                                printf("zPos: %f\n", pTModule->zPos[i]);
                                if (pTModule->upper_target_velocity[i][3] < 0)
                                {
                                    //pTModule->motors[i][3].target_velocity = 0;//到达限位后直接给0，没有缓坡加速
                                    // pTModule->upper_target_velocity[i][3] = 0;  
                                    // pTModule->upper_target_v_Flag[i] = 1;      //缓坡加速，200Hz
                                    handleStartSlight(pTModule,i,0,0,0,0,300);
    
                                }
                                Is_Zout_limit[i][1] = true;
                                    
                            } 
                            else if (pTModule->zPos[i] > 2.0)
                            {
                                printf("%d 号Z轴上限超出限位\n", i+1);
                                printf("zPos: %f\n", pTModule->zPos[i]);
                                if (pTModule->upper_target_velocity[i][3] > 0)
                                {
                                    //pTModule->motors[i][3].target_velocity = 0;//到达限位后直接给0，没有缓坡加速
                                    // pTModule->upper_target_velocity[i][3] = 0;  
                                    // pTModule->upper_target_v_Flag[i] = 1;      //缓坡加速，200Hz
                                    handleStartSlight(pTModule,i,0,0,0,0,300);
                                }
    
                                Is_Zout_limit[i][0] = true;
                                    
                            }
                            else
                            {
                                Is_Zout_limit[i][1] = false;
                                Is_Zout_limit[i][0] = false;
                            }
                        }
                        
                        
                        /*
                        //6.16 保存出错要显示的信息 -zjy
                        info_show_in_err.incln_acc_XY[i][0]  =  pTModule->incln_acc_XY[i][0];
                        info_show_in_err.incln_acc_XY[i][1]  =  pTModule->incln_acc_XY[i][1];
                        info_show_in_err.abs_stress[i]       =  pTModule->abs_stress[i];
                        
                        info_show_in_err.somanet_inf[i][0].Int_vel_value = pTModule->xMIntSpeed[i];
                        info_show_in_err.somanet_inf[i][1].Int_vel_value = pTModule->xSIntSpeed[i];
                        info_show_in_err.somanet_inf[i][2].Int_vel_value = pTModule->ySpeed[i];
                        info_show_in_err.somanet_inf[i][3].Int_vel_value = pTModule->zSpeed[i];

                        info_show_in_err.somanet_inf[i][0].Int_Pos_value =  pTModule->xMIntPos[i];
                        info_show_in_err.somanet_inf[i][1].Int_Pos_value =  pTModule->xSIntPos[i];
                        info_show_in_err.somanet_inf[i][2].Int_Pos_value =  pTModule->yPos[i];
                        info_show_in_err.somanet_inf[i][3].Int_Pos_value =  pTModule->zPos[i];
                        */
                        

                        new_data.incln_acc_XY[i][0]  =  pTModule->incln_acc_XY[i][0];
                        new_data.incln_acc_XY[i][1]  =  pTModule->incln_acc_XY[i][1];
                        new_data.abs_stress[i]       =  pTModule->abs_stress[i];
                        
                        new_data.somanet_inf[i][0].Int_vel_value = pTModule->xMIntSpeed[i];
                        new_data.somanet_inf[i][1].Int_vel_value = pTModule->xSIntSpeed[i];
                        new_data.somanet_inf[i][2].Int_vel_value = pTModule->ySpeed[i];
                        new_data.somanet_inf[i][3].Int_vel_value = pTModule->zSpeed[i];

                        new_data.somanet_inf[i][0].Int_Pos_value =  pTModule->xMIntPos[i];
                        new_data.somanet_inf[i][1].Int_Pos_value =  pTModule->xSIntPos[i];
                        new_data.somanet_inf[i][2].Int_Pos_value =  pTModule->yPos[i];
                        new_data.somanet_inf[i][3].Int_Pos_value =  pTModule->zPos[i];  
                    }
                }

                queue_enqueue(&queue, new_data);

            } else { //系统出错，将队列数据打包到上位机，直到队列为空
                Info_Show_In_Err item;
                if (queue_dequeue(&queue, &item)) {
                    // 计算数据年龄（纳秒）
                    packM_S_err_MSG(pTModule, &item);
                }
                //free(&item);

            }


        }
    }
    //发送吊杆主站控制信息到底盘
    if((runCnt) % 20==0)
    {
        //发送吊杆主站控制信息到底盘
        packM_S_chassisMSG(pTModule);
    }

    //10HZ发送到上位机,机械臂信息
    if((runCnt) % 100==0)
    {
        //验证插值算法
        //for (int i = 0; i < pTModule->armsSize; i++)
        {
            //只拷贝伺服电机的机械臂
            int i=3;
            if(pTModule->slaves_inf[i].childSlaveSize>4)
            {
                //interpn_value v = interpn_magic(&pTModule->mQuads[i],pTModule->xMIntPos[i],pTModule->yPos[i]);  
                //printf("%d interpn_v:(%f %f) (%f %f) \n",i, pTModule->xMIntPos[i],pTModule->yPos[i],v.x,v.y);
                //printf("%d \n",pTModule->arm_fsa[i]);
            }
        }

        //主站激活之后才发送消息到上位机
        if(pTModule->masterActivated)
        {
            packM_S_master_run_inf(pTModule);
        }
        pack_ms = pTModule->PACK_MSG_MS;
        if(pack_ms <10 )
            pack_ms = 10;
    }

  }

    sleep(1);

    // 清理资源
    queue_destroy(&queue);

    moduleEndUp(pTModule);
}

/******************************************************************************
* 功能：子线程：master使用ethercat交换pdo线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : args
* @return Descriptions
******************************************************************************/
static void reflash_tpdo_Ebus(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //读取交换somanet的数据
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //4个伺服电机、EK1100 Ek1110 ...
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            for (int j = 0; j < 4; j++)
            {
                //CSV、CSP模式下都需要刷新
                pTModule->motors[i][j].status_word    = EC_READ_U16(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].status_word);
                pTModule->motors[i][j].velocity_value = EC_READ_S32(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].velocity_value);
                pTModule->motors[i][j].Ext_Pos_value = EC_READ_S32(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].Ext_Pos_value);
                pTModule->motors[i][j].Int_Pos_value = EC_READ_S32(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].Int_Pos_value);
                
                //CVS模式下，算法要用到力矩偏置
                pTModule->motors[i][j].torque_act_value = EC_READ_U16(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].torque_act_value);
                pTModule->motors[i][j].torque_demand = EC_READ_U16(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].torque_demand);
            }
            //伺服里的PDO数据参数转换成使用单位
            somanets_i_to_f(pTModule,i);            
            //读取EC-3045传感器数据,绝压值
            pTModule->EC_3045_abs_stress[i] = EC_READ_U16(pTModule->domain1_pd + pTModule->EC_3045[i][6]);
            //读取EC-3045传感器数据,限位开关
            pTModule->EC_3045_switch[i] = EC_READ_U16(pTModule->domain1_pd + pTModule->EC_3045[i][5]);
        }
    }
}

static void reflash_rpdo_Ebus(ARMS_MASTER_THREAD_INFO *pTModule)
{
    //写入交换somanet的数据，1ms 刷新写入一次
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        //4个伺服电机、EK1100 Ek1110 ...
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {             
            //CVS模式下，算法要用到力矩偏置
            if(pTModule->somanetn_mode == MODE_CSV)
            {
                //目标速度存储
                pTModule->cmd_velocity[i][0] = ((float)pTModule->motors[i][1].target_velocity)*X_RPM_TO_V_MM*pTModule->AXIS_DIR[i][1];
                pTModule->cmd_velocity[i][1] = ((float)pTModule->motors[i][2].target_velocity)*X_RPM_TO_V_MM*pTModule->AXIS_DIR[i][2];
                pTModule->cmd_velocity[i][2] = ((float)pTModule->motors[i][0].target_velocity)*Y_RPM_TO_V_MM*pTModule->AXIS_DIR[i][0];
                pTModule->cmd_velocity[i][3] = ((float)pTModule->motors[i][3].target_velocity)*Z_RPM_TO_V_MM*pTModule->AXIS_DIR[i][3];
                for (int j = 0; j < 4; j++)
                {
                    EC_WRITE_U16(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].ctrl_word, pTModule->motors[i][j].ctrl_word);
                    EC_WRITE_U16(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].operation_mode, pTModule->motors[i][j].operation_mode);
                    EC_WRITE_S32(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].target_velocity, pTModule->motors[i][j].target_velocity);
                    EC_WRITE_S16(pTModule->domain1_pd + pTModule->somanet_pdo[i][j].torque_offset, pTModule->motors[i][j].torque_offset);
                }
            }
            //EC-3045输出
            //将从站输出的值转换成PDO类型的
            prop_to_ec3045(pTModule,i); 
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->EC_3045[i][0], pTModule->EC_3045_vari[i][0]);

            //模拟量输出
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->EC_3045[i][1], pTModule->EC_3045_vari[i][1]);
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->EC_3045[i][2], pTModule->EC_3045_vari[i][2]);
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->EC_3045[i][3], pTModule->EC_3045_vari[i][3]);
            EC_WRITE_U16(pTModule->domain1_pd + pTModule->EC_3045[i][4], pTModule->EC_3045_vari[i][4]);
        }
    }
}

void* ethercatBusThread(void* pModule)
{
    ETHERCAT_THREAD_INFO *pEModule = (ETHERCAT_THREAD_INFO *) pModule;
    ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *)pEModule->pTModule;

    unsigned int nDelay = 1000;        /* usec */
    //timer 定时器
    struct timespec  next, interval;
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int ret = 0;
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
        check_domain1_state(pTModule);
        //Check for master state
        check_master_state(pTModule);

        /******************************************实时传输PDO******************************************/
        //读取交换somanet的数据
        reflash_tpdo_Ebus(pTModule);
       
        /******************************************有限状态机******************************************/
        armsFsa(pTModule);

        /******************************************实时传输PDO******************************************/
        reflash_rpdo_Ebus(pTModule);
                 
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
