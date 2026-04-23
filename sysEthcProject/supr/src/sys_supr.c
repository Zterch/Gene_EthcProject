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
#include <stdarg.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "sys_supr.h"
#include "sys_upper_listener.h"
#include "sys_logger.h"
#include "sys_db_worker.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"
#include "sys_ctl.h"

//定义线程模块运行的状态
typedef enum
{
  M_STATE_START = 1,
  M_STATE_SDLE ,
  M_STATE_PHASE1 ,        //注冊 msg 隊列
  M_STATE_PHASE1_ACK ,
  M_STATE_PHASE2 ,        //modules 模塊做初始化
  M_STATE_PHASE2_ACK ,
  M_STATE_FIREUP,
  M_STATE_RUNNING,
  M_STATE_STOP,
  M_STATE_QUIT,
} M_RUN_STATE;

//模块结构体，线程名字，入口函数，pid
typedef struct _MODULEINFOS
{
  char    mName[64];
  int32_t mPri;
  pid_t   mPid;
  M_RUN_STATE mState;
} MODULEINFOS;

//cpu latency 禁止cpu进入节能模式
static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

//arms,tensions,logs threads and supr
MODULEINFOS gHiMInfo[MODULES_NUMS+1] = {0};

//socket
int32_t      mSurpSoket;
static int suprWorking = 1;

//thread info
LISTENER_THREAD_INFO mUpperListener;
LOG_THREAD_INFO mLogger;
DB_WORKER_INFO  mDBWorker;

static int32_t prepareEnv(void);
static int32_t initSupr(void);
static int32_t startModules(void);
static int32_t suprMainLoop();
static void signalHandle(int mSignal);
static void checkArmsWorking();
static int32_t getIniKeyValue(char *key, char* mnName, char *filename, float* value);
static void set_latency_target(void);


static int32_t  registMsgQForModules(int mId, pid_t mPid);
static int32_t  initModulesPhase1();
static int32_t  initModulesPhase1Ack(int32_t mModuleId, int32_t mNotice);
static int32_t  initModulesPhase2();
static int32_t  initModulesPhase2Ack(int32_t mModuleId, int32_t mNotice);
static int32_t  fireInAllModules();
static int32_t  triggerSuprChangeState(const int32_t mMsgId);
static int32_t  noticeAllModules(const int32_t mMsgId);
static int32_t  sendShortMsg(const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue, const char* mShortData, const int32_t mDSize);

static int32_t checkModuleStates(M_RUN_STATE mState);
static int32_t changeModuleStates(int mModuleId, M_RUN_STATE mState);

//leader function
static int32_t  sendLeaderNotice(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);

//upper cmd
static int32_t handleUpperMsg(MsgLongMsg *pUpperMsg);
static int32_t handleUpperActionMsg(UpperCmdMsg *pUpperMsg);

static void PrintfLog(const char *fm, ...);
static void PrintfData(const char *fm, ...);

static int32_t deInitSupr(void);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//static SUPR_BASE::M_STATE mSysState = SUPR_BASE::M_STATE_INIT;


static int32_t registMsgQForModules(int mModuleId, pid_t mPid)
{
  int iRet = 0;

  gHiMInfo[mModuleId].mPid  = mPid;
  gHiMInfo[mModuleId].mState = M_STATE_START;

  //检查是否都注册完毕
  iRet = checkModuleStates(M_STATE_START);

  printf("registMsgQForModules mID:%d iRet:%d\n",mModuleId,iRet);
  //进入init状态
  if(iRet == MODULES_NUMS)
  {
    triggerSuprChangeState(MSG_ID_MODULE_INIT1);
  }
  return iRet;
}

static int32_t checkModuleStates(M_RUN_STATE mState)
{
  int32_t iRet = 0;

  for (int i=1; i<MODULES_NUMS+1; ++i) {
    if(gHiMInfo[i].mState == mState)
      iRet++;
  }
  return iRet;
}

static int32_t changeModuleStates(int mModuleId, M_RUN_STATE mState)
{
  int32_t iRet = 0;
  gHiMInfo[mModuleId].mState = mState;

  return iRet;
}

static int32_t initModulesPhase1()
{
  int iRet = 0;
  //module go in init1
  char eno1_ip[IP_C_SIZE];
  //get_local_ip("eno1",eno1_ip);
  get_local_ip("wlp1s0",eno1_ip);
  int32_t frameId = 1;
  if(strcmp(eno1_ip,"192.168.11.11") == 0 || strcmp(eno1_ip,"192.168.13.13") == 0)
  {
    frameId = 1;
  }else if(strcmp(eno1_ip,"192.168.101.12") == 0 || strcmp(eno1_ip,"192.168.14.14") == 0)
  {
    frameId = 2;
  }

  for (int i=1; i<MODULES_NUMS+1; ++i) 
  {
    sendShortMsg(i, MSG_ID_MODULE_INIT1,frameId,0, 0, 0);
  }
  printf("supr initModulesPhase1\n");
  return iRet;
}

static int32_t initModulesPhase1Ack(int32_t mModuleId, int32_t mNotice)
{
  int iRet = 0;
  if(mNotice == 0)
    gHiMInfo[mModuleId].mState = M_STATE_PHASE1;

  //检查是否都注册完毕
  for (int i=1; i<MODULES_NUMS+1; ++i) {
    if(gHiMInfo[i].mState != M_STATE_PHASE1)
      return 0;
  }

  //TODU 先进入fireup 状态，后期可加入phase2 3 4...
  //进入fire状态
  triggerSuprChangeState(MSG_ID_MODULE_INIT2);
  printf("supr init2 to all\n");
  return iRet;
}

static int32_t initModulesPhase2()
{
  int iRet = 0;

  //检查是否都注册完毕,only (M_STATE_PHASE1 ,M_STATE_STOP) can into initModulesPhase2
  for (int i=1; i<MODULES_NUMS+1; ++i) {
    if((gHiMInfo[i].mState != M_STATE_PHASE1) && (gHiMInfo[i].mState != M_STATE_STOP))
      return -1;
  }

  //module go in init2
  for (int i=1; i<MODULES_NUMS+1; ++i) {
    sendShortMsg(i, MSG_ID_MODULE_INIT2, 0, 0, 0, 0);
  }
  return iRet;
}

static int32_t initModulesPhase2Ack(int32_t mModuleId, int32_t mNotice)
{
  int iRet = 0;
  if(mNotice == 0)
    gHiMInfo[mModuleId].mState = M_STATE_PHASE2;

  //检查是否都注册完毕
  for (int i=1; i<MODULES_NUMS+1; ++i) {
    if(gHiMInfo[i].mState != M_STATE_PHASE2)
      return 0;
  }

  //TODU 先进入fireup 状态，后期可加入phase 3 4...
  //进入fire状态
  iRet = triggerSuprChangeState(MSG_ID_FIRE_IN_THE_HOLE);

  return iRet;
}

static int32_t  fireInAllModules()
{
  //
  for (int i=0; i<MODULES_NUMS+1; ++i) {
    gHiMInfo[i].mState = M_STATE_FIREUP;
  }

  //发送到每个模块，进入fireinhole状态
  noticeAllModules(MSG_ID_FIRE_IN_THE_HOLE);
}

static int32_t triggerSuprChangeState(const int32_t mMsgId)
{
  int iRet = 0;
  iRet = MsgSendNotice(mSurpSoket,get_supr_port(), MN_ID_SUPR, MN_ID_SUPR, mMsgId, 0, 0);
  return iRet;
}

static int32_t  sendShortMsg(const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue, const char* mShortData, const int32_t mDSize)
{
  int iret = 0;

  MsgSendShortMsg(mSurpSoket, get_mn_port(mRecer), MN_ID_SUPR, mRecer, mMsgId, mNotice, mValue, mShortData, mDSize);

  return iret;
}

static int32_t  noticeAllModules(const int32_t mMsgId)
{
  int iret = 0;
  //const int mMsgQ, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue
  for (int i=1;i<MODULES_NUMS+1;++i)
  {
    MsgSendNotice(mSurpSoket,get_mn_port(i), MN_ID_SUPR, i, mMsgId, 0, 0);
  }
  return iret;
}

static int32_t  sendLeaderNotice(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
  int iret = 0;

  iret = sendShortMsg(MN_ID_LEADER, mMsgId, mNotice, mValue, 0, 0);

  return iret;
}

static void PrintfLog(const char *fm, ...)
{
  char mStr[256] = {0};
  va_list args;
  va_start(args, fm);
  vsnprintf(mStr, 255, fm, args);
  va_end( args );

  struct timespec  logtime;
  clock_gettime(CLOCK_MONOTONIC, &logtime);
  char mLogStr[300] = {0};
  sprintf(mLogStr, "%ld %ld %s",logtime.tv_sec,logtime.tv_nsec/1000,mStr);

  //BASE::MsgSendPriLog(mSurpSoket, BASE::MN_PORTS[BASE::MN_ID_LOGER],BASE::MN_ID_SUPR, BASE::MN_ID_LOGER,BASE::MSG_ID_LOGER_SAVE_LOG, mLogStr);
}

static void PrintfData(const char *fm, ...)
{
  char mStr[256] = {0};
  va_list args;
  va_start(args, fm);
  vsnprintf(mStr, 255, fm, args);
  va_end( args );

  struct timespec  logtime;
  clock_gettime(CLOCK_MONOTONIC, &logtime);
  char mLogStr[300] = {0};
  sprintf(mLogStr, "%ld %ld %s",logtime.tv_sec,logtime.tv_nsec/1000,mStr);

  //BASE::MsgSendPriLog(mSurpSoket,BASE::MN_PORTS[BASE::MN_ID_LOGER], BASE::MN_ID_SUPR, BASE::MN_ID_LOGER,BASE::MSG_ID_LOGER_SAVE_DATA, mLogStr);
}

static int32_t getIniKeyValue(char *key, char* mnName, char *filename, float* value)
{
  FILE *fp;

  char readbuf[512] = {0};
  char findbuf[100] = {0};
  strcpy(findbuf, mnName);
  strcat(findbuf, ".");
  strcat(findbuf, key);
  char *retbuf;
  retbuf = (char *)malloc(20);
  int line = 0;

  if((fp = fopen(filename, "a+")) == NULL)
  {
    printf("have  no  such   file \n");
    return -1;
  }
  while(fgets(readbuf, sizeof(readbuf), fp)) //逐行循环读取文件，直到文件结束
  {
    line++;
    if(!strncmp(readbuf, "#" ,1) || !strncmp(readbuf,"\n",1)) //忽略注释(#)和空行
      continue;
    if(strstr(readbuf, findbuf))     //查找配置文件名
    {
      char *p = strchr(readbuf, '=');  //确定“=”位置
      do
        p += 1;
      while(*p == ' ');

      sprintf(retbuf,"%s",p);
      printf("*****  %s\n", retbuf);
      *value = atof(retbuf);
     }
   }
   fclose(fp);
}


/* Latency trick
 * if the file /dev/cpu_dma_latency exists,
 * open it and write a zero into it. This will tell
 * the power management system not to transition to
 * a high cstate (in fact, the system acts like idle=poll)
 * When the fd to /dev/cpu_dma_latency is closed, the behavior
 * goes back to the system default.
 *
 * Documentation/power/pm_qos_interface.txt
 */
/******************************************************************************
* 功能：设置电源管理策略，此模块比避免cpu进入节能模式，影响定时器精度
* @return Descriptions
******************************************************************************/
static void set_latency_target(void)
{
    struct stat s;
    int err;

    errno = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1) {
        printf("WARN: stat /dev/cpu_dma_latency failed");
        return;
    }

    errno = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
        printf("WARN: open /dev/cpu_dma_latency");
        return;
    }

    errno = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1) {
        printf("# error setting cpu_dma_latency to %d!", latency_target_value);
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %du\n", latency_target_value);
}

/******************************************************************************
* 功能：初始化整个系统的运行环境
* @return Descriptions
******************************************************************************/
static int32_t prepareEnv(void) {
  int32_t iRet = 0;
  printf("supr ARMS APP STARTING\n");

  set_latency_target();
  //init_params();
  //TODO:: msg and msg q need refactor after first version.
  if(initSupr() < 0)
    return -1;

  iRet = startModules();

  return iRet;
}

/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(void) {
  //need handler error case
  int32_t iRet = 0;

  //创建soket
  mSurpSoket = createSoket(get_mn_port(MN_ID_SUPR), MN_LocalIpV4Str, 0, 5000);

  printf("createSoket:%d\n",mSurpSoket);

  iRet = mSurpSoket;
  //TUDO
  signal(SIGINT, signalHandle);

  //supr run in cpu0
  hiSetCpuAffinity(CPU_SUPR);
  hiSetThreadsched(pthread_self(), PRI_SUPR);

  return iRet;
}


/******************************************************************************
* 功能：各个线程模块初始化，并且启动个各模块线程
* @return Descriptions
******************************************************************************/
static int32_t startModules(void) {

  //上位机线程 thread
  mUpperListener.mWorking = true;
  mUpperListener.mInitOk  =false;
  mUpperListener.mCpuAffinity  = 1;
  strcpy(mUpperListener.mThreadName, "upper");
  mUpperListener.mthreadPid = hiCreateThread(mUpperListener.mThreadName, upper_threadEntry,50, &mUpperListener);
  
  //等待upper初始化完成，supr才进入循环
  int32_t waitTime = 0;
  while (!mUpperListener.mInitOk)
  {
    printf("supr waiting upper init...\n");
    sleep(1);
    waitTime++;
    if(waitTime>10)
      return -1;
  }

  //日志线程创建
  mLogger.mWorking = true;
  mLogger.mInitOk  =false;
  mLogger.mCpuAffinity  = 1;
  strcpy(mLogger.mThreadName, "logger");
  mLogger.mthreadPid = hiCreateThread(mLogger.mThreadName, logger_threadEntry,50, &mLogger);

  //数据库线程创建
  mDBWorker.mWorking = true;
  mDBWorker.mInitOk  =false;
  mDBWorker.mCpuAffinity  = 1;
  strcpy(mDBWorker.mThreadName, "dbworker");
  mDBWorker.mthreadPid = hiCreateThread(mDBWorker.mThreadName, db_threadEntry,50, &mDBWorker);

  return 0;
}


/******************************************************************************
* 功能：检查leader线程模块是否在运行
* @return Descriptions
******************************************************************************/
static void checkArmsWorking()
{
  ;
}


/******************************************************************************
* 功能：supr线程陷入函数，周期性的控制leader等线程运行
* @return Descriptions
******************************************************************************/
static int32_t suprMainLoop(){
  //TODU
  MsgHeader *mMsg;
  char mData[Msg_T_S];
  suprWorking = 1;
  mMsg = (MsgHeader *)mData;
  struct sockaddr_in  mPeerAddr;
  socklen_t mun = sizeof(mPeerAddr);

  while(suprWorking)
  {
      struct timespec  logS,logE;
      clock_gettime(CLOCK_MONOTONIC, &logS);
      //printf("suprMainLoop (%d:%d)\n\r",logS.tv_sec,logS.tv_nsec/1000);

    if((recvfrom(mSurpSoket, mData, Msg_T_S, 0, (struct sockaddr*)&(mPeerAddr), &mun)>0) && (MN_ID_SUPR == mMsg->mRecer))
    {
      switch (mMsg->mMsgId)
      {
        case MSG_ID_MODULE_REGISTER_MSG_Q_ACK:
        {
          MsgRegisterMsgQ *mRegisMsg = (MsgRegisterMsgQ *)mMsg;
          registMsgQForModules(mRegisMsg->mSender, mRegisMsg->mPid);
          printf("register ok\n");
          break;
        }
        case MSG_ID_MODULE_INIT1:
        {
          printf("supr init \n");
          initModulesPhase1();
          break;
        }
        case MSG_ID_MODULE_INIT1_ACK:
        {
          printf("supr init1 ack\n");
          initModulesPhase1Ack(mMsg->mSender, mMsg->mNotice);
          break;
        }
        case MSG_ID_MODULE_INIT2:
        {
          initModulesPhase2();
          printf("supr init2 \n");
          break;
        }
        case MSG_ID_MODULE_INIT2_ACK:
        {
          printf("supr init2 ack sender:%d\n",mMsg->mSender);
          initModulesPhase2Ack(mMsg->mSender, mMsg->mNotice);

          break;
        }
        case MSG_ID_FIRE_IN_THE_HOLE:
        {
          printf("supr fire in the hole\n");
          struct timespec  sendTime,endTimes;
          fireInAllModules();
          break;
        }
        case MSG_ID_MODULE_REQUEST_STOP_ALL:
        {
          PrintfLog("supr send stop\n");
          noticeAllModules(MSG_ID_MODULE_STOP_ALL);
          break;
        }
        case MSG_ID_MODULE_STOP_ALL_ACK:
        {
          MsgRegisterMsgQ *mRegisMsg = (MsgRegisterMsgQ *)mMsg;
          //change modules state
          changeModuleStates(mRegisMsg->mSender, M_STATE_STOP);
          //check modules state
          if(checkModuleStates(M_STATE_STOP) == MODULES_NUMS)
          {
            printf("all modules stoped\n");
            gHiMInfo[0].mState = M_STATE_STOP;
          }
          break;
        }
        case MSG_ID_MODULE_REQUEST_EXIT_ALL:
        {
          PrintfLog("supr send exit\n");
          noticeAllModules(MSG_ID_MODULE_EXIT_ALL);
          break;
        }
        case MSG_ID_MODULE_EXIT_ALL_ACK:
        {
          changeModuleStates(mMsg->mSender, M_STATE_QUIT);
          if(checkModuleStates(M_STATE_QUIT) == MODULES_NUMS)
          {
            printf("all modules quited\n");
            suprWorking = 0;
          }
          break;
        }
        case MSG_ID_UPPER_LOSE_LINK_MSG:
        {
          //request ethercat stop
          sendLeaderNotice(MSG_ID_REQUEST_ETHR_STOP, 0, 0);
          break;
        }
        case MSG_ID_UPPER_ACTIONG_MSG:
        {
          handleUpperMsg((MsgLongMsg *)mMsg);
          break;
        }
        default:
        {
          break;
        }
      } //end switch

    }//end if

  }// end while

  //
  mUpperListener.mWorking = 0;
  mLogger.mWorking = 0;
  mDBWorker.mWorking = 0;
} //end suprMainLoop

static int32_t handleUpperMsg(MsgLongMsg *pUpperMsg)
{
  int32_t iRet = 0;

  UpperCmdMsg *pCmdMsg = (UpperCmdMsg*)pUpperMsg->mData;

  switch (pCmdMsg->mCmd)
  {
    case 0:
    {

      printf("CMD_TYPE_MSGDATA\n");
      break;
    }
    default:
    {
      break;
    }

  }
  return iRet;
}


static int32_t handleUpperActionMsg(UpperCmdMsg *pUpperMsg)
{
  int32_t iRet = 0;

  switch (pUpperMsg->mCmd)
  {
    case CMD_LINK:
    {
      /*
      pthread_mutex_lock(&mshmSender->mShmMutex);
      mshmSender->mShmmNotice = BASE::ACK_NOTICE_LINK_OK;
      pthread_mutex_unlock(&mshmSender->mShmMutex);
      pthread_cond_signal(&mshmSender->mShmCond);
      */
      break;
    }
    case CMD_UNLINK:
    {
      //request ethercat stop
      sendLeaderNotice(MSG_ID_REQUEST_ETHR_STOP, 0, 0);

      /*
      pthread_mutex_lock(&mshmSender->mShmMutex);
      mshmSender->mShmmNotice = BASE::ACK_NOTICE_UNLINK_OK;
      pthread_mutex_unlock(&mshmSender->mShmMutex);
      pthread_cond_signal(&mshmSender->mShmCond);
      */
      break;
    }
    case CMD_SERVER_QUIT:
    {
      triggerSuprChangeState(MSG_ID_MODULE_REQUEST_EXIT_ALL);
      break;
    }
    default:
    {
      break;
    }
  }
  return iRet;
}

/******************************************************************************
* 功能：按键中断函数，ctrl+c结束系统运行
* @return Descriptions
******************************************************************************/
//TUDO
static void signalHandle(int mSignal)
{
  //mSysState = BASE::M_STATE_STOP;
  //mSysState = BASE::M_STATE_QUIT;
  //suprWorking = 0;
  triggerSuprChangeState(MSG_ID_MODULE_REQUEST_EXIT_ALL);
  printf("oops! stop!!!\n");
}

/******************************************************************************
* 功能：释放、销毁线程资源
* @return Descriptions
******************************************************************************/
static int32_t deInitSupr(void)
{
  int32_t iRet = 0;
  /* close the latency_target_fd if it's open */
  if (latency_target_fd >= 0)
    close(latency_target_fd);

  if(mSurpSoket>0)
  {
      close(mSurpSoket);
      mSurpSoket = -1;
  }
  printf("**********surp deInitSupr \n");

  return iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void clientTest()
{
//TUDO
}

/******************************************************************************
* 功能：系统入口函数，main函数调用
* @return Descriptions
******************************************************************************/
int32_t dmsAppStartUp() 
{
  int32_t iRet = 0;
  //父进程
  iRet = prepareEnv();

  get_mn_master_port(0);

  if (0 == iRet) {
    printf("supr runing now! ctrl+c to endup\n");
    iRet = suprMainLoop();

    pthread_join(mUpperListener.mthreadPid, NULL);
    pthread_join(mLogger.mthreadPid, NULL);
    pthread_join(mDBWorker.mthreadPid, NULL);
  }

  deInitSupr();
  printf("quit sysArms all modules,%d\n", iRet);
  return iRet;

}

