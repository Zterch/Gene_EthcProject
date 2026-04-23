/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_loger.c
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:日志模块，系统运行的情况记录在这个模块中。
*
********************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <string.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "list.h"
#include "sys_db_worker.h"
#include "sys_ctl.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_conf.h"
#include "sys_share_messages.h"
#include "sys_share_defs.h"

#define CONFIG_FILE_NAME "config"

static int32_t mLocalSoket = -1;

static int32_t initSupr(LISTENER_THREAD_INFO *pTModule);
static void LoopRecLocalMsg();

static int32_t  sendLeaderMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendMasterMsg(const int32_t mMasterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendAllMasterMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendArmsMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendArmsMasterNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);

/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(DB_WORKER_INFO *pTModule) {
  int32_t iRet = 0;
  //创建soket.非阻塞模式
  mLocalSoket = createSoket(get_port("DB_WORKER_PORT"), MN_LocalIpV4Str, 0, 0);

  if(mLocalSoket  < 0) {
      printf( "local socket creat Failed");
      return -1;
  }
  return iRet;
}

static void LoopRecLocalMsg()
{
    char mLocalData[Msg_T_S];

    struct sockaddr_in  mLocalPeerAddr;
    socklen_t localNum  = sizeof(mLocalPeerAddr);
    while(recvfrom(mLocalSoket, mLocalData, Msg_T_S, 0, (struct sockaddr*)&(mLocalPeerAddr), &localNum)>0)
    {
      MsgLongMsg *mMsg = (MsgLongMsg *)mLocalData;
      switch (mMsg->mMsgId)
      {
        case MSG_ID_LOGER_SAVE_DATA:
        {  
          //计算机械臂号的映射
          printf("%s \n",mMsg->mData);    
          break;
        }        
        default:
        {
          break;
        }
      } //end switch

    }
}


static int32_t  sendLeaderMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_port(MN_ID_LEADER), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  return iret;
}


static int32_t  sendMasterMsg(const int32_t mMasterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(mMasterId), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);

  return iret;
}

static int32_t  sendAllMasterMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_ARMS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_CHASSIS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  
  return iret;
}


static int32_t  sendArmsMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_ARMS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  
  return iret;
}

static int32_t  sendArmsMasterNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
   int iret = 0;
   iret = MsgSendNotice(mLocalSoket, get_mn_master_port(MN_ID_MASTER_ARMS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue);
  return iret; 
}

/******************************************************************************
* 功能：此函数销毁初始化模块，释放线程指针里的内容
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int moduleEndUp(DB_WORKER_INFO *pTModule)
{
  //删除创建的soket
  if(mLocalSoket>0)
  {
      close(mLocalSoket);
      mLocalSoket = -1;
  }
  printf("%s leader endup\n", pTModule->mThreadName);
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @return Descriptions
******************************************************************************/
void* db_threadEntry(void* pModule)
{
  DB_WORKER_INFO * pTModule =(DB_WORKER_INFO *) pModule;
  if(NULL == pTModule)
  {
    printf("null timer\n");
    return 0;
  }
  
  if(initSupr(pTModule)<0)
    return 0; 

  //初始化完成
  pTModule->mInitOk = true;
  static uint32_t runCnt=0;
  while(pTModule->mWorking)
  {
      //由于接收上位机和本地消息的套接字设置非阻塞模式，必须要sleep函数，避免cpu一直运行
      usleep(1000);

      //循环接收上位机和本地的消息。
      LoopRecLocalMsg();

      struct timespec  logtime;
      clock_gettime(CLOCK_MONOTONIC, &logtime);
      if(runCnt % 100 == 0)
      {
         //printf("**********:%s %ld %ld\n",pTModule->mThreadName,logtime.tv_sec,logtime.tv_nsec/1000);
      }
      runCnt++;
  }
  sleep(1);
  moduleEndUp(pTModule);
}

