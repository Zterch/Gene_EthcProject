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
#include "sys_logger.h"
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

//读写配置文件，获得参数数据
int read_config(int armid,float *pv);

static int32_t  sendLeaderMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendMasterMsg(const int32_t mMasterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendAllMasterMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendArmsMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendArmsMasterNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);

/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(LOG_THREAD_INFO *pTModule) {
  int32_t iRet = 0;
  //创建soket.非阻塞模式
  mLocalSoket = createSoket(get_port("LOGGER_LOCAL_PORT"), MN_LocalIpV4Str, 0, 0);

  if(mLocalSoket  < 0) {
      printf( "local socket creat Failed");
      return -1;
  }
  return iRet;
}

/**
 * @brief read_config
 * 从配置文件中 , 读取配置文件 键值对 信息
 * @return
 */
#if 0
int read_config(int armid,float *pv)
{
    // 局部变量 返回值 , 用于表示程序状态
    int ret = 0;
    char fileN[16];
    sprintf(fileN,"config%d.ini",armid);

    char key[256] = {0};      // Key 键
    char value[256] = {0};    // Value 值
    // 值 Value 的长度
    int value_len = 0;
    int pid = 0;
    while (1)
    {
        sprintf(key,"p%d",pid);
        value_len = 0;
        ret = read_config_file(fileN, key, value, &value_len);

        if(ret <0 || value_len<=0)
            break;
        pid++;
    }
    // 打印查询结果
    printf("Get Value Success , Value = %s \n", value);
}
#endif
/**
 * @brief write_update_config
 * 启动 写出 / 更新 配置项 模块 , 执行 写出 / 更新 配置项操作
 * @return
 */
#if 0
int write_update_config()
{
    // 局部变量 返回值 , 用于表示程序状态
    int ret = 0;

    // 写出 或 更新 的配置项
    // 数组声明会后 , 注意先进行初始化为 0 操作 , 否则其中的数据可能是随机的
    char key[256] = {0};      // Key 键
    char value[256] = {0};    // Value 值

    // 提示输入 Key 键
    printf("\nPlease Input Key : ");
    // 将 Key 存储到 name 字符串数组中
    scanf("%s", key);

    // 提示输入 Value 值
    printf("\nPlease Input Value : ");
    // 将 Value 值 存储到 value 字符串数组中
    scanf("%s", value);

    // 向 D:/File/config.ini 写出或更新 键值对 信息
    ret = write_or_update_config_file(CONFIG_FILE_NAME /*in*/, key /*in*/, value/*in*/,strlen(value) /*in*/);
    // 判定执行中是否出错
    if (ret != 0)
    {
        printf("error : write_or_update_config : %d \n", ret);
        return ret;
    }

    // 打印成功插入的键值对
    printf("Input %s = %s Success !\n", key , value);
    return ret;
}
#endif


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
static int moduleEndUp(LISTENER_THREAD_INFO *pTModule)
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
void* logger_threadEntry(void* pModule)
{
  LOG_THREAD_INFO * pTModule =(LOG_THREAD_INFO *) pModule;
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
  moduleEndUp(pTModule);
}

