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
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>

#include "sys_share_ipc.h"
#include "sys_share_messages.h"


int32_t MsgMgrInit(int magKey)
{
  int32_t iRet = -1;

  key_t mkey = (key_t)magKey;

  if(mkey > 0)
  {
    iRet = msgget(mkey, IPC_CREAT|0666);
    if(iRet < 0)
    {
      printf("##############msgget failed\n");
    }
  }
  return iRet;
}

int32_t  MsgMgrDeinit(int msqid)
{
  int iRet = 0;
  iRet = msgctl(msqid, IPC_RMID, NULL);
  return iRet;
}

int32_t MsgMgrSendMsg(const int32_t mSenderSoket, const int mRecerPort, const void *ptr, size_t length)
{
  int32_t iRet = 0;
  //peer
  struct  sockaddr_in  mPeerAddr;
  bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
  mPeerAddr.sin_family = AF_INET;
  mPeerAddr.sin_port = htons(mRecerPort);
  mPeerAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

  iRet = sendto(mSenderSoket, ptr, length, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));

  //printf("*********%d  %d %d\n",mSenderSoket,iRet, mRecerPort);
  return iRet;
}

int32_t MsgReceiveMsg(int msqid, void* ptr)
{
  int32_t iRet = 0;
  //BASE::MSG_t mMsg;
  //iRet = msgrcv (msqid, (char*)&mMsg, sizeof(BASE::MSG_t), 1, 0);

  //printf("********MsgReceiveMsg:%d  %d\n",iRet ,sizeof(BASE::MSG_t));

  if(iRet > 0)
  {
    //memcpy(ptr, mMsg.mData, Msg_T_S);
  }
  return iRet;
}

int32_t MsgSendNotice(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
  int32_t iRet = 0;
  MsgHeader mMsg;
  mMsg.mSender = mSender;
  mMsg.mRecer  = mRecer;
  mMsg.mMsgId  = mMsgId;
  mMsg.mNotice  = mNotice;
  mMsg.mValue1  = mValue;
  iRet = MsgMgrSendMsg(mSenderSoket,mRecerPort, (char*)&mMsg, sizeof(MsgHeader));
  return  iRet;
}

int32_t MsgSendShortMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                        const int32_t mNotice, const int32_t mValue, const char* mShortData, const int32_t mDSize)
{
  int32_t iRet = 0;
  MsgShortMsg mMsg;
  mMsg.mSender = mSender;
  mMsg.mRecer  = mRecer;
  mMsg.mMsgId  = mMsgId;
  mMsg.mNotice  = mNotice;
  mMsg.mValue1  = mValue;

  if(mDSize>0 && NULL != mShortData)
    memcpy(mMsg.mShortData, mShortData, mDSize);

  iRet = MsgMgrSendMsg(mSenderSoket,mRecerPort, (char*)&mMsg, sizeof(MsgShortMsg));
  return  iRet;
}

int32_t MsgSendLongMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                       const int32_t mNotice, const int32_t mValue, const char* mShortData, const int32_t mDSize)
{
  int32_t iRet = 0;
  MsgLongMsg mMsg;
  mMsg.mSender = mSender;
  mMsg.mRecer  = mRecer;
  mMsg.mMsgId  = mMsgId;
  mMsg.mNotice  = mNotice;
  mMsg.mValue1  = mValue;

  if(mDSize>0 && NULL != mShortData)
    memcpy(mMsg.mData, mShortData, mDSize);

  iRet = MsgMgrSendMsg(mSenderSoket, mRecerPort, (char*)&mMsg, sizeof(MsgLongMsg));
  return  iRet;
}

int32_t MsgSendRegisterMsgQ(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const pid_t mPid)
{
  int32_t iRet = 0;
  MsgRegisterMsgQ mMsg;
  mMsg.mSender = mSender;
  mMsg.mRecer  = mRecer;
  mMsg.mMsgId  = mMsgId;
  mMsg.mPid  = mPid;
  iRet = MsgMgrSendMsg(mSenderSoket, mRecerPort, (char*)&mMsg, sizeof(MsgRegisterMsgQ));
  return  iRet;
}

int32_t MsgSendPriLog(const int32_t mSenderSoket, const int mRecerPort,const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const char* mStr)
{
  int32_t iRet = 0;
  MsgLogMsg mMsg;
  mMsg.mSender = mSender;
  mMsg.mRecer  = mRecer;
  mMsg.mMsgId  = mMsgId;
  if((strlen(mStr)>0) && (strlen(mStr)<LOGER_MSG_SIZE))
  {
    strcpy(mMsg.mPriStr, mStr);
    iRet = MsgMgrSendMsg(mSenderSoket, mRecerPort, (char*)&mMsg, sizeof(MsgLogMsg));
  }
  return  iRet;
}

//shared mem

int32_t ShmCtl(const char* pFilePath, const int shmSize, int shmflg)
{
  int iRet = 0;
  key_t mkey = ftok(pFilePath, 0x11);
  if(mkey < 0)
  {
    perror("ftok failed\n");
    return -1;
  }

  if((iRet = shmget(mkey, shmSize, shmflg)) < 0)
  {
    perror("shmget failed\n");
    return -1;
  }
  return iRet;
}

int32_t ShmDestroy(const int shmId)
{
  int32_t iRet = 0;
  if(shmctl(shmId, IPC_RMID, NULL) < 0)
  {
    perror("destroy shm failed\n");
    return -1;
  }
  return iRet;
}

int32_t getShm(const char* pFilePath, const int shmSize)
{
  return ShmCtl(pFilePath, shmSize, IPC_CREAT);
}

int32_t createShm(const char* pFilePath, const int shmSize)
{
  return ShmCtl(pFilePath, shmSize, IPC_CREAT|IPC_EXCL|0666);
}


void setFdNonblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        printf("setFdNonblocking fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        printf("setFdNonblocking fcntl F_SETFL fail");
    }
}

void setFdblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        printf("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | ~O_NONBLOCK) < 0) {
        printf("setFdblocking fcntl F_SETFL fail");
    }
}

void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
    printf( "setsockopt failed:\n");
}

int32_t createSoket(uint32_t mMyPort,const char *mMyIpV4Str, int mWaitSec, int mWaitUsec) {
  int32_t iRet = 0;

  if((iRet = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      printf( "socket creat Failed");
      return -1;
  }

  if((mWaitSec <=0) && (mWaitUsec <= 0))
  {
      setFdNonblocking(iRet);
  } else
  {
      setFdTimeout(iRet, mWaitSec, mWaitUsec);
  }
  struct sockaddr_in  mMyAddr;
  //my
  bzero(&(mMyAddr), sizeof(mMyAddr));
  mMyAddr.sin_family = AF_INET;
  mMyAddr.sin_port = htons(mMyPort);
  mMyAddr.sin_addr.s_addr = inet_addr(mMyIpV4Str);

  if(bind(iRet, (struct sockaddr*)&(mMyAddr), sizeof(mMyAddr))<0)
  {
    printf("error= %d,err_str=%s,%s\n",errno,strerror(errno),mMyIpV4Str);
    return -1;
  }

  return iRet;
}

void push_back_strlist(strlist* strHeader, strlist * str)
{
    if(strHeader==NULL || str == NULL)
      return;
    //第一个元素
    if(strHeader->next == NULL)
    {
        strHeader->next = str;
        str->parent = strHeader;
        strHeader->parent = str;
    }
    else{
        str->parent = strHeader->parent;
        strHeader->parent->next = str;
        strHeader->parent = str;
    }
    strHeader->child_size++;
}

void destroy_strlist(strlist* strHeader)
{
    if(strHeader==NULL)
      return;
    
    strlist* strP;
    while (strHeader->child_size>0)
    {
        strP = strHeader->parent;
        strHeader->parent = strP->parent;
        free(strP);
        strHeader->child_size--;
    }
}
