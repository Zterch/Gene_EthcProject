/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-14 00:29:26
 * @LastEditors: lyq
 */
/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
#ifndef SYS_ARMS_IPC_HPP
#define SYS_ARMS_IPC_HPP
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>


//定义字符串链表结构体
typedef struct strlist{
    char str[128];/*> 最大存储128个字符*/

    int child_size;
    struct strlist* next;
    struct strlist* parent;
}strlist;

///Common part
int32_t MsgMgrInit(int magKey);
int32_t  MsgMgrDeinit(int msqid);
int32_t MsgMgrSendMsg(const int32_t mSenderSoket, const int mRecerPort, const void *ptr, size_t length);
int32_t  MsgReceiveMsg(int msqid, void* ptr);
int32_t MsgSendRegisterMsgQ(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const pid_t mPid);
int32_t MsgSendNotice(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);
int32_t MsgSendPriLog(const int32_t mSenderSoket, const int mRecerPort,const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const char* mStr);
int32_t MsgSendShortMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                        const int32_t mNotice, const int32_t mValue, const char* mShortData, const int32_t mDSize);

int32_t MsgSendLongMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                       const int32_t mNotice, const int32_t mValue, const char* mShortData, const int32_t mDSize);


//Shared Memory
int32_t ShmCtl(const char* pFilePath, const int shmSize, int shmflg);
int32_t ShmDestroy(const int shmId);
int32_t getShm(const char* pFilePath, const int shmSize);
int32_t createShm(const char* pFilePath, const int shmSize);


///////////////////////////////////////////////////////////////////////////////
int32_t createSoket(uint32_t mMyPort,const char *mMyIpV4Str, int mWaitSec, int mWaitUsec);
void setFdNonblocking(int sockfd);
void setFdblocking(int sockfd);

void setFdTimeout(int sockfd, const int mSec, const int mUsec);
//int32_t createSoket(uint32_t mMyPort,const char *mMyIpV4Str) ;

//////////////////////////////////////////////////////////////////////////////

void push_back_strlist(strlist* strHeader, strlist * str);
void destroy_strlist(strlist* strHeader);

#endif // SYS_ARMS_IPC_HPP
