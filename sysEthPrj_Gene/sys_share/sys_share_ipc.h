/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_share_ipc.h - IPC (UDP message send, socket create) for General EtherCAT
********************************************************************************/
#ifndef SYS_SHARE_IPC_H
#define SYS_SHARE_IPC_H

#include <stdint.h>
#include <sys/types.h>

// 字符串链表（用于部分命令返回）
typedef struct strlist {
    char str[128];
    int child_size;
    struct strlist *next;
    struct strlist *parent;
} strlist;

int32_t MsgMgrSendMsg(const int32_t mSenderSoket, const int mRecerPort, const void *ptr, size_t length);
int32_t MsgSendNotice(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);
int32_t MsgSendShortMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                        const int32_t mNotice, const int32_t mValue, const char *mShortData, const int32_t mDSize);
int32_t MsgSendLongMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                       const int32_t mNotice, const int32_t mValue, const char *mShortData, const int32_t mDSize);

int32_t createSoket(uint32_t mMyPort, const char *mMyIpV4Str, int mWaitSec, int mWaitUsec);
void setFdNonblocking(int sockfd);
void setFdblocking(int sockfd);
void setFdTimeout(int sockfd, const int mSec, const int mUsec);

void push_back_strlist(strlist *strHeader, strlist *str);
void destroy_strlist(strlist *strHeader);

#endif
