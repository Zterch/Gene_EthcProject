/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_share_ipc.c - UDP send, socket create (aligned with sysEthcProject)
********************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "sys_share_ipc.h"
#include "sys_share_messages.h"

int32_t MsgMgrSendMsg(const int32_t mSenderSoket, const int mRecerPort, const void *ptr, size_t length)
{
    int32_t iRet = 0;
    struct sockaddr_in mPeerAddr;
    memset(&mPeerAddr, 0, sizeof(mPeerAddr));
    mPeerAddr.sin_family = AF_INET;
    mPeerAddr.sin_port = htons(mRecerPort);
    mPeerAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    iRet = sendto(mSenderSoket, ptr, length, 0, (struct sockaddr *)&mPeerAddr, sizeof(mPeerAddr));
    return iRet;
}

int32_t MsgSendNotice(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
    int32_t iRet = 0;
    MsgHeader mMsg;
    mMsg.mSender = mSender;
    mMsg.mRecer = mRecer;
    mMsg.mMsgId = mMsgId;
    mMsg.mNotice = mNotice;
    mMsg.mValue1 = mValue;
    iRet = MsgMgrSendMsg(mSenderSoket, mRecerPort, (char *)&mMsg, sizeof(MsgHeader));
    return iRet;
}

int32_t MsgSendShortMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                        const int32_t mNotice, const int32_t mValue, const char *mShortData, const int32_t mDSize)
{
    int32_t iRet = 0;
    MsgShortMsg mMsg;
    memset(&mMsg, 0, sizeof(mMsg));
    mMsg.mSender = mSender;
    mMsg.mRecer = mRecer;
    mMsg.mMsgId = mMsgId;
    mMsg.mNotice = mNotice;
    mMsg.mValue1 = mValue;
    if (mDSize > 0 && mShortData != NULL && mDSize < SHORT_MSG_SIZE)
        memcpy(mMsg.mShortData, mShortData, mDSize);
    iRet = MsgMgrSendMsg(mSenderSoket, mRecerPort, (char *)&mMsg, sizeof(MsgShortMsg));
    return iRet;
}

int32_t MsgSendLongMsg(const int32_t mSenderSoket, const int mRecerPort, const int32_t mSender, const int32_t mRecer, const int32_t mMsgId,
                       const int32_t mNotice, const int32_t mValue, const char *mShortData, const int32_t mDSize)
{
    int32_t iRet = 0;
    MsgLongMsg mMsg;
    memset(&mMsg, 0, sizeof(mMsg));
    mMsg.mSender = mSender;
    mMsg.mRecer = mRecer;
    mMsg.mMsgId = mMsgId;
    mMsg.mNotice = mNotice;
    mMsg.mValue1 = mValue;
    if (mDSize > 0 && mShortData != NULL && mDSize < LONG_MSG_SIZE)
        memcpy(mMsg.mData, mShortData, mDSize);
    iRet = MsgMgrSendMsg(mSenderSoket, mRecerPort, (char *)&mMsg, sizeof(MsgLongMsg));
    return iRet;
}

void setFdNonblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag >= 0)
        fcntl(sockfd, F_SETFL, flag | O_NONBLOCK);
}

void setFdblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag >= 0)
        fcntl(sockfd, F_SETFL, flag & ~O_NONBLOCK);
}

void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
    struct timeval timeout;
    timeout.tv_sec = mSec;
    timeout.tv_usec = mUsec;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

int32_t createSoket(uint32_t mMyPort, const char *mMyIpV4Str, int mWaitSec, int mWaitUsec)
{
    int32_t iRet = socket(AF_INET, SOCK_DGRAM, 0);
    if (iRet < 0) {
        printf("socket create failed\n");
        return -1;
    }
    if (mWaitSec <= 0 && mWaitUsec <= 0)
        setFdNonblocking(iRet);
    else
        setFdTimeout(iRet, mWaitSec, mWaitUsec);
    struct sockaddr_in mMyAddr;
    memset(&mMyAddr, 0, sizeof(mMyAddr));
    mMyAddr.sin_family = AF_INET;
    mMyAddr.sin_port = htons(mMyPort);
    mMyAddr.sin_addr.s_addr = inet_addr(mMyIpV4Str);
    if (bind(iRet, (struct sockaddr *)&mMyAddr, sizeof(mMyAddr)) < 0) {
        printf("bind failed port %u errno %d %s\n", mMyPort, errno, strerror(errno));
        close(iRet);
        return -1;
    }
    return iRet;
}

void push_back_strlist(strlist *strHeader, strlist *str)
{
    if (strHeader == NULL || str == NULL) return;
    if (strHeader->next == NULL) {
        strHeader->next = str;
        str->parent = strHeader;
        strHeader->parent = str;
    } else {
        str->parent = strHeader->parent;
        strHeader->parent->next = str;
        strHeader->parent = str;
    }
    strHeader->child_size++;
}

void destroy_strlist(strlist *strHeader)
{
    if (strHeader == NULL) return;
    strlist *p;
    while (strHeader->child_size > 0) {
        p = strHeader->parent;
        strHeader->parent = p->parent;
        free(p);
        strHeader->child_size--;
    }
}
