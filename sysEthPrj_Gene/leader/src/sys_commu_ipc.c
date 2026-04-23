/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_commu_ipc.c - Send notice to leader, send data to supr (upper)
********************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "sys_share_conf.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"
#include "sys_ctl.h"

int32_t sendDataToUpper(int32_t mSoket, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue, const char *mData, const int32_t mDSize)
{
    if (mSoket < 0) return -1;
    return MsgSendLongMsg(mSoket, get_port("LISTENER_LOCAL_PORT"), MN_ID_LEADER, MN_ID_SUPR, mMsgId, mNotice, mValue, mData, mDSize);
}

int32_t sendNoticeToLeader(int32_t mSocket, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
    if (mSocket < 0) return -1;
    return MsgSendNotice(mSocket, get_mn_port(MN_ID_LEADER), MN_ID_LEADER, MN_ID_LEADER, mMsgId, mNotice, mValue);
}
