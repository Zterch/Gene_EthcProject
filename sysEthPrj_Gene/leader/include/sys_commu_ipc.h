/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_commu_ipc.h - Leader send to supr (UDP)
********************************************************************************/
#ifndef __COMMU_IPC_H__
#define __COMMU_IPC_H__

#include <stdlib.h>
#include <stdint.h>

int32_t sendNoticeToLeader(int32_t mSocket, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);
int32_t sendDataToUpper(int32_t mSoket, const int32_t mMsgId, const int32_t mNotice, const int32_t mValue, const char *mData, const int32_t mDSize);

#endif
