/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_daemon.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:创建线程模块（后期修改为多进程），修改线程优先级，线程绑定核模块
*
********************************************************************************/
#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "sys_share_conf.h"
#include "sys_commu_ipc.h"
#include "sys_share_ipc.h"

int32_t sendDataToUpper(int32_t mSoket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iRet = 0;

  if(mSoket<0)
    return -1;

  iRet = MsgSendLongMsg(mSoket, get_port("LISTENER_LOCAL_PORT"), MN_ID_LEADER, MN_ID_SUPR, mMsgId, mNotice, mValue, mData, mDSize);

  return iRet;
}

int32_t sendNoticeToLeader(int32_t mSocket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
  int iRet = 0;

  if(mSocket<0)
    return -1;

  iRet = MsgSendNotice(mSocket, get_mn_port(MN_ID_LEADER),MN_ID_LEADER, MN_ID_LEADER,mMsgId, mNotice, mValue);

  return iRet;
}

int32_t sendDataToChassis(int32_t mSoket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iRet = 0;

  if(mSoket<0)
    return -1;

  iRet = MsgSendLongMsg(mSoket, get_mn_master_port(MN_ID_MASTER_CHASSIS), MN_ID_LEADER, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);

  return iRet;
}
