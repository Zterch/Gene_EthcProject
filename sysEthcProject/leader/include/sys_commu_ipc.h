/*** 
 * @Description: 
 * 
 * @Author: lyq
 * @Date: 2024-06-16 23:03:55
 * @LastEditTime: 2024-06-16 23:56:13
 * @LastEditors: lyq
 */
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
* Description about this header file:，修改线程优先级，线程绑定核模块
*
********************************************************************************/
// 防止多次导入
#ifndef __COMMU_IPC_H__
#define __COMMU_IPC_H__
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "list.h"
#include "sys_ctl.h"


//发送给upper上位机
int32_t sendDataToUpper(int32_t mSoket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);

//发送给leader通知
int32_t sendNoticeToLeader(int32_t mSocket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);

//吊杆master发送数据到底盘master
int32_t sendDataToChassis(int32_t mSoket,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);

#endif // __COMMU_IPC_H__