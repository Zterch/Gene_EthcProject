/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-13 07:46:44
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_leader.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file: 定时器线程。该模块主要控制算法周期运行
*
********************************************************************************/
#ifndef SYS_MASTER_HPP
#define SYS_MASTER_HPP
#include "sys_ctl.h"
#include "sys_leader_defs.h"

void* master_arms_threadEntry(void* );

int32_t armsSetSpeed(ARMS_MASTER_THREAD_INFO *,int ,float ,float ,float ,float );
int32_t armsSetSpeedXY(ARMS_MASTER_THREAD_INFO *,int ,float ,float ,float );
int32_t armsSetSpeedZ(ARMS_MASTER_THREAD_INFO *,int ,float );

//算法模块出现意外，需要停止算法运行
int32_t magic_stop_i(ARMS_MASTER_THREAD_INFO *pTModule,int armsid);

#endif // SYS_MASTER_HPP
