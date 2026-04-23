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
#ifndef SYS_MASTER_CHASSIS_H
#define SYS_MASTER_CHASSIS_H
#include "sys_ctl.h"
#include "sys_leader_defs.h"

void* master_chassis_threadEntry(void* );
#endif // SYS_MASTER_CHASSIS_H