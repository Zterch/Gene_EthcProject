/********************************************************************************
* Copyright (c) 2026-2026 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_leader.hpp
* Module :
* Create on: 2026/2/5
* Author: 周佳燚
* Email: 2312947778@qq.com
* Description about this header file:系统中的领导线程模块，此模块负责创建所有线程，管理其他模块，周期的发送
*消息通知其他模块。
*
********************************************************************************/
#ifndef SYS_LEADER_HPP
#define SYS_LEADER_HPP


#include <stdint.h>

// leader 进程入口函数声明
int32_t leaderAppStartUp();

#endif 