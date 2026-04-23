/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-13 09:05:14
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
* Description about this header file:创建线程模块（后期修改为多进程），修改线程优先级，线程绑定核模块
*
********************************************************************************/
#ifndef SYS_ARMS_DAEMON_HPP
#define SYS_ARMS_DAEMON_HPP

#define __USE_GNU
#include <sched.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// 兼容 C++
#ifdef  __cplusplus
extern "C" {
#endif // __cplusplus

void hiCreateDaemon(const char *vDaemonName);

pthread_t hiCreateThread(const char * cThreadName,
                         void *(*thread_start)(void *),
                         int32_t iPriority,
                         void* pModule);

//void hiGetThreadPri(pthread_t pPid);
void hiSetThreadsched(pthread_t pPid, const int32_t iPriority);

void hiSetCpuAffinity(int mCpuI);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // SYS_ARMS_DAEMON_HPP
