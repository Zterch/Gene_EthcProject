/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_share_daemon.c - Thread create, CPU affinity (simplified, no SCHED_FIFO requirement)
********************************************************************************/
// 修改为条件定义
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>

#include "sys_share_daemon.h"

pthread_t hiCreateThread(const char *cThreadName, void *(*thread_start)(void *), int32_t iPriority, void *pModule)
{
    (void)iPriority;
    pthread_t tid;
    if (pthread_create(&tid, NULL, thread_start, pModule) != 0) {
        printf("pthread_create failed\n");
        return (pthread_t)0;
    }
    if (cThreadName && pthread_setname_np(tid, cThreadName) != 0) { /* ignore */ }
    return tid;
}

void hiSetThreadsched(pthread_t pPid, const int32_t iPriority)
{
    (void)pPid;
    (void)iPriority;
    /* Optional: set SCHED_FIFO if running as root */
}

void hiSetCpuAffinity(int mCpuI)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(mCpuI, &mask);
    pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
}
