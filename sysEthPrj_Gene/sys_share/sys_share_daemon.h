/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_share_daemon.h - Thread create, CPU affinity, priority
********************************************************************************/
#ifndef SYS_SHARE_DAEMON_H
#define SYS_SHARE_DAEMON_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sched.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

pthread_t hiCreateThread(const char *cThreadName, void *(*thread_start)(void *), int32_t iPriority, void *pModule);
void hiSetThreadsched(pthread_t pPid, const int32_t iPriority);
void hiSetCpuAffinity(int mCpuI);

#ifdef __cplusplus
}
#endif

#endif
