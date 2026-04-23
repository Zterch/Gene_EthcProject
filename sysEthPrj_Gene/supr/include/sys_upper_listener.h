/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_upper_listener.h - UDP listener: upper -> supr -> master (aligned with sysEthcProject)
********************************************************************************/
#ifndef SYS_UPPER_LISTENER_H
#define SYS_UPPER_LISTENER_H

#include <stdint.h>

typedef struct {
    int32_t  mWorking;
    char     mThreadName[15];
    int32_t  mInitOk;
    pthread_t mthreadPid;
    uint8_t   mCpuAffinity;
} LISTENER_THREAD_INFO;

int32_t initUpperListener(void);
void LoopRecUpperMsg(void);

#endif
