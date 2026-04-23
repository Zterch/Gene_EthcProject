/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_leader.c - Leader process: init socket, start master thread, wait (aligned with sysEthcProject)
********************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include "sys_leader.h"
#include "sys_master.h"
#include "sys_share_conf.h"
#include "sys_leader_defs.h"
#include "sys_share_messages.h"
#include "sys_share_ipc.h"
#include "sys_share_daemon.h"
#include "sys_ctl.h"
#include "sys_commu_ipc.h"

const char MN_MASTER_NAME[24] = {"MN_MASTER_ARMS"};

ARMS_MASTER_THREAD_INFO mArmsModule;
static int32_t mFrameId = 0;

static int32_t initSupr(void);
static int32_t startModules(void);

int32_t leaderAppStartUp(void)
{
    int32_t iRet = 0;
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);

    iRet = initSupr();
    if (iRet != 0) {
        printf("leader initSupr failed\n");
        return -1;
    }
    iRet = startModules();
    if (iRet != 0) {
        printf("leader startModules failed\n");
        return -1;
    }
    pthread_join(mArmsModule.mPid, NULL);
    printf("leader quit\n");
    return iRet;
}

static int32_t initSupr(void)
{
    hiSetCpuAffinity(CPU_LEAD);
    hiSetThreadsched(pthread_self(), PRI_LEAD);
    return 0;
}

static int32_t startModules(void)
{
    mArmsModule.mWorking = true;
    mArmsModule.mCpuAffinity = CPU_MASTER;
    mArmsModule.mthreadMasterId = 0;
    mArmsModule.masterId = 0;
    mArmsModule.slaves_size = 0;
    mArmsModule.mSerialWorker = NULL;
    mArmsModule.mIdentifier = mFrameId;
    mArmsModule.pTChassis = NULL;
    mArmsModule.mSoket = -1;
    usleep(10000);

    mArmsModule.mPid = hiCreateThread(MN_MASTER_NAME, master_arms_threadEntry, PRI_MASTER, &mArmsModule);
    if (mArmsModule.mPid == (pthread_t)0) {
        printf("leader create master thread failed\n");
        return -1;
    }
    return 0;
}
