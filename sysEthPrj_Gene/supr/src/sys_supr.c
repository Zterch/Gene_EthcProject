/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_supr.c - Supr 进程：初始化 upper listener，循环接收上位机命令并转发给 master (与 sysEthcProject 一致)
********************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include "sys_supr.h"
#include "sys_upper_listener.h"

static volatile int suprWorking = 1;

static void signalHandle(int sig)
{
    (void)sig;
    suprWorking = 0;
}

int32_t dmsAppStartUp(void)
{
    int32_t iRet = 0;
    signal(SIGINT, signalHandle);
    signal(SIGTERM, signalHandle);

    iRet = initUpperListener();
    if (iRet != 0) {
        printf("supr: initUpperListener failed\n");
        return -1;
    }
    printf("supr: 运行中，等待上位机 UDP 命令 (端口 LISTENER_TO_UPPER_PORT)，转发至 master\n");
    while (suprWorking) {
        LoopRecUpperMsg();
    }
    printf("supr: 退出\n");
    return iRet;
}
