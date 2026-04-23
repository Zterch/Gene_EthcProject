/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-06-07 07:18:48
 * @LastEditTime: 2024-06-14 00:27:45
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_loger.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:日志模块，系统运行的情况记录在这个模块中。
*
********************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <sys/time.h>
#include "sys_master_vision.h"
#include "sys_master.h"
#include "sys_leader_defs.h"
#include "sys_ctl.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"

#define     TIMESPEC2NS(T)       ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************网口使用***************************/
static int32_t mVisionSoket = -1;


/****************************************************************************/
static uint64_t system_time_ns(void);
static void handle_close(SERIAL_THREAD_INFO *pSModule);

/******************************************************************************
* 功能：tsnorm函数将timespec中的纳秒，妙格式化。按照1m = 1000000000ns，
* @param ts : ts是时间结构体，包含：妙 纳秒
* @return Descriptions
******************************************************************************/
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

//获取当前系统时间
static uint64_t system_time_ns(void)
{
    struct timespec  rt_time;
    clock_gettime(CLOCK_REALTIME, &rt_time);
    uint64_t time = TIMESPEC2NS(rt_time);
    return time;
}

static int32_t instervalus(float value,float *tension_vec,int filtertimes)
{
    for (int  i = 0; i < filtertimes-1; i++)
    {
        tension_vec[i] = tension_vec[i+1];
    }
    tension_vec[filtertimes-1] = value;
}

static float Slid_avg_filter(float *fvec,int times)
{
    if(times <= 0)
      return 0;
    /*拉力计滤波*/
    float sumT = 0;
    for (int  j = 0; j < times; j++)
    {
        sumT += fvec[j];
    }
    return (sumT/times);
}

static int32_t handle_open(SERIAL_THREAD_INFO *pSModule)
{
    handle_close(pSModule);
    //创建soket.非阻塞模式
    mVisionSoket = createSoket(get_port("VISION_PORT"),get_ip("VISION_IP"), 0, 0);
    if(mVisionSoket  < 0) {
        printf( "vision socket creat Failed\n");
        return -1;
    }
    return 0;
}

static void handle_close(SERIAL_THREAD_INFO *pSModule)
{
    //删除创建的soket
    if(mVisionSoket>0)
    {
        close(mVisionSoket);
        mVisionSoket = -1;
    }
}

static void handle_read(SERIAL_THREAD_INFO *pSModule)
{
    ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *)pSModule->pTModule;
    //读取vision视觉信息
    static uint32_t recCnt = 0;
    ssize_t recSize = 0;
    char mData[2000];
    struct sockaddr_in  mUpperPeerAddr;
    socklen_t upperNum;
    
    while((recSize=recvfrom(mVisionSoket, mData, sizeof(VISION_TYPE), MSG_DONTWAIT, (struct sockaddr*)&(mUpperPeerAddr), &upperNum))>0)
    {
        VISION_TYPE *pV = (VISION_TYPE *)mData;
        if(pV->mid>=0 && pV->mid<7)
        {
              FILTERING_PARAM * pf = (FILTERING_PARAM *)&pTModule->mF_Inf[pV->mid];
              instervalus(pV->x/1000.0, pf->camX_arr, pf->F_cam_n);
              instervalus(pV->y/1000.0, pf->camY_arr, pf->F_cam_n);
              instervalus(pV->tension, pf->tension_arr, pf->F_tension_n);

              pSModule->inclinxy[pV->mid][0] = pV->inclinx;
              pSModule->inclinxy[pV->mid][1] = pV->incliny;
        }

        struct timespec  timespec;
        clock_gettime(CLOCK_MONOTONIC, &timespec);
        recCnt++;
        //printf("recvfrom: %s %d %.4f %.4f time:(%d %d)\n", strerror(errno),recSize,pV->x,pV->y,timespec.tv_sec,timespec.tv_nsec/1000);
        //printf(" %d recvfrom:%.4f %.4f time:(%d %d)\n",pV->mid,pV->x,pV->y,timespec.tv_sec,timespec.tv_nsec/1000);
        pSModule->recVisionframesCnt[pV->mid]++;
    }    
    //if(recSize<0)
    //  printf("recvfrom: %s %d\n", strerror(errno),recSize);         
}

static void smooth_filter(SERIAL_THREAD_INFO *pSModule)
{
    ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *)pSModule->pTModule;
    //拉力计滤波
    for (int i = 0; i < pTModule->armsSize; i++)
    {
        if(pTModule->slaves_inf[i].childSlaveSize>4)
        {
            pTModule->camXY[i][0]= Slid_avg_filter(pTModule->mF_Inf[i].camX_arr,pTModule->mF_Inf[i].F_cam_n);
            pTModule->camXY[i][1]= Slid_avg_filter(pTModule->mF_Inf[i].camY_arr,pTModule->mF_Inf[i].F_cam_n);
            pTModule->tension[i] = Slid_avg_filter(pTModule->mF_Inf[i].tension_arr,pTModule->mF_Inf[i].F_tension_n);
        }
    }
}
/******************************************************************************
* 功能：子线程：master使用serial线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : args
* @return Descriptions
******************************************************************************/
void* visionThread(void* pModule)
{
    VISION_THREAD_INFO *pSModule = (VISION_THREAD_INFO *) pModule;
    ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *)pSModule->pTModule;

    unsigned int nDelay = 1000;        /* usec */
    //timer 定时器
    struct timespec  next, interval;
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int ret = 0;
    uint32_t  runCnt = 0;

    pSModule->mState = SR_NULL;
    memset(pSModule->recVisionframesCnt,0,sizeof(int32_t)*MASTER_ARMS_MAX_SIZE);
    uint32_t maxE = 0;
    while (pSModule->mWorking)
    {
        next.tv_sec  += interval.tv_sec;
        next.tv_nsec += interval.tv_nsec;
        tsnorm(&next);
        if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)))
        {
            if (ret != EINTR)
                printf("clock_nanosleep failed. errno:");
            continue;
        }

        struct timespec  st, et;
        clock_gettime(CLOCK_MONOTONIC, &st);  
        //数据滤波
        if(runCnt%5 == 0)
        {
            smooth_filter(pSModule);
        }

        //数据转换及滤波，0.005s
        //执行算法,5ms周期
        if(runCnt%3 == 0)
        {
            switch (pSModule->mState)
            {
                case SR_OPEN:{
                    handle_open(pSModule);
                    pSModule->mState = SR_RUN;
                    break;
                }
                case SR_CLOSE:{
                    handle_close(pSModule);
                    pSModule->mState = SR_NULL;
                    break;
                }
                case SR_RUN:{
                    //读取串口数据,并解析
                    handle_read(pSModule);
                    //printf("SR_RUN\n");
                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        //1s监控相机帧率
        if(runCnt%1000 == 0)
        {
            memcpy(pTModule->camFps,pSModule->recVisionframesCnt,sizeof(int32_t)*MASTER_ARMS_MAX_SIZE);
            memset(pSModule->recVisionframesCnt,0,sizeof(int32_t)*MASTER_ARMS_MAX_SIZE);
        }

        #if 0
        clock_gettime(CLOCK_MONOTONIC, &et);
        uint32_t newT = (et.tv_sec-st.tv_sec)*1000000 + (et.tv_nsec-st.tv_nsec)/1000;

        if(maxE< newT && pSModule->mState==SR_RUN)
          maxE = newT;
        printf("%d %d (%d %d) (%d %d)\n", newT,maxE,st.tv_sec,st.tv_nsec,et.tv_sec,et.tv_nsec);
        #endif
        runCnt++;
    }

    handle_close(pSModule);
    printf("serialThread exit\n\r");
}
