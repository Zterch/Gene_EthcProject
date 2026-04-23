/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C Header File(*.h)
* File Name :sys_arms_daemon.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:创建线程模块（后期修改为多进程），修改线程优先级，线程绑定核模块
*
********************************************************************************/
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>

#define __USE_GNU
#include <sched.h>
#include <pthread.h>

#include "sys_share_daemon.h"

/******************************************************************************
* @param vDaemonName : [in/out]待创建进程的名字，字符串类型

* @return Descriptions
* @TUDO
******************************************************************************/
void hiCreateDaemon(const char *vDaemonName) {

  //This is daemon since ppid==1
  if (getppid() == 1) {
    exit(0);
  }

  pid_t hiPid = fork();

  if (hiPid < 0)
  {
    //TODO Something wrong
    exit(1);
  }
  if (hiPid > 0)
  {
    exit(0);
  }

  pid_t hiSid = setsid();
  if (hiSid < 0) {
    //TODO Something wrong
    exit(2);
  }

  //Redirect standard files to /dev/null
  if (freopen("/dev/null", "r", stdin) == NULL) {
    //TODO Something wrong
  }
  if (freopen("/dev/null", "w", stdout) == NULL) {
    //TODO Something wrong
  }
  if (freopen("/dev/null", "w", stderr) == NULL) {
    //TODO Something wrong
  }

  //Set umask to 022, it mean file mode mask is 0644 */
  umask(022);

  //Change current working directory
  if (chdir("/") < 0) {
    exit(3);
  }

  //Cancel certain signals */
//  signal(SIGCHLD, SIG_DFL); /* A child process dies */
//  signal(SIGTSTP, SIG_IGN); /* Various TTY signals */
//  signal(SIGTTOU, SIG_IGN);
//  signal(SIGTTIN, SIG_IGN);
//  signal(SIGTERM, SIG_DFL); /* Die on SIGTERM */
  //syslog(5, "Create Daemon OK, %s", vDaemonName);
}


/******************************************************************************
* @param cThreadName : [in]待创建线程的名字，字符串类型
* @param thread_start : [in]待创建线程的函数入口
* @param iPriority : [in]待创建线程的优先级
* @param pModule : [in]待创建线程的参数指针
* @return Descriptions
* @exception pthread_t: 此函数返回新创建的线程的pid
******************************************************************************/
pthread_t hiCreateThread(const char *cThreadName,
                         void *(*thread_start)(void *),
                         int32_t iPriority,
                         void* pModule) {

  pthread_t pthreadId;
  int32_t iRet;
  pthread_attr_t attr;
  struct sched_param schedParam;

  // 1. 初始化线程属性对象
  // 为什么要做：为后续的属性设置准备一个干净的属性对象
  iRet = pthread_attr_init(&attr);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  // 2. 设置线程分离状态为可连接（JOINABLE）
  // 为什么要做：JOINABLE线程允许其他线程通过pthread_join等待其结束并获取返回值
  // 与DETACHED的区别：DETACHED线程结束后自动释放资源，但不能被join
  iRet = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  // 3. 设置调度策略继承方式为显式调度
  // 为什么要做：告诉系统不要从父线程继承调度策略，而是使用我们明确设置的策略
  // PTHREAD_EXPLICIT_SCHED：使用attr中显式设置的调度策略和参数
  // PTHREAD_INHERIT_SCHED：继承创建者的调度策略（默认）
  iRet = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  // 4. 设置调度策略为SCHED_FIFO（先进先出实时调度）
  // 为什么要做：SCHED_FIFO是实时调度策略，确保高优先级线程立即运行
  // SCHED_FIFO特点：相同优先级按队列顺序，高优先级可抢占低优先级
  // SCHED_RR：时间片轮转，更适合分时系统
  iRet = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  // 5. 获取当前的调度参数（主要是为了获取默认值）
  // 为什么要做：我们需要修改优先级参数，但保持其他参数不变
  iRet = pthread_attr_getschedparam(&attr, &schedParam);
  if (iRet != 0) {
//something wrong
      printf("error2\n");
  }

  // 6. 设置线程优先级
  // 为什么要做：实时线程需要明确的优先级设置以确保及时响应
  // Linux优先级范围：通常1-99（数字越大优先级越高），0为普通线程
  // 需要root权限或CAP_SYS_NICE能力才能设置高优先级
  schedParam.sched_priority = iPriority;
  iRet = pthread_attr_setschedparam(&attr, &schedParam);
  if (iRet != 0) {
//something wrong
      printf("error3\n");
  }

  // 7. 设置线程竞争范围为系统级（PTHREAD_SCOPE_SYSTEM）
  // 为什么要做：系统级竞争意味着线程与系统中所有线程竞争CPU
  // PTHREAD_SCOPE_SYSTEM：与系统所有线程竞争（实时系统常用）
  // PTHREAD_SCOPE_PROCESS：只与进程内线程竞争（较少使用）
  iRet = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
  if (iRet != 0) {
//something wrong
      printf("error4\n");
  }

  // 8. 创建线程（核心步骤）
  // 为什么要做：使用配置好的属性创建新线程
  // 参数解释：
  // &pthreadId: 输出参数，存储新线程ID
  // &attr: 线程属性（包含上面所有的设置）
  // thread_start: 线程入口函数
  // pModule: 传递给线程函数的参数
  iRet = pthread_create(&pthreadId, &attr, thread_start, pModule);
  if (iRet != 0) {
    //some thing wrong
    printf("error5:%d\n",iRet);
  }

  // 9. 设置线程名称（用于调试和系统监控）
  // 为什么要做：给线程起一个有意义的名称，便于调试和性能分析
  // 在gdb、htop、ps等工具中可以看到线程名称
  iRet = pthread_setname_np(pthreadId, cThreadName);
  if (iRet != 0) {
    //some thing wrong
  }

  // 10. 销毁线程属性对象
  // 为什么要做：属性对象只在创建线程时需要，创建完成后可以释放
  // 注意：销毁属性对象不会影响已创建的线程
  iRet = pthread_attr_destroy(&attr);
  if (iRet != 0) {
    //some thing wrong
  }

  return pthreadId;
}

/******************************************************************************
* @param pPid : [in]待设置线程的pid
* @param iPriority : [in]待设置线程的优先级
* @return Descriptions
* 此函数设置线程的优先级
******************************************************************************/
void hiSetThreadsched(pthread_t pPid, const int32_t iPriority) {
  struct sched_param schedParam;
  memset(&schedParam, 0, sizeof(schedParam));
  schedParam.sched_priority = iPriority;
  pthread_setschedparam(pPid, SCHED_FIFO, &schedParam);
  return;
}

/******************************************************************************
* @param mCpuI : [in]当前线程要绑定的cpu号，**特别注意自己控制器有多少核
* @return Descriptions
* 此函数将当前线程绑定到mCpuI核上
******************************************************************************/
void hiSetCpuAffinity(int mCpuI)
{
  //return;
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(mCpuI, &mask);

  pthread_setaffinity_np(pthread_self(), sizeof(mask),&mask);
}

//end of the file
