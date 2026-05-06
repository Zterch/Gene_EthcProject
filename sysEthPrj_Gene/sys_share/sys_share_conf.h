/********************************************************************************
* Copyright (c) 2026-2026 Zterch.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_share_conf.hpp
* Module :
* Create on: 2026/2/5
* Author: 周佳燚
* Email: 2312947778@qq.com
* Description about this header file: 系统主站以及电机的一些参数信息
*
********************************************************************************/
#ifndef SYS_SHARE_CONF_H
#define SYS_SHARE_CONF_H

//////////////////////////////////// System  /////////////////////////////////////
//定义线程模块优先级
#define PRI_SUPR    60
#define PRI_MASTER  50
#define PRI_LEAD    60
#define PRI_LOGER   60
//定义线程模块的CPU亲和度，绑定核
#define CPU_SUPR    1
#define CPU_MASTER  2
#define CPU_LEAD    3

//一个主站下，最多拥有的电机个数
#define MASTER_ARMS_MAX_SIZE 8

typedef enum { MN_ID_SUPR, MN_ID_LEADER, MN_ID_MAX } MODULE_NAME_ID;
typedef enum { MN_ID_MASTER_ARMS = 0 } MASTER_ID;
#define MN_LocalIpV4Str "127.0.0.1"

#endif // SYS_SHARE_CONF_H