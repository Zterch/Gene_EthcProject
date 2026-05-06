/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-28 09:22:19
 * @LastEditors: Zterch
 * @LastEditTime: 2026-04-29 16:59:17
 * @FilePath: /Gene_EthCtl_Bd/sysEthcProject/sys_share/sys_share_conf.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-18 12:11:19
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_share_def.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:
*
********************************************************************************/
#ifndef SYS_SHARE_CONF_H
#define SYS_SHARE_CONF_H

#include <stdint.h>

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
#define CPU_LOGER   4
#define CPU_LISTENER  2


//////////////////////////////////// System supr conf  ///////////////////////////////////////
//最大的模块数量,leader、timer、loger
#define MODULES_NUMS (1)
//进程模块的名字
//extern const char MN_MODULES_NAME[1][15];


/********************************leader conf************************************/
//master的名字

//一个主站下，最多拥有的机械臂个数
#define MASTER_ARMS_MAX_SIZE 7

//系统中所有线程模块ID,增加模块时要做对于修改
typedef enum {
  MN_ID_SUPR,
  MN_ID_LEADER,
  MN_ID_MAX,

} MODULE_NAME_ID;
/****************************supr leader使用ip及端口*************************/
#define    MN_LocalIpV4Str "127.0.0.1"



/****************************leader下的master线程使用*************************/
//master线程的id，0 1 2：上层、中层、底层
typedef enum {
  MN_ID_MASTER_ARMS,
  MN_ID_MASTER_CHASSIS,
} MASTER_ID;
//master线程使用的端口


/*********************************upper线程使用ip及端口,supr和leader线程都需要使用****************************/
//创建soket
//#define      mLocalUpperSoketPort 10021
//#define      mLocalLoggerSoketPort 10022


#endif // SYS_SHARE_CONF_H
