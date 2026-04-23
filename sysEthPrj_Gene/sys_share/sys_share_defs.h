/********************************************************************************
* Copyright (c) 2026-2026 Zterch.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_share_defs.hpp
* Module :
* Create on: 2026/2/5
* Author: 周佳燚
* Email: 2312947778@qq.com
* Description about this header file: 系统共享的一些定义
*
********************************************************************************/
#ifndef SYS_SHARE_DEFS_H
#define SYS_SHARE_DEFS_H

/////////////////////////////////////
//定义线程模块运行的状态
typedef enum
{
  M_STATE_INIT = 0,
  M_STATE_RUN,
  M_STATE_STOP,
  M_STATE_QUIT,
  M_STATE_WAIT,
} M_STATE;


//igh 扫描从站信息结构体
/*
 +1 Profile position mode
 +3 Profile velocity mode
 +4 Torque profile mode
 ©2024SynapticonGmbH|Steinbeisstraße1|D-71101Schönaich
 SOMANETNode
 Documentationv5.4.21|Built2024-05-27 329/1383
Value for 0x6060 Control Mode
 +6 Homing mode
 +8 Cyclic Syncronous Position Mode
 +9 Cyclic Syncronous Velocity Mode
 +10 Cyclic Syncronous Torque Mod
*/
typedef enum
{
  MODE_PP = 1,
  MODE_PV = 3,
  MODE_PT = 4,
  MODE_HOMING = 6,
  MODE_CSP = 8,
  MODE_CSV = 9,
  MODE_CST = 10,
} KAISER_MODE;

#endif // SYS_SHARE_DEFS_H