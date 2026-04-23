/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-18 13:35:08
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
#ifndef SYS_SHARE_DEFS_H
#define SYS_SHARE_DEFS_H
#include <stdio.h>

//////////////////////////////////// System internal structure  /////////////////////////////////////

////////////////////////////////////
//定义线程模块运行的状态
typedef enum
{
  M_STATE_INIT = 0,
  M_STATE_RUN,
  M_STATE_STOP,
  M_STATE_QUIT,
  M_STATE_WAIT,
} M_STATE;

//定义线程模块应答的状态
typedef enum
{
  ACK_STATE_NULL = 100,
  ACK_STATE_INIT_OK,
  ACK_STATE_QUIT_OK,
} ACK_STATE;


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
} SOMANETN_MODE;

//msg header
typedef struct {
  //在主站下的绝对位置
  int absPos;

  //别名位置
  int alias;

  //别名位置偏移
  int aliasOffset;
}SlavesPos;

//配置sdo的结构体
typedef struct
{
    uint8_t   armsId; /**< 上层编号0 1 2...，随后继续编号下层 */
    uint8_t   motorId; /**< 每一个arms下先连接EK1100 -> EK1110-> motor1-> motor2-> motor3-> motor4，然后链接硬石开发板、随后链接EC3045. */

    uint16_t  slave_position; /**< Slave position. */
    uint16_t  index; /**< Index of the SDO. */
    uint8_t   subindex; /**< Subindex of the SDO. */
    uint8_t   data[4] ;/**< Data buffer to download. */
    uint8_t   data_size; /**< Size of the data buffer. */
}sdo_download_t;

//读取sdo的结构体
typedef struct
{
    uint8_t   masterId; /**< master id 0 1 2 */

    uint16_t  slave_position; /**< Slave position. */
    uint16_t  index; /**< Index of the SDO. */
    uint8_t   subindex; /**< Subindex of the SDO. */
    uint8_t   target[4] ;/**< Data buffer upload. */
    uint8_t   target_size; /**< Size of the data buffer. */
}sdo_upload_t;

#endif // SYS_SHARE_DEFS_H
