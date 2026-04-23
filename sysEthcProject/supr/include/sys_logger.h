/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-04-17 07:07:15
 * @LastEditTime: 2024-06-16 23:26:13
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_logger.h
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file: 日志线程，主要是读写配置文件，打印loger日志运行
*
********************************************************************************/
#ifndef SYS_logger_H
#define SYS_logger_H

void* logger_threadEntry(void* );

#endif // SYS_logger_H
