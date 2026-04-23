/*** 
 * @Description: 
 * 
 * @Author: lyq
 * @Date: 2024-06-16 23:03:55
 * @LastEditTime: 2024-06-16 23:56:13
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_daemon.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:，修改线程优先级，线程绑定核模块
*
********************************************************************************/
// 防止多次导入
#ifndef __CFG_H__
#define __CFG_H__
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "list.h"
 //兼容 C++
#ifdef  __cplusplus
extern "C" {
#endif // __cplusplus

#define IP_C_SIZE     16
// 定义接口时 , 如果函数形参用作输入数据时 , 可以在形参名很后面添加 /*in*/ 注释
//
typedef struct param_dic
{
  char key[64];
  float value;
  char description[256];
}param_dic;

typedef struct table_entrys
{
  char **colValue;
}table_entrys;

typedef struct db_table
{
  int rows;
  int cols;
  char **colName;
  table_entrys *table_value;
}db_table;

typedef struct param_list
{
  //参数键值对
  param_dic key_value;

  int param_cnt;
  struct list_head list;
} param_list;

#define LIMIT_MAX(var,max) ((var)=((var)>(max) ? (max):(var) ))
#define LIMIT_MIN(var,min) ((var)=((var)<(min) ? (min):(var) ))

// 获取配置项
//int read_config_file(char *filename /*in*/, char *key /*in*/, char * value/*in out*/, int * value_len /*out*/);
// 写出 / 更新配置项
//int write_or_update_config_file(char *filename /*in*/, char *key /*in*/, char *value/*in*/, int value_len /*in*/);

//char * get_const_param_value_str(char *key);

char * get_ip(char *key);

int get_local_ip(const char *eth_inf, char *ip);

uint32_t get_port(char *key);

//得到模块:supr  leader ...的端口
int get_mn_port(int index);
int get_supr_port();

//得到leader下主站的端口
int get_mn_master_port(int index);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __CFG_H__