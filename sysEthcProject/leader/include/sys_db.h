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
#ifndef __DB_H__
#define __DB_H__
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "list.h"
#include "sys_ctl.h"

#define DB_FILE "/etc/arms/sys_arms.db"

extern const char * db_arms_tables[8];
extern const char *db_frame_table;
extern param_dic arms_params_names[];
extern const char * db_interpn_tables[8];

extern param_dic frame_params_names[];
/*******************************************配置参数、插值表**************************************/
typedef struct coordinate
{ 
  float x,y;
} coordinate,interpn_value;


typedef struct strs_list
{ 
    char str[100];
    int str_cnt;
    struct list_head list;
} strs_list;

typedef struct interpn_unit
{
  //行列主键
  int xi,yj;
  //电机位置、倾角仪角度
  coordinate xy;
  interpn_value vas;
  //预留
  float spare[2];
} interpn_unit;

//四边形结构体
typedef struct quads_unit
{ 
  coordinate left_up;
  coordinate left_down;
  coordinate right_up;
  coordinate right_down;

  interpn_value left_up_vaule;
  interpn_value left_down_vaule;
  interpn_value right_up_vaule;
  interpn_value right_down_vaule;
} quads_unit;

typedef struct quads_unit_head
{ 
  uint8_t isValid;
  int rows;
  int cols;
  quads_unit *pQus;
  //新插入的插值点指针
  interpn_unit *mNewInterpns;
} quads_unit_head;

quads_unit* generate_quads(interpn_unit* interpns,int inter_rows,int inter_cols);
quads_unit* find_quads(quads_unit_head* qHead,float coor_x,float coor_y);
interpn_value interpn_magic(quads_unit_head* qHead,float coor_x,float coor_y);

void deinit_strs_list(strs_list *p_list);
void db_tables_name(char* dbname,strs_list *strs);

/*******************************************配置参数、插值表 END**************************************/
void  deinit_params(param_list * p_list);
float get_list_value(param_list *plist,char *key);

void  deinit_table_params(db_table * p_table);
/******************************对数据库的操作,配置文件*********************************/
void create_param_table(char* ,const char* );
void read_param_db(char* ,const char* ,param_list *);
void insert_params_db(char* dbname,const char* table,param_dic * params);
void update_param_db(char* dbname,const char* table,char* keyname,float value);
void delete_table(char* dbname,const char* table);

void delete_table_entry(char* dbname,const char* table,const char* key,const char* value);
void read_param_db_table(char* ,const char* ,db_table *);
int get_value_from_dbtable(char* dbname,const char* p_table,char *key,double * value);

int update_table_entry(char* dbname,const char* table,const char* key,const char* key_const,const char* value_name,const char* value);
/**********************************对数据库的操作*************************************/

/**********************************对数据库的操作,插值表********************************/
void create_interpn_table(char* ,const char* );
void read_interpn_row_db(char* dbname,const char* table,int * interprow);
void read_interpn_col_db(char* dbname,const char* table,int * interpcol);
void read_interpn_db(char* dbname,const char* table,interpn_unit * interpns);
void insert_interpn_db(char* dbname,const char* table,interpn_unit *params,int row,int col);
//void update_interpn_db(char* dbname,const char* table,char* keyname,float value);

//以下是对interp 数组的操作。得到i 行 j列的数据
interpn_unit * get_interpn(interpn_unit * interpns, int i, int j, int cols);

/**********************************对数据库的操作*************************************/


#endif // __DB_H__