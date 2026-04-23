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
* Description about this header file:创建线程模块（后期修改为多进程），修改线程优先级，线程绑定核模块
*
********************************************************************************/
#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "sys_db.h"
#include "sys_share_conf.h"
#include <sqlite3.h>

const char *db_arms_tables[8]={"arm1","arm2","arm3","arm4","arm5","arm6","arm7","arm8"};
const char *db_frame_table="frame";
const char * db_interpn_tables[8] = {"interpn1","interpn2","interpn3","interpn4","interpn5","interpn6","interpn7","interpn8"};
param_dic arms_params_names[] = 
{
    {"FOLLOW_KP",1.5,"随动算法kp值"},
    {"FOLLOW_KD",0.0,"随动算法kd值"},
    {"FOLLOW_KI",0.07,"随动算法ki值"},
    {"FOLLOW_MAX_X_SPEED",0.1,"随动算法X向最大速度"},
    {"FOLLOW_MAX_Y_SPEED",0.1,"随动算法Y向最大速度"},
 
    {"CYLINDER_MEDIAN_POS",0.022,"气缸中位磁栅尺值"},
    {"MEDIAN_RATIO_MAX_U",4.4,"气缸内中阀的最大电压值"},
    
    {"FLOATING_WEIGHT_EBSL",0.8,"称重算法ebsl值"},
    {"FLOATING_WEIGHT_WN",2.0,"称重算法wn值"},
    {"FLOATING_AIR_EBSL",0.8,"垂直算法气缸ebsl"},
    {"FLOATING_AIR_WN",2.0,"垂直算法气缸wn"},
    {"FLOATING_AIR_TARGET_PRESS",124600,"垂直气缸目标压力"},
    {"FLOATING_WINDLASS_EBSL",0.5,"垂直算法卷扬机ebsl"},
    {"FLOATING_WINDLASS_WN",5,"垂直算法卷扬机wn"},
    {"FLOATING_MAX_Z_SPEED",0.15,"垂直算法Z向最大速度"},

    {"FILTER_SIKO_TIMES",8,"磁栅尺均值滤波次数"},
    {"FILTER_CAM_TIMES",8,"相机检测均值滤波次数"},
    {"FILTER_TENSION_TIMNES",10,"拉力计均值滤波次数"},
    {"FILTER_INCLN_TIMES",1,"倾角仪均值滤波次数"},
    {"FILTER_SPEED_TIMES",10,"电机速度均值滤波次数"},
    {"FILTER_ABS_PRESS_TIMES",10,"绝压传感器均值滤波次数"},

    {"FILTER_FLOATING_MOTOR_Z_V",4,"Z轴垂直伺服控制命令滤波次数"},
    {"FILTER_FOLLOW_MOTOR_XY_V",5,"XY轴随动伺服控制命令滤波次数"},

    {"MOTOR_Z_ZORE",0,"Z轴编码器零位点"},
    {"MOTOR_Y_ZORE",0,"Y轴编码器零位点"},
    {"MOTOR_XS_ZORE",0,"X副轴编码器零位点"},
    {"MOTOR_XM_ZORE",0,"X主轴编码器零位点"},

    {"X_SOFT_LEFT_LIMIT",0,"X轴左边软限位"},
    {"X_SOFT_RIGHT_LIMIT",0,"X轴右边软限位"},
    {"Y_SOFT_FRONT_LIMIT",0,"Y轴前边软限位"},
    {"Y_SOFT_BACK_LIMIT",0,"Y轴后边软限位"},
    {"Z_SOFT_UP_LIMIT",0,"Z轴上边软限位"},
    {"Z_SOFT_DOWN_LIMIT",0,"Z轴下边软限位"},
 
    {"VISION_COEFFICIENT_X",1.0,"相机X向系数"},
    {"VISION_COEFFICIENT_Y",1.0,"相机Y向系数"},

    {"DOWN_OR_UP",0,"0:不存在,1:下层,2:上层"},
    
    {"",0,""}
};

param_dic frame_params_names[] = 
{
    {"CHASSIS_MAX_W_V",15.0,"底盘行走电机最大速度 mm/s"},
    {"CHASSIS_MAX_R_V",10.0,"底盘旋转电机最大转速 度/s"},
    {"MASTER_UPLOAD_MSG_MS",10.0,"主站上传数据时间间隔ms"},
    {"CHASSIS1_ZORE",0.0,"舵轮1旋转轴零位"},
    {"CHASSIS2_ZORE",0.0,"舵轮2旋转轴零位"},
    {"CHASSIS3_ZORE",0.0,"舵轮3旋转轴零位"},
    {"CHASSIS4_ZORE",0.0,"舵轮4旋转轴零位"},

    {"",0,""}
};

static int read_table_vaule_index = 0;

int select_table_rowcol_callback(void *pd, int argc, char **argv, char **azColName);

/**********************************对数据库的操作*************************************/
int callback(void *pd, int argc, char **argv, char **azColName)
{
    for (int  i = 0; i < argc; i++)
    {
        printf("%s = %s \n",azColName[i],argv[i]);
    }
    return 0;
}

int read_db_tables_name_callback(void *pd, int argc, char **argv, char **azColName)
{
    strs_list *p_list = (strs_list *)pd;
    for (int  i = 0; i < argc; i++)
    {
        strs_list *child_p = (strs_list*)malloc(sizeof(strs_list));
        memset((char*)child_p,0,sizeof(strs_list));
        strcpy(child_p->str,argv[i]);
        list_add_tail(&child_p->list,&p_list->list);
        p_list->str_cnt++;
    }
    return 0;
}

int readParamcallback(void *pd, int argc, char **argv, char **azColName)
{
    static uint32_t calli = 0;
    int i;
    int pI = 0;
    param_list * p_list = (param_list *)pd;

    //读取参数
    if(argc>2)
    {   
        if(p_list !=NULL && p_list->param_cnt>0)
        {
            param_list *i = NULL;
            param_list *tmp = NULL;
            list_for_each_entry_safe(i, tmp, &p_list->list, list) 
            {
                if(strcmp(i->key_value.key,argv[0]) == 0)
                {
                    i->key_value.value = strtof(argv[1],0);
                    return 0;
                }
            }
        }

        //创建新的链表
        param_list *child_p = (param_list*)malloc(sizeof(param_list));
        memset((char*)child_p,0,sizeof(param_list));
        strcpy(child_p->key_value.key, argv[0]);
        child_p->key_value.value = strtof(argv[1],0);
        strcpy(child_p->key_value.description,argv[2]);
        list_add_tail(&child_p->list,&p_list->list);
        p_list->param_cnt++;
    }

   return 0;
}

int readParamTableColNamecallback(void *pd, int argc, char **argv, char **azColName)
{
    db_table *p_table = (db_table *)pd;
    p_table->cols = argc;
    p_table->colName = malloc(argc*sizeof(char*));
    for (int i = 0; i < argc; i++)
    {
        p_table->colName[i] = malloc(strlen(azColName[i])+1);
        strcpy(p_table->colName[i],azColName[i]);
    }
    return 0;
}

int readParamTableValuescallback(void *pd, int argc, char **argv, char **azColName)
{
    table_entrys * table_value = (table_entrys *)pd;
    table_value[read_table_vaule_index].colValue = malloc(argc*sizeof(char*));
    for (int i = 0; i < argc; i++)
    {
        table_value[read_table_vaule_index].colValue[i] = malloc(strlen(argv[i])+1);
        strcpy(table_value[read_table_vaule_index].colValue[i],argv[i]);
    }
    read_table_vaule_index++;
    return 0;
}

void create_param_table(char* dbname,const char* table)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc )
    {
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //创建表格
    char sql[1024];
    sprintf(sql,"CREATE TABLE IF NOT EXISTS %s (name TEXT PRIMARY KEY,value REAL,info  TEXT); ",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    printf("%s created or open successfully\n",table);
    
    //查找参数名字的字段

    sqlite3_close(db);  
}

void delete_table(char* dbname,const char* table)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //创建表格
    char sql[1024];
    sprintf(sql,"DROP TABLE IF EXISTS %s ; ",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    printf("%s delete successfully\n",table);
    
    //查找参数名字的字段

    sqlite3_close(db);  
}

void delete_table_entry(char* dbname,const char* table,const char* key,const char* value)
{
    printf("%s %s %s %s \n",dbname,table,key,value);
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //创建表格
    char sql[1024];
    sprintf(sql,"DELETE FROM %s WHERE %s='%s'; ",table,key,value);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }   
    //查找参数名字的字段

    sqlite3_close(db);   
}

int update_table_entry(char* dbname,const char* table,const char* key,const char* key_const,const char* value_name,const char* value)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc )
    {
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return -1;
    }

    //创建表格
    char sql[1024];
    char *endptr;
    double value_f = strtod(value,&endptr);
    printf("%s %s %s %s %s %s %f %f\n",dbname,table,key,key_const,value_name,value,value_f,strtod(value,&endptr));

    sprintf(sql,"UPDATE %s SET %s=%lf WHERE %s='%s'; ",table,value_name,value_f,key,key_const);

    //sprintf(sql,"UPDATE %s SET value = %f  WHERE name = '%s';",table,value,keyname);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return -1;
    }   
    //查找参数名字的字段
    sqlite3_close(db);  
    return 0;
}

float get_list_value(param_list *plist,char *key)
{
    if(plist !=NULL && plist->param_cnt>0)
    {
        param_list *i = NULL;
        param_list *tmp = NULL;
        list_for_each_entry_safe(i, tmp, &plist->list, list) 
        {
            if(strcmp(i->key_value.key,key) == 0)
            {
                return i->key_value.value;
            }
        }
    }
    return -1;
}


void read_param_db_table(char* dbname,const char* table_name,db_table *p_table)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc )
    {
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    char sql[1024];
    sprintf(sql,"SELECT * from %s LIMIT 1",table_name);

    //执行SQL语句.得到列数和列名字
    rc = sqlite3_exec(db, sql, readParamTableColNamecallback, p_table, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }

    //得到行数
    sprintf(sql,"SELECT COUNT(*) from %s ;",table_name);
    //执行SQL语句
    int rowCount = 0;
    rc = sqlite3_exec(db, sql, select_table_rowcol_callback, &rowCount, &zErrMsg);
    p_table->rows = rowCount;
    //malloc
    p_table->table_value = (table_entrys *)malloc(rowCount*sizeof(table_entrys *));

    //读取所有的表数据
    read_table_vaule_index = 0;
    sprintf(sql,"SELECT * from %s ;",table_name);
    rc = sqlite3_exec(db, sql, readParamTableValuescallback, p_table->table_value, &zErrMsg);
    if(rc == SQLITE_OK)
    {
        ;
    }
    //查找参数名字的字段
    sqlite3_close(db);
}

int selectArmsValuecallback(void *pd, int argc, char **argv, char **azColName)
{
    double *value = (double *)pd;
    char* errp;
    *value = strtod(argv[1],&errp);
    //printf("%s %s %s \n",argv[0],argv[1],argv[2]);
    return 0;
}

int get_value_from_dbtable(char* dbname,const char* p_table,char *key,double * value)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;
    rc = sqlite3_open(dbname, &db);
    if( rc )
    {
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return -1;
    }

    char sql[1024];
    sprintf(sql,"SELECT * from %s WHERE name='%s'; ",p_table,key);

    //执行SQL语句.得到列数和列名字
    rc = sqlite3_exec(db, sql, selectArmsValuecallback, value, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return -1;
    }
    //printf("%s %f \n",key,*value);
    //查找参数名字的字段
    sqlite3_close(db);
}

void read_param_db(char* dbname,const char* table, param_list *p_list)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc )
    {
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //创建表格
    //char sql[1024];
    //sprintf(sql,"DELETE from %s WHERE name='MOTOR_XM_DAMPING_RATIO' ;",table);
    //创建表格
    char sql[1024];
    sprintf(sql,"SELECT * from %s ",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, readParamcallback, p_list, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    //查找参数名字的字段
    sqlite3_close(db);
}

void insert_params_db(char* dbname,const char* table,param_dic * params)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }
    int pI = 0;
    while (strlen(params[pI].key)>0)
    {
        //插入数据库
        char sql[1024];
        sprintf(sql,"INSERT INTO %s (name, value, info) SELECT '%s',%f,'%s' where not exists(select * from %s where name = '%s');",
                     table,params[pI].key,params[pI].value,params[pI].description,table,params[pI].key);
        //执行SQL语句
        rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);
        if(rc != SQLITE_OK)
        {
            printf("SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        }
        pI++;
    }

    //查找参数名字的字段
    sqlite3_close(db);  
}

void db_tables_name(char* dbname,strs_list *strs)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //更新数据库
    char sql[1024];
    sprintf(sql,"SELECT name FROM sqlite_master WHERE type='table' order by name ;");
    //执行SQL语句
    rc = sqlite3_exec(db, sql, read_db_tables_name_callback, strs, &zErrMsg);
    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }
    //查找参数名字的字段
    sqlite3_close(db);  
}

void update_param_db(char* dbname,const char* table,char* keyname,float value)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //更新数据库
    char sql[1024];
    sprintf(sql,"UPDATE %s SET value = %f  WHERE name = '%s';",table,value,keyname);
    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);
    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }
    //查找参数名字的字段
    sqlite3_close(db);    
}

void deinit_strs_list(strs_list *p_list)
{
    if(p_list == NULL)
        return;

    if(p_list->str_cnt >0)
    {   
        strs_list *i = NULL;
        strs_list *tmp = NULL;
        list_for_each_entry_safe(i, tmp, &p_list->list, list) 
        {
            list_del(&i->list);
            free(i);
        }
        p_list->str_cnt = 0;
    }
}

/**********************************对数据库的操作 end*************************************/
void  deinit_params(param_list * p_list)
{
    if(p_list == NULL)
        return;

    if(p_list->param_cnt>0)
    {   
        param_list *i = NULL;
        param_list *tmp = NULL;
        list_for_each_entry_safe(i, tmp, &p_list->list, list) 
        {
            list_del(&i->list);
            free(i);
        }
        p_list->param_cnt = 0;
    }
}

void  deinit_table_params(db_table * p_table)
{
    if(p_table == NULL)
        return;
    //删除列名字
    if(p_table->cols>0 && p_table->rows>0)
    {
        for (int j = 0; j < p_table->cols; j++)
        {
            free(p_table->colName[j]);
        }
        free(p_table->colName);
    
        //删除列内容
        for (int i = 0; i < p_table->rows; i++)
        {
            table_entrys * p_values = &p_table->table_value[i];
            for (int j = 0; j < p_table->cols; j++)
            {
                free(p_values->colValue[j]);
            }
            free(p_values->colValue);
        }
        free(p_table->table_value);
        p_table->cols = 0; 
        p_table->rows = 0;
    }
}

void  deinit_str_lists(strs_list * p_list)
{
    if(p_list == NULL)
        return;

    if(p_list->str_cnt >0)
    {   
        strs_list *i = NULL;
        strs_list *tmp = NULL;
        list_for_each_entry_safe(i, tmp, &p_list->list, list) 
        {
            list_del(&i->list);
            free(i);
        }
        p_list->str_cnt = 0;
    }
}
/*********************************对插值表数据库的操作************************************/
/*
注意：线性插值表采集必须是按照从左到右，从上到下的顺序。
     坐标跟架子坐标系统一
*/

void create_interpn_table(char* dbname,const char* table)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }

    //创建表格
    char sql[1024];

    sprintf(sql,"CREATE TABLE IF NOT EXISTS %s (ID INTEGER PRIMARY KEY,I INTEGER,J INTEGER, X REAL,Y REAL,VAULEX REAL,VAULEY REAL,SPARE1 REAL,SPARE2 REAL); ",table);
    //sprintf(sql,"DROP TABLE %s;",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    printf("%s created or open successfully\n",table);
    
    //查找参数名字的字段
    sqlite3_close(db);    
}

int select_table_rowcol_callback(void *pd, int argc, char **argv, char **azColName)
{
    int * p = (int *)pd;

    //读取参数
    printf("%s=%s\n",azColName[0],argv[0]);
    *p = atoi(argv[0]);
    return 0;
}

int interpn_callback(void *pd, int argc, char **argv, char **azColName)
{
    interpn_unit * p_inters = (interpn_unit *)pd;
    //读取参数
    
    if(argc > 8)
    {
        int i = atoi(argv[0])-1;
        //printf("***** %s=%s %s=%s %s=%s  %d %s %s\n",azColName[0],argv[0],azColName[1],argv[1],azColName[2],argv[2],pd,argv[7],argv[8]);
        /*
        for (int i = 0; i < argc; i++)
        {
            printf("*****%d %s=%s  ", argc,azColName[i],argv[i]);
        }
        printf("\n");
        */

        p_inters[i].xi = atoi(argv[1]);
        p_inters[i].yj = atoi(argv[2]);
        
        p_inters[i].xy.x = strtof(argv[3],0);
        p_inters[i].xy.y = strtof(argv[4],0);

        p_inters[i].vas.x = strtof(argv[5],0);
        p_inters[i].vas.y = strtof(argv[6],0);

        p_inters[i].spare[0] = strtof(argv[7],0);
        p_inters[i].spare[1] = strtof(argv[8],0);
    }
    return 0;
}

void read_interpn_row_db(char* dbname,const char* table,int * interprow)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }
    //创建表格
    char sql[1024];
    sprintf(sql,"SELECT COUNT(*) from %s where I=0",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, select_table_rowcol_callback, interprow, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    //查找参数名字的字段
    sqlite3_close(db);
}

void read_interpn_col_db(char* dbname,const char* table,int * interpcol)
{
   sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }
    //创建表格
    char sql[1024];
    sprintf(sql,"SELECT COUNT(*) from %s where J=0",table);
    //sprintf(sql,"SELECT * from %s ;",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, select_table_rowcol_callback, interpcol, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    //查找参数名字的字段
    sqlite3_close(db);
}

void read_interpn_db(char* dbname,const char* table,interpn_unit * interpns)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }
    //创建表格
    char sql[1024];
    sprintf(sql,"SELECT * from %s ",table);

    //执行SQL语句
    rc = sqlite3_exec(db, sql, interpn_callback, interpns, &zErrMsg);

    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return ;
    }
    //查找参数名字的字段
    sqlite3_close(db);
}

void insert_interpn_db(char* dbname,const char* table,interpn_unit *params,int row,int col)
{
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    rc = sqlite3_open(dbname, &db);
    if( rc ){
        printf("Can't open database: %s\n", sqlite3_errmsg(db));
        return ;
    }
    
    char sql[1024];
    //插入数据之前，先删除表中的所有数据
    sprintf(sql,"DELETE FROM  %s ;",table);
    //执行SQL语句
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);
    if(rc != SQLITE_OK)
    {
        printf("SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }

    //分别插入行列数据到数据库
    for (int  i = 0; i < row; i++)
    {
        for (int  j = 0; j < col; j++)
        {            
            sprintf(sql,"INSERT INTO %s (ID, I,J,X ,Y ,VAULEX ,VAULEY ,SPARE1 ,SPARE2) SELECT %d ,%d, %d,%f,%f,%f,%f,%f,%f where not exists(select * from %s where ID = %d);",
                    table,
                    (i)*col+j+1,
                    params[i*col+j].xi,
                    params[i*col+j].yj,
                    params[i*col+j].xy.x,
                    params[i*col+j].xy.y,
                    params[i*col+j].vas.x,
                    params[i*col+j].vas.y,
                    params[i*col+j].spare[0],
                    params[i*col+j].spare[1],
                    table,
                    (i)*col+j+1);

            //执行SQL语句
            rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);
            if(rc != SQLITE_OK)
            {
                printf("SQL error: %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            }
        }
    }
    //查找参数名字的字段
    sqlite3_close(db);  
}

/**********************************对插值表数据库 end************************************/

/**********************************以下是对interp 数组的操作***********************************/
//。得到i 行 j列的数据
interpn_unit * get_interpn(interpn_unit * interpns, int i, int j, int cols)
{
    return &interpns[i*cols+j];
}

/**********************************以下是对interp 数组的操作 end***********************************/

float get_fork_z(coordinate p1,coordinate p2)
{
    return p1.x*p2.y - p1.y*p2.x;
}

quads_unit* generate_quads(interpn_unit* interpns,int inter_rows,int inter_cols)
{
    quads_unit* qu;
    qu = malloc(sizeof(quads_unit)*(inter_rows+1)*(inter_cols+1));
    float infinity = 10000000.0;
    //计算左上、右上、左下、右下的四边形
    qu[0].left_up.x = -infinity;
    qu[0].left_up.y =  infinity;
    qu[0].right_up.x = interpns[0].xy.x;
    qu[0].right_up.y =  infinity;
    qu[0].left_down.x = -infinity;
    qu[0].left_down.y =  interpns[0].xy.y;
    qu[0].right_down = interpns[0].xy;
    qu[0].left_up_vaule= interpns[0].vas;
    qu[0].right_up_vaule= interpns[0].vas;
    qu[0].left_down_vaule= interpns[0].vas;
    qu[0].right_down_vaule= interpns[0].vas;

    qu[inter_cols].left_up.x = interpns[inter_cols-1].xy.x;
    qu[inter_cols].left_up.y =  infinity;
    qu[inter_cols].right_up.x = infinity;
    qu[inter_cols].right_up.y = infinity;
    qu[inter_cols].left_down = interpns[inter_cols-1].xy;
    qu[inter_cols].right_down.x = infinity;
    qu[inter_cols].right_down.y = interpns[inter_cols-1].xy.y;
    qu[inter_cols].left_up_vaule= interpns[inter_cols-1].vas;
    qu[inter_cols].right_up_vaule= interpns[inter_cols-1].vas;
    qu[inter_cols].left_down_vaule= interpns[inter_cols-1].vas;
    qu[inter_cols].right_down_vaule= interpns[inter_cols-1].vas;

    int index = inter_rows*(inter_cols+1);
    qu[index].left_up.x = -infinity;
    qu[index].left_up.y = interpns[(inter_rows-1)*inter_cols].xy.y;
    qu[index].right_up = interpns[(inter_rows-1)*inter_cols].xy;
    qu[index].left_down.x = -infinity;
    qu[index].left_down.y = -infinity;
    qu[index].right_down.x = interpns[(inter_rows-1)*inter_cols].xy.x;
    qu[index].right_down.y = -infinity;
    qu[index].left_up_vaule= interpns[(inter_rows-1)*inter_cols].vas;
    qu[index].right_up_vaule= interpns[(inter_rows-1)*inter_cols].vas;
    qu[index].left_down_vaule= interpns[(inter_rows-1)*inter_cols].vas;
    qu[index].right_down_vaule= interpns[(inter_rows-1)*inter_cols].vas;

    index = (inter_rows+1)*(inter_cols+1)-1;
    qu[index].left_up = interpns[inter_rows*inter_cols-1].xy;
    qu[index].right_up.x = infinity;
    qu[index].right_up.y = interpns[inter_rows*inter_cols-1].xy.y;
    qu[index].left_down.x = interpns[inter_rows*inter_cols-1].xy.x;
    qu[index].left_down.y = -infinity;
    qu[index].right_down.x = infinity;
    qu[index].right_down.y = -infinity;
    qu[index].left_up_vaule= interpns[inter_rows*inter_cols-1].vas;
    qu[index].right_up_vaule= interpns[inter_rows*inter_cols-1].vas;
    qu[index].left_down_vaule= interpns[inter_rows*inter_cols-1].vas;
    qu[index].right_down_vaule= interpns[inter_rows*inter_cols-1].vas;

    //第1行
    for (int i = 0; i < inter_cols-1; i++)
    {
        qu[i+1].left_up.x = interpns[i].xy.x;
        qu[i+1].left_up.y = infinity;
        qu[i+1].right_up.x = interpns[i+1].xy.x;
        qu[i+1].right_up.y = infinity;
        qu[i+1].left_down = interpns[i].xy;
        qu[i+1].right_down = interpns[i+1].xy;
        qu[i+1].left_up_vaule= interpns[i].vas;
        qu[i+1].right_up_vaule= interpns[i+1].vas;
        qu[i+1].left_down_vaule= interpns[i].vas;
        qu[i+1].right_down_vaule= interpns[i+1].vas;
    }
    
    //最后一行
    index = inter_rows*(inter_cols+1);
    for (int i = 0; i < inter_cols-1; i++)
    {
        qu[index+i+1].left_up = interpns[(inter_rows-1)*inter_cols+i].xy;
        qu[index+i+1].right_up = interpns[(inter_rows-1)*inter_cols+i+1].xy;
        qu[index+i+1].left_down.x = interpns[(inter_rows-1)*inter_cols+i].xy.x;
        qu[index+i+1].left_down.y = -infinity;
        qu[index+i+1].right_down.x = interpns[(inter_rows-1)*inter_cols+i+1].xy.x;
        qu[index+i+1].right_down.y = -infinity;

        qu[index+i+1].left_up_vaule= interpns[(inter_rows-1)*inter_cols+i].vas;
        qu[index+i+1].right_up_vaule= interpns[(inter_rows-1)*inter_cols+i+1].vas;
        qu[index+i+1].left_down_vaule= interpns[(inter_rows-1)*inter_cols+i].vas;
        qu[index+i+1].right_down_vaule= interpns[(inter_rows-1)*inter_cols+i+1].vas;

        /*
        printf("***** %f %f %f %f %f %f %f %f \n",qu[index+i+1].left_up.x,qu[index+i+1].left_up.y,
                                            qu[index+i+1].right_up.x,qu[index+i+1].right_up.y,
                                            qu[index+i+1].left_down.x,qu[index+i+1].left_down.y,
                                            qu[index+i+1].right_down.x,qu[index+i+1].right_down.y
                                            );
        */
    }

    //第1列
    index = (inter_cols+1);
    for (int i = 0; i < inter_rows-1; i++)
    {
        qu[index*(i+1)].left_up.x = -infinity;
        qu[index*(i+1)].left_up.y = interpns[i*inter_cols].xy.y;
        qu[index*(i+1)].right_up =  interpns[i*inter_cols].xy;
        qu[index*(i+1)].left_down.x = -infinity;
        qu[index*(i+1)].left_down.y = interpns[(i+1)*inter_cols].xy.y;
        qu[index*(i+1)].right_down = interpns[(i+1)*inter_cols].xy;

        qu[index*(i+1)].left_up_vaule= interpns[i*inter_cols].vas;
        qu[index*(i+1)].right_up_vaule= interpns[i*inter_cols].vas;

        qu[index*(i+1)].left_down_vaule= interpns[(i+1)*inter_cols].vas;
        qu[index*(i+1)].right_down_vaule= interpns[(i+1)*inter_cols].vas;
    }
    
    //最后一列
    index = (inter_cols+1);
    for (int i = 0; i < inter_rows-1; i++)
    {
        qu[index*(i+2)-1].left_up = interpns[(i+1)*inter_cols-1].xy;
        qu[index*(i+2)-1].right_up.x =  infinity;
        qu[index*(i+2)-1].right_up.y =  interpns[(i+1)*inter_cols-1].xy.y;
        qu[index*(i+2)-1].left_down = interpns[(i+2)*inter_cols-1].xy;;
        qu[index*(i+2)-1].right_down.x = infinity;
        qu[index*(i+2)-1].right_down.y = interpns[(i+2)*inter_cols-1].xy.y;

        qu[index*(i+2)-1].left_up_vaule= interpns[(i+1)*inter_cols-1].vas;
        qu[index*(i+2)-1].right_up_vaule= interpns[(i+1)*inter_cols-1].vas;
        qu[index*(i+2)-1].left_down_vaule= interpns[(i+2)*inter_cols-1].vas;
        qu[index*(i+2)-1].right_down_vaule= interpns[(i+2)*inter_cols-1].vas;
    }

    //中间四边形方格子
    for (int i = 0; i < inter_rows-1; i++)
    {
        for (int j = 0; j < inter_cols-1; j++)
        {
            qu[(i+1)*(inter_cols+1)+(j+1)].left_up = interpns[(i)*inter_cols+j].xy;
            qu[(i+1)*(inter_cols+1)+(j+1)].right_up = interpns[(i)*inter_cols+j+1].xy;
            qu[(i+1)*(inter_cols+1)+(j+1)].left_down = interpns[(i+1)*inter_cols+j].xy;
            qu[(i+1)*(inter_cols+1)+(j+1)].right_down = interpns[(i+1)*inter_cols+j+1].xy;
            qu[(i+1)*(inter_cols+1)+(j+1)].left_up_vaule= interpns[(i)*inter_cols+j].vas;
            qu[(i+1)*(inter_cols+1)+(j+1)].right_up_vaule= interpns[(i)*inter_cols+j+1].vas;
            qu[(i+1)*(inter_cols+1)+(j+1)].left_down_vaule= interpns[(i+1)*inter_cols+j].vas;
            qu[(i+1)*(inter_cols+1)+(j+1)].right_down_vaule= interpns[(i+1)*inter_cols+j+1].vas;
        }
    }

    return qu;
}

//判断点Q是否在P1和P2的线段上
static int OnSegment(coordinate pi,coordinate pj,coordinate Q)
{
    if(fabs((Q.x-pi.x)*(pj.y-pi.y)-(pj.x-pi.x)*(Q.y-pi.y))<0.000001
        &&fmin(pi.x,pj.x)<=Q.x
        &&Q.x<=fmax(pi.x,pj.x)
        &&fmin(pi.y,pj.y)<=Q.y
        &&Q.y<=fmax(pi.y,pj.y))
    {
        return 1;
    }else{
        return 0;
    }
}

quads_unit* find_quads(quads_unit_head* qHead,float coor_x,float coor_y)
{
    if(qHead != NULL && qHead->isValid==1)
    {
        quads_unit *qus = qHead->pQus;
        if(qus==NULL || qHead->rows==0 || qHead->cols==0)
            return NULL;

        //注意：rows cols是原始的采集点个数。生成的四边形个数是（rows+1） (cols+1)
        int qu_nums = (qHead->rows+1)*(qHead->cols+1);
        coordinate P = {coor_x,coor_y};

        for (int i = 0; i < qu_nums; i++)
        {
            //在四边形的边上
            if (OnSegment(qus[i].left_up,qus[i].right_up,P) 
                ||OnSegment(qus[i].right_up,qus[i].right_down,P)
                ||OnSegment(qus[i].right_down,qus[i].left_down,P) 
                ||OnSegment(qus[i].left_down,qus[i].left_up,P) )
            {
                return &qus[i];
            }
            //判断是否在四边形之内.使用向量叉乘方法。适用于凸多边形
            coordinate P1,P2,P3,P4;
            P1.x = qus[i].left_up.x-P.x;
            P1.y = qus[i].left_up.y-P.y;

            P2.x = qus[i].left_down.x-P.x;
            P2.y = qus[i].left_down.y-P.y;

            P3.x = qus[i].right_down.x-P.x;
            P3.y = qus[i].right_down.y-P.y;

            P4.x = qus[i].right_up.x-P.x;
            P4.y = qus[i].right_up.y-P.y;

            //判断向量是否同向
            if((get_fork_z(P1,P2)>0 &&get_fork_z(P2,P3)>0 &&get_fork_z(P3,P4)>0 &&get_fork_z(P4,P1)>0) || 
            (get_fork_z(P1,P2)<0 &&get_fork_z(P2,P3)<0 &&get_fork_z(P3,P4)<0 &&get_fork_z(P4,P1)<0))
            {
                return &qus[i];
            }        
        }
    }
    return NULL;
}

interpn_value interpn_magic(quads_unit_head* qHead,float coor_x,float coor_y)
{
    interpn_value iRet = {0};
    if(qHead != NULL && (qHead->isValid==1))
    {
        quads_unit* qu = find_quads(qHead,coor_x,coor_y);
        if(qu != NULL)
        {
            coordinate P = {coor_x,coor_y};
            float inputX = P.x;
            float inputY = P.y;
        
            float ld_Tx = qu->left_down.x; 
            float lu_Tx = qu->left_up.x;
            float rd_Tx = qu->right_down.x;
            float rd_Ty = qu->right_down.y;  
            float ru_Tx = qu->right_up.x;
            float ru_Ty = qu->right_up.y; 

            float ld_Vx = qu->left_down_vaule.x;
            float ld_Vy = qu->left_down_vaule.y;  
            float lu_Vx = qu->left_up_vaule.x;
            float lu_Vy = qu->left_up_vaule.y;  
            float rd_Vx = qu->right_down_vaule.x;
            float rd_Vy = qu->right_down_vaule.y;  
            float ru_Vx = qu->right_up_vaule.x;
            float ru_Vy = qu->right_up_vaule.y; 

            float sikoDownx = ld_Vx + (P.x-ld_Tx)*(rd_Vx-ld_Vx)/(rd_Tx-ld_Tx);
            float sikoDowny = ld_Vy + (P.x-ld_Tx)*(rd_Vy-ld_Vy)/(rd_Tx-ld_Tx);
            float sikoUpx   = lu_Vx + (P.x-lu_Tx)*(ru_Vx-lu_Vx)/(ru_Tx-lu_Tx);
            float sikoUpy   = lu_Vy + (P.x-lu_Tx)*(ru_Vy-lu_Vy)/(ru_Tx-lu_Tx);
            
            iRet.x =sikoDownx + (P.y-rd_Ty)*(sikoUpx-sikoDownx)/(ru_Ty-rd_Ty);
            iRet.y =sikoDowny + (P.y-rd_Ty)*(sikoUpy-sikoDowny)/(ru_Ty-rd_Ty);

            /*
            printf("%f %f %f %f %f %f %f %f \n",
                qu->left_up.x,qu->left_up.y,
                qu->right_up.x,qu->right_up.y,
                qu->left_down.x,qu->left_down.y,
                qu->right_down.x,qu->right_down.y
            );
            */
        }
    }


    return iRet;
}