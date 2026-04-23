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
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <netdb.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "sys_ctl.h"
#include "sys_share_conf.h"

// 每行最长的大小
#define MAX_LINE 256

#define FILE_NAME "conf.in"

/*以下是不可修改的参数,供多线程使用*/
static param_dic const_params[] = 
{
    {"SUPR_PORTS",0,"30001"},
    {"LEADER_PORTS",0,"30002"},
    {"MASTER0_PORTS",0,"30011"},
    {"MASTER1_PORTS",0,"30012"},
    {"LISTENER_LOCAL_PORT",0,"30021"},
    {"LOGGER_LOCAL_PORT",0,"30022"},
    {"DB_WORKER_PORT",0,"30023"},
    //{"LISTENER_TO_UPPER_IP",0,"192.168.1.11"},
    {"LISTENER_TO_UPPER_PORT",0,"33333"},
   
    {"VISION_IP",0,"192.168.1.41"},
    {"VISION_PORT",0,"39001"},
    {"MASTER_60A3D_IP",0,"192.168.1.41"},
    {"MASTER_60A3D_PORT",0,"13131"},
    {"DAM_60A3D1_IP",0,"192.168.1.250"},
    {"DAM_60A3D1_PORT",0,"1030"},
    {"DAM_60A3D2_IP",0,"192.168.1.251"},
    {"DAM_60A3D2_PORT",0,"1030"},
    
    {"SERIAL_WORKER_IP",0,"192.168.1.41"},
    {"SERIAL_WORKER_PORT",0,"13132"},

    {"DAM_GW1800_IP",0,"192.168.1.253"},
    {"DAM_GW1800_PORT",0,"1030"},

    {"",0,""}
};
//static param_list p_list={0};


#if 0
/**
 * @brief read_config_file 读取配置文件
 * @param filename 文件名
 * @param key      键
 * @param value    值
 * @param value_len 值字符串长度
 * @return
 */
int read_config_file(char *filename /*in*/, char *key /*in*/, char *value/*in out*/, int *value_len /*out*/)
{

    // 返回值
    int ret = 0;

    // 文件指针
    FILE *fp = NULL;

    // 灵活使用的临时指针
    char *p = NULL;

    // 指向 Value 开始位置的指针
    char *start = NULL;

    // 指向 Value 结束位置的指针
    char *end = NULL;

    // 存储配置文件中的一行数据
    char line_buffer[MAX_LINE];

    // 以只读的方式打开文件
    fp = fopen(filename, "r");
    // 如果文件打开失败 , 说明文件不存在 , 直接退出
    if (fp == NULL)
    {
        ret = -1;
        return ret;
    }

    // 逐行遍历 配置文件 中的文本数据
    while (!feof(fp))
    {
        // 清空 line_buffer 中的遗留数据 , 避免被上一次写入的数据干扰
        memset(line_buffer, 0, sizeof(line_buffer));

        // 获取一行数据
        fgets(line_buffer, MAX_LINE, fp);

        // 查找 '=' 字符
        p = strchr(line_buffer, '=');
        // 如果没有找到 '=' 字符 , 则退出 , 继续执行下一次循环
        if (p == NULL)
        {
            continue;
        }

        // 查找 Key 值
        // 如果找到了 Key 关键字 , 则返回的指针 p 指向 Key 关键字出现的首地址中
        p = strstr(line_buffer, key);
        // 如果没有找到 Key 关键字 , 退出执行下一次循环换
        if (p == NULL)
        {
            continue;
        }

        // 越过 Key 关键字 , 从 Key 关键字后面的内容遍历
        p = p + strlen(key);

        // 查找 '=' 字符
        p = strchr(p, '=');
         // 如果没有找到 '=' 字符 , 则退出 , 继续执行下一次循环
        if (p == NULL)
        {
            continue;
        }

        // 越过 '=' 字符 , 从 '=' 字符 后面的内容遍历
        p = p + 1;

        // 获取 Value 起始位置
        for(;;)
        {
            // 去掉开始位置的空格
            if (*p == ' ')
            {
                p ++ ;
            }
            else
            {
                start = p;
                if (*start == '\n')
                {
                    // 进入到该分支 , 说明 Value 值是空的
                    // 直接退出即可
                    goto End;
                }
                break;
            }
        }

        // 获取 Value 结束位置
        // 从 Value 的不为空格的位置开始遍历
        for(;;)
        {
            // 遇到空格或回车 , 说明读取到了最后的位置, 或者换行位置
            if ((*p == ' ' || *p == '\n'))
            {
                break;
            }
            else
            {
                p ++;
            }
        }
        end = p;

        // 通过 间接赋值 设置 Value 值长度
        *value_len = end - start;

        // 通过 间接赋值 设置 Value 值数据内容
        memcpy(value, start, end - start);
    }

End:
    // 关闭文件
    if (fp == NULL)
    {
        fclose(fp);
    }
    return 0;
}
#endif

/**
 * @brief write_or_update_config_file 写出或更新配置项
 * 遍历每行数据 , 检查 key 键 是否存在
 * 如果存在 , 就更新对应的 value 值
 * 如果不存在 , 在文件末尾添加该键值对信息
 * 格式为 :
 * key = value
 *
 * @param filename 文件名称
 * @param key 键
 * @param value 值
 * @param value_len 值的长度
 * @return
 */
#if 0
int write_or_update_config_file(char *filename /*in*/, char *key /*in*/, char * value/*in*/, int value_len /*in*/)
{
    // 返回值
    int ret = 0;

    // 查询配置文件中 Key 是否存在
    int key_exist = 0;

    // 统计的配置文件大小
    int file_length = 0;

    // 文件指针
    FILE *fp = NULL;

    // 存储读取的每一行配置文件信息
    char line_buffer[MAX_LINE];

    // 临时指针
    char *p = NULL;

    // 文件数据缓存区
    char file_buffer[1024 * 4] = {0};

    // 验证指针有效性
    if (filename == NULL || key == NULL || value == NULL)
    {
        ret = -1;
        printf("error : filename == NULL || key == NULL || value == NULL \n");
        goto End;
    }

    // 使用读写方式打开 filename 文件
    fp = fopen(filename, "r+");
    // 如果打开失败 提示失败信息
    if (fp == NULL)
    {
        ret = -2;
        printf("error : fopen \n");
    }

    // 如果文件打开失败 , 说明没有文件
    if (fp == NULL)
    {
        // 以写的方式 , 打开文本文件 , 如果文件不存在 , 则创建文件
        fp = fopen(filename, "w+t");
        // 打开失败 , 直接退出
        if (fp == NULL)
        {
            ret = -3;
            printf("error : fopen \n");
            goto End;
        }
    }

    // 统计文件大小

    // 将文件指针移动到末尾
    fseek(fp, 0L, SEEK_END);

    // 获取当前指针位置 , 当前指针位置就是文件大小
    file_length = ftell(fp);

    // 将文件指针指向开始位置
    fseek(fp, 0L, SEEK_SET);

    // 文件大小不能超过 4K
    if (file_length > 1024 * 4)
    {
        ret = -3;
        printf("File Size More Than 4K\n");
        goto End;
    }


    // 逐行遍历配置文件
    while (!feof(fp))
    {
        // 将 line_buffer 数据清空
        memset(line_buffer, 0, sizeof(line_buffer));

        // 获取 fp 文件的一行数据 , 保存到 line_buffer 数组中 ,  最多获取 MAX_LINE 字节
        p = fgets(line_buffer, MAX_LINE, fp);
        // 如果获取失败 , 则返回 NULL
        // 获取成功 , 返回的是 line_buffer 地址
        if (p == NULL)
        {
            break;
        }

        // 查询 本行字符数组中是否包含 键 Key
        p = strstr(line_buffer, key);
        // 本行不包含 Key , 将数据行 line_buffer
        // 追加拷贝到 file_buffer 数组中
        if (p == NULL)
        {
            strcat(file_buffer, line_buffer);
            continue;
        }
        else
        {
            // 如果 Key 关键字 在本行 , 则使用新的数据替换原来的数据 , 最后拷贝到 file_buffer 中
            // 替换本行数据
            sprintf(line_buffer, "%s = %s\n", key, value);

            // 将替换的数据 , 追加拷贝到 file_buffer 数组中
            strcat(file_buffer, line_buffer);

            // 设置 Key 存在标志位
            key_exist = 1;
        }
    }

    // 如果 Key 关键字不存在 , 直接将数据追加到文件末尾即可
    if (key_exist == 0)
    {
        fprintf(fp, "%s = %s\n", key, value);
    }
    else // 如果 Key 关键字存在 , 则需要重新写出该文件的数据 , 原来的数据直接删除覆盖
    {
        // 先关闭之前的 文件指针
        if (fp != NULL)
        {
            fclose(fp);
            fp = NULL;
        }

        // 重新打开文件
        fp = fopen(filename, "w+t");
        if (fp == NULL)
        {
            ret = -4;
            printf("fopen() err. \n");
            goto End;
        }
        // 将文件的完整数据 , 写出到 fp 中
        // 注意此处的文件数据 , 没有原来的 键值对数据
        // 写入了要更新的键值对数据
        fputs(file_buffer, fp);

        // 也可以使用 fwrite 函数 , 向文件中写出数据
        //fwrite(filebuf, sizeof(char), strlen(filebuf), fp);
    }

End:
    // 关闭文件
    if (fp != NULL)
    {
        fclose(fp);
    }
    return ret;
}
#endif
#if 0
char* get_param_value_str(param_dic *params,char *key)
{
    int pI = 0;
    char *p = NULL;
    while (1)
    {
        if(strlen(params[pI].key)>0 && (strcmp(key,params[pI].key) == 0))
        {
            p = params[pI].description;
            break;
        }
        pI++;
    }
}
#endif

int get_local_ip(const char *eth_inf, char *ip)
{
    int sd;
    struct sockaddr_in sin;
    struct ifreq ifr;
 
    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sd)
    {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }
 
    strncpy(ifr.ifr_name, eth_inf, IFNAMSIZ);
    ifr.ifr_name[IFNAMSIZ - 1] = 0;
 
    // if error: No such device
    if (ioctl(sd, SIOCGIFADDR, &ifr) < 0)
    {
        printf("ioctl error: %s\n", strerror(errno));
        close(sd);
        return -1;
    }
 
    memcpy(&sin, &ifr.ifr_addr, sizeof(sin));
    snprintf(ip, IP_C_SIZE, "%s", inet_ntoa(sin.sin_addr));
 
    close(sd);
    return 0;
}

char * get_const_param_value_str(char *key)
{
   int pI = 0;
    char *p = NULL;
    while (1)
    {
        if(strlen(const_params[pI].key)>0 && (strcmp(key,const_params[pI].key) == 0))
        {
            p = const_params[pI].description;
            break;
        }
        pI++;
    }
}

char * get_ip(char *key)
{
    int pI = 0;
    char *p = NULL;
    while (1)
    {
        if(strlen(const_params[pI].key)>0 && (strcmp(key,const_params[pI].key) == 0))
        {
            p = const_params[pI].description;
            break;
        }
        pI++;
    } 
}

uint32_t get_port(char *key)
{
    return strtol(get_const_param_value_str(key),NULL,10);
}

/**
 * @brief get_config_mn_port 得到mn_port配置值
 * 遍历每行数据 , 检查 key 键 是否存在
 * 如果存在 , 就更新对应的 value 值
 * 如果不存在 , 在文件末尾添加该键值对信息
 * 格式为 :
 * key = value
 *
 * @param filename 文件名称
 * @param key 键
 * @param value 值
 * @param value_len 值的长度
 * @return
 */
int get_mn_port(int index)
{
    int iRet = 0;
    if(MN_ID_SUPR == index)
        iRet = strtol(get_const_param_value_str("SUPR_PORTS"),NULL,10);
    if(MN_ID_LEADER == index)
        iRet = strtol(get_const_param_value_str("LEADER_PORTS"),NULL,10);

    return iRet;
}

int get_supr_port()
{
    return strtol(get_const_param_value_str("SUPR_PORTS"),NULL,10);
}
/**
 * @brief get_mn_master_port 得到mn_port配置值
 * 遍历每行数据 , 检查 key 键 是否存在
 * 如果存在 , 就更新对应的 value 值
 * 如果不存在 , 在文件末尾添加该键值对信息
 * 格式为 :
 * key = value
 *
 * @param filename 文件名称
 * @param key 键
 * @param value 值
 * @param value_len 值的长度
 * @return
 */
int get_mn_master_port(int index)
{
    int iRet = 0;
    if(0 == index)
        iRet = strtol(get_const_param_value_str("MASTER0_PORTS"),NULL,10);
    if(1 == index)
        iRet = strtol(get_const_param_value_str("MASTER1_PORTS"),NULL,10);
    
    return iRet;
}

