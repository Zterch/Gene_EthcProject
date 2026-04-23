/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-06-07 07:18:48
 * @LastEditTime: 2024-06-14 00:27:45
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_loger.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:日志模块，系统运行的情况记录在这个模块中。
*
********************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <sys/time.h>
#include "sys_master_serial.h"
#include "sys_master.h"
#include "sys_leader_defs.h"
#include "sys_ctl.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"

#define     TIMESPEC2NS(T)       ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/* CRC余式表 */
const unsigned int crc_table[256] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040,
};


/*****************************************************************************************************************/

/*智岍物联网口模块60A3D 主要模拟量输出模块，控制比例调压阀*/
static int32_t mSoket = -1;
static char mSoketRecBuff[2000] = {0};
static uint16_t soketRecNum = 0;
static uint32_t Polling_modbusid = 0;

/*智岍物联网口模块GW1800 主要控制继电器*/
static int32_t mGW1800Soket = -1;
static uint16_t relay_port = 1030;
//无线拉力计，使用的串口
static int32_t  serialFd[] ={-1,-1,-1,-1,-1,-1,-1,-1};
static uint8_t  recBuff[8][500];
static int32_t recLen[8];

/*BMS串口分配,使用RS232，网线的3 7 6 ：ttyS0 */
const char * ttys_bms = "/dev/ttyM3";
const char * ttys_bms_MI = "/dev/ttyMI3";
static int32_t  sFd_bms = -1;
static uint8_t  recBuff_bms[500];
static int32_t recLen_bms;

/*PCI转485串口一共有4个。比例调压阀串口分配：ttyM0 是ZQWL-DAM-5F03D，主要是采集模拟量数据*/
const char * ttys_pressure = "/dev/ttyM0";
const char * ttys_pressure_MI = "/dev/ttyMI0";
static int32_t  sFd_press = -1;
static uint8_t  recBuff_press[500];
static int32_t recLen_press;

/*PCI转485串口一共有4个。串口分配：ttyM1 是底盘变送器1-5，气压表 主要是采集称重计*/
const char * ttys_weight = "/dev/ttyM1";
const char * ttys_weight_MI = "/dev/ttyMI1";
static int32_t  sFd_weight = -1;
static uint8_t  recBuff_weight[500];
static int32_t recLen_weight;


/****************************************************************************/
static unsigned short do_crc_table(unsigned char *ptr, int len);
static int32_t data_swap_u32(uint8_t *dst,uint8_t *src);
static int32_t data_swap_u16(uint8_t *dst,uint8_t *src);
static inline void tsnorm(struct timespec *ts);
static uint64_t system_time_ns(void);
static int32_t setNewOptions(int32_t mFd, const speed_t cBaudRate, const int mDatabits,
                             const int mStop, const bool sHandshake, const bool hHandshake);
static int32_t openUart(const char ttyUsb[],speed_t bautrate);
static int32_t closeUart(int32_t cFd);
static int32_t readBMS(SERIAL_THREAD_INFO *pSM);
static int32_t sendBMS(SERIAL_THREAD_INFO *pSM);
static float Slid_avg_filter(float *fvec,int times);
void setFdNonblockingm(int sockfd);
void setFdTimeoutm(int sockfd, const int mSec, const int mUsec);
static int32_t handle_open(SERIAL_THREAD_INFO *pSModule);
static void handle_close(SERIAL_THREAD_INFO *pSModule);

static void handle_proportional_write(SERIAL_THREAD_INFO *pSModule);

static void modbus_send_press(SERIAL_THREAD_INFO *pSModule);
static void modbus_read_press(SERIAL_THREAD_INFO *pSModule);

static void modbus_send_weight(SERIAL_THREAD_INFO *pSModule,int modbus_id);
static void modbus_read_weight(SERIAL_THREAD_INFO *pSModule,int modbus_id);


//static void handle_socket_read(SERIAL_THREAD_INFO *pSModule,int mod_id);
static void smooth_filter(SERIAL_THREAD_INFO *pSModule);

//GW1800操作
static void read_relays_state(SERIAL_THREAD_INFO *pSModule);
static void arm24_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void arm48_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void chassis24_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void chassis48_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void chassis_brk_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void arm_gas_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void chassis_gas_on_off(SERIAL_THREAD_INFO *pSModule,int flag);
static void handle_gw1800_read(SERIAL_THREAD_INFO *pSModule);
/****************************************************************************/
static int32_t instervalus(float value,float *tension_vec,int filtertimes);

//查表法计算crc
static unsigned short do_crc_table(unsigned char *ptr, int len)
{
    unsigned short crc = 0xFFFF;
    while(len--) 
    {
        crc = (crc >> 8) ^ crc_table[(crc ^ *ptr++) & 0xff];
    }
    return (crc);
}

//32位整形的高低位转换
static int32_t data_swap_u32(uint8_t *dst,uint8_t *src)
{
    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];
}
//16位整形的高低位转换
static int32_t data_swap_u16(uint8_t *dst,uint8_t *src)
{
    dst[0] = src[1];
    dst[1] = src[0];
}

/******************************************************************************
* 功能：tsnorm函数将timespec中的纳秒，妙格式化。按照1m = 1000000000ns，
* @param ts : ts是时间结构体，包含：妙 纳秒
* @return Descriptions
******************************************************************************/
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

//获取当前系统时间
static uint64_t system_time_ns(void)
{
    struct timespec  rt_time;
    clock_gettime(CLOCK_REALTIME, &rt_time);
    uint64_t time = TIMESPEC2NS(rt_time);
    return time;
}


static int32_t setNewOptions(int32_t mFd, const speed_t cBaudRate, const int mDatabits,
                             const int mStop, const bool sHandshake, const bool hHandshake){
  struct termios newtio;
  memset(&newtio, 0, sizeof(newtio));
  if (tcgetattr(mFd, &newtio)!=0) {
    printf("tcgetattr() 3 failed");
  }
  cfsetospeed(&newtio, (speed_t)cBaudRate);
  cfsetispeed(&newtio, (speed_t)cBaudRate);

   /* We generate mark and space parity ourself. */
  switch (mDatabits) {
    case 5: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
      break;
    }
    case 6: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
      break;
    }
    case 7: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
      break;
    }
    case 8:
    default: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
      break;
    }
  }
  newtio.c_cflag |= CLOCAL | CREAD;

  //parity
  newtio.c_cflag &= ~(PARENB | PARODD);

  newtio.c_cflag &= ~CRTSCTS;

  //stopbits
  if (mStop == 2) {
    newtio.c_cflag |= CSTOPB;
  } else {
    newtio.c_cflag &= ~CSTOPB;
  }

  newtio.c_iflag=IGNBRK;

  //software handshake
  if (sHandshake) {
    newtio.c_iflag |= IXON | IXOFF;
  } else {
    newtio.c_iflag &= ~(IXON|IXOFF|IXANY);
  }

  newtio.c_lflag=0;
  newtio.c_oflag=0;

  newtio.c_cc[VTIME]=1;
  newtio.c_cc[VMIN]=60;

  tcflush(mFd, TCIFLUSH);
  if (tcsetattr(mFd, TCSANOW, &newtio)!=0) {
    printf("tcsetattr() 1 failed");
  }

  int mcs=0;
  ioctl(mFd, TIOCMGET, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(mFd, TIOCMSET, &mcs);

  if (tcgetattr(mFd, &newtio)!=0) {
    printf("tcgetattr() 4 failed");
  }

  //hardware handshake
  if (hHandshake) {
    newtio.c_cflag |= CRTSCTS;
  } else {
    newtio.c_cflag &= ~CRTSCTS;
  }

  if (tcsetattr(mFd, TCSANOW, &newtio)!=0) {
    perror("tcsetattr() 2 failed");
  }
  return 0;
}

static int32_t openUart(const char ttyUsb[],speed_t bautrate)
{
  int32_t fd = -1;
  fd = open(ttyUsb, O_RDWR|O_NDELAY);
  if (-1 == fd) {
    printf("Can't Open Serial Port:%s\n",ttyUsb);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);

  int n = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, n & ~O_NDELAY);
  if (fd < 0) {
    return -1;
  }
  if (setNewOptions(fd, bautrate, 8, 1, 0, 0) < 0) {
    return -2;
  }

  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  printf("Open Serial Port:%s OK \n",ttyUsb);
  return fd;
}

static int32_t closeUart(int32_t cFd)
{
  int32_t iRet = 0;
  if (cFd < 0) {
    perror("fd is not exist");
    return -1;
  }

  iRet = close(cFd);
  if(iRet != 0){
    perror("close fd failed");
    return -2;
  }
  return iRet;
}

static int32_t readBMS(SERIAL_THREAD_INFO *pSM)
{
    int32_t iRet = -1;
    static uint32_t rec_timeout = 0;
    /*
        默认的波特率9600 地址 33. 发送modbus标准协议，读取7个地址的。
        Battery Status	NA	0xA701	UINT16	2	0
        Battery Voltage	V	0xA702~0xA703	UINT32	4	1
        Battery Current	A	0xA704~0xA705	INT32	4	1
        Battery Temperature	degC	0xA706	INT16	2	0
        Remaining Battery Capacity Percent	%	0xA707	INT16	2	0
        Battery Backup Time	h	0xA708~0xA709	UINT32	4	1
        Battery SOH	%	0xA70A	UINT16	2	0
    */ 
    if(sFd_bms < 0)
        return -1;

    recLen_bms = 0;
    recLen_bms = read(sFd_bms, recBuff_bms+recLen_bms, 30);
    if(recLen_bms >0)
    {    
        //21 03 0E 00 0B 00 02 00 C9 04 68 00 02 FF FF FF FF 4D D8  
        while(recLen_bms>=19)
        {
            if(recBuff_bms[0] == 0x21 && recBuff_bms[1] == 0x03 && recBuff_bms[2] == 0x0E)
            {
                unsigned short crc = do_crc_table(recBuff_bms,17);
                if(crc == *(unsigned short*)(recBuff_bms+17))
                {
                    uint8_t cud16[2] = {0,0};
                    uint8_t cud32[4] = {0,0,0,0};
                    //电池电压
                    data_swap_u32(cud32,recBuff_bms+5);
                    pSM->chserinf.Bat_Voltage = ((float)*(uint32_t*)(cud32))/10.0;
                    //电池电流
                    data_swap_u32(cud32,recBuff_bms+9);
                    pSM->chserinf.Bat_Current = ((float)*(int32_t*)(cud32))/10.0;
                    //电池温度
                    data_swap_u16(cud16,recBuff_bms+13);
                    pSM->chserinf.Bat_Temperature = *(int16_t*)(cud16);
                    //电池剩余容量
                    data_swap_u16(cud16,recBuff_bms+15);
                    pSM->chserinf.Bat_Remaining = *(int16_t*)(cud16);

                    //接收到BMS，清空timeout计数
                    rec_timeout = 0;
                }   
                recLen_bms -= 19;
                for (int i = 0; i < recLen_bms; i++)
                    recBuff_bms[i] = recBuff_bms[i+19];
            }else
            {
                recLen_bms -= 1;
                for (int  i = 0; i < recLen_bms; i++)
                    recBuff_bms[i] = recBuff_bms[i+1];
            }
        }
    }
    else
    {
        //10次未读取到，设置BMS显示都0
        if(rec_timeout > 10)
        {
            //电池电压
            pSM->chserinf.Bat_Voltage = 0;
            //电池电流
            pSM->chserinf.Bat_Current = 0.0;
            //电池温度
            pSM->chserinf.Bat_Temperature = 0;
            //电池剩余容量
            pSM->chserinf.Bat_Remaining = 0;
        }
        rec_timeout++;
    }
    return iRet;
}

static int32_t sendBMS(SERIAL_THREAD_INFO *pSM)
{
    int32_t iRet = -1;
    //整流模块接的PCIE的RS485的第八路
    /*
        默认的波特率9600 地址 33. 发送modbus标准协议，读取7个地址的。
        Battery Status	NA	0xA701	UINT16	2	0
        Battery Voltage	V	0xA702~0xA703	UINT32	4	1
        Battery Current	A	0xA704~0xA705	INT32	4	1
        Battery Temperature	degC	0xA706	INT16	2	0
        Remaining Battery Capacity Percent	%	0xA707	INT16	2	0
        Battery Backup Time	h	0xA708~0xA709	UINT32	4	1
        Battery SOH	%	0xA70A	UINT16	2	0
    */ 
    if(sFd_bms < 0)
        return -1;

    //先读取bms数据
    readBMS(pSM);

    uint8_t writeBuff[] = {0x21,0x03,0xA7,0x01,0x00,0x07,0x71,0xdc};
    write(sFd_bms, writeBuff, 8);
    return iRet;
}

static float Slid_avg_filter(float *fvec,int times)
{
    if(times <= 0)
      return 0;
    /*拉力计滤波*/
    float sumT = 0;
    for (int  j = 0; j < times; j++)
    {
        sumT += fvec[j];
    }
    return (sumT/times);
}

void setFdNonblockingm(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        printf("setFdNonblocking fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        printf("setFdNonblocking fcntl F_SETFL fail");
    }
}

void setFdTimeoutm(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
    printf( "setsockopt failed:\n");
}

static int32_t handle_open(SERIAL_THREAD_INFO *pSModule)
{
    ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *)pSModule->pTModule;
    CHASSIS_MASTER_THREAD_INFO *pTChassisModule = (CHASSIS_MASTER_THREAD_INFO *)pSModule->pTChassisModule;

    //打开比例调压阀使用的串口
    recLen_press = 0;
    sFd_press = openUart(ttys_pressure,B9600);
    if(sFd_press < 0)
    {
        //ttyMI0
        sFd_press = openUart(ttys_pressure_MI,B9600);       
    }

    //打开变送器及气压计使用的串口
    recLen_weight = 0;
    sFd_weight = openUart(ttys_weight,B19200);
    if(sFd_weight <0)
    {
        sFd_weight = openUart(ttys_weight_MI,B19200);
    }

    /********测试*/
    recLen_bms = 0;
    sFd_bms = openUart(ttys_bms,B9600);
    if(sFd_bms < 0)
    {
        //ttyMI3
        sFd_bms = openUart(ttys_bms_MI,B9600);       
    }

    mSoket = createSoket(get_port("MASTER_60A3D_PORT"), get_ip("MASTER_60A3D_IP"), 0, 0);
    if(mSoket  < 0) 
    {
        printf( "******** serial socket creat Failed\n");
        return -1;
    }

    mGW1800Soket = createSoket(get_port("SERIAL_WORKER_PORT"), get_ip("SERIAL_WORKER_IP"), 0, 0);
    if(mGW1800Soket  < 0) 
    {
        printf( "******** mGW1800Soket socket creat Failed\n");
        return -1;
    }

    //设置比例调压阀置零.
    for (int i = 0; i < 13; i++)
    {
        pSModule->cmd_proportion[i] = 0;
    }
    /*
    //设置气缸初始气压0.2MPa
    pSModule->cmd_proportion[1] = 0.2;
    pSModule->cmd_proportion[3] = 0.2;
    pSModule->cmd_proportion[5] = 0.2;
    pSModule->cmd_proportion[8] = 0.2;
    pSModule->cmd_proportion[10] = 0.2;
    pSModule->cmd_proportion[12] = 0.2;
    */
    soketRecNum = 0;
    return 0;
}

static void handle_close(SERIAL_THREAD_INFO *pSModule)
{
    //关闭拉力计使用的串口
    for (int  i = 0; i < 8; i++)
    {
        if(serialFd[i]>0)
          closeUart(serialFd[i]);
    }

    //关闭比例调压阀使用的串口
    if(sFd_press>0)
        closeUart(sFd_press);

    //关闭变送器使用的串口
    if(sFd_weight>0)
        closeUart(sFd_weight);

    //删除创建的soket
    if(mSoket>0)
    {
        close(mSoket);
        mSoket = -1;
    }

    if(mGW1800Soket > 0)
    {
        close(mGW1800Soket);
        mGW1800Soket = -1;
    }
}

static void handle_proportional_write(SERIAL_THREAD_INFO *pSModule)
{
    if(mSoket>0)
    {
        //写入13个比例调压阀的气压
        //01 10 00 50 00 08 10 03 E8 4E 20 3A 98 3A 98 3A 98 3A 98 00 00 00 
        //先写第一个 ZQWL-DAM-60A3D-10V的7个口，在写第二个ZQWL-DAM-60A3D-10V的前6个口
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(get_port("DAM_60A3D1_PORT"));
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_60A3D1_IP"));

        uint16_t cmd_presss[13];
        for (int  i = 0; i < 13; i++)
        {
            //真实的气压 = 真实电压值*4 /0.09 传输到模拟量输出模块里.
            cmd_presss[i] = (uint16_t)(pSModule->cmd_proportion[i]*4/0.089);
        }
        
        //第一个网口转模拟量的
        uint8_t writeBuff1[] = {0x01,0x10,0x00,0x50,0x00,0x0A,0x14,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
        memcpy(writeBuff1+7,cmd_presss,sizeof(uint16_t)*7);
        uint16_t crc1 = do_crc_table(writeBuff1,27);
        *((uint16_t*) (writeBuff1+27)) = crc1;
        sendto(mSoket, writeBuff1, 29, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));

        //第二个网口转模拟量的
        mPeerAddr.sin_port = htons(get_port("DAM_60A3D2_PORT"));
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_60A3D2_IP"));
        uint8_t writeBuff2[] = {0x01,0x10,0x00,0x50,0x00,0x0A,0x14,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
        memcpy(writeBuff2+7,&cmd_presss[7],sizeof(uint16_t)*6);
        uint16_t crc2 = do_crc_table(writeBuff2,27);
        *((uint16_t*) (writeBuff2+27)) = crc2;
        sendto(mSoket, writeBuff2, 29, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));
    }
}

static void modbus_send_weight(SERIAL_THREAD_INFO *pSModule,int modbus_id)
{
    //modbus读取标准协议.读取变送器
    if(modbus_id>=1 && modbus_id<=5)
    {
        uint8_t writeBuff[] = {0x01,0x03,0x03,0x00,0x00,0x06,0x00,0x00};
        writeBuff[0] = (uint8_t)modbus_id;
        *(uint16_t*)(writeBuff+6) = do_crc_table(writeBuff,6);
        write(sFd_weight, writeBuff, 8);
    }

    //modbus读取标准协议.读取气压计1和2
    if(modbus_id>=6 && modbus_id<=7)
    {
        uint8_t writeBuff[] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
        writeBuff[0] = (uint8_t)modbus_id;
        *(uint16_t*)(writeBuff+6) = do_crc_table(writeBuff,6);
        write(sFd_weight, writeBuff, 8);
    }
}

static void modbus_read_weight(SERIAL_THREAD_INFO *pSModule,int mod_id)
{
    int32_t iRet = -1;
    if(sFd_weight < 0)
        return ;

    recLen_weight = 0;
    if((recLen_weight = read(sFd_weight, recBuff_weight, 30))<=0)
        return;
    
    //轮询气压计
    if(mod_id == 6 || mod_id == 7)
    {
        while(recLen_weight>=7)
        {
            //气压计
            if((recBuff_weight[0]==0x06 || recBuff_weight[0]==0x07) && recBuff_weight[1] == 0x03 && recBuff_weight[2] == 0x02)
            {
                //读取气压计
                uint8_t cud[2] = {0,0};
                data_swap_u16(cud,recBuff_weight+3);
                //int ide = recBuff_weight[0]-6;
                if(recBuff_weight[0]==0x06)
                    pSModule->chserinf.gas_press_high= ((float)*(uint16_t*)(cud))/100.0;
                if(recBuff_weight[0]==0x07)
                    pSModule->chserinf.gas_press_low= ((float)*(uint16_t*)(cud))/100.0;
                //printf("%f \n",((float)*(uint16_t*)(cud)));
                recLen_weight -= 7;
            }else{
                recLen_weight -= 1;
                for (int  i = 0; i < recLen_weight; i++)
                    recBuff_weight[i] = recBuff_weight[i+1];
            }
        }
    }

    //轮询变送器
    if(mod_id >= 1 && mod_id <= 5)
    {
        while(recLen_weight>=17)
        {
            //5路变送器
            if(recBuff_weight[0] >= 0x01 && recBuff_weight[0]<=0x05 && recBuff_weight[1] == 0x03 && recBuff_weight[2] == 0x0C)
            {   
                //11个称重计
                float p1,p2,p3;
                //高低位转换
                uint8_t cud[4] = {0,0,0,0};
                data_swap_u32(cud,recBuff_weight+3);
                p1 = ((float)*(int32_t*)(cud))/10.0;
                data_swap_u32(cud,recBuff_weight+7);
                p2 = ((float)*(int32_t*)(cud))/10.0;
                data_swap_u32(cud,recBuff_weight+11);
                p3 = ((float)*(int32_t*)(cud))/10.0;
                  /*
                    1号变送器
                    Ch1：左前气足
                    Ch2：左前舵轮
                    Ch3：左边过缝气足
                    2号变送器
                    Ch1：左后气足
                    Ch2：左后舵轮
                    3号变送器
                    Ch1：右前气足
                    Ch2：右前舵轮
                    Ch3：右边过缝气足
                    4号变送器
                    Ch1：右后气足
                    Ch2：右后舵轮
                    5号变送器
                    Ch1：中间气足
                */
                if(recBuff_weight[0] == 0x01)
                {
                    pSModule->chserinf.chassis_weight[0]= p1;
                    pSModule->chserinf.chassis_weight[1]= p2;
                    pSModule->chserinf.chassis_weight[2]= p3;
                }else if(recBuff_weight[0] == 0x02)
                {
                    pSModule->chserinf.chassis_weight[3]= p1;
                    pSModule->chserinf.chassis_weight[4]= p2;
                }else if(recBuff_weight[0] == 0x03)
                {
                    pSModule->chserinf.chassis_weight[5]= p1;
                    pSModule->chserinf.chassis_weight[6]= p2;
                    pSModule->chserinf.chassis_weight[7]= p3;
                }else if(recBuff_weight[0] == 0x04)
                {
                    pSModule->chserinf.chassis_weight[8]= p1;
                    pSModule->chserinf.chassis_weight[9]= p2;
                }else if(recBuff_weight[0] == 0x05)
                {
                    pSModule->chserinf.chassis_weight[10]= p1;
                }

                recLen_weight -= 17;
            }else{
                recLen_weight -= 1;
                for (int  i = 0; i < recLen_weight; i++)
                    recBuff_weight[i] = recBuff_weight[i+1];
            }
        }
    }
}


static void modbus_send_press(SERIAL_THREAD_INFO *pSModule)
{
    modbus_read_press(pSModule);

    if(sFd_press > 0)
    {
        uint8_t writeBuff[] = {0x01,0x03,0x00,0x00,0x00,0x0D,0x84,0x0F};
        write(sFd_press, writeBuff, 8);
    }
}

static void modbus_read_press(SERIAL_THREAD_INFO *pSModule)
{
    //读取拉力计串口信息
    int32_t iRet = -1;
    if(sFd_press < 0)
        return ;

    int len = read(sFd_press, recBuff_press+recLen_press, 60);
    if(len >0)
    {
        //13个模拟量采集数据。
        //01 03 10 FF FF EC 78 00 01 00 01 EC 78 00 02 00 02 00 04 31 48
        recLen_press += len;  
        while(recLen_press>=31)
        {
            if(recBuff_press[0] == 0x01 && recBuff_press[1] == 0x03 && recBuff_press[2] == 0x1A)
            {
                unsigned short crc = do_crc_table(recBuff_press,29);
                if(crc == *(unsigned short*)(recBuff_press+29))
                {
                    uint16_t modbus_presss[13];
                    memcpy(modbus_presss,recBuff_press+3,sizeof(uint16_t)*13);
                    //根据实际采集的16位模拟量值，转换成压力
                    for (int  i = 0; i < 13; i++)
                    {
                        uint8_t *p = (uint8_t*)&modbus_presss[i];
                        uint8_t ptemp = p[0];
                        p[0] = p[1];
                        p[1] = ptemp;
                        //转换到0-4V以内
                        float ptV = ((float)modbus_presss[i]-1000)/1000.0;
                        pSModule->chserinf.proportion_presss[i] = ptV*(0.9/4.0);
                    }
                }
                recLen_press -= 31;
                for (int i = 0; i < recLen_press; i++)
                    recBuff_press[i] = recBuff_press[i+31];
            }else
            {
                recLen_press -= 1;
                for (int  i = 0; i < recLen_press; i++)
                    recBuff_press[i] = recBuff_press[i+1];
            }
        }
    }
}

static void handle_socket_read(SERIAL_THREAD_INFO *pSModule,int mod_id)
{
    ssize_t recSize = 0;
    struct sockaddr_in  mUpperPeerAddr;
    socklen_t upperNum;
    /*
    char ipAddress[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(mUpperPeerAddr.sin_addr), ipAddress, INET_ADDRSTRLEN);
    */
       /*
    第一个GW1800链接的件：  
                        * TODU
                        第一个485：端口1030，挂载8串口继电器
                        第二个485：端口1031，挂载8串口继电器
                        第三个485：端口1032，挂载8串口继电器
                        第四个485：端口1033，挂载8串口继电器
     */

    soketRecNum = 0;
    while((recSize=recvfrom(mSoket, mSoketRecBuff+soketRecNum, 50, MSG_DONTWAIT, (struct sockaddr*)&(mUpperPeerAddr), &upperNum))>0)
    {
        soketRecNum += recSize;
    } 

    //轮询电磁阀的反馈数据.忽略不处理
    if(mod_id == 1)
    {
        while(soketRecNum>=10)
        {
            //8串口电磁阀
            if(mSoketRecBuff[0]==0x48 && mSoketRecBuff[1]==0x3a&&mSoketRecBuff[8]==0x45&&mSoketRecBuff[9]==0x44)
            {
                recLen_press -= 10;
            }else{
                recLen_press--;
            }
        }
    }

    #if 0
    //轮询气压计
    if(mod_id == 2 || mod_id == 3)
    {
        while(soketRecNum>=7)
        {
            //气压计
            if((mSoketRecBuff[0]==0x02 || mSoketRecBuff[0]==0x03) && mSoketRecBuff[1] == 0x03 && mSoketRecBuff[2] == 0x02)
            {
                //读取气压计
                printf("气压计1\n");
            }else{
                soketRecNum -= 1;
                for (int  i = 0; i < soketRecNum; i++)
                    mSoketRecBuff[i] = mSoketRecBuff[i+1];
            }
        }
    }
    #endif

}

static void smooth_filter(SERIAL_THREAD_INFO *pSModule)
{
    ARMS_MASTER_THREAD_INFO *pT = (ARMS_MASTER_THREAD_INFO *)pSModule->pTModule;
    //拉力计滤波
    for (int i = 0; i < pT->armsSize; i++)
    {
        if(pT->slaves_inf[i].childSlaveSize>4)
        {
            //pT->tension[i] = Slid_avg_filter(pT->mF_Inf[i].tension_arr,pT->mF_Inf[i].F_tension_n);
        }
    }
}


static void read_relays_state(SERIAL_THREAD_INFO *pSModule)
{
         /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */

    //定期的查询继电器状态
    relay_port++;

    if(relay_port == 1034)
        relay_port = 1030;

    if(mGW1800Soket>0)
    {
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(relay_port);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));
        //modbus读取标准协议
        uint8_t writeBuff[] = {0x48,0x3a,0x01,0x53,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x26,0x45,0x44};
        int sensize =  sendto(mGW1800Soket, writeBuff, 15, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));
        //printf("************* %d \n",sensize);
    }
}
static void arm24_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
     /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1030);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));
        //modbus读取标准协议
        uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x01, 0x01, 0x00, 0x00, 0x45, 0x44};
        for (int i = 0; i < 7; i++)
        {
            if(pSModule->upperMsg.mModuleFlag[i] == 1)
            {
                writeBuff[4] = i+1;
                writeBuff[5] = flag;
                printf("arm24%d \n",i);
                sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr)); 
            }
        } 
    }
}

static void arm48_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
   /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1031);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));
        //modbus读取标准协议
        uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x01, 0x01, 0x00, 0x00, 0x45, 0x44};
        for (int i = 0; i < 7; i++)
        {
            if(pSModule->upperMsg.mModuleFlag[i] == 1)
            {
                writeBuff[4] = i+1;
                writeBuff[5] = flag;
                printf("*****  arm48: %d \n",i);
                int iRet = sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));
            }
        } 
    }
}

static void chassis24_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
   /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1030);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));
        //modbus读取标准协议
        uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x08, 0x01, 0x00, 0x00, 0x45, 0x44};
        writeBuff[5] = flag;
        sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr)); 
    }
}


static void chassis48_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
   /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1031);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));
        //modbus读取标准协议
        uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x08, 0x01, 0x00, 0x00, 0x45, 0x44};
        writeBuff[5] = flag;
        sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr)); 
    }
}

static void arm_gas_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
    /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1033);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));

        //第一路RS485是8路继电器模块
        uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x01, 0x01, 0x00, 0x00, 0x45, 0x44};
        writeBuff[5] = flag;
        sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));
    }
}

static void chassis_gas_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
    /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1033);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));

        //第一路RS485是8路继电器模块
        uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x02, 0x01, 0x00, 0x00, 0x45, 0x44};
        writeBuff[5] = flag;
        sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));
    }
}

static void chassis_brk_on_off(SERIAL_THREAD_INFO *pSModule,int flag)
{
   /*
    第一个GW1800链接的件：  
                        第一个485：端口1030，挂载8串口24V供电继电器modbusid：1 
                        第二个485：端口1031，挂载8串口48V供电继电器modbusid：1 
                        第三个485：端口1032，挂载8串口48V抱闸继电器modbusid：1 
                        第四个485：端口1033，挂载8串口继电器。两路气路控制
     */
    if(mGW1800Soket>0)
    {
        #if 0
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1032);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));
        //modbus读取标准协议
        uint8_t writeBuff[] = {0x48,0x3a,0x01,0x57,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0xe2,0x45,0x44};
        writeBuff[4] = flag;
        writeBuff[5] = flag;
        writeBuff[6] = flag;
        writeBuff[7] = flag;
        writeBuff[8] = flag;
        writeBuff[9] = flag;
        writeBuff[10] = flag;
        writeBuff[11] = flag;
        if(flag == 0)
            writeBuff[11] = 0xDA;
        
        sendto(mGW1800Soket, writeBuff, 15, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr)); 
        #endif
        //udp透传模式
        struct  sockaddr_in  mPeerAddr;
        bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
        mPeerAddr.sin_family = AF_INET;
        mPeerAddr.sin_port = htons(1032);
        mPeerAddr.sin_addr.s_addr = inet_addr(get_ip("DAM_GW1800_IP"));

        for (int i = 0; i < 5; i++)
        {
            if(pSModule->upperMsg.mModuleFlag[i] == 1)
            {

                //modbus读取标准协议
                uint8_t writeBuff[] = {0x48, 0x3a, 0x01, 0x70, 0x01, 0x01, 0x00, 0x00, 0x45, 0x44};
                writeBuff[4] = i+1; 
                writeBuff[5] = flag;       
                sendto(mGW1800Soket, writeBuff, 10, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr)); 
            }
        }
   }
}

static void handle_gw1800_read(SERIAL_THREAD_INFO *pSModule)
{
    ssize_t recSize = 0;
    struct sockaddr_in  mUpperPeerAddr;
    socklen_t upperNum;
    /*
    第一个GW1800链接的件：  
                        * TODU
                        第一个485：端口1030，挂载8串口继电器
                        第二个485：端口1031，挂载8串口继电器
                        第三个485：端口1032，挂载8串口继电器
                        第四个485：端口1033，挂载8串口继电器
     */

    soketRecNum = 0;
    uint8_t gw1800_buff[100];
    if((recSize=recvfrom(mGW1800Soket, gw1800_buff, 100, MSG_DONTWAIT, (struct sockaddr*)&(mUpperPeerAddr), &upperNum))>0)
    {
        soketRecNum += recSize;

        for(int i=0;i<recSize;i++)
        {
            //printf("%x ",gw1800_buff[i]);
        }
        //printf("\n");
        
        
        //读取继电器状态信息
        if(gw1800_buff[0] == 0x48 && gw1800_buff[1] == 0x3A && gw1800_buff[2] == 0x01 && gw1800_buff[3] == 0x54)
        {
            if(relay_port == 1030)
            {
                memcpy(pSModule->chserinf.relays_24V,gw1800_buff+4,8);
            }
            if(relay_port == 1031)
            {
                memcpy(pSModule->chserinf.relays_48V,gw1800_buff+4,8);
            }
            if(relay_port == 1032)
            {
                memcpy(pSModule->chserinf.relays_brk_48V,gw1800_buff+4,8);
            }            
            if(relay_port == 1033)
            {
                memcpy(pSModule->chserinf.relays_gas,gw1800_buff+4,8);
            }
        }
    } 
}

static void handle_upper_msg(SERIAL_THREAD_INFO *pSModule)
{
    switch (pSModule->upperMsg.mCmd)
    {
        case CMD_VALVE_ARMS24_ON:{
          printf("CMD_VALVE_ARMS24_ON\n");
          arm24_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_ARMS24_OFF:{
          printf("CMD_VALVE_ARMS24_OFF\n");
          arm24_on_off(pSModule,0);
          break;
        }
        case CMD_VALVE_CHASSIS24_ON:{
          printf("CMD_VALVE_CHASSIS24_ON\n");
          chassis24_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_CHASSIS24_OFF:{
          printf("CMD_VALVE_CHASSIS24_OFF\n");
          chassis24_on_off(pSModule,0);
          break;
        }
        case CMD_VALVE_ARMS48_ON:{
          printf("CMD_VALVE_ARMS48_ON\n");
          arm48_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_ARMS48_OFF:{
          printf("CMD_VALVE_ARMS48_OFF\n");
          arm48_on_off(pSModule,0);
          break;
        }
        case CMD_VALVE_CHASSIS48_ON:{
          printf("CMD_VALVE_CHASSIS48_ON\n");
          chassis48_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_CHASSIS48_OFF:{
          printf("CMD_VALVE_CHASSIS48_OFF\n");
          chassis48_on_off(pSModule,0);
          break;
        }
        case CMD_VALVE_ARM_GAS_ON:{
          printf("CMD_VALVE_ARM_GAS_ON\n");
          arm_gas_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_ARM_GAS_OFF:{
          printf("CMD_VALVE_ARM_GAS_OFF\n");
          arm_gas_on_off(pSModule,0);
          break;
        }
        case CMD_VALVE_CHASSIS_GAS_ON:{
          printf("CMD_VALVE_CHASSIS_GAS_ON\n");
          chassis_gas_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_CHASSIS_GAS_OFF:{
          printf("CMD_VALVE_CHASSIS_GAS_OFF\n");        
          chassis_gas_on_off(pSModule,0);
          break;
        }
        case CMD_VALVE_CHASSIS_BRK_ON:{
          printf("CMD_VALVE_CHASSIS_BRK_ON\n");
          chassis_brk_on_off(pSModule,1);
          break;
        }
        case CMD_VALVE_CHASSIS_BRK_OFF:{
          printf("CMD_VALVE_CHASSIS_BRK_OFF\n");
          chassis_brk_on_off(pSModule,0);
          break;
        }
        default:{
          break;
        }
    }
}

/******************************************************************************
* 功能：子线程：master使用serial线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : args
* @return Descriptions
******************************************************************************/
void* serialThread(void* pModule)
{
    SERIAL_THREAD_INFO *pSModule = (SERIAL_THREAD_INFO *) pModule;
    //ARMS_MASTER_THREAD_INFO *pTModule = (ARMS_MASTER_THREAD_INFO *)pSModule->pTModule;

    unsigned int nDelay = 1000;        /* usec */
    //timer 定时器
    struct timespec  next, interval;
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int ret = 0;
    uint32_t  runCnt = 0;

    pSModule->mState = SR_OPEN;
    pSModule->newUpperMsg = false;
    uint32_t maxE = 0;
    while (pSModule->mWorking)
    {
        next.tv_sec  += interval.tv_sec;
        next.tv_nsec += interval.tv_nsec;
        tsnorm(&next);
        if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)))
        {
            if (ret != EINTR)
                printf("clock_nanosleep failed. errno:");
            continue;
        }
 
        //数据滤波
        if(runCnt%10 == 0)
        {
            smooth_filter(pSModule);
        }

        //轮询. 200ms一次
        if((pSModule->mState ==SR_RUN) && (runCnt%200 == 0))
        {
            /************************第二路485.1-5个变送器 气压计1 2轮询采集********************/
            modbus_read_weight(pSModule,Polling_modbusid);
            Polling_modbusid++;
            if(Polling_modbusid >=8)
            {
                Polling_modbusid = 1;
            }
            modbus_send_weight(pSModule,Polling_modbusid);
            /************************第二路485.1-5个变送器 气压计1 2轮询采集 end********************/
           
            //发送modbus读取模拟量采集数据.5F03D
            modbus_send_press(pSModule);
            //发送BMS读取命令。
            sendBMS(pSModule);
            /*
            printf("左前气足：%f 左前舵轮：%f 左边过缝：%f 左后气足：%f 左后舵轮：%f 右前气足：%f 右前舵轮：%f 右边过缝：%f 右后气足：%f 右后舵轮：%f 中间气足:%f\n",
                    pSModule->chserinf.chassis_weight[0],
                    pSModule->chserinf.chassis_weight[1],
                    pSModule->chserinf.chassis_weight[2],
                    pSModule->chserinf.chassis_weight[3],
                    pSModule->chserinf.chassis_weight[4],
                    pSModule->chserinf.chassis_weight[5],
                    pSModule->chserinf.chassis_weight[6],
                    pSModule->chserinf.chassis_weight[7],
                    pSModule->chserinf.chassis_weight[8],
                    pSModule->chserinf.chassis_weight[9],
                    pSModule->chserinf.chassis_weight[10]);
            */
        }                   

        if(runCnt % 30 == 0)
        {
            handle_gw1800_read(pSModule);
        }
        if(runCnt %200 == 0)
        {
            //循环的读取继电器状态
            read_relays_state(pSModule);
        }

        //数据转换及滤波，0.005s
        //执行算法,5ms周期
        if(runCnt%8 == 0)
        {
            switch (pSModule->mState)
            {
                case SR_OPEN:{
                    handle_open(pSModule);
                    pSModule->mState = SR_RUN;
                    break;
                }
                case SR_CLOSE:{
                    handle_close(pSModule);
                    pSModule->mState = SR_NULL;
                    break;
                }
                case SR_RUN:{
                    //写入模拟量命令
                    handle_proportional_write(pSModule);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        //处理上位机发送的消息
        if(pSModule->newUpperMsg)
        {
            if(pSModule->mState ==SR_RUN)
            {
                handle_upper_msg(pSModule);
            }
            pSModule->newUpperMsg = false;
        }
        
        runCnt++;
    }
    sleep(1);
    handle_close(pSModule);
    printf("serialThread exit\n\r");
}

static int32_t instervalus(float value,float *tension_vec,int filtertimes)
{
    for (int  i = 0; i < filtertimes-1; i++)
    {
        tension_vec[i] = tension_vec[i+1];
    }
    tension_vec[filtertimes-1] = value;
}