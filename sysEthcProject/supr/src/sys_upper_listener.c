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
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <signal.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "sys_upper_listener.h"
#include "sys_ctl.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_conf.h"
#include "sys_share_messages.h"
#include "sys_share_defs.h"


//定义字符串链表结构体
typedef struct upd_str{
    char str[512];/*> 最大存储512个字符*/
    struct sockaddr_in  mAddr; /*> 要发送的地址*/
    int ackType;

    int listSize;

    struct upd_str *front;
    struct upd_str *next; 
}upd_str;

static int32_t mRecUpperSoket = -1;
static int32_t mLocalSoket = -1;
static upd_str mUdpStrs={0};

struct sockaddr_in  mUpperPeerAddr;
socklen_t upperNum;

struct sockaddr_in  mLocalPeerAddr;
socklen_t localNum;

//上位机的ip和port
static char MN_UpperIpV4Str[16] = "192.168.2.99";
static uint16_t mUpperSoketPort = 10001;
//显示屏电脑的ip和port
static char MN_TVIpV4Str[16] = "192.168.1.99";
static uint16_t mTvSoketPort = 10001;
//心跳包标志位
static uint16_t mUpperTimeOut = 0;
uint64_t lastRecHeartTime = 0;


//将上层、中层机械臂映射，1 2 3 4 5 6 7 代表映射到上层的号，11 12 13 14 15 16 17 代表映射到中层的号
static int arms_id_map[8];

static int32_t initSupr(LISTENER_THREAD_INFO *pTModule);
static void LoopRecLocalMsg();

static void amrsIdToMasterId(uint8_t *armsFlag,uint8_t *master0Arms,uint8_t *master1Arms);

static void LoopRecUpperMsg();
static void LoopRecUpperScanSlaves();
static void sendSMgToUpper(char* ptr, int length,char*ip,uint16_t port);
static void sendSMgToUpperAddr(char* ptr, int length,struct sockaddr* address);

static int32_t  sendLeaderMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendMasterMsg(const int32_t mMasterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendAllMasterMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendArmsMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendChassisMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);
static int32_t  sendMasterNoticeMsg(MASTER_ID masterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);
static int32_t  sendAllNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue);
static int32_t  sendLoggerNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize);

static int32_t handleRWSlaveInf(int masterid);
static int32_t handleRWMasterGraph(int masterid);
static int32_t handleRWSlavsSdo(int masterid,int spos);
static int32_t handleRWSlavsAlias(int mid,int spos,int salias);
static int32_t handleRWSlavsUpload(int mid,int spos,char*pstr);
static int32_t handleRWSlavsDownload(int mid,int spos,char*pstr1,char*pstr2,char*pstr3);
static uint64_t getCurrentTime();

//系统调用的接口,查询igh的信息
static int32_t  systemPopen(char * cmdStr,strlist *strHead);
static int32_t  udpStrlistInsert(upd_str* strH,char* strs,struct sockaddr_in addr,int ackType);
static int32_t  udpStrlistsInsert(upd_str* strH,strlist* strs,struct sockaddr_in addr,int ackType);

/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(LISTENER_THREAD_INFO *pTModule) {
  int32_t iRet = 0;

  //创建soket.非阻塞模式
  char eno1_ip[IP_C_SIZE];
  //get_local_ip("eno1",eno1_ip);
  get_local_ip("wlp1s0",eno1_ip);

  mRecUpperSoket = createSoket(get_port("LISTENER_TO_UPPER_PORT"), eno1_ip, 0, 0);

  if(mRecUpperSoket  < 0) {
      printf( "listener socket creat Failed\n");
      return -1;
  }

  //创建soket.非阻塞模式
  mLocalSoket = createSoket(get_port("LISTENER_LOCAL_PORT"), MN_LocalIpV4Str, 0, 0);

  if(mLocalSoket  < 0) {
      printf( "local socket creat Failed");
      return -1;
  }

  return iRet;
}

/******************************************************************************
* 函数名称:  LoopRecUpperMsg
* 功能说明:  循环接收上位机消息
            该函数通过udp接收
* 输入参数:  pucByteArray: 掩码字节数组
            ucByteNum   : 掩码字节数组待转换的有效字节数目
            ucBaseVal   : 掩码字符串起始字节对应的数值
* 输出参数:  pStrSeq     ：掩码字符串，以','、'-'间隔
            形如0xD7(0b'11010111)  ---> "0-1,3,5-7"
* 返 回 值:  pStr        ：pStrSeq的指针备份，可用于strlen等链式表达式
* 用法示例:  INT8U aucByteArray[8] = {0xD7, 0x8F, 0xF5, 0x73};
            CHAR szSeq[64] = {0};
            ByteArray2StrSeq(aucByteArray, 4, 0, szSeq);
               ----> "0-1,3,5-8,12-19,21,23,25-27,30-31"
            memset(szSeq, 0, sizeof(szSeq));
            ByteArray2StrSeq(aucByteArray, 4, 1, szSeq);
               ----> "1-2,4,6-9,13-20,22,24,26-28,31-32"
* 注意事项:  因本函数内含strcat，故调用前应按需初始化pStrSeq
******************************************************************************/
static void LoopRecUpperMsg()
{
    char mUpperData[Msg_T_S];

    ssize_t recSize = 0;
    while((recSize=recvfrom(mRecUpperSoket, mUpperData, Msg_T_S, 0, (struct sockaddr*)&(mUpperPeerAddr), &upperNum))>0)
    {
      SUPR_R_MSG *mMsg = (SUPR_R_MSG *)mUpperData;
      switch (mMsg->mCmd)
      {
        //链接主站
        case CMD_LINK:{
          uint16_t *pPort = (uint16_t*)mMsg->mData;
          mUpperSoketPort = *pPort;
          strcpy(MN_UpperIpV4Str,mMsg->mData+2);
          printf("ip: %s",MN_UpperIpV4Str);
          break;
        }
        //断开链接主站
        case CMD_UNLINK:{
          sendAllMasterMsg(CMD_DESTROY_MASTER,0,0,0,0);
          break;
        }
        //心跳包消息
        case CMD_HEARTBEAT:
        {
          //接收到心跳包
          break;
        }
        //扫描从站
        case CMD_IGH_CREATE_MASTERS:
        case CMD_DESTROY_MASTER:
        case CMD_IGH_SCAN_SLAVES:
        {
          sendAllMasterMsg(mMsg->mCmd,0,0,0,0);
          break;
        }
        case CMD_MASTER_OP_KEYDOWN:
        {
          sendAllMasterMsg(mMsg->mCmd,0,*mMsg->mData,0,0);
          break;
        }
        case CMD_IGH_SDO_DOWNLOAD:
        case CMD_IGH_SDO_UPLOAD:
        {
          //sendArmsMasterMsg(MSG_ID_UPPER_SDO_DOWNLOAD,0,0,mMsg->mData,sizeof(sdo_download_t));
          break;
        }
        case CMD_IGH_CONF_SERVO_PDOS:
        case CMD_IGH_START_OP:
        {
          sendMasterNoticeMsg(MN_ID_MASTER_ARMS,mMsg->mCmd,0,0);
          break;
        }
        case CMD_IGH_CLEAR_ERROR:
        {
          sendChassisMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          sendArmsMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_HAND_VEL_MM:
        case CMD_HAND_RELA_POS_MM:
        case CMD_HAND_ABS_POS_M:
        case CMD_EC3045_PROPORTION:
        case CMD_AUTO_VERTICAL_WEIGHT:
        case CMD_AUTO_VERTICAL:
        case CMD_IGH_SIKO_ZERO:
        case CMD_IGH_X_EXT_ABS_POS:
        case CMD_INTERPN_START_COLLECT:
        case CMD_INTERPN_COLLECT:
        case CMD_INTERPN_COLLECT_OVER:
        case CMD_IGH_CTRL_WORD:
        {
          sendArmsMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_IGH_Z_ZERO:
        case CMD_IGH_Y_ZERO:
        case CMD_IGH_XS_ZERO:
        case CMD_IGH_XM_ZERO:
        case CMD_CONF_R_DB_TABLES_NAME:
        case CMD_CONF_R_TABLE_ENTYR:
        case CMD_CONF_D_TABLE_ENTYR:
        case CMD_CONF_UPDATE_TABLE_ENTYR:
        {
            sendLeaderMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
            break;
        }
        case CMD_IGH_FSA_SHUTDOWN:
        case CMD_IGH_FSA_SWITH_ON:
        case CMD_IGH_FSA_ENOP:
        case CMD_IGH_FSA_HALT:
        {
          IsSystemError = true;
          sendAllNoticeMsg(mMsg->mCmd,0,0);
          break;
        }
        case CMD_AUTO_FOLLOW:
        case CMD_AUTO_FOLLOW_VERTICAL:
        case CMD_ALL_STOP:
        {
          sendChassisMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          sendArmsMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_CHASSIS_MAGIC_ENABLE:
        case CMD_AUTO_FOLLOW_CHASSIS:
        {
          sendChassisMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_IGH_CHASSIS_VEL:
        case CMD_CHASSIS_WHEEL_AGNGLE:
        case CMD_CHASSIS_MOVE:
        case CMD_CHASSIS_STOP:
        {
          //每次查询只有一个值返回
          sendChassisMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_IGH_RW_SLAVES_INF:{
          handleRWSlaveInf(*((int32_t*)mMsg->mData));
          break;
        }
        case CMD_IGH_RW_MASTER_GRAPH:{
          handleRWMasterGraph(*((int32_t*)mMsg->mData));
          break;
        }
        case CMD_IGH_RW_SLAVE_SDO:{
          handleRWSlavsSdo(*((int32_t*)mMsg->mData),*((int32_t*)(mMsg->mData+4)));
          break;
        }
        case CMD_IGH_RW_SLAVE_ALIAS:{
          handleRWSlavsAlias(*((int32_t*)mMsg->mData),*((int32_t*)(mMsg->mData+4)),*((int32_t*)(mMsg->mData+8)));
          break;
        }
        case CMD_IGH_RW_SLAVE_UPLOAD:{
          //每次查询只有一个值返回
          handleRWSlavsUpload(*((int32_t*)mMsg->mData),*((int32_t*)(mMsg->mData+4)),mMsg->mData+8);
          break;
        }
        case CMD_IGH_RW_SLAVE_DOWNLOAD:{
          //每次查询只有一个值返回
          handleRWSlavsDownload(*((int32_t*)mMsg->mData),*((int32_t*)(mMsg->mData+4)),mMsg->mData+8,mMsg->mData+18,mMsg->mData+30);
          break;
        }
        case CMD_VALVE_ARMS24_ON:
        case CMD_VALVE_ARMS24_OFF:
        case CMD_VALVE_ARM_GAS_ON:
        case CMD_VALVE_ARM_GAS_OFF:
        case CMD_VALVE_ARMS48_ON:
        case CMD_VALVE_ARMS48_OFF:
        { 
          sendArmsMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_VALVE_CHASSIS24_ON:
        case CMD_VALVE_CHASSIS24_OFF:
        case CMD_VALVE_CHASSIS48_ON:
        case CMD_VALVE_CHASSIS48_OFF:
        case CMD_VALVE_CHASSIS_GAS_ON:
        case CMD_VALVE_CHASSIS_GAS_OFF:
        case CMD_VALVE_CHASSIS_BRK_ON:
        case CMD_VALVE_CHASSIS_BRK_OFF:
        case CMD_PROPORTIONAL_VALVE:
        case CMD_PROPORTIONAL_VALVES:
        {
          sendChassisMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg,recSize);
          break;
        }
        case CMD_IGH_VERTICAL_LIMIT:
        {
          sendArmsMasterDataMsg(mMsg->mCmd,0,0,(char*)mMsg->mData,recSize);
          // Is_Zlimit_Open = (mMsg->mData[0] == 'Y'?true : false);
          
        }
        default:{
          break;
        }
      } //end switch
      lastRecHeartTime = getCurrentTime();
    }

    //循环的将strlist链表字符串数据udp发送出去。
    if(mUdpStrs.listSize >0 )
    {
        upd_str *pNext = mUdpStrs.next;
        if(pNext != NULL)
        {
            ARMS_S_IGH_MSG *sendstrMsg=malloc(sizeof(ARMS_S_IGH_MSG)+strlen(pNext->str)+1);
            memcpy(sendstrMsg->mData,pNext->str,strlen(pNext->str)+1);
            sendstrMsg->mMsgType = pNext->ackType;
            sendSMgToUpperAddr((char*)sendstrMsg,sizeof(ARMS_S_IGH_MSG)+strlen(pNext->str)+1,(struct sockaddr*)&pNext->mAddr);
            mUdpStrs.next = pNext->next;
            mUdpStrs.listSize--;
            free(pNext);
            free(sendstrMsg);
        }
    }
  
}

static int32_t handleRWSlaveInf(int masterid)
{
    char cmdPopen[64];
    //sprintf(cmdPopen,"sudo ethercat -m %d slave",*((int32_t*)mMsg->mData));
    sprintf(cmdPopen,"sudo ethercat -m %d slave",masterid);
    strlist sHeadRet={0};
    systemPopen(cmdPopen,&sHeadRet);
    udpStrlistsInsert(&mUdpStrs,&sHeadRet,mUpperPeerAddr,ACK_NOTICE_IGH_RW_SLAVES);
    destroy_strlist(&sHeadRet);  
}

static int32_t  handleRWMasterGraph(int masterid)
{
    char cmdPopen[64];
    sprintf(cmdPopen,"sudo ethercat -m %d graph",masterid);
    strlist sHeadRet={0};
    systemPopen(cmdPopen,&sHeadRet);
    udpStrlistsInsert(&mUdpStrs,&sHeadRet,mUpperPeerAddr,ACK_NOTICE_IGH_RW_GRAPH);
    destroy_strlist(&sHeadRet);
}

static int32_t handleRWSlavsSdo(int masterid,int spos)
{
    char cmdPopen[64];
    //sprintf(cmdPopen,"sudo ethercat -m %d -p %d sdos",*((int32_t*)mMsg->mData),*((int32_t*)(mMsg->mData+4)));
    sprintf(cmdPopen,"sudo ethercat -m %d -p %d sdos",masterid,spos);
    strlist sHeadRet={0};
    systemPopen(cmdPopen,&sHeadRet);
    udpStrlistsInsert(&mUdpStrs,&sHeadRet,mUpperPeerAddr,ACK_NOTICE_IGH_RW_SDO);
    destroy_strlist(&sHeadRet);
}

static int32_t handleRWSlavsAlias(int mid,int spos,int salias)
{
    char cmdPopen[64];
    sprintf(cmdPopen,"sudo ethercat -m %d -p %d alias %d ",mid,spos,salias);
    strlist sHeadRet={0};
    systemPopen(cmdPopen,&sHeadRet);
}

static int32_t handleRWSlavsUpload(int mid,int spos,char*pstr)
{
  char cmdPopen[64];
  sprintf(cmdPopen,"sudo ethercat -m %d -p %d upload %s ",mid,spos,pstr);
  strlist sHeadRet={0};
  systemPopen(cmdPopen,&sHeadRet);
  //udpStrlistsInsert(&mUdpStrs,&sHeadRet,mUpperPeerAddr,ACK_NOTICE_IGH_RW_UPLOAD);
  if (sHeadRet.child_size == 1)
  {   
      strlist* pNext = sHeadRet.next;
      ARMS_S_IGH_MSG *sendstrMsg=malloc(sizeof(ARMS_S_IGH_MSG)+strlen(pNext->str)+1);
      memcpy(sendstrMsg->mData,pNext->str,strlen(pNext->str)+1);
      sendstrMsg->mMsgType = ACK_NOTICE_IGH_RW_UPLOAD;
      strcpy(sendstrMsg->addr, pstr);
      sendSMgToUpperAddr((char*)sendstrMsg,sizeof(ARMS_S_IGH_MSG)+strlen(pNext->str)+1,(struct sockaddr*)&mUpperPeerAddr);
      free(sendstrMsg);
  } 
  destroy_strlist(&sHeadRet);
}

static uint64_t getCurrentTime()
{
  uint64_t currT = 0;
  struct timespec  nowTime;
  clock_gettime(CLOCK_MONOTONIC, &nowTime);

  currT = ((int64_t)(nowTime.tv_sec))*1000 + (nowTime.tv_nsec/1000000);

  return currT;
}

static int32_t handleRWSlavsDownload(int mid,int spos,char*pstr1,char*pstr2,char*pstr3)
{
    char cmdPopen[64];
    sprintf(cmdPopen,"sudo ethercat -m %d -p %d -t %s download %s %s",mid,spos,pstr1,pstr2,pstr3);
    strlist sHeadRet={0};
    systemPopen(cmdPopen,&sHeadRet);
    destroy_strlist(&sHeadRet);

    //修改到伺服驱动文件里，掉电不丢失
    sprintf(cmdPopen,"sudo ethercat -m %d -p %d -t uint32 download 0x1010 01 0x65766173",mid,spos);
    systemPopen(cmdPopen,&sHeadRet);
    destroy_strlist(&sHeadRet);
}

static void sendSMgToUpper(char* ptr, int length,char*ip,uint16_t port)
{
    int32_t iRet = 0;
    //peer
    struct  sockaddr_in  mPeerAddr;
    bzero(&(mPeerAddr), sizeof(struct sockaddr_in));
    mPeerAddr.sin_family = AF_INET;

    mPeerAddr.sin_port = htons(port);
    mPeerAddr.sin_addr.s_addr = inet_addr(ip);

    iRet = sendto(mRecUpperSoket, ptr, length, 0,(struct sockaddr *)&(mPeerAddr), sizeof(mPeerAddr));
}

static void sendSMgToUpperAddr(char* ptr, int length,struct sockaddr* address)
{
    int32_t iRet = 0;
    iRet = sendto(mRecUpperSoket, ptr, length, 0,address, sizeof(struct sockaddr_in));
}


static void amrsIdToMasterId(uint8_t *armsFlag,uint8_t *master0Arms,uint8_t *master1Arms)
{
    memset(master0Arms,0,sizeof(uint8_t)*MASTER_ARMS_MAX_SIZE);
    memset(master1Arms,0,sizeof(uint8_t)*MASTER_ARMS_MAX_SIZE);

    //memcpy(master0Arms,armsFlag,sizeof(uint8_t)*mMaster_graphs[0].arms_size);
   //memcpy(master1Arms,&armsFlag[mMaster_graphs[0].arms_size],sizeof(uint8_t)*mMaster_graphs[1].arms_size);
}
static void LoopRecUpperScanSlaves()
{
   
}

static void LoopRecLocalMsg()
{
    char mLocalData[Msg_T_S];

    while(recvfrom(mLocalSoket, mLocalData, Msg_T_S, 0, (struct sockaddr*)&(mLocalPeerAddr), &localNum)>0)
    {
      MsgHeader *mMsg = (MsgHeader *)mLocalData;

      switch (mMsg->mMsgId)
      {
        case MSG_ID_MASTER_GRAPH:
        {  
          //计算机械臂号的映射    
          break;
        }
        case MSG_ID_MASTER_SHOW_INF:
        {
          MsgLongMsg* mLMsg = (MsgLongMsg *)mLocalData;
          sendSMgToUpper(mLMsg->mData,mLMsg->mValue1,MN_UpperIpV4Str,mUpperSoketPort);

          //发送到显示屏
          //sendSMgToUpper(mLMsg->mData,mLMsg->mValue1,MN_TVIpV4Str,mTvSoketPort);
          break;
        }
        case MSG_ID_MASTER_SHOW_PARAMS:
        case MSG_ID_UPPER_NOTICE:
        {
          MsgLongMsg* mLMsg = (MsgLongMsg *)mLocalData;
          sendSMgToUpper(mLMsg->mData,mLMsg->mValue1,MN_UpperIpV4Str,mUpperSoketPort);
          break;
        }
        case MSG_ID_MASTER_ALL_STOP:
        {
            //主动停止算法运行
            SUPR_R_MSG mMsg;
            memset(mMsg.mModuleFlag,1,sizeof(uint8_t)*8);
            sendChassisMasterDataMsg(CMD_ALL_STOP,0,0,(char*)&mMsg,sizeof(SUPR_R_MSG));
            sendArmsMasterDataMsg(CMD_ALL_STOP,0,0,(char*)&mMsg,sizeof(SUPR_R_MSG));
            break;
        }
        case MSG_ID_MASTER_SHOW_SAMPLE_INF:
        {
          MsgLongMsg* mLMsg = (MsgLongMsg *)mLocalData;

          //发送到显示屏
          sendSMgToUpper(mLMsg->mData,mLMsg->mValue1,MN_TVIpV4Str,mTvSoketPort);
          break;
        }
        default:
        {
          break;
        }
      } //end switch

    }
}


static int32_t  sendLeaderMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_port(MN_ID_LEADER), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  return iret;
}


static int32_t  sendMasterMsg(const int32_t mMasterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(mMasterId), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);

  return iret;
}

static int32_t  sendAllMasterMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_ARMS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_CHASSIS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  
  return iret;
}


static int32_t  sendArmsMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_ARMS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  
  return iret;
}

static int32_t  sendChassisMasterDataMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
  int iret = 0;

  iret = MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_CHASSIS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
  
  return iret;
}


static int32_t  sendMasterNoticeMsg(MASTER_ID masterId,const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
   int iret = 0;
   iret = MsgSendNotice(mLocalSoket, get_mn_master_port(masterId), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue);
   return iret; 
}

static int32_t  sendAllNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue)
{
    int iret = 0;
    iret = sendMasterNoticeMsg(MN_ID_MASTER_ARMS,mMsgId,mNotice,mValue);
    iret = sendMasterNoticeMsg(MN_ID_MASTER_CHASSIS,mMsgId,mNotice,mValue);
    return iret;
}

static int32_t  sendLoggerNoticeMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue,const char* mData, const int32_t mDSize)
{
   int iret = 0;
   iret = MsgSendLongMsg(mLocalSoket, get_port("LOGGER_LOCAL_PORT"), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
   return iret; 
}

static int32_t  systemPopen(char * cmdStr,strlist *strHead)
{
    FILE*fp = popen(cmdStr,"r");
    int slavesize = 0;
    if (fp != NULL)
    {   
        char buffer[128];
        while (fgets(buffer, 128,fp) != NULL)
        {
            strlist *strtemp = (strlist*)malloc(sizeof(strlist));
            memset(strtemp,0,sizeof(strlist));
            strcpy(strtemp->str,buffer);
            push_back_strlist(strHead,strtemp);   
        }
        fclose(fp);          
    }

}

static int32_t  udpStrlistInsert(upd_str* strH,char* strs,struct sockaddr_in addr,int ackType)
{
   if(strH == NULL)
      return -1;

    upd_str* strT = (upd_str*)malloc(sizeof(upd_str));
    memset(strT,0,sizeof(upd_str));
    strT->mAddr = addr;
    strT->ackType = ackType;
    strT->str[0]='\0';
    strcpy(strT->str,strs); 
    if(strH->next == NULL)
    {
        strT->front = strH;
        strH->next = strT;
        strH->front = strT;
    }else
    {
        strT->front = strH->front;
        strH->front->next = strT;
        strH->front = strT;
    }
    strH->listSize++;
}

static int32_t  udpStrlistsInsert(upd_str* strH,strlist* strs,struct sockaddr_in addr,int ackType)
{
    if(strH==NULL || strs==NULL)
        return -1;
    strlist* strP = strs->next;
    for (int i = 0; i < strs->child_size; i++)
    {
        udpStrlistInsert(strH,strP->str,addr,ackType);
        strP = strP->next;
    }    
}


/******************************************************************************
* 功能：此函数销毁初始化模块，释放线程指针里的内容
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int moduleEndUp(LISTENER_THREAD_INFO *pTModule)
{
  //删除创建的soket
  if(mRecUpperSoket>0)
  {
      close(mRecUpperSoket);
      mRecUpperSoket = -1;
  }

  if(mLocalSoket>0)
  {
      close(mLocalSoket);
      mLocalSoket = -1;
  }

  printf("%s leader endup\n", pTModule->mThreadName);
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @return Descriptions
******************************************************************************/
void* upper_threadEntry(void* pModule)
{
  LISTENER_THREAD_INFO * pTModule =(LISTENER_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    printf("null timer\n");
    return 0;
  }
  
  if(initSupr(pTModule)<0)
    return 0; 

  //初始化完成
  pTModule->mInitOk = true;

  upperNum = sizeof(mUpperPeerAddr);
  localNum = sizeof(mLocalPeerAddr);
  uint32_t static runCnt = 0;

  while(pTModule->mWorking)
  {
      //由于接收上位机和本地消息的套接字设置非阻塞模式，必须要sleep函数，避免cpu一直运行
      usleep(2000);

      //循环接收上位机和本地的消息。
      LoopRecUpperMsg();

      //循环接收上位机和本地的消息。
      LoopRecLocalMsg();

      //心跳包判断是否断开,1s判断一次心跳包
      if(runCnt % 2000 == 0)
      {
        if(abs(getCurrentTime()- lastRecHeartTime)>2000)
        {
          printf("***************%d abs(getCurrentTime()- lastRecHeartTime)>1000\n",runCnt);
          if(mUpperTimeOut < 3)
          {
            //销毁主站。
            sendAllMasterMsg(CMD_DESTROY_MASTER,0,0,0,0);
            mUpperTimeOut++;
          }
        }else{
            mUpperTimeOut = 0;
        }
      }
      runCnt++;
  }
  sleep(1);
  moduleEndUp(pTModule);
}

