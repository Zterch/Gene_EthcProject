/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_upper_listener.c - 接收上位机 UDP 命令，转发给 leader/master (与 sysEthcProject 一致)
********************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "sys_upper_listener.h"
#include "sys_ctl.h"
#include "sys_share_ipc.h"
#include "sys_share_conf.h"
#include "sys_share_messages.h"

static int32_t mRecUpperSoket = -1;
static int32_t mLocalSoket = -1;
static struct sockaddr_in mUpperPeerAddr;
static socklen_t upperNum = sizeof(mUpperPeerAddr);

// 【新增】辅助函数：从接收数据中提取轴掩码
static inline uint8_t extract_axis_mask(const char *data, ssize_t size) 
{
    // SUPR_R_MSG 头部大小 = sizeof(SUPR_R_MSG) = 20字节 (mCmd + mFrameId + mModuleFlag + mDataLength + mCRC)
    // 轴掩码位于 mData[0]
    const size_t SUPR_MSG_HEADER_SIZE = 20;  // sizeof(SUPR_R_MSG) 的实际大小
    
    if (size > (ssize_t)SUPR_MSG_HEADER_SIZE) {
        return (uint8_t)data[SUPR_MSG_HEADER_SIZE];
    }
    return 0x3F;  // 默认6轴全部选中
}

static int32_t sendMasterMsg(const int32_t mMsgId, const int32_t mNotice, const int32_t mValue, const char *mData, const int32_t mDSize)
{
    if (mLocalSoket < 0) return -1;
    return MsgSendLongMsg(mLocalSoket, get_mn_master_port(MN_ID_MASTER_ARMS), MN_ID_SUPR, MN_ID_LEADER, mMsgId, mNotice, mValue, mData, mDSize);
}

int32_t initUpperListener(void)
{
    char eno1_ip[IP_C_SIZE];
    if (get_local_ip("enx00e04c680022", eno1_ip) != 0 && get_local_ip("eno1", eno1_ip) != 0)
        snprintf(eno1_ip, sizeof(eno1_ip), "0.0.0.0");
    mRecUpperSoket = createSoket(get_port("LISTENER_TO_UPPER_PORT"), eno1_ip, 0, 0);
    if (mRecUpperSoket < 0) {
        printf("supr: LISTENER_TO_UPPER_PORT socket failed\n");
        return -1;
    }
    mLocalSoket = createSoket(get_port("LISTENER_LOCAL_PORT"), MN_LocalIpV4Str, 0, 0);
    if (mLocalSoket < 0) {
        printf("supr: LISTENER_LOCAL_PORT socket failed\n");
        close(mRecUpperSoket);
        mRecUpperSoket = -1;
        return -1;
    }
    printf("supr: upper listener init ok (upper port %u, local port %u)\n",
           (unsigned)get_port("LISTENER_TO_UPPER_PORT"), (unsigned)get_port("LISTENER_LOCAL_PORT"));
    return 0;
}

void LoopRecUpperMsg(void)
{
    char mUpperData[Msg_T_S];
    ssize_t recSize = recvfrom(mRecUpperSoket, mUpperData, sizeof(mUpperData), 0, (struct sockaddr *)&mUpperPeerAddr, &upperNum);
    if (recSize <= 0) return;
    
    SUPR_R_MSG *mMsg = (SUPR_R_MSG *)mUpperData;
    int32_t cmd = mMsg->mCmd;
    
    printf("supr: 收到上位机命令 cmd=%d, size=%zd\n", cmd, recSize);
    
    switch (cmd) {
        case CMD_LINK: {
            /* 转发CMD_LINK并携带上位机地址，供master正确回发PID/状态等响应 */
            char linkData[6];
            linkData[0] = (char)(mUpperPeerAddr.sin_port & 0xFF);
            linkData[1] = (char)((mUpperPeerAddr.sin_port >> 8) & 0xFF);
            memcpy(linkData + 2, &mUpperPeerAddr.sin_addr.s_addr, 4);
            printf("supr: 转发 CMD_LINK -> master, 上位机 %s:%d\n",
                   inet_ntoa(mUpperPeerAddr.sin_addr), ntohs(mUpperPeerAddr.sin_port));
            sendMasterMsg(CMD_LINK, 0, 0, linkData, 6);
            break;
        }
        case CMD_UNLINK:
            sendMasterMsg(CMD_DESTROY_MASTER, 0, 0, NULL, 0);
            break;
        case CMD_HEARTBEAT:
            break;
        case CMD_IGH_CREATE_MASTERS:
            printf("supr: 转发 CMD_IGH_CREATE_MASTERS -> master\n");
            sendMasterMsg(CMD_IGH_CREATE_MASTERS, 0, 0, NULL, 0);
            break;
        case CMD_IGH_SCAN_SLAVES:
            printf("supr: 转发 CMD_IGH_SCAN_SLAVES -> master\n");
            sendMasterMsg(CMD_IGH_SCAN_SLAVES, 0, 0, NULL, 0);
            break;
        case CMD_IGH_CONF_SERVO_PDOS:
            printf("supr: 转发 CMD_IGH_CONF_SERVO_PDOS -> master\n");
            sendMasterMsg(CMD_IGH_CONF_SERVO_PDOS, 0, 0, NULL, 0);
            break;
        case CMD_IGH_START_OP:
            printf("supr: 转发 CMD_IGH_START_OP -> master\n");
            sendMasterMsg(CMD_IGH_START_OP, 0, 0, NULL, 0);
            break;
            
        // 【修复】所有FSA命令添加轴掩码提取和转发
        case CMD_IGH_FSA_SHUTDOWN: {
            uint8_t mask = extract_axis_mask(mUpperData, recSize);
            printf("supr: 转发 CMD_IGH_FSA_SHUTDOWN -> master, mask=0x%02X\n", mask);
            // 确保至少4字节数据，与主站解析兼容
            char maskData[4] = {mask, 0, 0, 0};
            sendMasterMsg(CMD_IGH_FSA_SHUTDOWN, 0, 0, maskData, sizeof(maskData));
            break;
        }
            
        case CMD_IGH_FSA_SWITCH_ON: {
            uint8_t mask = extract_axis_mask(mUpperData, recSize);
            printf("supr: 转发 CMD_IGH_FSA_SWITCH_ON -> master, mask=0x%02X\n", mask);
            char maskData[4] = {mask, 0, 0, 0};
            sendMasterMsg(CMD_IGH_FSA_SWITCH_ON, 0, 0, maskData, sizeof(maskData));
            break;
        }
            
        case CMD_IGH_FSA_ENOP: {
            uint8_t mask = extract_axis_mask(mUpperData, recSize);
            printf("supr: 转发 CMD_IGH_FSA_ENOP -> master, mask=0x%02X\n", mask);
            char maskData[4] = {mask, 0, 0, 0};
            sendMasterMsg(CMD_IGH_FSA_ENOP, 0, 0, maskData, sizeof(maskData));
            break;
        }
            
        case CMD_IGH_FSA_HALT: {
            uint8_t mask = extract_axis_mask(mUpperData, recSize);
            printf("supr: 转发 CMD_IGH_FSA_HALT -> master, mask=0x%02X\n", mask);
            char maskData[4] = {mask, 0, 0, 0};
            sendMasterMsg(CMD_IGH_FSA_HALT, 0, 0, maskData, sizeof(maskData));
            break;
        }
            
        case CMD_IGH_CLEAR_ERROR: {
            uint8_t mask = extract_axis_mask(mUpperData, recSize);
            printf("supr: 转发 CMD_IGH_CLEAR_ERROR -> master, mask=0x%02X\n", mask);
            char maskData[4] = {mask, 0, 0, 0};
            sendMasterMsg(CMD_IGH_CLEAR_ERROR, 0, 0, maskData, sizeof(maskData));
            break;
        }
            
        case CMD_DESTROY_MASTER:
            printf("supr: 转发 CMD_DESTROY_MASTER -> master\n");
            sendMasterMsg(CMD_DESTROY_MASTER, 0, 0, NULL, 0);
            break;
            
        case CMD_MASTER_OP_KEYDOWN:
            sendMasterMsg(CMD_MASTER_OP_KEYDOWN, 0, 0, (const char *)mMsg, (int32_t)recSize);
            break;
            
        case CMD_HAND_VEL_MM: {
            size_t dataLen = (size_t)recSize > (ssize_t)sizeof(SUPR_R_MSG) ? (size_t)recSize - sizeof(SUPR_R_MSG) : 0;
            printf("supr: 转发 CMD_HAND_VEL_MM -> master, dataLen=%zu\n", dataLen);
            sendMasterMsg(CMD_HAND_VEL_MM, 0, 0, mMsg->mData, (int32_t)dataLen);
            break;
        }
            
        case CMD_HAND_RELA_POS_MM:
        case CMD_HAND_ABS_POS_M: {
            size_t dataLen = (size_t)recSize > (ssize_t)sizeof(SUPR_R_MSG) ? (size_t)recSize - sizeof(SUPR_R_MSG) : 0;
            sendMasterMsg(mMsg->mCmd, 0, 0, mMsg->mData, (int32_t)dataLen);
            break;
        }

        case CMD_PID_READ_PARAMS:
            printf("supr: 转发 CMD_PID_READ_PARAMS -> master\n");
            sendMasterMsg(CMD_PID_READ_PARAMS, 0, 0, mMsg->mData, 
                         (int32_t)(recSize - sizeof(SUPR_R_MSG)));
            break;
            
        case CMD_PID_WRITE_PARAMS:
            printf("supr: 转发 CMD_PID_WRITE_PARAMS -> master\n");
            sendMasterMsg(CMD_PID_WRITE_PARAMS, 0, 0, mMsg->mData, 
                         (int32_t)(recSize - sizeof(SUPR_R_MSG)));
            break;
            
        case CMD_PID_SAVE_TO_FLASH:
            printf("supr: 转发 CMD_PID_SAVE_TO_FLASH -> master\n");
            sendMasterMsg(CMD_PID_SAVE_TO_FLASH, 0, 0, mMsg->mData, 
                         (int32_t)(recSize - sizeof(SUPR_R_MSG)));
            break;

        default:
            printf("supr: 未知命令 cmd=%d, 转发给master\n", cmd);
            sendMasterMsg(cmd, 0, 0, NULL, 0);
            break;
    }
}