/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_ctl.c - Port/config (const table) for General EtherCAT
********************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "sys_ctl.h"
#include "sys_share_conf.h"

#define MAX_LINE 64

typedef struct {
    char key[64];
    float value;
    char description[256];
} param_dic;

static param_dic const_params[] = {
    {"SUPR_PORTS", 0, "30001"},
    {"LEADER_PORTS", 0, "30002"},
    {"MASTER0_PORTS", 0, "30011"},
    {"LISTENER_LOCAL_PORT", 0, "30021"},
    {"LISTENER_TO_UPPER_PORT", 0, "33333"},
    {"", 0, ""}
};

char *get_const_param_value_str(char *key)
{
    int pI = 0;
    while (strlen(const_params[pI].key) > 0) {
        if (strcmp(key, const_params[pI].key) == 0)
            return const_params[pI].description;
        pI++;
    }
    return NULL;
}

char *get_ip(char *key)
{
    return get_const_param_value_str(key);
}

uint32_t get_port(char *key)
{
    char *p = get_const_param_value_str(key);
    return p ? (uint32_t)strtol(p, NULL, 10) : 0;
}

int get_mn_port(int index)
{
    if (index == MN_ID_SUPR)
        return (int)strtol(get_const_param_value_str("SUPR_PORTS"), NULL, 10);
    if (index == MN_ID_LEADER)
        return (int)strtol(get_const_param_value_str("LEADER_PORTS"), NULL, 10);
    return 0;
}

int get_mn_master_port(int index)
{
    if (index == 0)
        return (int)strtol(get_const_param_value_str("MASTER0_PORTS"), NULL, 10);
    return 0;
}

int get_local_ip(const char *eth_inf, char *ip)
{
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, eth_inf, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sd, SIOCGIFADDR, &ifr) < 0) {
        printf("ioctl error: %s\n", strerror(errno));
        close(sd);
        snprintf(ip, IP_C_SIZE, "127.0.0.1");
        return -1;
    }
    struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
    snprintf(ip, IP_C_SIZE, "%s", inet_ntoa(sin->sin_addr));
    close(sd);
    return 0;
}
