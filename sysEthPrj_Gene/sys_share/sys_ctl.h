/********************************************************************************
* Copyright (c) 2026 Zterch.
* sys_ctl.h - Port/config getters for General EtherCAT
********************************************************************************/
#ifndef SYS_CTL_H
#define SYS_CTL_H

#include <stdint.h>

#define IP_C_SIZE 16

char *get_const_param_value_str(char *key);
char *get_ip(char *key);
uint32_t get_port(char *key);
int get_mn_port(int index);
int get_mn_master_port(int index);
int get_local_ip(const char *eth_inf, char *ip);

#endif
