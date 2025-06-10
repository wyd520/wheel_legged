/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 11:17:58
 * @LastEditTime: 2022-11-13 17:16:18
 */
#ifndef TRANSMIT_H
#define TRANSMIT_H

#include "unistd.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "ethercat.h"
#include "sys/time.h"



extern EtherCAT_Msg Rx_Message[];
extern EtherCAT_Msg Tx_Message[];



void EtherCAT_Init(char *ifname);
void EtherCAT_Run();
void EtherCAT_Run_Once();//只用于跑一次程的程序
void EtherCAT_Command_Set();

void EtherCAT_Data_Get();
void EtherCAT_Command_Set();

void Thread_EtherCAT_Communicate();
#endif // PROJECT_RT_ETHERCAT_H