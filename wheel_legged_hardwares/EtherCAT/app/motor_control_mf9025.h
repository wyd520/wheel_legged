/*
 * @Description:
 * @Author: hy_ren
 * @Date: 2024-08-15 
 * @LastEditTime: 2024-08-15 17:16:19
 */
#ifndef MOTOR_CONTROL_MF9025_H
#define MOTOR_CONTROL_MF9025_H

#include <inttypes.h>
#include <string.h>
#include "config.h"

#include "transmit.h"
#include "motor_control.h"

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define comm_ack 0x00
#define comm_auto 0x01


void MF9025_set_motor_cur_tor(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, int16_t cur_tor);
void MF9025_getFeedback_motor_cur_tor(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg);

void MF9025_motor_turn_on(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);
void MF9025_motor_turn_off(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);
void MF9025_motor_stop(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);

void MF9025_get_motor_state2(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);
void MF9025_getFeedback_motor_state2(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg);

void MF9025_set_encoder_zero(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);

// void MF9025_get_motor_acc(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);
// void MF9025_getFeedback_motor_acc(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg);

void CyberGear_set_encoder_zero(EtherCAT_Msg *TxMessage,uint8_t passage, uint32_t motor_id);
// void CyberGear_getFeedback(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg); //EtherCat 不支持

#endif
