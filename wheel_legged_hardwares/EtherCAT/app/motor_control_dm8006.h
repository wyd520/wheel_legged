/*
 * @Description:
 * @Author: hy_ren
 * @Date: 2025-03-25 
 * @LastEditTime: 2025-03-25 17:16:19
 */
#ifndef MOTOR_CONTROL_DM8006_H
#define MOTOR_CONTROL_DM8006_H

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



void DM8006_MIT_cmd(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float kp, float kd, float q, float dq, float tau);
void DM8006_getFeedback(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg);

void DM8006_motor_turn_on(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);
void DM8006_motor_turn_off(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);

void DM8006_set_encoder_zero(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);




#endif
