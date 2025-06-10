/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:45:09
 * @LastEditTime: 2022-11-13 17:16:19
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <inttypes.h>
#include <string.h>
#include "config.h"

#include "transmit.h"

#define NO_MOTOR 0
#define REST_MODE 1
#define MOTOR_MODE 2
#define RESET_POSITION 3
#define RUN_MODE 4

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

typedef struct
{
    uint16_t motor_id;
    uint8_t INS_code;  // instruction code.
    uint8_t motor_fbd; // motor CAN communication feedback.
} MotorCommFbd;

typedef struct
{
    int comm_type = 0;
    uint16_t motor_id = 0;
    uint8_t error = 0;
    float angle_actual_float = 0;
    float speed_actual_float = 0;
    float torque_actual_float = 0;

    float angle_desired_float = 0;
    float speed_desired_float = 0;
    float torque_desired_float = 0;

    float kp_desired = 0;
    float kd_desired = 0;
    uint16_t angle_actual_int; //used
    uint16_t angle_desired_int;
    int16_t speed_actual_int;
    int16_t speed_desired_int;
    int16_t current_actual_int;
    int16_t current_desired_int;
    float speed_actual_rad;  //used
    float speed_desired_rad;
    float angle_actual_rad;  //used
    float angle_desired_rad;
    uint8_t temperature;  //used
    float current_actual_float; //used
    float current_desired_float;
    float power;
    uint16_t acceleration;
    uint16_t linkage_KP;
    uint16_t speed_KI;
    uint16_t feedback_KP;
    uint16_t feedback_KD;

    float acceleration_rad; // newly created for datatype adaptation
    uint16_t encoderOffset;   
    uint8_t awk; 
} OD_Motor_Msg;

extern OD_Motor_Msg rv_motor_msg[6];
extern uint16_t motor_id_check;

void MotorIDReset(EtherCAT_Msg *TxMessage);
void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new);
void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd);
void MotorSetZero(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id);
void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id);
void MotorIDReading(EtherCAT_Msg *TxMessage);

void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float cur);
void set_motor_position(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
void set_motor_speed(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
void set_motor_cur_tor(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);
void set_motor_acceleration(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t acc, uint8_t ack_status);
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status);
void set_motor_feedbackKP(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t fdbKP, uint8_t ack_status);
void get_motor_parameter(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint8_t param_cmd);

void RV_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id);
void ckyf_Ethercat_rv_data_repack(EtherCAT_Msg *RxMessage,int i, uint8_t comm_mode, OD_Motor_Msg *motor_msg);
void ckyf_Ethercat_rv_data_repack_0x7FF(EtherCAT_Msg *RxMessage,int i);

int float_to_uint(float x, float x_min, float x_max, int bits);

#endif
