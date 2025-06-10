

#ifdef __cplusplus 
extern "C" {
#endif  //__cplusplus
 #include "math_ops.h"
#ifdef __cplusplus
}
#endif  //__cplusplus


#include "motor_control_mf9025.h"

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f


void MF9025_motor_turn_on(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id+0x140;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0x88;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = 0x00;
    TxMessage->motor[passage-1].data[5] = 0x00;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;
}

void MF9025_motor_turn_off(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id+0x140;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0x80;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = 0x00;
    TxMessage->motor[passage-1].data[5] = 0x00;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;
}

void MF9025_motor_stop(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id+0x140;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0x81;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = 0x00;
    TxMessage->motor[passage-1].data[5] = 0x00;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;
}


void MF9025_set_motor_cur_tor(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, int16_t cur_tor)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id+0x140;
    TxMessage->motor[passage-1].dlc = 8;

    if (cur_tor > 2048)
        cur_tor = 2048;
    else if (cur_tor < -2048)
        cur_tor = -2048;
    
    TxMessage->motor[passage-1].data[0] = 0xA1;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = (uint8)(cur_tor &0xff);
    TxMessage->motor[passage-1].data[5] = (uint8)(cur_tor >>8) & 0xff;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;

}



/*发送转矩电流指令收到的返回，用此函数处理
  返回电机实际电流：-16.5A～16.5A
  返回电机温度：单位摄氏度
  返回电机实际速度：单位rad/s
  返回电机编码器位置：-pi～pi
*/
void MF9025_getFeedback_motor_cur_tor(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg)
{
    int8_t temp_int= 0;
    int16_t cur_int = 0;
    int16_t spd_int = 0;
    uint16_t pos_int = 0;

 
    if (RxMessage->motor[i].dlc == 0) return;//数据长度为0，表示没有数据
    else
    {
        motor_msg->motor_id = RxMessage->motor[i].id - 0x140; //MF9025反馈的canID加了0x140
        motor_msg->awk=RxMessage->motor[i].data[0];

        temp_int= RxMessage->motor[i].data[1] ;
        cur_int = RxMessage->motor[i].data[2]  | RxMessage->motor[i].data[3]<< 8;
        spd_int = RxMessage->motor[i].data[4]  | RxMessage->motor[i].data[5]<< 8;
        pos_int = RxMessage->motor[i].data[6]  | RxMessage->motor[i].data[7]<< 8;
      
        motor_msg->temperature = (uint8_t)(temp_int & 0x7f);  // 类型转换，忽略temp的最高位符号位，默认正数
        motor_msg->current_actual_float = (float)(cur_int)*(16.5/2048.0);  // 单位A
        motor_msg->speed_actual_rad = (float)(spd_int)*(PI/180.0); 
        motor_msg->angle_actual_int = pos_int;
        motor_msg->angle_actual_rad = uint_to_float(pos_int, -3.14159f, 3.14159f, 16);
        motor_msg->torque_actual_float=motor_msg->current_actual_float*0.32;  //for MF9025

    }
}

void MF9025_get_motor_state2(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id+0x140;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0x9C;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = 0x00;
    TxMessage->motor[passage-1].data[5] = 0x00;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;
}

void MF9025_getFeedback_motor_state2(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg)
{
    int8_t temp_int= 0;
    int16_t cur_int = 0;
    int16_t spd_int = 0;
    uint16_t pos_int = 0;

 
    if (RxMessage->motor[i].dlc == 0) return;//数据长度为0，表示没有数据
    else
    {
        motor_msg->motor_id = RxMessage->motor[i].id - 0x140; //MF9025反馈的canID加了0x140
        motor_msg->awk=RxMessage->motor[i].data[0];

        temp_int= RxMessage->motor[i].data[1] ;
        cur_int = RxMessage->motor[i].data[2]  | RxMessage->motor[i].data[3]<< 8;
        spd_int = RxMessage->motor[i].data[4]  | RxMessage->motor[i].data[5]<< 8;
        pos_int = RxMessage->motor[i].data[6]  | RxMessage->motor[i].data[7]<< 8;
      
        motor_msg->temperature = (uint8_t)(temp_int & 0x7f);  // 类型转换，忽略temp的最高位符号位，默认正数
        motor_msg->current_actual_float = (float)(cur_int)*(16.5/2048.0);  // 单位A
        motor_msg->speed_actual_rad = (float)(spd_int)*(PI/180.0); 
        motor_msg->angle_actual_int = pos_int;
        motor_msg->angle_actual_rad = uint_to_float(pos_int, -3.14159f, 3.14159f, 16);
        motor_msg->torque_actual_float=motor_msg->current_actual_float*0.32; //for MF9025

    }
}


void MF9025_set_encoder_zero(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id+0x140;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0x19;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = 0x00;
    TxMessage->motor[passage-1].data[5] = 0x00;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;
}

// void MF9025_get_motor_acc(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
// {
//     TxMessage->can_ide = 0;
//     TxMessage->motor[passage-1].rtr = 0;
//     TxMessage->motor[passage-1].id = motor_id+0x140;
//     TxMessage->motor[passage-1].dlc = 8;

//     TxMessage->motor[passage-1].data[0] = 0x33;
//     TxMessage->motor[passage-1].data[1] = 0x00;
//     TxMessage->motor[passage-1].data[2] = 0x00;
//     TxMessage->motor[passage-1].data[3] = 0x00;
//     TxMessage->motor[passage-1].data[4] = 0x00;
//     TxMessage->motor[passage-1].data[5] = 0x00;
//     TxMessage->motor[passage-1].data[6] = 0x00;
//     TxMessage->motor[passage-1].data[7] = 0x00;
// }

// void MF9025_getFeedback_motor_acc(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg)
// {
//     int32_t acc_int=0;

//     if (RxMessage->motor[i].dlc == 0) return;//数据长度为0，表示没有数据
//     else
//     {
//         motor_msg->motor_id = RxMessage->motor[i].id;
//         motor_msg->awk=RxMessage->motor[i].data[0];

//         acc_int = RxMessage->motor[i].data[4]  | RxMessage->motor[i].data[5]<< 8 | RxMessage->motor[i].data[6]<< 16 | RxMessage->motor[i].data[7]<< 24;
//         motor_msg->acceleration_rad = (acc_int*360.0/2147483648.0)*PI/180.0;  // rad/s2

//     }
// }

void CyberGear_set_encoder_zero(EtherCAT_Msg *TxMessage,uint8_t passage, uint32_t motor_id)
{
    TxMessage->can_ide = 1;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id | 0x06000000;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0x01;
    TxMessage->motor[passage-1].data[1] = 0x00;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x00;
    TxMessage->motor[passage-1].data[4] = 0x00;
    TxMessage->motor[passage-1].data[5] = 0x00;
    TxMessage->motor[passage-1].data[6] = 0x00;
    TxMessage->motor[passage-1].data[7] = 0x00;
}




