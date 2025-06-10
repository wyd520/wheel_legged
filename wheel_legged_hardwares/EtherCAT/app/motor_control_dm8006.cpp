

#ifdef __cplusplus 
extern "C" {
#endif  //__cplusplus
 #include "math_ops.h"
#ifdef __cplusplus
}
#endif  //__cplusplus


#include "motor_control_dm8006.h"

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -45.0f
#define SPD_MAX 45.0f
#define T_MIN -40.0f
#define T_MAX 40.0f
#define I_MIN -40.0f
#define I_MAX 40.0f


void DM8006_motor_turn_on(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0xFF;
    TxMessage->motor[passage-1].data[1] = 0xFF;
    TxMessage->motor[passage-1].data[2] = 0xFF;
    TxMessage->motor[passage-1].data[3] = 0xFF;
    TxMessage->motor[passage-1].data[4] = 0xFF;
    TxMessage->motor[passage-1].data[5] = 0xFF;
    TxMessage->motor[passage-1].data[6] = 0xFF;
    TxMessage->motor[passage-1].data[7] = 0xFC;
}

void DM8006_motor_turn_off(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0xFF;
    TxMessage->motor[passage-1].data[1] = 0xFF;
    TxMessage->motor[passage-1].data[2] = 0xFF;
    TxMessage->motor[passage-1].data[3] = 0xFF;
    TxMessage->motor[passage-1].data[4] = 0xFF;
    TxMessage->motor[passage-1].data[5] = 0xFF;
    TxMessage->motor[passage-1].data[6] = 0xFF;
    TxMessage->motor[passage-1].data[7] = 0xFD;
}

void DM8006_MIT_cmd(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float kp, float kd, float q, float dq, float tau)
{
    // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    };

    uint16_t q_uint = float_to_uint(q, POS_MIN, POS_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, SPD_MIN,SPD_MAX, 12);
    uint16_t kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, T_MIN, T_MAX, 12);

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = (q_uint >> 8) & 0xff;
    TxMessage->motor[passage-1].data[1] = q_uint & 0xff;
    TxMessage->motor[passage-1].data[2] = dq_uint >> 4;
    TxMessage->motor[passage-1].data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    TxMessage->motor[passage-1].data[4] = kp_uint & 0xff;
    TxMessage->motor[passage-1].data[5] = kd_uint >> 4;
    TxMessage->motor[passage-1].data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    TxMessage->motor[passage-1].data[7] = tau_uint & 0xff;


}


void DM8006_getFeedback(EtherCAT_Msg *RxMessage,int i, OD_Motor_Msg *motor_msg)
{
    uint16_t cur_uint = 0;
    uint16_t spd_uint = 0;
    uint16_t pos_uint = 0;

    // static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
    //     float span = xmax - xmin;
    //     float data_norm = float(x) / ((1 << bits) - 1);
    //     float data = data_norm * span + xmin;
    //     return data;
    // };
 
    // if (RxMessage->motor[i].dlc == 0) return;//数据长度为0，表示没有数据
    // else
    // {
    //     motor_msg->motor_id = RxMessage->motor[i].id; //DM8006反馈的canID对应电机自己的ID
    //     motor_msg->error=RxMessage->motor[i].data[0];

    //     pos_uint = (uint16_t(RxMessage->motor[i].data[1]) << 8) | RxMessage->motor[i].data[2];
    //     spd_uint = (uint16_t(RxMessage->motor[i].data[3]) << 4) | (RxMessage->motor[i].data[4] >> 4);
    //     tau_uint = (uint16_t(RxMessage->motor[i].data[4] & 0xf) << 8) | RxMessage->motor[i].data[5];
      
    //     motor_msg->angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
    //     motor_msg->speed_actual_rad = uint_to_float(spd_uint, SPD_MIN, SPD_MAX, 12);
    //     motor_msg->torque_actual_float=uint_to_float(tau_uint, T_MIN, T_MAX, 12);
    // }
}



void DM8006_set_encoder_zero(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    TxMessage->motor[passage-1].data[0] = 0xFF;
    TxMessage->motor[passage-1].data[1] = 0xFF;
    TxMessage->motor[passage-1].data[2] = 0xFF;
    TxMessage->motor[passage-1].data[3] = 0xFF;
    TxMessage->motor[passage-1].data[4] = 0xFF;
    TxMessage->motor[passage-1].data[5] = 0xFF;
    TxMessage->motor[passage-1].data[6] = 0xFF;
    TxMessage->motor[passage-1].data[7] = 0xFE;
}






