

#ifdef __cplusplus 
extern "C" {
#endif  //__cplusplus
 #include "math_ops.h"
#ifdef __cplusplus
}
#endif  //__cplusplus


#include "motor_control.h"

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
#define I_MIN -60.0f
#define I_MAX 60.0f





union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[6];




// MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd)
{
    // EtherCAT_Msg TxMessage;

    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].dlc = 4;

    if (cmd == 0)
        return;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = cmd;
}

// Add Can_passage for suit the different canID motor
void MotorSetZero(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id){
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].id = 0x7FF;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].dlc = 4;



    TxMessage->motor[passage-1].data[0] = motor_id >> 8;
    TxMessage->motor[passage-1].data[1] = motor_id & 0xff;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x03;
}

// Reset Motor ID
void MotorIDReset(EtherCAT_Msg *TxMessage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = 0x7F;
    TxMessage->motor[0].data[1] = 0x7F;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x05;
    TxMessage->motor[0].data[4] = 0x7F;
    TxMessage->motor[0].data[5] = 0x7F;
}
// set motor new ID
void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x04;
    TxMessage->motor[0].data[4] = motor_id_new >> 8;
    TxMessage->motor[0].data[5] = motor_id_new & 0xff;
}
// read motor communication mode
void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x81;
}
// read motor ID
void MotorIDReading(EtherCAT_Msg *TxMessage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = 0xFF;
    TxMessage->motor[0].data[1] = 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x82;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    if (tor > T_MAX)
        tor = T_MAX;
    else if (tor < T_MIN)
        tor = T_MIN;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

    TxMessage->motor[passage-1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[passage-1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[passage-1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage-1].data[3] = pos_int >> 8;
    TxMessage->motor[passage-1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage-1].data[5] = spd_int >> 4;
    TxMessage->motor[passage-1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage-1].data[7] = tor_int & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
pos:float
spd:0~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_position(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    if (ack_status > 3)
        return;

    rv_type_convert.to_float = pos;
    TxMessage->motor[passage-1].data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    TxMessage->motor[passage-1].data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    TxMessage->motor[passage-1].data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    TxMessage->motor[passage-1].data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    TxMessage->motor[passage-1].data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    TxMessage->motor[passage-1].data[5] = (spd & 0x3FC) >> 2;
    TxMessage->motor[passage-1].data[6] = (spd & 0x03) << 6 | (cur >> 6);
    TxMessage->motor[passage-1].data[7] = (cur & 0x3F) << 2 | ack_status;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
spd:-18000~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_speed(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 7;

    rv_type_convert.to_float = spd;
    TxMessage->motor[passage-1].data[0] = 0x40 | ack_status;
    TxMessage->motor[passage-1].data[1] = rv_type_convert.buf[3];
    TxMessage->motor[passage-1].data[2] = rv_type_convert.buf[2];
    TxMessage->motor[passage-1].data[3] = rv_type_convert.buf[1];
    TxMessage->motor[passage-1].data[4] = rv_type_convert.buf[0];
    TxMessage->motor[passage-1].data[5] = cur >> 8;
    TxMessage->motor[passage-1].data[6] = cur & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
cur:-3000~3000
ctrl_status:
    0:current control
    1:torque control
    2:variable damping brake control(also call full brake)
    3:dynamic brake control
    4:regenerative brake control
    5:NON
    6:NON
    7:NON
ack_status:0~3
*/
void set_motor_cur_tor(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 3;

    if (ack_status > 3)
        return;
    if (ctrl_status > 7)
        return;
    if (ctrl_status) // enter torque control mode or brake mode
    {
        if (cur_tor > 3000)
            cur_tor = 3000;
        else if (cur_tor < -3000)
            cur_tor = -3000;
    }
    else
    {
        if (cur_tor > 2000)
            cur_tor = 2000;
        else if (cur_tor < -2000)
            cur_tor = -2000;
    }

    TxMessage->motor[passage-1].data[0] = 0x60 | ctrl_status << 2 | ack_status;
    TxMessage->motor[passage-1].data[1] = cur_tor >> 8;
    TxMessage->motor[passage-1].data[2] = cur_tor & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
acc:0~2000
ack_status:0~3
*/
void set_motor_acceleration(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t acc, uint8_t ack_status)
{


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 4;

    if (ack_status > 2)
        return;
    if (acc > 2000)
        acc = 2000;

    TxMessage->motor[passage-1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage-1].data[1] = 0x01;
    TxMessage->motor[passage-1].data[2] = acc >> 8;
    TxMessage->motor[passage-1].data[3] = acc & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
linkage:0~10000
speedKI:0~10000
ack_status:0/1
*/
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 6;

    if (ack_status > 2)
        return;
    if (linkage > 10000)
        linkage = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    TxMessage->motor[passage-1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage-1].data[1] = 0x02;
    TxMessage->motor[passage-1].data[2] = linkage >> 8;
    TxMessage->motor[passage-1].data[3] = linkage & 0xff;
    TxMessage->motor[passage-1].data[4] = speedKI >> 8;
    TxMessage->motor[passage-1].data[5] = speedKI & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
fdbKP:0~10000
fbdKD:0~10000
ack_status:0/1
*/
void set_motor_feedbackKP_KD(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 6;

    if (ack_status > 2)
        return;
    if (fdbKP > 10000)
        fdbKP = 10000;
    if (fdbKD > 10000)
        fdbKD = 10000;

    TxMessage->motor[passage-1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage-1].data[1] = 0x03;
    TxMessage->motor[passage-1].data[2] = fdbKP >> 8;
    TxMessage->motor[passage-1].data[3] = fdbKP & 0xff;
    TxMessage->motor[passage-1].data[4] = fdbKD >> 8;
    TxMessage->motor[passage-1].data[5] = fdbKD & 0xff;
}
// This function use in ask communication mode.
/*void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, uint16_t motor_id, float kp, float kd, float pos, float spd, float cur);
void set_motor_position(EtherCAT_Msg *TxMessage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
void set_motor_speed(EtherCAT_Msg *TxMessage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
void set_motor_cur_tor(EtherCAT_Msg *TxMessage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);
void set_motor_acceleration(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t acc, uint8_t ack_status);
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status);
void set_motor_feedbackKP(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t fdbKP, uint8_t ack_status);
void get_motor_parameter(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t param_cmd);
motor_id:1~0x7FE
param_cmd:1~9
*/
void get_motor_parameter(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint8_t param_cmd)
{


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 2;

    TxMessage->motor[passage-1].data[0] = 0xE0;
    TxMessage->motor[passage-1].data[1] = param_cmd;
}


void Rv_Message_Print(uint8_t ack_status)
{
    if (ack_status == 0)
    {
        if (motor_comm_fbd.motor_fbd == 0x01)
        {
            printf("自动模式.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x02)
        {
            printf("问答模式.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x03)
        {
            printf("零点设置成功.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x04)
        {
            printf("新设置的Id为: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x05)
        {
            printf("重置Id成功.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x06)
        {
            printf("当前电机Id: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x80)
        {
            printf("查询失败.\n");
        }
    }
    else
    {
        for (int i = 0; i < 6; ++i)
        {
            if (rv_motor_msg[i].motor_id == 0)
            {
                continue;
            }
            switch (ack_status)
            {
            case 1:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_rad: %f\n", rv_motor_msg[i].angle_actual_rad);
                printf("speed_actual_rad: %f\n", rv_motor_msg[i].speed_actual_rad);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                break;
            case 2:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                break;
            case 3:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                break;
            case 4:
                if (motor_comm_fbd.motor_fbd == 0)
                {
                    printf("配置成功.\n");
                }
                else
                {
                    printf("配置失败.\n");
                }
                break;
            case 5:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                if (motor_comm_fbd.INS_code == 1)
                {
                    printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 2)
                {
                    printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 3)
                {
                    printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 4)
                {
                    printf("power: %f\n", rv_motor_msg[i].power);
                }
                else if (motor_comm_fbd.INS_code == 5)
                {
                    printf("acceleration: %d\n", rv_motor_msg[i].acceleration);
                }
                else if (motor_comm_fbd.INS_code == 6)
                {
                    printf("linkage_KP: %d\n", rv_motor_msg[i].linkage_KP);
                }
                else if (motor_comm_fbd.INS_code == 7)
                {
                    printf("speed_KI: %d\n", rv_motor_msg[i].speed_KI);
                }
                else if (motor_comm_fbd.INS_code == 8)
                {
                    printf("feedback_KP: %d\n", rv_motor_msg[i].feedback_KP);
                }
                else if (motor_comm_fbd.INS_code == 9)
                {
                    printf("feedback_KD: %d\n", rv_motor_msg[i].feedback_KD);
                }
                break;
            case 6:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_int: %d\n", rv_motor_msg[i].angle_actual_int);
                printf("speed_actual_int: %d\n", rv_motor_msg[i].speed_actual_int);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                break;
            default:
                break;
            }
        }
    }
}

uint16_t motor_id_check = 0;
void RV_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id)//该函数为一个从站上六个电机的共用函数
{
    printf("slave %d msg:\n", slave_id);
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    for (int i = 0; i < 6; ++i)
    {
        if (RxMessage->motor[i].dlc == 0)
            continue;
        printf("motor%d message:\n", i);
        if (RxMessage->motor[i].id == 0x7FF)
        {
            if (RxMessage->motor[i].data[2] != 0x01) // determine whether it is a motor feedback instruction
                return;                              // it is not a motor feedback instruction
            if ((RxMessage->motor[i].data[0] == 0xff) && (RxMessage->motor[i].data[1] == 0xFF))
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
                motor_comm_fbd.motor_fbd = 0x06;
            }
            else if ((RxMessage->motor[i].data[0] == 0x80) && (RxMessage->motor[i].data[1] == 0x80)) // inquire failed
            {
                motor_comm_fbd.motor_id = 0;
                motor_comm_fbd.motor_fbd = 0x80;
            }
            else if ((RxMessage->motor[i].data[0] == 0x7F) && (RxMessage->motor[i].data[1] == 0x7F)) // reset ID succeed
            {
                motor_comm_fbd.motor_id = 1;
                motor_comm_fbd.motor_fbd = 0x05;
            }
            else
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            Rv_Message_Print(0);
        }
        else if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) // Response mode
        {
            ack_status = RxMessage->motor[i].data[0] >> 5;
            motor_id_t = RxMessage->motor[i].id - 1;
            motor_id_check = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].motor_id = motor_id_check;
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[0] & 0x1F;
            if (ack_status == 1) // response frame 1
            {
                pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
                spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
                cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];

                rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12);
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[6] - 50) / 2;
            }
            else if (ack_status == 2) // response frame 2
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 3) // response frame 3
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 4) // response frame 4
            {
                if (RxMessage->motor[i].dlc != 3)
                    return;
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
            }
            else if (ack_status == 5) // response frame 5
            {
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                if (motor_comm_fbd.INS_code == 1 && RxMessage->motor[i].dlc == 6) // get position
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 2 && RxMessage->motor[i].dlc == 6) // get speed
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 3 && RxMessage->motor[i].dlc == 6) // get current
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 4 && RxMessage->motor[i].dlc == 6) // get power
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 5 && RxMessage->motor[i].dlc == 4) // get acceleration
                {
                    rv_motor_msg[motor_id_t].acceleration = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 6 && RxMessage->motor[i].dlc == 4) // get linkage_KP
                {
                    rv_motor_msg[motor_id_t].linkage_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 7 && RxMessage->motor[i].dlc == 4) // get speed_KI
                {
                    rv_motor_msg[motor_id_t].speed_KI = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 8 && RxMessage->motor[i].dlc == 4) // get feedback_KP
                {
                    rv_motor_msg[motor_id_t].feedback_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 9 && RxMessage->motor[i].dlc == 4) // get feedback_KD
                {
                    rv_motor_msg[motor_id_t].feedback_KD = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
            }

            Rv_Message_Print(ack_status);
        }
        else if (comm_mode == 0x01 && RxMessage->motor[i].dlc != 0) // automatic feedback mode
        {
            motor_id_t = RxMessage->motor[i].id - 0x205;
            rv_motor_msg[motor_id_t].motor_id = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1]);
            rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3]);
            rv_motor_msg[motor_id_t].current_actual_int = (RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5]);
            rv_motor_msg[motor_id_t].temperature = RxMessage->motor[i].data[6];
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[7];
            Rv_Message_Print(6);
        }
    }
}


//该函数由前期判定是由前期ID来判定缓存里的数据是属于哪个电机的，在交给这个电机来判定处理，
void ckyf_Ethercat_rv_data_repack(EtherCAT_Msg *RxMessage,int i, uint8_t comm_mode, OD_Motor_Msg *motor_msg){
    
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;

    if (RxMessage->motor[i].dlc == 0)//数据长度为0，表示没有数据
        return;
        

    if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) // Response mode
    {
        ack_status = RxMessage->motor[i].data[0] >> 5;
        motor_id_t = RxMessage->motor[i].id - 1;
        motor_id_check = RxMessage->motor[i].id;
        motor_msg->motor_id = motor_id_check;
        motor_msg->error = RxMessage->motor[i].data[0] & 0x1F;
        if (ack_status == 1) // response frame 1
        {
            pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
            spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
            cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];

            motor_msg->angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
            motor_msg->speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
            motor_msg->current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12);
            motor_msg->temperature = (RxMessage->motor[i].data[6] - 50) / 2;
            motor_msg->torque_actual_float=motor_msg->current_actual_float*2.1; //for 8112
        }
        else if (ack_status == 2) // response frame 2
        {
            rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
            rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
            rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
            rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
            motor_msg->angle_actual_float = rv_type_convert.to_float;
            motor_msg->current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
            motor_msg->temperature = (RxMessage->motor[i].data[7] - 50) / 2;
            motor_msg->current_actual_float = motor_msg->current_actual_int / 100.0f;
            motor_msg->torque_actual_float=motor_msg->current_actual_float*2.1; //for 8112
        }
        else if (ack_status == 3) // response frame 3
        {
            rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
            rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
            rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
            rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
            motor_msg->speed_actual_float = rv_type_convert.to_float;
            motor_msg->current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
            motor_msg->temperature = (RxMessage->motor[i].data[7] - 50) / 2;
            motor_msg->current_actual_float = motor_msg->current_actual_int / 100.0f;
            motor_msg->torque_actual_float=motor_msg->current_actual_float*2.1; //for 8112
        }
        else if (ack_status == 4) // response frame 4
        {
            if (RxMessage->motor[i].dlc != 3)
                return;
            motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
            motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
        }
        else if (ack_status == 5) // response frame 5
        {
            motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
            if (motor_comm_fbd.INS_code == 1 && RxMessage->motor[i].dlc == 6) // get position
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                motor_msg->angle_actual_float = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 2 && RxMessage->motor[i].dlc == 6) // get speed
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                motor_msg->speed_actual_float = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 3 && RxMessage->motor[i].dlc == 6) // get current
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                motor_msg->current_actual_float = rv_type_convert.to_float;
                motor_msg->torque_actual_float=motor_msg->current_actual_float*2.1; //for 8112
            }
            else if (motor_comm_fbd.INS_code == 4 && RxMessage->motor[i].dlc == 6) // get power
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                motor_msg->power = rv_type_convert.to_float;
            }
            else if (motor_comm_fbd.INS_code == 5 && RxMessage->motor[i].dlc == 4) // get acceleration
            {
                motor_msg->acceleration = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
            }
            else if (motor_comm_fbd.INS_code == 6 && RxMessage->motor[i].dlc == 4) // get linkage_KP
            {
                motor_msg->linkage_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
            }
            else if (motor_comm_fbd.INS_code == 7 && RxMessage->motor[i].dlc == 4) // get speed_KI
            {
                motor_msg->speed_KI = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
            }
            else if (motor_comm_fbd.INS_code == 8 && RxMessage->motor[i].dlc == 4) // get feedback_KP
            {
                motor_msg->feedback_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
            }
            else if (motor_comm_fbd.INS_code == 9 && RxMessage->motor[i].dlc == 4) // get feedback_KD
            {
                motor_msg->feedback_KD = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
            }
        }

        Rv_Message_Print(ack_status);
    }
    else if (comm_mode == 0x01 && RxMessage->motor[i].dlc != 0) // automatic feedback mode
    {
        motor_id_t = RxMessage->motor[i].id - 0x205;
        motor_msg->motor_id = RxMessage->motor[i].id;
        motor_msg->angle_actual_int = (uint16_t)(RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1]);
        motor_msg->speed_actual_int = (int16_t)(RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3]);
        motor_msg->current_actual_int = (RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5]);
        motor_msg->temperature = RxMessage->motor[i].data[6];
        motor_msg->error = RxMessage->motor[i].data[7];
        Rv_Message_Print(6);
    }
    
}


//该函数由前期判定是由前期ID来判定缓存里的数据是属于哪个电机的，在交给这个电机来判定处理，该函数主要用来判断返回帧ID是0x7FF部分
void ckyf_Ethercat_rv_data_repack_0x7FF(EtherCAT_Msg *RxMessage,int i){
    
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;

    if (RxMessage->motor[i].dlc == 0)//数据长度为0，表示没有数据
        return;
        
    if (RxMessage->motor[i].id == 0x7FF)
    {
        if (RxMessage->motor[i].data[2] != 0x01) // determine whether it is a motor feedback instruction
            return;                              // it is not a motor feedback instruction
        if ((RxMessage->motor[i].data[0] == 0xff) && (RxMessage->motor[i].data[1] == 0xFF))
        {
            motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
            motor_comm_fbd.motor_fbd = 0x06;
        }
        else if ((RxMessage->motor[i].data[0] == 0x80) && (RxMessage->motor[i].data[1] == 0x80)) // inquire failed
        {
            motor_comm_fbd.motor_id = 0;
            motor_comm_fbd.motor_fbd = 0x80;
        }
        else if ((RxMessage->motor[i].data[0] == 0x7F) && (RxMessage->motor[i].data[1] == 0x7F)) // reset ID succeed
        {
            motor_comm_fbd.motor_id = 1;
            motor_comm_fbd.motor_fbd = 0x05;
        }
        else
        {
            motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
            motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
        }
        Rv_Message_Print(0);
    }
    
    
}







