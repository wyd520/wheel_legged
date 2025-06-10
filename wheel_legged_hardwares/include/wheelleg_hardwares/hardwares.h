
#ifndef _HARDWARES_H
#define _HARDWARES_H

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include </usr/local/include/eigen3/Eigen/Core>
#include </usr/local/include/eigen3/Eigen/LU>
#include "vector"
#include <mutex>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#include "../EtherCAT/app/motor_control.h"
#include "../EtherCAT/app/motor_control_mf9025.h"
#include <wheelleg_controllers/Legkinematics.h>
#include <wheelleg_controllers/tools.h>


class Hardwares;
extern Hardwares* four_wheelleg;
extern bool joy_cmd_exit;
extern Eigen::Matrix<float, 10, 1> Xsense_msg;
extern OD_Motor_Msg motor_msg_chassis[6];
extern OD_Motor_Msg motor_msg_upper[6];


class Hardwares{
public:
    Hardwares(ros::NodeHandle &nh_);
    void getJointStatesFromEtherCAT();
    void publishImuData();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void jointForceCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void motor_control(EtherCAT_Msg *TxMessage_0, EtherCAT_Msg *TxMessage_1);
    void set_Joint_zeroPotion(EtherCAT_Msg *TxMessage_0);
    void test_printf();


private:
    ros::Publisher joint_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber joint_forceCmd_sub_;


    bool STOP_FLAG = true; //控制电机启停的标志位
    std::mutex mutex_; //互斥锁
    
    std::vector<std::string> Joint_Name; //描述实际电机的名称
    Eigen::VectorXf Joint_Sign; //描述实际电机的正方向与模型的一致性
    Eigen::VectorXf Joint_Offset; //描述实际电机的零点位置相对于模型的偏置
    Eigen::VectorXf Joint_K; //描述实际电机的电流和力矩的转换系数 扭矩=电流A*扭矩常数
    Eigen::VectorXf Joint_Current_K; //描述实际电机电流值与输入编码值的系数
    Eigen::VectorXf Joint_Force_target; //描述实际电机速度值与输入编码值的系数

};

#endif 
