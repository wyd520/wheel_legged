//
// Created by yeyinong on 2023/3/29.
//

#include "hardwares.h"


bool joy_cmd_exit = false;
Hardwares* four_wheelleg = nullptr;
Eigen::Matrix<float, 10, 1> Xsense_msg = Eigen::Matrix<float, 10, 1>::Zero(); 
OD_Motor_Msg motor_msg_chassis[6];//挂载在slave0上
OD_Motor_Msg motor_msg_waist[6];//挂载在slave1上
OD_Motor_Msg motor_msg_upper[6];//挂载在slave2上

Hardwares::Hardwares(ros::NodeHandle &nh_) {
	// 初始化发布者
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/wheelleg/joint_states", 10);
	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/wheelleg/imu_msg", 10);
    joy_sub_ = nh_.subscribe("/joy", 10, &Hardwares::joyCallback, this);
    joint_forceCmd_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/wheelleg/joint_force_cmd", 10, &Hardwares::jointForceCmdCallback, this);

	Joint_Name.resize(13);
	Joint_Sign.resize(13);
	Joint_Offset.resize(13);
	Joint_K.resize(13);
	Joint_Current_K.resize(13);
    Joint_Force_target.resize(13);

	Joint_Name = {"Joint_LBhip", "Joint_LBknee", "Joint_LWheel", "Joint_RBhip", "Joint_RBknee", "Joint_RWheel", "Joint_Labad", "Joint_Lhip", "Joint_Lknee", "Joint_Rabad", "Joint_Rhip", "Joint_Rknee", "Joint_waist"};
	Joint_Sign <<  -1.0, 1.0, 1.0, -1.0, -1.0, 1.0 , -1.0 ;   //实际电机转轴与仿真中设定的正负关系
    Joint_Offset <<  0.0f, -0.32f , 0.32f , -0.32f , 0.32f , 0.0f , 0.0f ;//表示的是关节位置偏置，是关节处于零点的实际位置   用于关节抵至上限位的参数
    Joint_Current_K << 100.0f , 100.0f , 2048.0f/16.5f , 100.0f , 100.0f , 100.0f  , 2048.0f/16.5f , 1.0f , 1.0f , 1.0f , 1.0f , 1.0f , 1.0f, 1.0f ;//各个电机输入电流与编码器的系数
    Joint_K <<  2.1f , 2.1f , 0.32f , 2.1f , 2.1f , 2.1f , 0.32f;    //各个电机的力矩常数
    Joint_Force_target << 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f ;    //各个电机的力矩目标值初始化为0

}

void Hardwares::getJointStatesFromEtherCAT(){

    // 定义joint数组消息，并初始化为13个关节
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(13);
    joint_state.position.resize(13);
    joint_state.velocity.resize(13);
    joint_state.effort.resize(13);

	//处理solve1上的数据
	joint_state.name[0]=Joint_Name[0];
	joint_state.position[0]=Joint_Sign(0)*motor_msg_upper[0].angle_actual_rad+Joint_Offset(0);
	joint_state.velocity[0]=Joint_Sign(0)*motor_msg_upper[0].speed_actual_rad;
	joint_state.effort[0]=Joint_Sign(0)*motor_msg_upper[0].torque_actual_float;
	
	//处理solve0上的数据
    for (int i = 0; i < 6; i++)
    {
        joint_state.name[i+1]=Joint_Name[i+1];
        joint_state.position[i+1]=Joint_Sign(i+1)*motor_msg_chassis[i].angle_actual_rad+Joint_Offset(i+1);
        joint_state.velocity[i+1]=Joint_Sign(i+1)*motor_msg_chassis[i].speed_actual_rad;
        joint_state.effort[i+1]=Joint_Sign(i+1)*motor_msg_chassis[i].torque_actual_float;
    }
	
	// 发布JointState消息
	joint_pub_.publish(joint_state);

}

void Hardwares::jointForceCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!STOP_FLAG){
        Joint_Force_target(0) = limitRange(msg->data[0],-25,25); 
        Joint_Force_target(1) = limitRange(msg->data[1],-25,25);
        Joint_Force_target(2) = limitRange(msg->data[2],-25,25);
        Joint_Force_target(3) = limitRange(msg->data[3],-25,25);
        Joint_Force_target(4) = limitRange(msg->data[4],-25,25);
        Joint_Force_target(5) = limitRange(msg->data[5],-4,4);
        Joint_Force_target(6) = limitRange(msg->data[6],-4,4);
    }
    else{
        Joint_Force_target << 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f;    //各个电机的力矩目标值初始化为0
    }
}

void Hardwares::publishImuData() {

    sensor_msgs::Imu imu_msg_;	
	//填充Imu消息
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.header.frame_id = "base_link"; 

	imu_msg_.orientation.x = Xsense_msg[0];
    imu_msg_.orientation.y = Xsense_msg[1];
    imu_msg_.orientation.z = Xsense_msg[2];
    imu_msg_.orientation.w = Xsense_msg[3];

    imu_msg_.angular_velocity.x = Xsense_msg[4];
    imu_msg_.angular_velocity.y = Xsense_msg[5];
    imu_msg_.angular_velocity.z = Xsense_msg[6];

    imu_msg_.linear_acceleration.x = Xsense_msg[7];
    imu_msg_.linear_acceleration.y = Xsense_msg[8];
    imu_msg_.linear_acceleration.z = Xsense_msg[9];

    //发布Imu消息
    imu_pub_.publish(imu_msg_);

}

void Hardwares::test_printf(){
    // std::cout << "Xsense_msg:" << Xsense_msg.transpose() << std::endl;

}

void Hardwares::motor_control(EtherCAT_Msg *TxMessage_0, EtherCAT_Msg *TxMessage_1){

    std::lock_guard<std::mutex> lock(mutex_);
    // salve 0
    set_motor_cur_tor(TxMessage_0,1,1,Joint_Sign(1)*Joint_Force_target(1)*100,1,1); //这里乘以100是默认力矩模式 对照协议
    set_motor_cur_tor(TxMessage_0,2,2,Joint_Sign(2)*Joint_Force_target(2)*100,1, 1);
    set_motor_cur_tor(TxMessage_0,3,3,Joint_Sign(3)*Joint_Force_target(3)*100,1, 1); //这里乘以100是默认力矩模式 对照协议
    set_motor_cur_tor(TxMessage_0,4,4,Joint_Sign(4)*Joint_Force_target(4)*100,1, 1);
    MF9025_set_motor_cur_tor(TxMessage_0,5,5,Joint_Sign(5)*(Joint_Force_target(5)/Joint_K(5))*Joint_Current_K(5));
    MF9025_set_motor_cur_tor(TxMessage_0,6,6,Joint_Sign(6)*(Joint_Force_target(6)/Joint_K(6))*Joint_Current_K(6));
    // salve 1
    set_motor_cur_tor(TxMessage_1,1,1,Joint_Sign(0)*Joint_Force_target(0)*100,1, 1);
}

void Hardwares::set_Joint_zeroPotion(EtherCAT_Msg *TxMessage_0){
    for(int i=0;i<4;i++)
    {
        MotorSetZero(TxMessage_0,i+1, i+1);
    }   
}

void Hardwares::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg){

    if(joy_msg->buttons[1] == 1)
    {
        STOP_FLAG = !STOP_FLAG;
        if(STOP_FLAG) std::cout<<"stop!!!"<<std::endl;
        else  std::cout<<"run~~~"<<std::endl;
    }

}

void EtherCAT_Command_Prepare(EtherCAT_Msg *TxMessage){

    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
		//这里设置控制电机指令
		// MotorIDReading(&Tx_Message[slave]);
		// MotorCommModeReading(&Tx_Message[slave], 1);
		// MotorSetting(&Tx_Message[slave], 1, 3);
		// MotorSetting(&Tx_Message[slave], 2, 3);
		// MotorSetting(&Tx_Message[slave], 3, 3);
		// MotorSetting(&Tx_Message[slave], 4, 3);
		// MotorSetZero(&Tx_Message[slave],1, 1);
		// MotorSetZero(&Tx_Message[slave],2, 2);
		// MotorSetZero(&Tx_Message[slave],3, 3);
		// MotorSetZero(&Tx_Message[slave],4, 4);
		// MotorIDReset(&Tx_Message[1]);
		// MotorIDSetting(&Tx_Message[slave], 3, 4);
		// send_motor_ctrl_cmd(&Tx_Message[slave], 1,1, 0, 0, 0, 0, 0);
		// set_motor_position(&Tx_Message[slave], 1,2, 12, 23, 12, 2);
		// set_motor_speed(&Tx_Message[slave], i,i, 20, 500, 2);
		// set_motor_cur_tor(&Tx_Message[slave], 1,1, 100, 1, 1);
		// set_motor_cur_tor(&Tx_Message[slave], 2,2, -100, 1, 1);
		// set_motor_cur_tor(&Tx_Message[slave], 3,4, -100, 1, 1);
		// set_motor_cur_tor(&Tx_Message[slave], 4,4, 100, 1, 1);
		// get_motor_parameter(&Tx_Message[slave],i, i, 1);

		// set_motor_cur_tor(&Tx_Message[0], 1,1, -300, 1, 1);
		// set_motor_cur_tor(&Tx_Message[0], 2,2, -300, 1, 1);
		// set_motor_cur_tor(&Tx_Message[0], 3,3, 300, 1, 1);
		// set_motor_cur_tor(&Tx_Message[0], 4,4, 300, 1, 1);
		// MF9025_set_motor_cur_tor(&Tx_Message[0],5,5, 500);
		// MF9025_set_motor_cur_tor(&Tx_Message[0],6,6, -500);
		// set_motor_cur_tor(&Tx_Message[1], 1,1, 500, 1, 1);

		// MF9025_set_motor_cur_tor(&Tx_Message[0],5,5, 100.00);
		// MF9025_set_motor_cur_tor(&Tx_Message[0],6,6, 100.00);
		// MF9025_set_encoder_zero(&Tx_Message[slave],5,5);
		// MF9025_motor_stop(&Tx_Message[slave] , 5 , 5);
			



		/*--------------------------------以上为Test----------------------------------------*/
		four_wheelleg->motor_control(&Tx_Message[0], &Tx_Message[1]);   

		// four_wheelleg->set_Joint_zeroPotion(&Tx_Message[0]);     //给底盘四个电机调零
		// MotorSetZero(&Tx_Message[1],1,1);                        //单独给waist电机调零
            

        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest)
            *(EtherCAT_Msg *)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
    }


}	

void EtherCAT_Data_Explain(EtherCAT_Msg* RxMessage){
    const int chassis_slave=0;//底盘和轮子挂载的EtherCAT载板的从板ID
	const int upper_slave=1;//上肢挂载的EtherCAT载板的从板ID

    EtherCAT_Msg *slave_src0 = (EtherCAT_Msg *)(ec_slave[chassis_slave + 1].inputs);
	EtherCAT_Msg *slave_src1 = (EtherCAT_Msg *)(ec_slave[upper_slave + 1].inputs);//
    if (slave_src0)
        RxMessage[chassis_slave] = *(EtherCAT_Msg *)(ec_slave[chassis_slave + 1].inputs);
    if (slave_src1)
        RxMessage[upper_slave] = *(EtherCAT_Msg *)(ec_slave[upper_slave + 1].inputs);
		

    for(int i=0;i<6;i++){//底盘一块板子上最多挂了六个设备
        int can_id = RxMessage[chassis_slave].motor[i].id ;
		// printf("%x -----  \n",RxMessage[chassis_slave].can_ide);
        switch (can_id){
            case 326:{
                MF9025_getFeedback_motor_cur_tor(&RxMessage[chassis_slave],i,&motor_msg_chassis[5]);
                // std::cout<<"heyhey6"<<std::endl;
                break;
            }
            case 325:{
                MF9025_getFeedback_motor_cur_tor(&RxMessage[chassis_slave],i,&motor_msg_chassis[4]);
                // std::cout<<"heyhey5"<<std::endl;
                break;
            }
            case 4:{
                ckyf_Ethercat_rv_data_repack(&RxMessage[chassis_slave],i,comm_ack,&motor_msg_chassis[3]);
                // std::cout<<"heyhey4"<<std::endl;
                break;
            }
            case 3:{
                ckyf_Ethercat_rv_data_repack(&RxMessage[chassis_slave],i,comm_ack,&motor_msg_chassis[2]);
                // std::cout<<"heyhey3"<<std::endl;
                break;
            }
            case 2:{
                ckyf_Ethercat_rv_data_repack(&RxMessage[chassis_slave],i,comm_ack,&motor_msg_chassis[1]);
                // std::cout<<"heyhey2"<<std::endl;
                break;
            }
            case 1:{
                ckyf_Ethercat_rv_data_repack(&RxMessage[chassis_slave],i,comm_ack,&motor_msg_chassis[0]);
                // std::cout<<"heyhey1"<<std::endl;                
                break;
            }
            case 0x7FF:{
                ckyf_Ethercat_rv_data_repack_0x7FF(&RxMessage[chassis_slave],i);
                break;
            }
            default:{
                break;
            }
        }
    }

    for(int i=0;i<6;i++){//这块板子上挂了一个设备
        int can_id = RxMessage[upper_slave].motor[i].id ;
		// printf("%x -----  \n",RxMessage[chassis_slave].can_ide);
        switch (can_id){
            case 1:{
                ckyf_Ethercat_rv_data_repack(&RxMessage[upper_slave],i,comm_ack,&motor_msg_upper[0]);         
                break;
            }
            case 0x7FF:{
                ckyf_Ethercat_rv_data_repack_0x7FF(&RxMessage[upper_slave],i);
                break;
            }
            default:{
                break;
            }
        }
    }

}