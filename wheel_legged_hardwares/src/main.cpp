extern "C"
{
#include <stdio.h>
#include "config.h"
#include "ethercat.h"
#include "unistd.h"
}
#include "thread"
#include "cmath"
#include <vector>
#include "mti_receive_data.h"
#include "hardwares.h"



int main(int argc,char** argv){

    ros::init(argc, argv, "wheelleg_hardwares");
    ros::NodeHandle node_handle_;
    ros::Rate loop_rate(1000); // 1000 Hz
    four_wheelleg = new Hardwares(node_handle_);
    if(ros::ok()) ros::Duration(1).sleep(); // 等待接口初始化完成


    // ROS环境中需要用sudo su进入管理员权限，然后source ./devel/setup.bash来启动
    // 注意:不要在管理员权限界面编译代码，容易他妈的出问题！！！
    // 记得设置imu串口低延迟模式 "sudo setserial /dev/ttyUSB0 low_latency" 每次IMU掉电重上电就需要设置一下
    std::thread th_imu = thread(receiveImu, ref(Xsense_msg));
    if(ros::ok()) ros::Duration(3).sleep(); // 等待IMU初始化完成

    std::thread ThreadEtherCAT(Thread_EtherCAT_Communicate); 
    if(ros::ok()) ros::Duration(10).sleep(); // 等待EtherCAT初始化 

    while(ros::ok())
    {
        ros::spinOnce(); // 处理回调函数

        // four_wheelleg->test_printf();
        four_wheelleg->publishImuData();
        four_wheelleg->getJointStatesFromEtherCAT();
      
        loop_rate.sleep();  // 控制循环频率
    }

    th_imu.join();
    ThreadEtherCAT.join();

	return 0;
}
