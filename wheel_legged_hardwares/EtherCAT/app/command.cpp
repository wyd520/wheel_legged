// //
// // Created by bismarck on 11/19/22.
// //

#include "command.h"
// #include "../../Robot/Robot.h"

unsigned test(const std::vector<std::string> &) {
    std::cout << "你是大笨蛋嘻嘻！\n"
        << "\tMotionInit\n";
    return 0;
}




std::vector<double> t;
std::vector<double> pos_vector;
std::vector<double> vel_vector;
std::vector<double> cur_vector;
//打印电机的图像
void Print_task(){



    int i=0;

    plt::ion();
    plt::figure();
    plt::figure();
    plt::figure();
    while(1){

        sleep(1);
        i++;
        // if(t.size()>1000){
        //    t.erase(t.begin());
        //    pos_vector.erase(pos_vector.begin());
        //    vel_vector.erase(vel_vector.begin());
        //    cur_vector.erase(cur_vector.begin());
        // }
        plt::figure(1);
        plt::clf(); // 清空当前图表
        plt::plot(t, pos_vector,{{"color", "c"}, {"marker", "."}, {"linestyle", "--"}}); // 绘制更新后的曲线
        plt::named_plot("Position",t, pos_vector);
        plt::title("Position");
        plt::legend();

        plt::figure(2);
        plt::clf(); // 清空当前图表
        plt::plot(t, vel_vector,{{"color", "r"}});
        plt::named_plot("Velocity",t, vel_vector);
        plt::title("Velocity");
        plt::legend();

        plt::figure(3);
        plt::clf(); // 清空当前图表
        plt::plot(t, cur_vector,{{"color", "r"}});
        plt::named_plot("Current",t, cur_vector);
        plt::title("Current");
        plt::legend();

        plt::pause(0.01);

    }

}



// const float left_hip_rad_init = 21.6406/180.*M_PI;
// const float left_knee_rad_init = -40/180.*M_PI;
// const float left_abduction_rad_init = -10./180.*M_PI;

// const float right_hip_rad_init = 21.6349/180.*M_PI;
// const float right_knee_rad_init = -40/180.*M_PI;
// const float right_abduction_rad_init = 10./180.*M_PI;

// float pos_init[6] = {left_abduction_rad_init,left_hip_rad_init, left_knee_rad_init, right_abduction_rad_init,  right_hip_rad_init, right_knee_rad_init};

// unsigned help(const std::vector<std::string> &) {
//     std::cout << "Available Commands:\n"
//         << "\tMotionInit\n"
//         << "\tMotorRestSet <SlaveId> <MotorId>\n"
//         << "\tMotorMotorSet <SlaveId> <MotorId>\n"
//         << "\tMotorZeroSet <SLaveId> <MotorId>\n"
//         << "\tMotorSpeedSet <SlaveId> <MotorId> <Speed>(0.) <Kd>(1)\n"
//         << "\tMotorPositionSet <SlaveId> <MotorId> <Position>(0) <Kp>(5) <Kd>(1)\n"
//         << "\tMotorTorqueSet <SlaveId> <MotorId> <Torque>(0)\n";
//     return 0;
// }

// unsigned motorRestSetCmd(const std::vector<std::string> & input) {
//     int slaveId;
//     int motor_id ;
//     switch (input.size()-1) {
//         case 2:
//             slaveId = std::stoi(input[1]);
//             motor_id = std::stoi(input[2]);
//             break;
//         default:
//             std::cout << "Command format error\n" <<
//                       "\tShould be \"MotorSpeedSet <SlaveId> <OldMotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\"\n";
//             return 1;
//     }

//     motorRestSet(slaveId, motor_id);
//     return 0;
// }

// unsigned motorMotorSetCmd(const std::vector<std::string> & input) {
//     int slaveId;
//     int motor_id ;
//     switch (input.size()-1) {
//         case 2:
//             slaveId = std::stoi(input[1]);
//             motor_id = std::stoi(input[2]);
//             break;
//         default:
//             std::cout << "Command format error\n" <<
//                       "\tShould be \"MotorSpeedSet <SlaveId> <OldMotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\"\n";
//             return 1;
//     }
//     motorMotorSet(slaveId, motor_id);
//     return 0;
// }


// unsigned motorZeroSetCmd(const std::vector<std::string> & input) {
//     int slaveId;
//     int motor_id ;
//     switch (input.size()-1) {
//         case 2:
//             slaveId = std::stoi(input[1]);
//             motor_id = std::stoi(input[2]);
//             break;
//         default:
//             std::cout << "Command format error\n" <<
//                       "\tShould be \"MotorSpeedSet <SlaveId> <OldMotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\"\n";
//             return 1;
//     }
//     motorZeroSet(slaveId, motor_id);
//     return 0;
// }

// unsigned motorSpeedSetCmd(const std::vector<std::string> & input) {
//     int slaveId;
//     int motor_id ;
//     float spd = 0;
//     float Kd = 1;
//     switch (input.size()-1) {
//         case 4:
//             Kd = std::stof(input[4]);
//         case 3:
//             spd = std::stof(input[3]);
//         case 2:
//             slaveId = std::stoi(input[1]);
//             motor_id = std::stoi(input[2]);
//             break;
//         default:
//             std::cout << "Command format error\n" <<
//                       "\tShould be \"MotorSpeedSet <SlaveId> <OldMotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\"\n";
//             return 1;
//     }
//     motorSpeedSet(slaveId, motor_id, spd, Kd);
//     return 0;
// }

// unsigned motorPositionSetCmd(const std::vector<std::string> & input) {
//     int slaveId;
//     int motor_id;
//     float pos = 0;
//     float Kp = 5;
//     float Kd = 1;
//     switch (input.size()-1) {
//         case 5:
//             Kd = std::stof(input[5]);
//         case 4:
//             Kp = std::stof(input[4]);
//         case 3:
//             pos = std::stof(input[3]);
//         case 2:
//             slaveId = std::stoi(input[1]);
//             motor_id = std::stoi(input[2]);
//             break;
//         default:
//             std::cout << "Command format error\n" <<
//                       "\tShould be \"MotorPositionSet <SlaveId> <OldMotorId> <Position>(0) <Speed>(50) <Current>(500) <AckStatus>(2)\"\n";
//             return 1;
//     }
//     motorPositionSet(slaveId, motor_id, pos, Kp, Kd);
//     return 0;
// }

// unsigned motorTorqueSetCmd(const std::vector<std::string> & input) {
//     int slaveId;
//     int motor_id;
//     float tor = 0;
//     switch (input.size()-1) {
//         case 3:
//             tor = std::stof(input[3]);
//         case 2:
//             slaveId = std::stoi(input[1]);
//             motor_id = std::stoi(input[2]);
//             break;
//         default:
//             std::cout << "Command format error\n" <<
//                       "\tShould be \"MotorPositionSet <SlaveId> <OldMotorId> <Position>(0) <Speed>(50) <Current>(500) <AckStatus>(2)\"\n";
//             return 1;
//     }
//     motorTorqueSet(motor_id, tor);
//     return 0;
// }

// unsigned motionInitCmd(const std::vector<std::string> & input){
//     // 电机上电
//     for (int i = 0; i < 6; ++i) {
//         motorMotorSet(0, i+1);
//     }
//     sleep(1);
//     cout << "Motor Set!" << endl;
//     for (int i = 0; i < 6; ++i) {
//         motorPositionSet(0, i+1, 0, 5, 1);
//     }
//     sleep(1);
//     cout << "Zero Position Arrived!" << endl;

//     for (int count = 0; count < 100; ++count) {
//         for (int i = 0; i < 6; ++i) {
//             motorPositionSet(0, i+1, pos_init[i]/100.*count, 15, 0.5);
//         }
//         usleep(20000);
//     }
//     cout << "Init Position Arrived!" << endl;

//     motion_init = true;
// }

// unsigned motionRunCmd(const std::vector<std::string> & input){
//     long dt;
//     struct timeval prev, now;
// //    sleep(5);
//     while (!joy_cmd_exit){
//         gettimeofday(&prev, NULL);

//         mutex_robbie.lock();
//         robbie->update(0.001);
//         mutex_robbie.unlock();

// //        robbie->controlMotor();

//         gettimeofday(&now, NULL);

//         dt = (now.tv_sec-prev.tv_sec)*1000000 + now.tv_usec - prev.tv_usec;
//         if (1000>dt)
//             usleep(1000 - dt);
//         else
//             cout << "time out:" << dt << endl;
//     }
//     cout << "robot run stop!" << endl;
//     return 0;
// }