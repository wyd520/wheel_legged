// ros_to_ocs2_bridge.cpp
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_core/Types.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_state.h>
#include <ocs2_msgs/mpc_input.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <mutex>
#include <wheelleg_controllers/Legkinematics.h>
#include <wheelleg_controllers/tools.h>
#include <wheelleg_controllers/PID.h>





class Ros2Ocs2Bridge {
public:
    Ros2Ocs2Bridge(ros::NodeHandle &nh_) {
        // 初始化发布者和订阅者

        imu_sub_ = nh_.subscribe("/wheelleg/imu_msg", 1, &Ros2Ocs2Bridge::imuCallback, this);
        joint_sub_ = nh_.subscribe("/wheelleg/joint_states", 1, &Ros2Ocs2Bridge::jointStateCallback, this);
        mpcInput_sub_ = nh_.subscribe("/wheelleg_mpc_input", 1, &Ros2Ocs2Bridge::inputCallback, this);
        nqnv_pub_ = nh_.advertise<ocs2_msgs::mpc_observation>("/wheelleg_mpc_observation", 1);

        // 初始化关节力矩发布者
        joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/wheelleg/joint_force_cmd", 1);
        
        // ... 其他关节类似初始化
        q_.resize(7);
        dq_.resize(7);
        imuData.resize(9);
        stateObserved.resize(12);
        input_.value.resize(5,0.0);
        stateObserved << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        delta.resize(4);
        phi1_4_l.resize(2);
        phi1_4_r.resize(2);
        dphi1_4_l.resize(2);
        dphi1_4_r.resize(2);
        left_legPose.resize(4);
        right_legPose.resize(4);
        delta << -0.09, 0.1, -0.05, 0.06;

        PID_Init(&leftlengthPID.outer, 30, 0, 1, 0, 300);  //腿长PID  //25 1
        PID_Init(&leftlengthPID.inner, 3000, 0, 5, 0, 300);  
        PID_SetErrLpfRatio(&leftlengthPID.outer, 0.01);	

        PID_Init(&rightlengthPID.outer, 30, 0, 1, 0, 300);
        PID_Init(&rightlengthPID.inner, 3000, 0, 5, 0, 300); 
        PID_SetErrLpfRatio(&rightlengthPID.outer, 0.01);

        PID_Init(&rollPID.outer, 500, 0, 1, 0, 150);      //翻滚PID
        // PID_Init(&rollPID.inner, 3000, 0, 1, 0, 150);
        PID_SetErrLpfRatio(&rollPID.outer, 0.005);

        PID_Init(&thetaPID.outer, 10, 0, 0, 0, 50);          //腿部协调PID
        PID_SetErrLpfRatio(&thetaPID.outer, 0.25);
    }

// private:
    ros::Publisher joint_pub_;
    ros::Subscriber imu_sub_, joint_sub_, mpcInput_sub_;
    ros::Publisher nqnv_pub_;
    
    ocs2_msgs::mpc_observation observation_;
    ocs2_msgs::mpc_input input_;

    ros::Time last_update_time_; // 用于记录上次更新时间

    ocs2::vector_t imuData;
    ocs2::vector_t q_, dq_; // joint states
    ocs2::vector_t phi1_4_l,phi1_4_r,dphi1_4_l,dphi1_4_r; //五连杆平面角度和角速度
    ocs2::vector_t delta; // joint state offset left_leg1 left_leg4 right_leg1 right_leg4
    ocs2::vector_t stateObserved; //[s  ds  yaw  dyaw  thetaL_l  dthetaL_l  thetaL_r  dthetaL_r  phi  dphi]
    ocs2::vector_t left_legPose; //实时计算的腿长和角度 [legLenth, legAngle,dot_legLenth, dot_legAngle]
    ocs2::vector_t right_legPose;

    CascadePID leftlengthPID,rightlengthPID;
    CascadePID rollPID , thetaPID;
    double legLength_target = 0.20;
    double roll_target = 0.0;//0.0125;
    double yawAngle_LocalOffset = 0.0;
    bool verbose1 = false;
    bool verbose2 = true;
    int print_count = 0;
    const int print_fq = 1000;
    bool hardwareInitialized_flag = false;
    std::mutex mtx_;

    void isInitialized(){
        hardwareInitialized_flag = checkQRange();
    }

    bool checkQRange() {
        // 检查关节位置是否在合理范围内 这里给的是50
        for (size_t i = 0; i < dq_.size(); ++i) {
            if (dq_[i] < -50.0 || dq_[i] > 50.0) {
                return false;
            }
        }
        return true;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        std::vector<std::string> joint_names = {
            "upper_Joint", "left_leg1_Joint", "left_leg4_Joint", "right_leg1_Joint",
            "right_leg4_Joint", "left_wheel_Joint", "right_wheel_Joint"
        };
        
        // 处理关节位置和速度
        for (size_t i = 0; i < joint_names.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
            if (it != msg->name.end()) {
                int idx = std::distance(msg->name.begin(), it);
                q_[i] = msg->position[idx];
                dq_[i] = msg->velocity[idx];
            }
        }
        // 修正模型腿部关节初始状态角度
        q_[1] = q_[1] + delta[0]; //left_leg1
        q_[2] = q_[2] + delta[1]; //left_leg4
        q_[3] = q_[3] + delta[2]; //right_leg1
        q_[4] = q_[4] + delta[3]; //right_leg4
    }

    void jointAngle2fiveBarAngle() {
        std::lock_guard<std::mutex> lock(mtx_);
        // 修正五连杆角度值 q&dq方向反过来
        phi1_4_l[0] = -1.0 * q_[1] +  M_PI; //left_leg1
        phi1_4_l[1] = -1.0 * q_[2]; //left_leg4
        phi1_4_r[0] = -1.0 * q_[3] +  M_PI; //right_leg1
        phi1_4_r[1] = -1.0 * q_[4];
        dphi1_4_l[0] = -1.0 * dq_[1]; //left_leg1
        dphi1_4_l[1] = -1.0 * dq_[2]; //left_leg4
        dphi1_4_r[0] = -1.0 * dq_[3]; //right_leg1
        dphi1_4_r[1] = -1.0 * dq_[4]; //right_leg4
        // 此时的phi1_4和dphi1_4是五连杆模型中的关节角度和速度可以之间传递给legpose
    }


    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        // QuaterniondToEulerAngles
        EulerAngles euler_angles;
        Eigen::Quaterniond quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        euler_angles = ToEulerAngles(quaterniond);

        imuData[0] = euler_angles.roll;
        imuData[1] = euler_angles.pitch;
        imuData[2] = euler_angles.yaw;

        imuData[3] = msg->angular_velocity.x;
        imuData[4] = msg->angular_velocity.y;
        imuData[5] = msg->angular_velocity.z;

        imuData[6] = msg->linear_acceleration.x; //从imu中得到的线加速度
        imuData[7] = msg->linear_acceleration.y;
        imuData[8] = msg->linear_acceleration.z;

    }

    void inputCallback(const ocs2_msgs::mpc_input::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        input_ = *msg;
    } 

    void legPose_Cal(){
        double legPos[2], legSpd[2];
        double legpos[2], legspd[2];
    
        //计算左腿位置
        leg_pos(phi1_4_l[0],phi1_4_l[1], legPos);
        left_legPose[0] = legPos[0]; //length
        left_legPose[1] = legPos[1]; //angle
        
        //计算左腿速度
        leg_spd(dphi1_4_l[0],dphi1_4_l[1],phi1_4_l[0],phi1_4_l[1], legSpd);
        left_legPose[2] = legSpd[0]; //dLength
        left_legPose[3] = legSpd[1]; //dAngle
    
    
        //计算右腿位置
        leg_pos(phi1_4_r[0],phi1_4_r[1], legpos);
        right_legPose[0] = legpos[0]; //length
        right_legPose[1] = legpos[1]; //angle
    
        //计算右腿速度
        leg_spd(dphi1_4_r[0],dphi1_4_r[1],phi1_4_r[0],phi1_4_r[1],legspd);
        right_legPose[2] = legspd[0]; //dLength
        right_legPose[3] = legspd[1]; //dAngle
    
    
    }
    void states_Cal()
    {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        last_update_time_ = current_time; // 更新时间戳

        // ds = R_w/2*(dtheta_wl + dtheta_wr) + ...
        stateObserved[1] = 0.1/2 * (dq_[5] + dq_[6]) + left_legPose[0]/2*cos(M_PI/2 - left_legPose[1] + imuData[1])*(-1.0 * left_legPose[3] + imuData[4]) +right_legPose[0]/2*cos(M_PI/2 - right_legPose[1] + imuData[1])*(-1.0 * right_legPose[3] + imuData[4]); 

        // s = R_w/2*(theta_wl + theta_wr) + ...
        stateObserved[0] = stateObserved[0] + stateObserved[1] * dt; //此处采用ds的积分

        // yaw = R_w/(2*R_l)*(theta_wr - theta_wl)-l_l/(2*R_l)*sin(theta_ll)+l_r/(2*R_l)*sin(theta_lr)
        stateObserved[2] = imuData[2] - yawAngle_LocalOffset; //此处采用IMU的yaw角 这里减去了yawAngle_LocalOffset

        // dyaw = R_w/(2*R_l)*(ddtheta_wr - ddtheta_wl)-l_l/(2*R_l)*cos(theta_ll)*ddtheta_ll+l_r/(2*R_l)*cos(theta_lr)*ddtheta_lr+l_l/(2*R_l)*sin(theta_ll)*dtheta_ll*dtheta_ll-l_r/(2*R_l)*sin(theta_lr)*dtheta_lr*dtheta_lr
        stateObserved[3] = imuData[5];//此处采用IMU的yaw角

        //thetaL_l = pi/2-legAngle+pitch 
        stateObserved[4] = M_PI/2 - left_legPose[1] + imuData[1];

        //dthetaL_l = -dot_legAngle+dpitch 
        stateObserved[5] = -1.0 * left_legPose[3] + imuData[4];

        //thetaL_r = pi/2-legAngle+pitch
        stateObserved[6] = M_PI/2 - right_legPose[1] + imuData[1];

        //dthetaL_r = -dot_legAngle+dpitch 
        stateObserved[7] = -1.0 * right_legPose[3] + imuData[4];

        //phi = pitch
        stateObserved[8] = imuData[1];

        //dphi = dpitch
        stateObserved[9] = imuData[4];

        //alpha = waist_q + pitch
        stateObserved[10] = q_[0] + imuData[1];

        //dalpha = waist_dq + dpitch
        stateObserved[11] = dq_[0] + imuData[4];
    } 

    void sendStates() {
        std::lock_guard<std::mutex> lock(mtx_);
        ocs2_msgs::mpc_observation obs;
        obs.time = ros::Time::now().toSec();
        obs.input = input_;
        obs.state.value.clear(); 
        obs.state.value.insert(obs.state.value.end(), stateObserved.data(), stateObserved.data() + stateObserved.size());

        nqnv_pub_.publish(obs);
    }

    void sendTorque() {
        std::lock_guard<std::mutex> lock(mtx_);
        /*       关节力矩解算       */ 
        double Tp_templeft[2],Tp_tempright[2];
        double F_l, F_r; //腿长PID输出，左右不一致
        double F_roll, F_inertial; //重力补偿,左右一致；rollPID输出、侧向惯性补偿,左右差分
        // double delta_length_forRoll = (0.24/2)*tan(roll_target); //roll角对应的腿长变化量 test!!!!!!
        //更新PID输出
        PID_CascadeCalc(&rollPID,roll_target,imuData[0],imuData[3]);
        PID_SingleCalc(&thetaPID.outer,0.0,stateObserved[6]-stateObserved[4]);

        PID_CascadeCalc(&leftlengthPID, legLength_target,         left_legPose[0], left_legPose[2]);               
        PID_CascadeCalc(&rightlengthPID,legLength_target + 0.010,right_legPose[0],right_legPose[2]);
        // PID_SingleCalc(&leftlengthPID.outer,legLength_target,left_legPose[0]);
        // PID_SingleCalc(&rightlengthPID.outer,legLength_target+ 0.0055,right_legPose[0]);

        F_roll = 0.0 *rollPID.outer.output; // -1.0
        F_inertial = 0.0; //待优化！！！！

        F_l =        F_roll + leftlengthPID.output  + 115/cos(stateObserved[4]) + -1.0 * F_inertial;  // F_gravity 110/cos(stateObserved[4])
        F_r = -1.0 * F_roll + rightlengthPID.output + 115/cos(stateObserved[6]) +        F_inertial;  // F_gravity 110/cos(stateObserved[6]) 

        if(verbose1 && (print_count % print_fq == 0))
        {
            std::cout << "leftlengthPID.output: " << leftlengthPID.output << std::endl;
            std::cout << "rightlengthPID.output: " << rightlengthPID.output << std::endl;
            std::cout << "thetaPID.output: " << rollPID.outer.output << std::endl;

            std::cout << "F_l: "<< F_l <<"  "<< "F_r: " << F_r <<"  "<< "F_roll: "<< F_roll << std::endl;
        }

        // leg_conv()这个函数的Tp的正方向是-y，输出的Tp1和Tp4的正方向也是-y，所以后面发布关节力矩时要反号
        leg_conv(F_l, -1.0 * (input_.value[2]-1.0*thetaPID.outer.output), phi1_4_l[0],  phi1_4_l[1], Tp_templeft); 
        leg_conv(F_r, -1.0 * (input_.value[3]+1.0*thetaPID.outer.output), phi1_4_r[0],  phi1_4_r[1], Tp_tempright);

        std_msgs::Float64MultiArray forceCmd; // T_waist Tp1_l Tp4_l  Tp1_r Tp4_r Tw_l Tw_r
        forceCmd.data = {input_.value[4], -1.0 * Tp_templeft[0], -1.0 * Tp_templeft[1], -1.0 * Tp_tempright[0], -1.0 * Tp_tempright[1], input_.value[0], input_.value[1]};
        joint_pub_.publish(forceCmd);

        if(verbose2 && (print_count % print_fq == 0))
        {
            std::cout << "mpc_input: " << input_.value[0] << " " << input_.value[1] << " " << input_.value[2] << " " << input_.value[3] << " " << input_.value[4] << std::endl;
            std::cout << "腰部: " << forceCmd.data[0] <<"左1: " <<  forceCmd.data[1] <<"  "<< "左4: " <<  forceCmd.data[2] <<"  "<< "左轮: " <<  forceCmd.data[5] <<"  "<< "右1: " <<  forceCmd.data[3] <<"  "<< "右4: " <<  forceCmd.data[4] <<"  "<< "右轮: " <<  forceCmd.data[6] << std::endl;
            std::cout << "---------------------------------------------------------------------------------------" << std::endl;
        }

        if(print_count < 100000) print_count++; // 防止溢出
        else print_count = 0;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_toocs2_bridge");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(1000); // 1000 Hz
    Ros2Ocs2Bridge processor(node_handle);


    if(ros::ok()) ros::Duration(0.01).sleep(); // 等待0.1秒 修复第一次读取不到数据的问题导致s积分无穷大

    do {
        ros::spinOnce(); // 处理一次回调函数
        ROS_INFO_STREAM("Waiting for hardware initialization...");
        if(ros::ok()) ros::Duration(1).sleep(); //每1秒检查一次
        processor.isInitialized();
    } while (processor.hardwareInitialized_flag == false);
    ROS_INFO_STREAM("Hardware communication initialization completed!!!");

    processor.yawAngle_LocalOffset = processor.imuData[2]; //这里先把yaw角Local偏移量赋值给yawAngle_LocalOffset

    processor.last_update_time_ = ros::Time::now(); // 用于修正状态计算中的时间差

    int print_counter = 0; // 用于控制打印频率
    const int print_frequency = 1000; // 1000 Hz / 1 Hz = 1000

    while(ros::ok())
    {
      ros::spinOnce(); // 处理回调函数

      processor.jointAngle2fiveBarAngle();
      processor.legPose_Cal();
      processor.states_Cal(); 

      if(print_counter % print_frequency == 0)
      {
        std::cout << "q: " << processor.q_.transpose() << std::endl;
        std::cout << "dq: " << processor.dq_.transpose() << std::endl;
        std::cout << "phi1_4_l: " << processor.phi1_4_l.transpose() << std::endl;
        std::cout << "phi1_4_r: " << processor.phi1_4_r.transpose() << std::endl;
        std::cout << "left_legPose: " << processor.left_legPose.transpose() << std::endl;
        std::cout << "right_legPose: " << processor.right_legPose.transpose() << std::endl;
        std::cout << "imuData: "<< processor.imuData.transpose() << std::endl;
        std::cout << "stateObserved: " << processor.stateObserved.transpose() << std::endl;
        std::cout << "----------------------------------------------------------------------" << std::endl;
      }
      processor.sendStates(); // 发送mpc_state消息 "/wheelleg_mpc_observation"
      processor.sendTorque(); // 发送关节力矩消息 "/wheelleg_joint_torque"
      
      loop_rate.sleep();  // 控制循环频率
      print_counter++;
    }
    ros::shutdown();
    return 0;
}