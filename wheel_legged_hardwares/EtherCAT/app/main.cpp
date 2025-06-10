/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */



extern "C"
{
#include <stdio.h>
#include "config.h"
#include "ethercat.h"

#include "unistd.h"
}
#include "transmit.h"
#include "motor_control.h"
#include "matplotlibcpp.h"
#include "thread"
#include "cmath"

void EtherCAT_Data_Explain(EtherCAT_Msg* RxMessage){

}
void EtherCAT_Command_Prepare(EtherCAT_Msg *TxMessage){

}
namespace plt = matplotlibcpp;


std::vector<double> t;
std::vector<double> pos_vector;
std::vector<double> vel_vector;
std::vector<double> cur_vector;
//打印电机的图像
void Thread_Print(){



    int i=0;

    plt::ion();
    plt::figure();
    plt::figure();
    plt::figure();
    while(1){

        sleep(1);
        i++;
        //if(t.size()>1000){
        //    t.erase(t.begin());
        //    pos_vector.erase(pos_vector.begin());
        //    vel_vector.erase(vel_vector.begin());
        //    cur_vector.erase(cur_vector.begin());
        //}
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


int main()
{

    std::thread Printfthread(Thread_Print);
    printf("SOEM 主站测试\n");

    //这里填自己电脑上的网卡
    EtherCAT_Init("enp2s0");  //工位PC：enp2s0   NUC: enp86s0

    if (ec_slavecount <= 0)
    {
        printf("未找到从站, 程序退出！");
        return 1;
    }
    else
        printf("从站数量： %d\r\n", ec_slavecount);

    int i=0;
    while (1)
    {

        EtherCAT_Run();
        // EtherCAT_Run_Once();
        usleep(2000);


        t.push_back(i++);
        pos_vector.push_back(rv_motor_msg[0].angle_actual_float);
        //vel_vector.push_back(rv_motor_msg[0].speed_actual_float);
        vel_vector.push_back(sin(i/500.0));
        cur_vector.push_back(rv_motor_msg[0].current_actual_float);
        if(t.size()>3000){
            t.erase(t.begin());
            pos_vector.erase(pos_vector.begin());
            vel_vector.erase(vel_vector.begin());
            cur_vector.erase(cur_vector.begin());

        }


    }
    Printfthread.join();
    return 0;
}
