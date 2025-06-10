//
// Created by yeyinong on 2023/12/20.
//

#include "plot.h"
#include "matplotlibcpp.h"
// #include "../../Robot/Robot.h"

namespace plt = matplotlibcpp;

mutex mutex_plot_data;

vector<float> position(2000, 0);
vector<float> velocity(2000, 0);
vector<float> torque(2000, 0);
vector<vector<float>> plotData{position, velocity, torque};
vector<float> plotXAxis;

int beginIndex = 0;
const int plotSize = 2000;

const string plotName[3] = {"position", "velocity", "torque"};

void plotGraph(){
    if (plotXAxis.empty()){
        for (int i = 0; i < plotSize; ++i) {
            plotXAxis.push_back(i);
        }
    }
    plt::figure();
    plt::ion();
    struct timeval lastTime;
    struct timeval now;
    while (true){
        
        // for (int i = 0; i < 3; ++i) {
        //     plt::subplot2grid(1,3, 0, i);

        //     plt::xlim( 0., (double )plotSize);
        //     plt::ylim(-M_PI, M_PI);
        //     named_plot_user(plotName[i], plotXAxis, plotData[i]);
        //     plt::legend();
        // }
        plt::clf();
        named_plot_user(plotName[0], plotXAxis, plotData[0]);

        plt::pause(0.001);

    }
    plt::show();
}
