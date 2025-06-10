//
// Created by bismarck on 11/19/22.
//

#ifndef MASTERSTACK_COMMAND_H
#define MASTERSTACK_COMMAND_H
//extern "C" {
#include "config.h"
#include "motor_control.h"
#include "transmit.h"
//}

#include <iostream>
#include <vector>
#include <unistd.h>
#include <chrono>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;



unsigned test(const std::vector<std::string> &);
void Print_task();
// unsigned help(const std::vector<std::string> &);

// unsigned motorZeroSetCmd(const std::vector<std::string> & input);
// unsigned motorMotorSetCmd(const std::vector<std::string> & input);
// unsigned motorRestSetCmd(const std::vector<std::string> & input);
// unsigned motorSpeedSetCmd(const std::vector<std::string> & input);
// unsigned motorPositionSetCmd(const std::vector<std::string> & input);
// unsigned motorTorqueSetCmd(const std::vector<std::string> & input);
// unsigned motionInitCmd(const std::vector<std::string> & input);
// unsigned motionRunCmd(const std::vector<std::string> & input);

#endif //MASTERSTACK_COMMAND_H
