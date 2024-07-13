#ifndef __LINEAR_SLIDER_HARDWARE_HPP__
#define __LINEAR_SLIDER_HARDWARE_HPP__

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "linear_slider_hardware_interface/system_status.h"

struct Interface{
    slidersystem::SystemStatus system_status = slidersystem::SYSTEM_STANDBY;
    double pos = 0.0;
    double vel = 0.0;
    int rpm = 0;
    double lim_switch_neg {false}; // hardware_interface::StateInterface only accepts double as a value, not bool. TODO: Switch to GPIO??
    double lim_switch_pos {false};
    double* limit_switches[2] = {&lim_switch_neg, &lim_switch_pos};
};

class LinearSliderHardware {
    public:
        std::string name;
        Interface state;
        Interface command;
        std::vector<std::string> joint_names = {"joint1"};

        double pos_min = -0.4;
        double pos_max = 0.4;
        double start_velocity = 0.0;

        LinearSliderHardware();
        LinearSliderHardware(const std::string& system_name);
        ~LinearSliderHardware();

        void begin(const std::string& system_name);
        double rpm_to_vel(int rpm);
        int vel_to_rpm(double& vel);
};

#endif // __LINEAR_SLIDER_HARDWARE_HPP__