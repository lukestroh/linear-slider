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
};

class LinearSliderHardware {
    public:
        std::string name;
        Interface state;
        Interface command;
        std::vector<std::string> joint_names = {"joint1"};

        // int rpm_cmd = 0;
        // int rpm_state = 0;
        // double vel_cmd = 0;
        // double vel_state = 0;
        bool lim_switch_pos = false;
        bool lim_switch_neg = false;

        LinearSliderHardware();
        LinearSliderHardware(const std::string& system_name);
        ~LinearSliderHardware();

        void begin(const std::string& system_name);
        double rpm_to_vel(int rpm);
        int vel_to_rpm(double& vel);
};

#endif // __LINEAR_SLIDER_HARDWARE_HPP__