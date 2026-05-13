#ifndef __LINEAR_SLIDER_HARDWARE_HPP__
#define __LINEAR_SLIDER_HARDWARE_HPP__

#include <cstdint>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "linear_slider_hardware_interface/system_status.h"

struct Interface{
    slidersystem::SystemStatus system_status = slidersystem::SYSTEM_STANDBY;
    double pos = 0.0; // TODO: get from initial_positions file?
    double vel = 0.0;
    int rpm = 0;
    int32_t pos_steps = 0;
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

        const double pos_min = -0.4; // TODO: get these limits from yaml file
        const double pos_max = 0.4;
        const double start_velocity = 0.0;

        const double revs_per_cm = 2.0;
        const double revs_per_meter = 200.0;
        double steps_per_rev = 800.0; // TODO: set from MSP / ClearPath config.
        double steps_per_meter = steps_per_rev * revs_per_meter;

        LinearSliderHardware();
        LinearSliderHardware(const std::string& system_name);
        ~LinearSliderHardware();

        void begin(const std::string& system_name);
        void set_steps_per_rev(double steps);
        double rpm_to_vel(int rpm);
        int vel_to_rpm(double& vel);

        int32_t meters_to_steps(double meters);
        double steps_to_meters(int32_t steps);
};

#endif // __LINEAR_SLIDER_HARDWARE_HPP__
