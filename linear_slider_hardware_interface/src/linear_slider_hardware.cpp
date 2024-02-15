#include "linear_slider_hardware_interface/linear_slider_hardware.hpp"

LinearSliderHardware::LinearSliderHardware(){
    name = "default";
}

LinearSliderHardware::LinearSliderHardware(const std::string& system_name) {
    begin(system_name);
}

LinearSliderHardware::~LinearSliderHardware(){}

void LinearSliderHardware::begin(const std::string& system_name) {
    name = system_name;
}

double LinearSliderHardware::rpm_to_vel(int rpm) {
    /* Convert revolutions per minute of the motor to linear velocity of the slider 
       two turns for each cm of distance
       1 rev/min * 1 min/60s * 1cm/2rev * 1m/100cm = m/s
    */
    return static_cast<double>(rpm / 60.0 / 2.0 / 100.0);
}

int LinearSliderHardware::vel_to_rpm(double& vel) {
    /* Convert linear velocity of the slider to revolultions per minute of the motor 
       1cm of distance for two turns
       1 m/s * 60s/min * 100cm/m * 2rev/cm = rev/min
    */
   return static_cast<int>(vel * 60 * 100 * 2);
}
