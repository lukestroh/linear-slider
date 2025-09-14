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
