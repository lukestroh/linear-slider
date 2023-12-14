#include "linear_slider_controller/linear_slider_system_interface.hpp"

#include <chrono>
#include <memory>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace linear_slider_system_interface 
{

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Allocate enough memory for our velocities
    hw_states_velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        // The linear slider has one state and one command interface on the single prismatic joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("LinearSliderSystemHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        
    }


}

} // namespace linear_slider_system_interface