#include "linear_slider_hardware_interface/linear_slider_system_interface.hpp"

#include <chrono>
#include <memory>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define _LOGGER rclcpp::get_logger("LinearSliderSystemHardware")

namespace linear_slider_system_interface 
{

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_init(const hardware_interface::HardwareInfo& info) {
    /* Checks whether the hardware interface matches the robot description */
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Set up motor
    linear_slider_.begin("linear_slider_hardware");

    // TODO: Can probably delete these, since we have a custom object representing our motor
    // // Allocate enough memory for our velocities
    // hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Set hardware configs from the linear_slider.ros2_control.xacro file
    config_.device_name = info_.hardware_parameters["device_name"];

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        // The linear slider has one state and one command interface on the single prismatic joint, make sure they exist
        // Command interface check
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                _LOGGER,
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(),
                joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)) {
            RCLCPP_FATAL(
                _LOGGER,
                "Joint '%s' has %s command interface. Expected %s.",
                joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // State interface check
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(
                _LOGGER,
                "Joint '%s' has %zu state interfaces. 1 expected.",
                joint.name.c_str(),
                joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)) {
            RCLCPP_FATAL(
                _LOGGER,
                "Joint '%s' has %s state interface. Expected %s.",
                joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> LinearSliderSystemInterface::export_command_interfaces() {
    /* Tells the rest of ros2_control which control interfaces are accessible */
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i] 
    //     ));
    // }
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        linear_slider_.name, hardware_interface::HW_IF_VELOCITY, &linear_slider_.vel_cmd
    ));
    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> LinearSliderSystemInterface::export_state_interfaces() {
    /* Tells the rest of ros2_control which state interfaces are accessible */
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (std::size_t i = 0; i<info_.joints.size(); ++i) {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]
    //     ));
    // }
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        linear_slider_.name, hardware_interface::HW_IF_VELOCITY, &linear_slider_.vel_state
    ));
    return state_interfaces;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    /* Set up the comms */
    RCLCPP_INFO(_LOGGER, "Configuring system, please wait...");
    comms_.begin();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    /* Close comms connection */
    RCLCPP_INFO(_LOGGER, "Cleaning up, please wait...");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(_LOGGER, "Activating hardware, please wait...");

    RCLCPP_INFO(_LOGGER, "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(_LOGGER, "Deactivating hardware, please wait...");

    RCLCPP_INFO(_LOGGER, "Successfully deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LinearSliderSystemInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    /* Read data from the linear slider. Message formatted as JSON string. Converts RPM speeds to linear velocities */
    char* msg = comms_.read_data();
    if (msg[0] != '\0'){
        // parse message
        Json::CharReaderBuilder builder;
        Json::CharReader* reader = builder.newCharReader();
        Json::Value msg_json;
        std::string errors;
        bool parsingSuccessful = reader->parse(msg, msg + strlen(msg), &msg_json, &errors);
        delete reader;

        if (!parsingSuccessful) {
            RCLCPP_ERROR(_LOGGER, "Failed to parse JSON message: %s", errors.c_str());
            return hardware_interface::return_type::ERROR;
        }

        // Get system status and store in linear_slider_.system_status
        linear_slider_.system_status = msg_json["status"].asInt();
        // Get motor RPM, convert to float velocity, store in linear_slider_.rpm_state. Additionally, update linear_slider_.vel_state
        linear_slider_.rpm_state = msg_json["servo_rpm"].asInt();
        linear_slider_.vel_state = linear_slider_.rpm_to_vel(linear_slider_.rpm_state); // TODO: this should probably either be completely internal, or completely external, but not both.
        linear_slider_.lim_switch_pos = msg_json["lim_switch_pos"].asBool();
        linear_slider_.lim_switch_neg = msg_json["lim_switch_neg"].asBool();
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type LinearSliderSystemInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    /* Write data to the linear slider. Converts linear velocities to RPM speeds */

    // convert linear_slider_.vel_cmd to linear_slider_.rpm_cmd. Convert this value to str, send via comms_
    linear_slider_.rpm_cmd = linear_slider_.vel_to_rpm(linear_slider_.vel_cmd); // TODO: this should probably either be completely internal, or completely external, but not both.
    std::string cmd = std::to_string(linear_slider_.rpm_cmd);
    comms_.send_data(cmd.c_str());
    return hardware_interface::return_type::OK;
}

} // namespace linear_slider_system_interface


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    linear_slider_system_interface::LinearSliderSystemInterface, hardware_interface::SystemInterface
)