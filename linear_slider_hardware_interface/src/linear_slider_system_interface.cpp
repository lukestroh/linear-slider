#include "linear_slider_hardware_interface/linear_slider_system_interface.hpp"

#include <chrono>
#include <memory>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define _LOGGER rclcpp::get_logger("LinearSliderSystemHardware")

namespace linear_slider_system_interface 
{

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_init(const hardware_interface::HardwareInfo& info) {
    /* Checks whether the hardware interface matches the robot description */
    // _pub = this->create_publisher<std_msgs::msg::String>("test_messages", 10);

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
    config_.ip_addr = info_.hardware_parameters["device_ip"];
    config_.port = info_.hardware_parameters["device_port"];

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
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                _LOGGER,
                "Joint '%s' has %zu state interfaces. 2 expected.",
                joint.name.c_str(),
                joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (std::size_t i=0; i > joint.state_interfaces.size(); ++i) {
            if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY) || !(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION)) {
                RCLCPP_FATAL(
                    _LOGGER,
                    "Joint '%s' has %s state interface. Expected %s. or %s",
                    joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION,
                    hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
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
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &linear_slider_.command.vel
    ));
    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> LinearSliderSystemInterface::export_state_interfaces() {
    /* Tells the rest of ros2_control which state interfaces are accessible.
       Linear slide contains a single joint and two limit switches.
    */
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (std::size_t i = 0; i<info_.joints.size(); ++i) {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]
    //     ));
    // }

    // Export joint state interface. We have just one joint, so no for loop needed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &linear_slider_.state.pos
    ));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &linear_slider_.state.vel
    ));

    // Export sensor state interface (adding limit switches into the exported state)
    for (std::size_t i=0; i<info_.sensors.size(); ++i) {
        for (std::size_t j=0; j<info_.sensors[i].state_interfaces.size(); ++j) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, linear_slider_.state.limit_switches[i]
            ));
        }
    }
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.sensors
    // ));
    return state_interfaces;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    /* Set up the comms */
    RCLCPP_INFO(_LOGGER, "Configuring system, please wait...");
    RCLCPP_INFO(_LOGGER, "Setting up communication...");
    if (comms_.begin()) {return hardware_interface::CallbackReturn::SUCCESS;}
    else {return hardware_interface::CallbackReturn::FAILURE;}
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    /* Close comms connection */
    RCLCPP_INFO(_LOGGER, "Cleaning up, please wait...");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    /* Activate the hardware by sending a calibration request if the system is in standby. */
    RCLCPP_INFO(_LOGGER, "Activating hardware, please wait...");
    while (true) {
        hardware_interface::return_type read_success = read(rclcpp::Clock().now(), rclcpp::Duration(0, 0));

        // RCLCPP_WARN(_LOGGER, "Slider status: %d", linear_slider_.state.system_status);

        if (read_success == hardware_interface::return_type::OK) {
            // Don't do anything if system is normal
            if (linear_slider_.state.system_status == slidersystem::SYSTEM_OK) {
                linear_slider_.command.system_status = slidersystem::SYSTEM_OK;
                break;
            }
            // If in standby, calibrate
            else if (linear_slider_.state.system_status == slidersystem::SYSTEM_STANDBY) {
                // make sure this message only gets sent once...
                if (!calibration_cmd_sent) {
                    linear_slider_.command.system_status = slidersystem::SYSTEM_CALIBRATING;
                    this->write(rclcpp::Clock().now(), rclcpp::Duration(0,0));
                    calibration_cmd_sent = true;
                    RCLCPP_INFO(_LOGGER, "Calibration request sent.");
                }
            }
            // If E-stop, don't do anything
            else if (linear_slider_.state.system_status == slidersystem::E_STOP) {
                RCLCPP_ERROR(_LOGGER, "Linear slider E-stop triggered.");
                return hardware_interface::CallbackReturn::FAILURE;
            }
            // If the system is calibrating, don't do anything
            else if (linear_slider_.state.system_status == slidersystem::SYSTEM_CALIBRATING) {
                RCLCPP_INFO(_LOGGER, "System calibrating...");
            }
            // If at the switches, update position and return to normal operation
            else if (linear_slider_.state.system_status == slidersystem::NEG_LIM) {
                linear_slider_.state.pos = linear_slider_.pos_min; // TODO: remove hardcode
                linear_slider_.command.system_status = slidersystem::SYSTEM_OK;
                linear_slider_.command.vel = linear_slider_.start_velocity;
                this->write(rclcpp::Clock().now(), rclcpp::Duration(0,0));
            }
            else if (linear_slider_.state.system_status == slidersystem::POS_LIM) {
                linear_slider_.state.pos = linear_slider_.pos_max; // TODO: remove hardcode
                linear_slider_.command.system_status = slidersystem::SYSTEM_OK;
                linear_slider_.command.vel = linear_slider_.start_velocity;
                this->write(rclcpp::Clock().now(), rclcpp::Duration(0,0));
            }
        }
    }

    RCLCPP_INFO(_LOGGER, "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinearSliderSystemInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    /* Put the slider into SYSTEM_STANDBY status */
    RCLCPP_INFO(_LOGGER, "Deactivating hardware, please wait...");
    // Send hardware to standby mode, set velocity to 0.
    linear_slider_.command.system_status = slidersystem::SYSTEM_STANDBY;
    linear_slider_.command.rpm = 0.0;
    this->write(rclcpp::Clock().now(), rclcpp::Duration(0,0));

    RCLCPP_INFO(_LOGGER, "Successfully deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LinearSliderSystemInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    /* Read data from the linear slider. Message formatted as JSON string. Converts RPM speeds to linear velocities */
    char* msg = comms_.read_data();

    // RCLCPP_WARN(_LOGGER, "Read time: %f", time.nanoseconds() / 1e9);

    if (msg[0] != '\0'){
        const std::unique_ptr<Json::CharReader> reader(Json::CharReaderBuilder().newCharReader());
        Json::Value msg_json;
        const std::unique_ptr<std::string> errors;
        
        bool parsingSuccessful = reader->parse(msg, msg + strlen(msg), &msg_json, errors.get());

        if (!parsingSuccessful) {
            RCLCPP_ERROR(_LOGGER, "Failed to parse JSON message: %s", (*errors).c_str());
            return hardware_interface::return_type::ERROR;
        }

        // Get system status and store in linear_slider_.system_status
        linear_slider_.state.system_status = static_cast<slidersystem::SystemStatus>(msg_json["status"].asInt());
        // Get motor RPM, convert to float velocity, store in linear_slider_.rpm_state. Additionally, update linear_slider_.vel_state
        linear_slider_.state.rpm = msg_json["servo_rpm"].asInt();
        linear_slider_.state.vel = linear_slider_.rpm_to_vel(linear_slider_.state.rpm); // TODO: this should probably either be completely internal, or completely external, but not both.
        linear_slider_.state.lim_switch_neg = msg_json["lim_switch_neg"].asBool();
        linear_slider_.state.lim_switch_pos = msg_json["lim_switch_pos"].asBool();

        // rclcpp::Duration last_read_duration = time - last_read_time;
        linear_slider_.state.pos += period.nanoseconds() / 1e9 * linear_slider_.state.vel;
         
        // RCLCPP_INFO(_LOGGER, "State Position: %f, Velocity: %f, Duration: %f", linear_slider_.state.pos, linear_slider_.state.vel, period.nanoseconds() / 1e9);
        // RCLCPP_INFO(_LOGGER, "Command Position: %f, Velocity: %f", linear_slider_.command.pos, linear_slider_.command.vel);

    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type LinearSliderSystemInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    /* Write data to the linear slider. Converts linear velocities to RPM speeds */

    // convert linear_slider_.vel_cmd to linear_slider_.rpm_cmd. Convert this value to str, send via comms_
    // RCLCPP_WARN(_LOGGER, "Write time: %f", time.nanoseconds() / 1e9);

    linear_slider_.command.rpm = linear_slider_.vel_to_rpm(linear_slider_.command.vel); // TODO: this should probably either be completely internal, or completely external, but not both.
    std::string status_cmd = std::to_string(linear_slider_.command.system_status);
    std::string rpm_cmd = std::to_string(linear_slider_.command.rpm);

    comms_.send_data((status_cmd + "," + rpm_cmd).c_str());

    // RCLCPP_WARN(_LOGGER, "Write rpm: %s", (status_cmd + "," + rpm_cmd).c_str());
    // RCLCPP_WARN(_LOGGER, "Write period: %f", period.nanoseconds() / 1e9);

    return hardware_interface::return_type::OK; 
}

} // namespace linear_slider_system_interface


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    linear_slider_system_interface::LinearSliderSystemInterface, hardware_interface::SystemInterface
)