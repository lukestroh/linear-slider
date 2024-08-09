/*
Copyright 2024, Luke Strohbehn, Grimmlins Lab - Oregon State Robotics

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Author(s): Luke Strohbehn
Date: 2024-07-26
*/

#include "linear_slider_controllers/limit_switch_state_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"

namespace rclcpp_lifecycle {
    class State;
} // namespace rclcpp_lifecycle

namespace linear_slider_controllers {

#define _LOGGER this->get_node()->get_logger()

controller_interface::CallbackReturn LimitSwitchStateBroadcaster::on_init() {
    try {
        // Create parameter listener and get parameters
        param_listener_ = std::make_shared<limit_switch_state_broadcaster::ParamListener>(this->get_node());
        params_ = param_listener_->get_params();
        RCLCPP_INFO(_LOGGER, "Loading LimitSwitchBroadcaster with prefix: %s", params_.prefix.c_str()); // TODO
    } catch (std::exception& e) {
        // get_node() may throw, logging raw
        fprintf(stderr, "Exception thrown during init state with message: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LimitSwitchStateBroadcaster::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration LimitSwitchStateBroadcaster::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string prefix = params_.prefix;
    config.names.push_back(prefix + "lim_switch_neg/state"); // TODO: figure out what this does
    config.names.push_back(prefix + "lim_switch_pos/state");

    return config;
}

controller_interface::CallbackReturn LimitSwitchStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /* previous_state */) {
    if (!param_listener_) {
        RCLCPP_ERROR(_LOGGER, "Error encountered during init.");
        return controller_interface::CallbackReturn::ERROR;
    }

    // update the dynamic map parameters
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    publish_rate_ = params_.state_publish_rate;

    RCLCPP_INFO(_LOGGER, "Publisher rate set to: %.1f Hz", publish_rate_);

    // Create the publisher
    try {
        limit_switch_state_publisher_ = this->get_node()->create_publisher<linear_slider_msgs::msg::LimitSwitchSensors>("~/sensor_states", rclcpp::SystemDefaultsQoS());
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during configuration state with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LimitSwitchStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
    RCLCPP_INFO(_LOGGER, "Activating LimitSwitchStateBroadcaster");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LimitSwitchStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
    RCLCPP_INFO(_LOGGER, "Deactivating LimitSwitchStateBroadcaster");
    return controller_interface::CallbackReturn::SUCCESS;
}p"

PLUGINLIB_EXPORT_CLASS(linear_slider_controllers::LimitSwitchStateBroadcaster, controller_interface::ControllerInterface)


controller_interface::return_type LimitSwitchStateBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (publish_rate_ > 0 && period > rclcpp::Duration(1.0 / publish_rate_, 0.0)) {
        for (const hardware_interface::LoanedStateInterface& state : this->state_interfaces_) {
            limit_switch_state_msg_.state = state.get_value();
            limit_switch_state_msg_.name = state.get_name();
            limit_switch_state_msg_.header.stamp.sec = time.seconds();
            limit_switch_state_msg_.header.stamp.nanosec = time.nanoseconds();
            limit_switch_state_msg_.header.frame_id = state.get_name();
            // Publish limit switch state
            limit_switch_state_publisher_->publish(limit_switch_state_msg_);

        }
        // limit_switch_state_msg_.pos_limit = this->state_interfaces_[1].get_value();

    }
    return controller_interface::return_type::OK;
}

} // namespace linear_slider_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(linear_slider_controllers::LimitSwitchStateBroadcaster, controller_interface::ControllerInterface)
