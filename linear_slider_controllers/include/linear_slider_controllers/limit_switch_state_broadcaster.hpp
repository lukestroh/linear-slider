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

#ifndef __LINEAR_SLIDER_CONTROLLERS__LIMIT_SWITCH_STATE_BROADCASTER__
#define __LINEAR_SLIDER_CONTROLLERS__LIMIT_SWITCH_STATE_BROADCASTER__

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "linear_slider_msgs/msg/limit_switch_sensors.hpp"
#include "limit_switch_state_broadcaster_parameters.hpp"

namespace linear_slider_controllers {

class LimitSwitchStateBroadcaster : public controller_interface::ControllerInterface {
  public:
    LimitSwitchStateBroadcaster() = default;
    ~LimitSwitchStateBroadcaster() override = default;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /* previous_state */) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /* previous_state */) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) override;
    
  protected:
    std::vector<std::string> sensor_names_;
    double publish_rate_;

    std::shared_ptr<rclcpp::Publisher<linear_slider_msgs::msg::LimitSwitchSensors>> limit_switch_state_publisher_;// TODO make custom msg type.
    linear_slider_msgs::msg::LimitSwitchSensors limit_switch_state_msg_;

    // Parameters from ROS (find in CMakeLists file for generated build)
    std::shared_ptr<limit_switch_state_broadcaster::ParamListener> param_listener_;
    limit_switch_state_broadcaster::Params params_;
};

} // namespace linear_slider_controllers
#endif // __LINEAR_SLIDER_CONTROLLERS__LIMIT_SWITCH_STATE_BROADCASTER__
