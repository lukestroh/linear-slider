#ifndef __LINEAR_SLIDER_CONTROLLER__HARDWARE_INTERFACE_H__
#define __LINEAR_SLIDER_CONTROLLER__HARDWARE_INTERFACE_H__

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "linear_slider_controller/visibility_control.h"

namespace linear_slider_system_interface
{
    class LinearSliderSystemInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(LinearSliderSystemInterface);

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override; // publishes to the rest of ros2_control so that we know what we can read/write to

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            hardware_interface::return_type prepare_command_mode_switch(
                const std::vector<std::string>& start_interfaces,
                const std::vector<std::string>& stop_interfaces
            ) override;

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            hardware_interface::return_type read(const rclcpp::Time& time, rclcpp::Duration& period) override;

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            hardware_interface::return_type write(const rclcpp::Time& time, rclcpp::Duration& period) override;
    };
} // namespace linear_slider_system_interface

#endif // __LINEAR_SLIDER_CONTROLLER__HARDWARE_INTERFACE_H__