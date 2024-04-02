#ifndef __LINEAR_SLIDER__HARDWARE_INTERFACE_H__
#define __LINEAR_SLIDER__HARDWARE_INTERFACE_H__

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

#include "linear_slider_hardware_interface/visibility_control.h"

#include "linear_slider_hardware_interface/clearcore_comms.hpp"
#include "linear_slider_hardware_interface/linear_slider_hardware.hpp"

namespace linear_slider_system_interface
{
    class LinearSliderSystemInterface : public hardware_interface::SystemInterface
    {

        struct Config {
            std::string device_name = "";
        };

        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(LinearSliderSystemInterface)

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override; // publishes to the rest of ros2_control so that we know what we can read/write to

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) override;

            LINEAR_SLIDER_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;


        private:
            // data structures for holding velocity data
            // std::vector<double> hw_states_velocities_;
            // std::vector<double> hw_commands_velocities_;

            // Comms
            ClearCoreComms comms_;

            // LinearSlider hardware (Motor, Limit switches)
            LinearSliderHardware linear_slider_;

            // Config
            Config config_;

            rclcpp::Time last_read_time;
    };
} // namespace linear_slider_system_interface

#endif // __LINEAR_SLIDER__HARDWARE_INTERFACE_H__