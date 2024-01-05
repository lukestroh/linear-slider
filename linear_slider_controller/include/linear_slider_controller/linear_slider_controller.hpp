#ifndef ___LINEAR_SLIDER_CONTROLLER_HPP___
#define ___LINEAR_SLIDER_CONTROLLER_HPP___

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>

#include "visibility_control.h"

// https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

namespace linear_slider_controller_interface
{
    class LinearSliderController: public controller_interface::ControllerInterface
    {
        public:
            LINEAR_SLIDER_CONTROLLER_PUBLIC
            LinearSliderController();

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            
}