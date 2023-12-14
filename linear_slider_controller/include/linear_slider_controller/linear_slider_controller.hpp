#ifndef ___LINEAR_SLIDER_CONTROLLER_HPP___
#define ___LINEAR_SLIDER_CONTROLLER_HPP___

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>


namespace linear_slider
{
    class LinearSliderController: public controller_interface::ControllerInterface
    {
        public:
            LINEAR_SLIDER_CONTROLLER_PUBLIC
            LinearSliderController();

            LINEAR_SLIDER_CONTROLLER_PUBLIC
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            
}