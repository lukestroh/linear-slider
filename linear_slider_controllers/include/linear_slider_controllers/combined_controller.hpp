#ifndef LINEAR_SLIDER_CONTROLLERS__COMBINED_CONTROLLER_HPP_
#define LINEAR_SLIDER_CONTROLLERS__COMBINED_CONTROLLER_HPP_

#include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include <rclcpp/rclcpp.hpp>
// #include "scaled_joint_trajectory_controller_parameters.hpp"

// #include "ur_controllers/scaled_joint_trajectory_controller.hpp"
#include <memory>
#include <vector>
#include "lifecycle_msgs/msg/state.hpp"

namespace linear_slider_controllers
{

class CombinedController : public controller_interface::ControllerInterface {
    public:
        // CombinedController() = default;
        // ~CombinedController() override = default;
        CombinedController() = default;
        // controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        // controller_interface::return_type init(const std::string& controller_name) override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        
    protected:
        // struct TimeData {
        //     TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0.0), uptime(0.0)) {}
        //     rclcpp::Time time;
        //     rclcpp::Duration period;
        //     rclcpp::Time uptime;
        // };
        rclcpp::Logger logger_ = this->get_node()->get_logger();
        std::string _linear_slider_joint_prefix = this->get_node()->get_parameter("prefix").as_string();
        std::string _ur_joint_prefix = this->get_node()->get_parameter("tf_prefix").as_string();

    

        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _sub_trajectory;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pub_linear_slider_controller;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pub_ur_controller;

        void _sub_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    // private:
    //     double scaling_factor_{};
    //     realtime_tools::RealtimeBuffer<TimeData> time_data_;

    //     std::shared_ptr<scaled_trajectory_controller::ParamListener> scaled_param_listener_;
    //     scaled_joint_trajectory_controller::Params scaled_params_;
    };   
} // namespace linear_slider_controllers

#endif // LINEAR_SLIDER_CONTROLLERS__COMBINED_CONTROLLER_HPP_