#include "linear_slider_controllers/combined_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace linear_slider_controllers
{
// controller_interface::return_type CombinedController::init(const std::string& controller_name) {
//     RCLCPP_INFO(logger_, "CombinedController running init() method.");
//     return controller_interface::return_type::OK;
// }

controller_interface::CallbackReturn CombinedController::on_init() {
    // Init base class
    // auto ret = ControllerInterface::init("combined_controller");
    // if (ret != controller_interface::return_type::OK) {
    //     return controller_interface::CallbackReturn::ERROR;
    // }

    // // Init subscriber for planner's command
    // _sub_trajectory = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    //     "/combined_controller/joint_trajectory",
    //     1,
    //     std::bind(&CombinedController::_sub_trajectory_callback, this, std::placeholders::_1)
    // );

    // Init publishers for both controllers
    _pub_ur_controller = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 1);
    _pub_linear_slider_controller = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>("linear_slider_controller/joint_trajectory", 1);
    

    return controller_interface::CallbackReturn::SUCCESS;
}

void CombinedController::_sub_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // Forward the command to both controllers
    _pub_ur_controller->publish(*msg);
    _pub_linear_slider_controller->publish(*msg);
}

controller_interface::return_type CombinedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    RCLCPP_INFO(logger_, "Update function run.");
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration CombinedController::command_interface_configuration() const {
    RCLCPP_INFO(logger_, "Command interface configuration run");
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // can select all interfaces or none
    config.names = {
        "joint1/velocity",
        "elbow_joint/position",
        "shoulder_lift_joint/position",
        "shoulder_pan_joint/position",
        "wrist_1_joint/position",
        "wrist_2_joint/position",
        "wrist_2_joint/position",
    };

    return config;
}

controller_interface::InterfaceConfiguration CombinedController::state_interface_configuration() const {
    RCLCPP_INFO(logger_, "State interface configuration run");
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "joint1/velocity",
        "joint1/position",
        "elbow_joint/position",
        "shoulder_lift_joint/position",
        "shoulder_pan_joint/position",
        "wrist_1_joint/position",
        "wrist_2_joint/position",
        "wrist_2_joint/position",
    };
    return config;
}

} // namespace linear_slider_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(linear_slider_controllers::CombinedController, controller_interface::ControllerInterface)