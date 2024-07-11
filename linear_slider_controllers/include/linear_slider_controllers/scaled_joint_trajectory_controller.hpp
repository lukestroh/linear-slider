#ifndef LINEAR_SLIDER_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define LINEAR_SLIDER_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include <rclcpp/rclcpp.hpp>


namespace linear_slider_controllers
{

class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
    public:
        ScaledJointTrajectoryController() = default;
        ~ScaledJointTrajectoryController() override = default;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_init() override;

    protected:
        struct TimeData {
            TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0.0)), uptime(0.0) {}
            rclcpp::Time time;
            rclcpp::Duration period;
            rclcpp::Time uptime;
        };

    private:
        double scaling_factor_{};
        realtime_tools::RealtimeBuffer<TimeData> time_data_;
        // std::shared_ptr<scaled_joint_trajectory_controller::ParamListener> scaled_param_listener_;
        // scaled_joint_trajectory_controller::Params scaled_params_;

};

}




#endif // LINEAR_SLIDER_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_