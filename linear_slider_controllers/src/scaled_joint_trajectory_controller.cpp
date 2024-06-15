#include <memory>
#include <vector>
#include "linear_slider_controllers/scaled_joint_trajectory_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"


controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  // Create the parameter listener and get the parameters
  scaled_param_listener_ = std::make_shared<scaled_joint_trajectory_controller::ParamListener>(get_node());
  scaled_params_ = scaled_param_listener_->get_params();

  return JointTrajectoryController::on_init();
}