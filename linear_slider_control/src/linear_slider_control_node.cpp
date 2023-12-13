#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <realtime_tools/thread_priority.hpp>

const int kScheduledPriority = 50;


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Set up executor
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Set up controller manager node
    auto controller_manager_node = std::make_shared<controller_manager::ControllerManager>(executor, "controller_manager");
    RCLCPP_INFO(controller_manager_node->get_logger(), "Controller manager node update rate is %d Hz", controller_manager_node->get_update_rate());

    std::thread controller_manager_thread([controller_manager_node]() {
        if (realtime_tools::has_realtime_kernel()) {
            if (!realtime_tools::configure_sched_fifo(kScheduledPriority)) {
                RCLCPP_WARN(controller_manager_node->get_logger(), "Failed to set scheduler to FIFO with priority %d", kScheduledPriority);
            }
        }
        else {
            RCLCPP_INFO(controller_manager_node->get_logger(), "Realtime kernel not detected. Running controller manager in normal priority thread.");
        }
    

        // Calculate sleep time
        auto const period = std::chrono::nanoseconds(static_cast<int>(1e9 / controller_manager_node->get_update_rate()));
        auto const controller_manager_now = std::chrono::nanoseconds(controller_manager_node->now().nanoseconds());
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time {controller_manager_now};

        // Calculate measured period of the loop
        rclcpp::Time previous_time = controller_manager_node->now();

        while (rclcpp::ok()) {
            // Calculate measured period
            auto const current_time = controller_manager_node->now();
            auto const measured_period = current_time - previous_time;
            previous_time = current_time;

            // Update controller manager
            controller_manager_node->read(controller_manager_node->now(), measured_period);
            controller_manager_node->update(controller_manager_node->now(), measured_period);
            controller_manager_node->write(controller_manager_node->now(), measured_period);

            next_iteration_time += period;
            std::this_thread::sleep_until(next_iteration_time);
        }
    });

    // Load controllers
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    controller_manager_node->load_controller("joint_state_controller", "joint_state_controller/JointStateController");
    controlelr_manager_node->load_controller("joint_command_controller", "joint_command_controller/JointCommandController"); // could be linear_slider controller?

    // Configure controllers
    controller_manager_node->configure_controller("joint_state_controller");
    controller_manager_node->configure_controller("joint_command_controller");

    // Start controllers
    start_controllers.push_back("joint_state_controller");
    start_controllers.push_back("joint_command_controller");
    controller_manager_node->switch_controller(start_controllers, stop_controllers, 1, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);

    // Run the node
    executor->add_node(controller_manager_node);
    executor->spin();

    // Exit
    rclcpp::shutdown();

    return 0;
}