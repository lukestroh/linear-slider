# Linear Slider Bringup

This package defines initial configuration settings and launch files to allow the running of the linear slider in multiple environments. These environments include real-world implementation, a mock hardware system, and interfaces hosted by Gazebo.

## Controller definition
Controller implementations and their parameters for the linear slider are defined in `linear_slider_bringup/config/linear_slider_controllers`. Currently, the linear slider only works with the JointTrajectoryController using position for both state and command interfaces.

## Mock hardware
Mock hardware is a system hosted by the ros2_control framework, extending fake interfaces to the /controller_manager. This allows developers to quickly debug their systems before moving on to more complex environments such as Gazebo or the real world.

Launching the system with mock hardware requires arguments to be passed from the launch file to the xacro command. This looks like the following command:
```
ros2 launch linear_slider_bringup linear_slider.launch.py
```

## Gazebo Classic
This feature does not currently exist, but a [Robotics Stack Exchange question](https://robotics.stackexchange.com/questions/110116/gazebo-ros2-control-does-not-load-joints-or-sensors-from-urdf) regarding my bug is available for viewing.

## Gazebo Ignition
This feature does not currently exist.

## Real World
While the system hardware interface has been defined in `linear_slider_hardware_interface`, proper visualization and controller tools are required. Unfortunately this means that the real-world hardware interface has not yet been tested.