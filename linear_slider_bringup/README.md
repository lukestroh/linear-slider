# Linear Slider Bringup

## TODO:

1. Add lifecycle node that triggers limit switch states if using mock hardware.

This package defines initial configuration settings and launch files to allow the running of the linear slider in multiple environments. These environments include real-world implementation, a mock hardware system, and interfaces hosted by Gazebo.

## Controller definition
Controller implementations and their parameters for the linear slider are defined in `linear_slider_controllers/config/linear_slider_controllers`. Currently, the linear slider only works with the JointTrajectoryController using position for both state and command interfaces.

## Mock hardware
Mock hardware is a system hosted by the ros2_control framework, extending fake interfaces to the /controller_manager. This allows developers to quickly debug their systems before moving on to more complex environments such as Gazebo or the real world.

Launching the system with mock hardware requires arguments to be passed from the launch file to the xacro command. This looks like the following command:
```
ros2 launch linear_slider_bringup linear_slider.launch.py
```

## Gazebo Classic
The model successfully launches in Gazebo after having fixed some strange bugs. Important bugs to note:
1. When calling the mesh files into the URDF, RViz allows for the package to be called using the command `filename="package://<package_name>/path/to/file"`. Gazebo did not like this, however, and requires the URDF to find the file with the command `filename="file://$(find <package_name>)/path/to/file"`.
2. The model must also not interact with the ground, so in the launch file, the object is spawned at a height of z=1.

Launching the linear slider in Gazebo Classic can be done with the following command.  
```
ros2 launch linear_slider_bringup linear_slider_sim_gazebo.launch.py sim_gazebo_classic:=true
```

## Gazebo Ignition
This feature does not currently exist.

## Real World
While the system hardware interface has been defined in `linear_slider_hardware_interface`, proper visualization and controller tools are required. Unfortunately this means that the real-world hardware interface has not yet been tested.    