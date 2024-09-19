# TODO:

1.  fix limit switch broadcaster (doesn't publish 1 to 1). 
1. When starting program on lim switch, sometimes initial burst of movement off the switch. Why??
1. Remove /tmp/ folder stuff for kinematics launch file stuff (remove old way, moveitconfigsbuilder is better)
1. Refactor all of the MoveIt stuff, it's a mess. (Maybe check out Jazzy? Looks like the MoveItConfigsBuilder is what they're working towards...)
1.  Add Isaac Sim instead of Gazebo: [DIRECTIONS HERE](https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html). 
1.  Detect if slider starts up on limit switch. Otherwise calibration violates safety.
    1. Check why accelerates away from switch sometimes
1.  ~~Allow for backwards movement after limit switch hit.~~
1.  ~~If program is restart while ClearCore controller is not, it accelerates off in the wrong direction of calibration.~~
1.  ~~Add limit switches to URDF system state. Update Hardware Interface to reflect these values. Find out where they're represented in the global ROS system state~~ 
1.  Allow for calibration at any point.
    1. Calibration on either side? 
1.  Get X-box controller to move slider with L2/R2 
1.  Check if `on_deactivate()` runs for linear_slider_hardware when control-c is hit. 
1.  Refactor linear slider bringup to take a robot arm as launch argument, build subsequent launch files and URDFs from there.  
1.  Let the high-level launch pass the URDF to the MoveIt launch.... 
1.  Add polling for E-stop for reset-detection (i.e. can we un-set the estop and resume operation without restarting the controller?). 
1.  Create centralized velocity and position limits. Make sure each interface receives them. 
1.  SHUTDOWN SAFTETY
    1.  Code shutdown 
    1.  Physical link to UR5 (can be wired in --> Jostan)   
1.  Merge ur_with_linear_slider into linear_slider as launch option


# UR Launch arguments
```
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/64d44d4c345ceaaa4e02fa4baa21c80d1a5fbce7/ur_robot_driver/src/hardware_interface.cpp#L309
```

# Linear Slider Drivers

This is a repository that includes packages for the hardware description, hardware interface, and controller interface for the linear slider. Each package within this repository includes their own README for more detailed explanations.

## Package Contents

`linear_slider_bringup`: Launches all of the interfaces.

`linear_slider_controllers`: Manages the control node for interfacing with ROS2, hosts custom controller configurations and implementations.

`linear_slider_description`: Defines the geometry and links in a URDF file. Also specifies controller interfaces through `<ros2_control>` tag.

`linear_slider_hardware_interface`: Defines the hardware and controller interfaces for the linear slider

`linear_slider_moveit`: Defines the interface by assigning it to a MoveIt controller "Group" so that path motion planning can easily be done in RViz. This package was generated using the MoveIt Setup Assistant and should not be edited directly.

`linear_slider_test`: Provides an ament_python environment for writing simple scripts to test the functionality of all linear slider packages.

## Running the linear slider

### Simulation

#### Mock Hardware

Build the packages and run the following command:

```
ros2 launch linear_slider_bringup linear_slider.launch.py use_mock_hardware:=true mock_sensor_commands:=true
```

This command should load the mock hardware interface and assign a Joint Trajectory Controller to it.

The joint trajectory controller can be tested by running either of the following commands:

```
# GUI option
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

# Action server option
ros2 run linear_slider_test test_joint_trajectory_controller_node
```

#### Gazebo Classic Simulation

Launching the linear slider in Gazebo Classic can be done with the following command.

```
ros2 launch linear_slider_bringup linear_slider_sim_gazebo.launch.py sim_gazebo_classic:=true
```

#### Gazebo Ignition Simulation

Gazebo Ignition functionality not available at this time. If you wish to view the current bugs with this launch, please run the following command:

```
ros2 launch linear_slider_bringup linear_slider_sim_gazebo.launch.py sim_gazebo:=true
```

### Real-world

- Not available at this time.

Once the hardware interface is properly configured, the system will be able to be launched using the following command:

```
ros2 launch linear_slider_bringup linear_slider.launch.py
```

#### Actual hardware

The real-world launch requires a proper hardware setup and safety configuration.

Communication: The `linear_slider_hardware_interface` communicates with a Teknic ClearCore microcontroller via a UDP Ethernet interface.

Safety: The ClearCore microcontroller is attached to an emergency stop that should interrupt the controller's main execution loop at any point. The slider also implements two optical safety stops at the limits of the slider. Be sure these are operational before running the linear slider.

## Testing the linear slider

Each of the packages has a set of launch files that help guide the creation of your drivers. A brief description of each of the test files is below:

1. `linear_slider_description/launch/view_robot.launch.py` -- Spawns a `joint_state_publisher_gui` window, allowing the user to monitor the URDF's build process.

2. `linear_slider_bringup/launch/linear_slider.launch.py` -- Spawns the `/controller_manager` node, allowing the activation of various controller libraries. This also links the controller interface to the hardware interface.

## Linear Slider Description:

An xacro file can be easily checked using the following command line argument:

```
xacro model.urdf.xacro > tmp.urdf && check_urdf tmp.urdf
```

Default state interface values can be found in `linear_slider_description/config/initial_state.yaml`.

Under the `<ros2_control>` tag, there are two [ros2_control hardware interface types](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html), **Joints** and **Sensors**. The `<joint>` tag defines the state and command interfaces for the controllers defined in `linear_slider_controllers/config/linear_slider_controllers.yaml`.

## Linear Slider Bringup:

The `linear_slider_bringup` package links together the various `ros2_control` implementations. First, it forms a working URDF generated from the xacro files defined in `linear_slider_description`. Parameters are passed from the bringup launch files to the xacro executable, which allows the correct hardware interface plugin to be selected.

### Linear Slider Description: Defines the physical robot. This includes STL meshes, URDFs, and initial state information.

Things to look at:

```
https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html
```

## Hardware Interface

The hardware interface for the linear slider translates world frame commands into motor commands and sends them via the UDP Ethernet communication interface to the ClearCore Microcontroller. More detail on the package can be found within the package's README.

## Controller Interface

The current controller interface only relies on generalized controller libraries from ros2_control. Additional controllers can be defined in the `linear_slider_controllers` package.

### Generalized controllers

[Commonly used controllers for the ros2_control framework](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html) are well set up to be used "out of the box" by MoveIt2 and Nav2.

### Custom controllers

[The main documentation](https://control.ros.org/humble/doc/ros2_controllers/doc/writing_new_controller.html) describes writing a new controller interface. Other good examples include the [Universal Robots ROS2 Drivers](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).

## MoveIt

The construction of the MoveIt package is still underway, as the URDF should be fully-functional before proceeding with this step. While the URDF seems mostly done, issues with Gazebo currently point to errors in the URDF.

## Test

The `linear_slider_test` package provides an ament_python environemtn for testing various packages of the linear slider. Currently, it features a joint_trajectory_controller interface by sending positional goals for the JointTrajectoryController action interface.
