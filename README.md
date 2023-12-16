# Linear Slider Drivers

This is a package that includes the hardware description, hardware interface, and controller interface for the linear slider.

## Package Contents

### Linear Slider Control: Manages the control node for interfacing with ROS2 core.

### Linear Slider Controller: Defines the hardware and controller interfaces for the linear slider

A basic tutorial of the interfaces is as follows:

#### Hardware Interface

`on_init()`: runs at the startup, reads parameters, allocates memory, etc. Hardware enters unconfigured state.

`on_configure()`: Establish comms. Takes us to the inactive state

`on_activate()`: Engage actuators. Takes us to the active state. Here, we can send read/write commands.

`on_deactivate()`: Disengage actuators. Takes us back to the inactive state.

`on_cleanup()`: Takes us from the inactive state to the unconfigured state, disabling comms.

`on_shutdown()`: Shuts down the whole thing (gracefully).

### Linear Slider Description: Defines the physical robot. This includes STL meshes, URDF and SRDF files, MoveIt configurations, etc.

Things to look at:

```
https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html
```