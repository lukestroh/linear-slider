# Linear Slider Hardware Interface

This package defines the UDP communication with the external microcontroller, and presents a custom `LinearSliderHardware` class to store the state of the system. The class also translates the rpm of the motor the horizontal distance of the `moving_base` and vice versa. The class is an attribute within the high-level `LinearSliderSystemInterface` class.

This high-level class inherits the boiler-plate ros2_control `hardware_interface::SystemInterface` class, where several class methods must be overwritten:

* `on_init()`: runs at the startup, reads parameters, allocates memory, etc. Hardware enters unconfigured state.

* `on_configure()`: Establish comms. Takes us to the inactive state

* `on_activate()`: Engage actuators. Takes us to the active state. Here, we can send read/write commands.

* `on_deactivate()`: Disengage actuators. Takes us back to the inactive state.

* `on_cleanup()`: Takes us from the inactive state to the unconfigured state, disabling comms.

* `on_shutdown()`: Shuts down the whole thing (gracefully).

* `read()`: Read the value from the external device.

* `write()`: Write a value to the external device.

These methods provide us with a simple way to engage, communicate, and disengate with our external hardware. Much of the functionality takes place within the `read` and `write` methods. The values are stored in `LinearSliderSystemInterface::linear_slider_` and eventually published to the `/joint_states` topic.

### Interface 
The official documentation provides a useful diagram of [the architecture of `ros2_control`](https://control.ros.org/master/doc/getting_started/getting_started.html#architecture).

<figure>
    <center>
        <img src="../images/components_architecture.png" width=400>
    </center>
</figure>

The Controller Manager provides an interface for loading, activating, deactivating, and unloading various controllers. It publishes several service types to allow for this, and these service topics can also be called using the `ros2 control` command line interface. A detailed description on how to implement a hardware interface can be [found in the documentation](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html), and there are good examples provided by the [ROSCon Workshop](https://github.com/ros-controls/roscon2022_workshop/tree/master/controlko_hardware_interface) and [Articulated Robotics](https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/).