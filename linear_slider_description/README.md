# Linear Slider Description

This package provides a URDF description for the Linear Slider and is created using the XML Macro language (`xacro`).

## Viewing the URDF
To view the basic linear slider, build this package and launch the `view_linear_slider.launch.py` file using 

```
ros2 launch linear_slider_description view_linear_slider.launch.py
```
The linear slider can be moved around with the `joint_state_publisher_gui` window.

## Testing the URDF

The URDF can be tested for errors using the `xacro` CLI command:
```
xacro model.urdf.xacro > tmp.urdf && check_urdf tmp.urdf
```
This command should inform the user where the various bugs are. However, there are times when it fails to catch some bugs. One notable area is the loading of YAML parameter files. This description package imports an `initial_state.yaml` file from `linear_slider_description/config/initial_state.yaml`, and provides initial state values for the joint positions and the sensors. These initial sensor values cannot describe the state as `False` -- doing so will generate the error `what():  Failed converting string to real number`. This initial state value must be an integer or a double. This bug is not reported when running the `xacro` command, but rather only when the `ros2_control_node` attempts to parse the already-formed URDF.

## URDF/Xacro Syntax

Lots of information about URDF construction can be gathered from the [official ROS website](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File), though there are often gaps in the documentation.

### Macros
Xacro macros are useful tools for separating out sections of your URDF. The highest-level xacro file defined in this package is `linear_slider_description/urdf/linear_slider.urdf.xacro`. Taking a look at this file, we can see that there are some arguments defined at the top, followed by some `<xacro:include>` tags. The contents of the files included by these tags are then populated when the macro is called below with additional arguments. For example, the tags defining the `ros2_control` interface are located within a macro named `<xacro:linear_slider_ros2_control>`.

### Conditional `xacro`
Further down the file, we see that different versions of Gazebo can be launched depending on the xacro arguments, which can be passed in via a launch file or the command line. This is done through the use of the `<xacro:if>` tag. An opposite `<xacro:unless>` tag can be specified when desiring false values.

### Arguments and Parameters
Arguments are globally accessible from within any included file or macro, and are accessed via `$(arg <arg_name>)`.

Parameters are values passed to xacro macros and are accessed via `${<param_name>}`. Similar to arguments, default values can be set for each parameter. This takes place within the macro definition like the one in `linear_slider_description/urdf/linear_slider/linear_slider.ros2_control.xacro`.

When importing higher level parameters from a launch file, the caret `^` indicates to use the outer-scope property (with same name). The pipe `|` indicates to use the given fallback if the property is not defined in the outer-scope.

### ros2_control
The `ros2_control` tag provides a definition to describe the hardware definition. For a custom definition like the one in `linear_slider_hardware_interface`, it must be exported using the pluginlib library and called in this block using the `<plugin>` tag. The ros2_control framework also provides a `mock_components/GenericSystem` interface that extends a fake interface to the main ROS2 system, allowing developers a chance to debug their URDF and their controllers. Finally, Gazebo can extend an interface to ros2_control, allowing users to link the controllers and the hardware within a simulated environment.


## Editing the model

Edits to the design of the linear slider should be done in your preferred CAD and exported as STL files to `linear_slider_description/meshes/`. If the names of these files change, they must also be updated in the `linear_slider_macro.xacro` file, under `linear_slider_description/urdf/linear_slider/linear_slider_macro.xacro`.

If editing the design, be sure to update the `<collision>` and `<inertial>` tags in the `linear_slider_description/urdf/linear_slider/linear_slider_macro.xacro` file. These will help a physics simulator such as Gazebo to properly update the model. Similarly, additional hardware parameters and joint configurations are set in the `linear_slider_macro.xacro file`.

The hardware interface definition is not in the `linear_slider_macro.xacro` file. Instead, edits to the ros2_control hardware interface description should be made under `linear_slider_description/urdf/linear_slider/linear_slider.ros2_control.xacro`.