# Linear Slider Description

## Description package for the Linear Slider

This package provides a URDF description for the Linear Slider. To view the basic linear slider, build this package and launch the `view_linear_slider.launch.py` file using 

```
ros2 launch linear_slider_description view_linear_slider.launch.py
```
The linear slider can be moved around with the `joint_state_publisher_gui` window.

Edits to the design of the linear slider should be done in your preferred CAD and exported as STL files to `linear_slider_description/meshes/`. If the names of these files change, they must also be updated in the `linear_slider_macro.xacro` file, under `linear_slider_description/urdf/linear_slider/linear_slider_macro.xacro`.

Similarly, hardware parameters and joint configurations are set in the `linear_slider_macro.xacro file`.

Edits to the ros2_control hardware interface description should be made under `linear_slider_description/urdf/linear_slider/linear_slider.ros2_control.xacro`.