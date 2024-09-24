# Linear Slider Controllers

## Custom Controllers

### Limit Switch State Broadcaster
The LimitSwitchStateBroadcaster is a general broadcast controller that reports the boolean status of our limit switches. 

### Linear Slider Controller
The LinearSliderController is an extension of the default JointTrajectoryController from `ros2_control`. It is a velocity-controlled controller. PID values can be set in the yaml config file.

### 

## Pesky Bugs
If the controller `update_rate` too is too low, ...