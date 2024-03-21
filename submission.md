# Notes about the submission

<ol>
    <li> Each package contains its own README, and there also exists a high-level README for the repository.
    <li> Gazebo doesn't work.
    <li>Velocity controller doesn't work
    <li>This build process needs a better way to check for errors in the build.
</ol>

### Each package contains its own README.
For organizational purposes, each package within this repository contains its own README. Please read each README to get a full understanding of the package.

### Gazebo doesn't work
I cannot get Gazebo Classic to correctly read in the joint, and the controllers are unable to load. A more detailed explanation of this bug can be [found in this Robotics Stack Exchange post](https://robotics.stackexchange.com/questions/110116/gazebo-ros2-control-does-not-load-joints-or-sensors-from-urdf) that I made.

### Velocity controller doesn't work
The motor I have only accepts position/position ***or*** velocity/velocity command/state interfaces. I originally began this project using a velocity/velocity interface, but publishing to the `/velocity_controller/commands` topic did not work for some reason. I switched to position/position so that I could interface with the popular Joint Trajectory Controller and things began working in my Mock Hardware interface.

### This build process needs a better way...
There are quite a few times where bugs are not readily apparent, making debugging the system a nightmare. This is especially true once passing conditional arguments and initial value parameters to `xacro` through a launch file. Not only could the debug system be improved, but I found myself repeating quite a few of the interface types and names, writing them in xacro files, yaml files, and C++ code for the hardware interface. While I haven't brainstormed how, it feels like this process could potentially be streamlined.