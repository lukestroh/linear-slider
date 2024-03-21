# Notes about the submission

<ol>
    <li> Each package contains its own README, and there also exists a high-level README for the repository.
    <li> Gazebo Ignition doesn't work
    <li>Velocity controller doesn't work
    <li>This build process needs a better way to check for errors in the build.
    <li>Actual hardware not safe enough
</ol>

### Each package contains its own README.
For organizational purposes, each package within this repository contains its own README. Please read each README to get a full understanding of the package.

### Gazebo Ignition doesn't work
Ignition cannot be installed alongside Gazebo Classic, and therefore I focused only on Gazebo Classic.

### Velocity controller doesn't work
The motor I have only accepts position/position ***or*** velocity/velocity command/state interfaces. I originally began this project using a velocity/velocity interface, but publishing to the `/velocity_controller/commands` topic did not work for some reason. I switched to position/position so that I could interface with the popular Joint Trajectory Controller and things began working in my Mock Hardware interface.

### This build process needs a better way...
There are quite a few times where bugs are not readily apparent, making debugging the system a nightmare. This is especially true once passing conditional arguments and initial value parameters to `xacro` through a launch file. Not only could the debug system be improved, but I found myself repeating quite a few of the interface types and names, writing them in xacro files, yaml files, and C++ code for the hardware interface. While I haven't brainstormed how, it feels like this process could potentially be streamlined.

### Actual hardware not safe enough
Though I have implemented an emergency stop button, I still have yet to install the limit switches for the slider. Without these switches, I do not feel comfortable running the slider, especially with the arm on it.