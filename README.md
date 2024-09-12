# Universal Robots ROS 2 tutorials
This package contains tutorials around the ROS 2 packages for Universal Robots.

## Getting started
To use the tutorials from this repository, please make sure to [install ROS
2](https://docs.ros.org/en/rolling/Installation.html) on your system. Currently, only ROS Jazzy and
Rolling are supported.

With that, please create a workspace, clone this repo into the workspace, install the dependencies
and build the workspace.

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspaces/ur_tutorials
   mkdir -p $COLCON_WS/src
   ```

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials.git src/ur_tutorials
   rosdep update && rosdep install --ignore-src --from-paths src -y
   ```

1. Create a colcon workspace:
   ```
   cd $COLCON_WS
   colcon build
   ```

1. Source your workspace
   ```
   source $COLCON_WS/install/setup.bash
   ```

1. Launch an example
   e.g. the custom workcell example
   ```
   ros2 launch my_robot_cell_control start_robot.launch.py use_mock_hardware:=true
   ```
