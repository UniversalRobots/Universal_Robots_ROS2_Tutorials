# Universal Robots ROS 2 tutorials
This package contains tutorials around the ROS 2 packages for Universal Robots.

## Branches & ROS distributions
This package intends to show use-cases and examples for the ROS 2 packages for Universal Robots.
The main branch of this repository will be used to show examples for the latest ROS 2 distribution,
ROS Rolling. For older ROS 2 distributions. This is valid for all active ROS 2 distributions, as
long as there is no specialized branch for that distribution. The following table shows the
branches and their corresponding ROS 2 distributions:

| ROS 2 Distro | Branch                                                                                   | Documentation                                                                                                                                                                        |
|--------------|------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Rolling**  | [main](https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/main)     | [![Documentation](https://img.shields.io/badge/docs-latest-blue)](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_tutorials/tutorial_index.html) |
| **Jazzy**    | [jazzy](https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/jazzy)   | [![Documentation](https://img.shields.io/badge/docs-jazzy-blue)](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_tutorials/tutorial_index.html)    |
| **Humble**   | [humble](https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/humble) | --                                                                                                                                                                                   |

## Getting started
To use the tutorials from this repository, please make sure to [install ROS
2](https://docs.ros.org/en/rolling/Installation.html) on your system.

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
