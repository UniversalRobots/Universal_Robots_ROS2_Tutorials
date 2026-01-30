:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/blob/main/my_dual_robot_cell/doc/index.rst

.. _dual_arm_tutorial:

===================
Dual robot arm cell
===================

Example about integrating two UR arms into a custom workspace. We will build a custom workcell
description containing two robot arms, start the driver and create a MoveIt config to enable trajectory planning with MoveIt.

Please see the `package source code
<https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/main/my_robot_cell>`_ for
all files relevant for this example.

.. toctree::
   :maxdepth: 4
   :caption: Contents:

   assemble_urdf
   start_ur_driver

MoveIt! Configuration
---------------------
For inspiration on how to configure MoveIt! 2 using the setup assistant, please see the `single arm robot cell tutorial
<https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_tutorials/my_robot_cell/doc/build_moveit_config.html>`_
