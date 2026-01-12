:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/blob/main/my_dual_robot_cell/doc/start_ur_driver.rst

=========================
Start the ur_robot_driver
=========================

In the last chapter we created a custom scene description containing our two robots. In order to make
that description usable to ``ros2_control`` and hence the ``ur_robot_driver``, we need to add
control information. We will also add a custom launchfile to start up our demonstrator.

We will generate a new package called `my_dual_robot_cell_control <https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/main/my_dual_robot_cell/my_dual_robot_cell_control>`_ for that purpose.

Create a description with ros2_control tag
------------------------------------------

The first step is to create a description containing the control instructions:

.. literalinclude:: ../my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro
    :language: xml
    :linenos:
    :caption: my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro

This URDF is very similar to the one we have already assembled. We simply need to include the ros2_control macro,

.. literalinclude:: ../my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro
    :language: xml
    :start-at:   <xacro:include filename="$(find ur_robot_driver)/urdf/ur.ros2_control.xacro" />
    :end-at:   <xacro:include filename="$(find ur_robot_driver)/urdf/ur.ros2_control.xacro"/>
    :caption: my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro

define the necessary arguments that need to be passed to the macro,

.. literalinclude:: ../my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro
    :language: xml
    :start-at: <xacro:arg name="alice_use_mock_hardware" default="false" />
    :end-at: <xacro:arg name="ur_input_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_input_recipe.txt" />
    :caption: my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro

and then call the macro by providing all the specified arguments.

.. literalinclude:: ../my_dual_robot_cell_control/urdf/my_dual_robot_cell_controlled.urdf.xacro
    :language: xml
    :start-at:    <xacro:ur_ros2_control
    :end-before:   </robot>
    :caption: my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro

Extract the calibration
-----------------------

One very important step is to extract each robot's specific calibration and save it to our
workcell's startup package. For details, please refer to `our documentation on extracting the calibration information <https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/robot_setup.html#extract-calibration-information>`_.
For now, we just copy the default ones for the UR3e and UR5e.

.. code-block::

   cp $(ros2 pkg prefix ur_description)/share/ur_description/config/ur3e/default_kinematics.yaml \
     my_dual_robot_cell_control/config/alice_calibration.yaml

    cp $(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/default_kinematics.yaml \
     my_dual_robot_cell_control/config/bob_calibration.yaml


Create robot_state_publisher launchfile
---------------------------------------

To use the custom controlled description, we need to generate a launchfile loading that description
(Since it contains less / potentially different) arguments than the "default" one. In that
launchfile we need to start a ``robot_state_publisher`` (RSP) node that will get the description as a
parameter and redistribute it via the ``robot_description`` topic:

.. literalinclude:: ../my_dual_robot_cell_control/launch/rsp.launch.py
    :language: py
    :start-at: from launch import LaunchDescription
    :linenos:
    :caption: my_dual_robot_cell_control/launch/rsp.launch.py

Before we can start our workcell we need to create a launchfile that will launch the two robot descriptions correctly.

Create start_robot launchfile
-----------------------------

Here we create a launch file which will launch the driver with the correct description and robot state publisher, as well as start all of the controllers necessary to use the two robots.

.. literalinclude:: ../my_dual_robot_cell_control/launch/start_robots.launch.py
    :language: py
    :linenos:
    :caption: my_dual_robot_cell_control/launch/start_robots.launch.py

With that we can start the robot using

.. code-block:: bash

   ros2 launch my_dual_robot_cell_control start_robots.launch.py alice_use_mock_hardware:=true bob_use_mock_hardware:=true

Testing everything
------------------

Before we can test our code, it's essential to build and source our Colcon workspace:

.. code-block:: bash

    #cd to your colcon workspace root, e.g.
    cd ~/colcon_ws

    #source and build your workspace
    colcon build
    source install/setup.bash

We can start the system in a mocked simulation

.. code-block:: bash

    #start the driver with mocked hardware
    ros2 launch my_dual_robot_cell_control start_robots.launch.py alice_use_mock_hardware:=true bob_use_mock_hardware:=true

Or to use it with a real robot:

.. code-block:: bash

    #start the driver with real hardware (or the docker simulator included in this example)
    ros2 launch my_dual_robot_cell_control start_robots.launch.py

.. note::
   We extracted the calibration information from the robot and saved it in the
   ``my_robot_cell_control`` package. If you have a different robot, you need to extract the
   calibration information from that, and pass that to the launch file. Changing the ``ur_type``
   parameter only will lead to a wrong model, as it will still be using the example kinematics from
   a UR3e and UR5e. To use a UR10e and UR20, for example, you can do

   .. code-block:: bash

      ros2 launch my_dual_robot_cell_control start_robots.launch.py bob_ur_type:=ur10e bob_use_mock_hardware:=true \
        bob_kinematics_parameters_file:=$(ros2 pkg prefix ur_description)/share/ur_description/config/ur10e/default_kinematics.yaml \
        alice_ur_type:=ur20 alice_use_mock_hardware:=true \
        alice_kinematics_parameters_file:=$(ros2 pkg prefix ur_description)/share/ur_description/config/ur20/default_kinematics.yaml
