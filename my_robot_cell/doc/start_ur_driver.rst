=========================
Start the ur_robot_driver
=========================

In the last chapter we created a custom scene description containing our robot. In order to make
that description usable to ``ros2_control`` and hence the ``ur_robot_driver``, we need to add
control information. We will also add a custom launchfile to startup our demonstrator.

We will generate a new package called `my_robot_cell_control <https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/tree/main/my_robot_cell/my_robot_cell_control>`_ for that purpose.


Create a description with ros2_control tag
------------------------------------------

The first step is to create a description containing the control instructions:

.. literalinclude:: ../my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro
    :language: xml
    :linenos:
    :caption: my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro

This URDF is very similar to the one we have already assembled. We simply need to include the ros2_control macro,

.. literalinclude:: ../my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro
    :language: xml
    :start-at:   <xacro:include filename="$(find ur_description)/urdf/ur.ros2_control.xacro"/>
    :end-at:   <xacro:include filename="$(find ur_description)/urdf/ur.ros2_control.xacro"/>
    :caption: my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro


define the necessary arguments that need to be passed to the macro,

.. literalinclude:: ../my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro
    :language: xml
    :start-at: <xacro:arg name="robot_ip" default="0.0.0.0"/>
    :end-at: <xacro:arg name="fake_sensor_commands" default="false" />
    :caption: my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro


and then call the macro by providing all the specified arguments.

.. literalinclude:: ../my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro
    :language: xml
    :start-at:    <xacro:ur_ros2_control
    :end-at:   />
    :caption: my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro

Extract the calibration
-----------------------

One very important step is to extract the robot's specific calibration and save it to our
workcell's startup package. For details, please refer to `our documentation on extracting the calibration information <https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/robot_setup.html#extract-calibration-information>`_.
For now, we just copy the default one for the ur20.

.. code-block::

   cp $(ros2 pkg prefix ur_description)/share/ur_description/config/ur20/default_kinematics.yaml \
     my_robot_cell_control/config/my_robot_calibration.yaml

Create start_robot launchfile
-----------------------------

To launch a controlled robot with our custom description, we need to generate a launchfile. We
include the ``ur_control.launch.py`` file from the driver and set its parameters to match our
custom workcell.

.. literalinclude:: ../my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :linenos:
    :caption: my_robot_cell_control/launch/start_robot.launch.py

With that we can start the robot using

.. code-block:: bash

   ros2 launch my_robot_cell_control start_robot.launch.py use_fake_hardware:=true

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
    ros2 launch my_robot_cell_control start_robot.launch.py use_fake_hardware:=true

Or to use it with a real robot:

.. code-block:: bash

    #start the driver with real hardware
    ros2 launch my_robot_cell_control start_robot.launch.py
