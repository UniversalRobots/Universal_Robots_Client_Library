:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/direct_torque_control.rst

.. _direct_torque_control_example:

Torque control example
======================

In the ``examples`` subfolder you will find a minimal example for commanding torques to the robot.
It moves the robot in the 5th axis back and forth, while reading the joint positions. To run it
make sure to

* have an instance of a robot controller / URSim running at the configured IP address (or adapt the
  address to your needs)
* have PolyScope version 5.23.0 / 10.10.0 or later installed on the robot.
* run it from the package's main folder (the one where the README.md file is stored), as for
  simplicity reasons it doesn't use any sophisticated method to locate the required files.

This page will walk you through the `full_driver.cpp
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/full_driver.cpp>`_
example.

Initialization
--------------

At first, we create a ``ExampleRobotWrapper`` object giving it the robot's IP address, script file and RTDE
recipes.

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-at: // --------------- INITIALIZATION END -------------------

.. note::
   This example requires PolyScope version 5.23.0 / 10.10.0 or later, as it uses the direct_torque_control
   mode which is only available in these versions and later. If you try to run it on an older
   software version, this example will print an error and exit.

Robot control loop
------------------

This example reads the robot's joint positions, commands a torque for the 5th axis and sends that
back as a joint command for the next cycle. This way, the robot will move its wrist first until a
positive limit and then back to 0.

To read the joint data, the driver's RTDE client is used:


.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Once RTDE communication is started
   :end-before: // Open loop control

The first read operation will initialize the target buffer with the current robot position. Next,
the target joint torques are set based on the current joint positions:


.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Open loop control
   :end-at: target_torques[JOINT_INDEX] = cmd_torque;

To send the control command, the robot's :ref:`reverse_interface` is used via the
``writeJointCommand()`` function:

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Setting the RobotReceiveTimeout
   :end-before: URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg->toString().c_str());
