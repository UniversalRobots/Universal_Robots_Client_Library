:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/ur_driver.rst

.. _example-driver:

Example driver
==============
In the ``examples`` subfolder you will find a minimal example of a running driver. It starts an
instance of the ``UrDriver`` class and prints the RTDE values read from the controller. To run it make
sure to

* have an instance of a robot controller / URSim running at the configured IP address (or adapt the
  address to your needs)
* run it from the package's main folder (the one where the README.md file is stored), as for
  simplicity reasons it doesn't use any sophisticated method to locate the required files.

This page will walk you through the `full_driver.cpp
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/full_driver.cpp>`_
example.

Initialization
--------------

At first, we create a ``ExampleRobotWrapper`` object giving it the robot's IP address, script file and RTDE
recipes.

.. literalinclude:: ../../examples/full_driver.cpp
   :language: c++
   :caption: examples/full_driver.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-at: // --------------- INITIALIZATION END -------------------

Robot control loop
------------------

This example reads the robot's joint positions, increments / decrements the 5th axis and sends that
back as a joint command for the next cycle. This way, the robot will move its wrist first until a
positive limit and then back to 0.

To read the joint data, the driver's RTDE client is used:


.. literalinclude:: ../../examples/full_driver.cpp
   :language: c++
   :caption: examples/full_driver.cpp
   :linenos:
   :lineno-match:
   :start-at: // Once RTDE communication is started
   :end-before: // Open loop control

The first read operation will initialize the target buffer with the current robot position. Next,
the target joint configuration is calculated:


.. literalinclude:: ../../examples/full_driver.cpp
   :language: c++
   :caption: examples/full_driver.cpp
   :linenos:
   :lineno-match:
   :start-at: // Open loop control
   :end-at: joint_target[5] += increment

To send the control command, the robot's :ref:`reverse_interface` is used via the
``writeJointCommand()`` function:

.. literalinclude:: ../../examples/full_driver.cpp
   :language: c++
   :caption: examples/full_driver.cpp
   :linenos:
   :lineno-match:
   :start-at: // Setting the RobotReceiveTimeout
   :end-before: URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg->toString().c_str());
