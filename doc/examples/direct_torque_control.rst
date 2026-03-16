:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/direct_torque_control.rst

.. _direct_torque_control_example:

Torque control example
======================

In the ``examples`` subfolder you will find a minimal example for commanding torques to the robot.
It applies a sinusoidal torque to the 6th joint (index 5) while reading the joint positions. To run
it make sure to

* have an instance of a robot controller / URSim running at the configured IP address (or adapt the
  address to your needs)
* have PolyScope version 5.25.1 / 10.12.1 or later installed on the robot.
* run it from the package's main folder (the one where the README.md file is stored), as for
  simplicity reasons it doesn't use any sophisticated method to locate the required files.

This page will walk you through the `direct_torque_control.cpp
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/direct_torque_control.cpp>`_
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
   This example requires PolyScope version 5.25.1 / 10.12.1 or later, as it uses
   ``setFrictionScales()`` which is only available in these versions and later. If you try to run it
   on an older software version, this example will skip gracefully.

Friction compensation setup
----------------------------

Before starting the control loop, friction compensation scales are configured using
``setFrictionScales()``. Since this example only controls a single joint (``JOINT_INDEX``), friction
compensation for the other joints is set to zero to keep them more steady. The controlled joint
gets full compensation (scale factor 1.0) for both viscous and Coulomb friction.

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Scale each individual joint's friction compensation
   :end-at: setFrictionScales(viscous_scale, coulomb_scale);

Move to start position
----------------------

Before entering the torque control loop, the robot is moved to a known start position using
the ``InstructionExecutor``. This ensures a predictable initial state for the sinusoidal motion.
First, RTDE communication is started and the current joint positions are read. Then, the target
position for ``JOINT_INDEX`` is set to ``start_position`` and the robot is moved there using
``moveJ()``.

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Once RTDE communication is started
   :end-at: instruction_executor->moveJ(g_joint_positions, 1.4, 1.04, 0.1);

Robot control loop
------------------

The main control loop applies a sinusoidal torque to the controlled joint for 10 full periods.
In each cycle, the latest RTDE data package is read using ``getDataPackage()``. This call blocks
until new data arrives from the robot (with an internal timeout), so the robot's RTDE cycle
effectively controls the timing of this loop. The current joint positions are then extracted from
the received package:

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Run 10 periods of the sinusoidal
   :end-before: // Open loop control

The target torque for the controlled joint is computed as a sinusoidal function of time:

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Open loop control
   :end-at: target_torques[JOINT_INDEX] = amplitude * std::sin(omega * time);

To send the control command, the robot's :ref:`reverse_interface` is used via the
``writeJointCommand()`` function:

.. literalinclude:: ../../examples/direct_torque_control.cpp
   :language: c++
   :caption: examples/direct_torque_control.cpp
   :linenos:
   :lineno-match:
   :start-at: // Setting the RobotReceiveTimeout
   :end-before: URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg.toString().c_str());
