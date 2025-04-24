:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/ur_driver.rst

.. _ur_driver:

UrDriver
========

The ``UrDriver`` class is the core of this library. It creates an interface to the robot containing
all components from this library.

When creating an instance of the ``UrDriver`` class, the created object will contain

- a :ref:`DashboardClient <dashboard_client>`
- a :ref:`ReverseInterface <reverse_interface>`
- an :ref:`RTDEClient <rtde_client>`
- a :ref:`ScriptCommandInterface <script_command_interface>`
- a :ref:`ScriptSender <script_sender>`
- a :ref:`TrajectoryPointInterface <trajectory_point_interface>`.
- a rudimentary client to the robot's primary interface
- a couple of helper functions

As this page is not meant to be a full-blown API documentation, not every public method will be
explained here. For a full list of public methods, please inspect class definition in the
`ur_client_library/ur/ur_driver.h
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/include/ur_client_library/ur/ur_driver.h>`_
header file.

Many functions from the individual components are directly wrapped in the ``UrDriver`` class, for
example there is ``UrDriver::startForceMode(...)`` which is a wrapper around the
``ScriptCommandInterface::startForceMode(...)`` function.

As an example on how to use this class, please see the `full_driver.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/full_driver.cpp>`_ example.

Public helper functions
-----------------------

``checkCalibration(const std::string checksum)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This function opens a connection to the primary interface where it will receive a calibration
information as the first message. The checksum from this calibration info is compared to the one
given to this function. Connection to the primary interface is dropped afterwards.

``sendScript(const std::string& program)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This function sends given URScript code directly to the primary interface. The
``sendRobotProgram()`` function is a special case that will send the script code given in the
``RTDEClient`` constructor.



