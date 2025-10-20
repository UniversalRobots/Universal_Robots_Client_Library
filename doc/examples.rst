:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples.rst

Usage examples
==============

This library contains a number of examples how this library can be used. You can use them as a
starting point for your own applications.

All examples take a robot's IP address as a first argument and some of them take a maximum run
duration (in seconds) as second argument. If the second argument is omitted, some of the examples
may be running forever until manually stopped.

.. note:: Most of these examples use the driver's headless mode. Therefore, on an e-Series (PolyScope 5) robot, the robot has to be
   in *remote control mode* to work.

.. toctree::
   :maxdepth: 1

   examples/dashboard_client
   examples/force_mode
   examples/freedrive
   examples/instruction_executor
   examples/primary_pipeline
   examples/primary_pipeline_calibration
   examples/rtde_client
   examples/external_fts_through_rtde
   examples/script_command_interface
   examples/script_sender
   examples/spline_example
   examples/tool_contact_example
   examples/direct_torque_control
   examples/trajectory_point_interface
   examples/ur_driver
