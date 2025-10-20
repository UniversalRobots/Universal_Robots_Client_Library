:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/external_fts_through_rtde.rst

External FTS example
====================

It is possible to feed data of an external Force-Torque-Sensor (FTS) into the robot controller
through RTDE. This feature has to be explicitly enabled by calling the URScript function
`ft_rtde_input_enable
<https://www.universal-robots.com/manuals/EN/HTML/SW10_10/Content/prod-scriptmanual/all_scripts/ft_rtde_input_enable.htm?Highlight=ft_rtde_input_enable>`_.

Usually, that functionality would be used with a URCap running on the robot but data can also be
routed through an external computer with a matching driver for that FTS. Keep in mind that the
latency introduced by the network connection may have a negative impact on the performance.

Another use case could be to inject force-torque measurements into a simulated robot for testing
purposes.

This example allows setting force values that are sent to the robot as external ft measurements.
The terminal user interface (TUI) is pretty simple:

- Pressing the keys ``x``, ``y``, ``z``, ``a``, ``b``, or ``c`` will increase the respective
  force/torque component by 10 (N or Nm). (``a``, ``b``, and ``c`` correspond to torques around the
  x, y, and z axis respectively.)
- Pressing the keys ``X``, ``Y``, ``Z``, ``A``, ``B``, or ``C`` will decrease the respective
  force/torque component by 10 (N or Nm).
- Pressing ``0`` will reset all force/torque components to zero.
- Pressing ``q`` will exit the program.

The example's source code can be found in `external_fts_through_rtde.cpp
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/external_fts_through_rtde.cpp>`_.


.. note:: This example requires the robot to be in *remote control mode*.

Robot initialization
--------------------

The robot initialization is handed off to the ``ExampleRobotWrapper`` class:

.. literalinclude:: ../../examples/external_fts_through_rtde.cpp
   :language: c++
   :caption: external_fts_through_rtde/external_fts_through_rtde.cpp
   :linenos:
   :lineno-match:
   :start-at: g_my_robot = std::make_unique
   :end-before: // Enable using the force-torque

In order to use the RTDE input for the FTS, we have to enable it first by calling the
``ft_rtde_input_enable`` function. This is done by sending a script to the robot through the
``ftRtdeInputEnable()`` method of the ``UrDriver`` class.

.. literalinclude:: ../../examples/external_fts_through_rtde.cpp
   :language: c++
   :caption: external_fts_through_rtde/external_fts_through_rtde.cpp
   :linenos:
   :lineno-match:
   :start-at: // Enable using the force-torque
   :end-at: g_my_robot->getUrDriver()->ftRtdeInputEnable(true);

RTDE communication is moved to a separate thread in order to use the main thread for handling
keyboard input commands.

Input key handling
------------------

For handling the keyboard input, define a ``getChar()`` function that reads a single character from
the terminal without waiting for a newline character. This has to be done differently on Windows
and Linux (and other unix-like systems), therefore the code is split using preprocessor directives.

.. literalinclude:: ../../examples/external_fts_through_rtde.cpp
   :language: c++
   :caption: external_fts_through_rtde/external_fts_through_rtde.cpp
   :linenos:
   :lineno-match:
   :start-at: // Platform-specific implementation of getChar()
   :end-at: #endif

Setting the external force-torque values
----------------------------------------

The result from the tui function is synchonized to the RTDE communication thread using a buffer ``g_FT_VEC`` and a mutex.

.. literalinclude:: ../../examples/external_fts_through_rtde.cpp
   :language: c++
   :caption: external_fts_through_rtde/external_fts_through_rtde.cpp
   :linenos:
   :lineno-match:
   :start-at: std::scoped_lock<std::mutex> lock(g_FT_VEC_MUTEX);
   :end-at: g_FT_VEC = local_ft_vec;

.. literalinclude:: ../../examples/external_fts_through_rtde.cpp
   :language: c++
   :caption: external_fts_through_rtde/external_fts_through_rtde.cpp
   :linenos:
   :lineno-match:
   :start-at: if (g_FT_VEC_MUTEX.try_lock())
   :end-at: }

Note that the ``try_lock()`` method is used in the RTDE thread to avoid blocking the thread if the
tui thread is currently holding the lock. The new values will be available in a later cycle then.

.. note::
   The values reported from the robot are rotated to the base coordinate system of the robot.
   Hence, they will not be the same as the ones sent to the robot. For them to be the same, align
   the TCP frame with robots base frame.

Example output
--------------
The following shows an example output.
.. code::

   [1760084283.005655] INFO ur_client_library/src/primary/primary_client.cpp 67: Starting primary client pipeline
   [1760084283.056577] INFO ur_client_library/src/ur/dashboard_client.cpp 76: Connected: Universal Robots Dashboard Server

   [1760084283.060021] INFO ur_client_library/src/example_robot_wrapper.cpp 201: Robot ready to start a program
   [1760084283.060652] INFO ur_client_library/src/helpers.cpp 95: SCHED_FIFO OK, priority 99
   [1760084283.061806] INFO ur_client_library/src/rtde/rtde_client.cpp 139: Negotiated RTDE protocol version to 2.
   [1760084283.061944] INFO ur_client_library/src/rtde/rtde_client.cpp 291: Setting up RTDE communication with frequency 500.000000
   Program running: false

   [1760084284.095662] INFO ur_client_library/src/primary/primary_client.cpp 67: Starting primary client pipeline
   [1760084284.225584] INFO ur_client_library/src/control/reverse_interface.cpp 225: Robot connected to reverse interface. Ready to receive control commands.
   Program running: true

   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   [1760084284.225944] INFO ur_client_library/src/helpers.cpp 95: SCHED_FIFO OK, priority 99
   [1760084284.227630] INFO ur_client_library/src/control/reverse_interface.cpp 238: Connection to reverse interface dropped.
   Program running: false

   Force-torque reported by robot: [0, 0, 0, 0, 0, 0]
   Force-torque reported by robot: [0, 0, 0, 0, 0, 0]
   <'x' pressed>
   Artificial FT input: [10, 0, 0, 0, 0, 0]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   <'y' pressed>
   Artificial FT input: [10, 10, 0, 0, 0, 0]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   <'y' pressed>
   Artificial FT input: [10, 20, 0, 0, 0, 0]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   Force-torque reported by robot: [10, 20, 2.05103e-09, 0, 0, 0]
   Force-torque reported by robot: [10, 20, 2.05103e-09, 0, 0, 0]
   Force-torque reported by robot: [10, 20, 2.05103e-09, 0, 0, 0]
   Force-torque reported by robot: [10, 20, 2.05103e-09, 0, 0, 0]
   <'Y' pressed>
   Artificial FT input: [10, 10, 0, 0, 0, 0]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   Force-torque reported by robot: [10, 10, -3.67436e-15, 0, 0, 0]
   Force-torque reported by robot: [10, 10, -3.67436e-15, 0, 0, 0]
   <'c' pressed>
   Artificial FT input: [10, 10, 0, 0, 0, 10]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   Force-torque reported by robot: [10, 10, -3.67436e-15, 2.05104e-09, -2.05103e-09, 10]
   <'X' pressed>
   Artificial FT input: [0, 10, 0, 0, 0, 10]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   Force-torque reported by robot: [2.05103e-09, 10, 2.05103e-09, 2.05104e-09, -2.05103e-09, 10]
   <'0' pressed>
   Artificial FT input: [0, 0, 0, 0, 0, 0]
   Press x,y,z to increase the respective axes, 0 for reset, q for quit.
   Force-torque reported by robot: [0, 0, 0, 0, 0, 0]
   Force-torque reported by robot: [0, 0, 0, 0, 0, 0]
   <'q' pressed>
   Artificial FT input: [0, 0, 0, 0, 0, 0]
   [1760084296.656255] INFO ur_client_library/src/primary/primary_client.cpp 61: Stopping primary client pipeline
