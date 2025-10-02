:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/script_command_interface.rst

Script command interface
========================

The :ref:`script_command_interface` allows sending predefined commands to the robot while there is
URScript running that is connected to it.

An example to utilize the script command interface can be found in the `freedrive_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/script_command_interface.cpp>`_.

In order to use the ``ScriptCommandInterface``, there has to be a script code running on the robot
that connects to the ``ScriptCommandInterface``. This happens as part of the big
`external_control.urscript <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/resources/external_control.urscript>`_. In order to reuse that with this example, we will create a full
``UrDriver`` and leverage the ``ScriptCommandInterface`` through this.

At first, we create a ``ExampleRobotWrapper`` object in order to initialize communication with the
robot.

.. literalinclude:: ../../examples/script_command_interface.cpp
   :language: c++
   :caption: examples/script_command_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: g_my_robot =
   :end-at: std::thread script_command_send_thread(sendScriptCommands);

The script commands will be sent in a separate thread which will be explained later.

Since the connection to the script command interface runs as part of the bigger external_control
script, we'll wrap the calls alongside a full ``ExampleRobotWrapper``. Hence, we'll have to send
keepalive signals regularly to keep the script running:

.. literalinclude:: ../../examples/script_command_interface.cpp
   :language: c++
   :caption: examples/script_command_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: std::chrono::duration<double> time_done(0);
   :end-at: g_my_robot->getUrDriver()->stopControl();

Sending script commands
-----------------------

Once the script is running on the robot, a connection to the driver's script command interface
should be established. The ``UrDriver`` forwards most calls of the ``ScriptCommandInterface`` and
we will use that interface in this example. To send a script command, we can e.g. use
``g_my_robot->getUrDriver()->zeroFTSensor()``.

In the example, we have wrapped the calls into a lambda function that will wait a specific timeout,
print a log output what command will be sent and then call the respective command:

.. literalinclude:: ../../examples/script_command_interface.cpp
   :language: c++
   :caption: examples/script_command_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: run_cmd(
   :end-before: URCL_LOG_INFO("Script command thread finished.");

The lambda itself looks like this:

.. literalinclude:: ../../examples/script_command_interface.cpp
   :language: c++
   :caption: examples/script_command_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: auto run_cmd =
   :end-before: // Keep running all commands in a loop

For a list of all available script commands, please refer to the ``ScriptCommandInterface`` class
`here <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/include/ur_client_library/control/script_command_interface.h>`_.
