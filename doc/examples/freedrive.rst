:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/freedrive.rst

Freedrive Mode example
======================

`Freedrive
<https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/freedrive_mode.htm>`_
allows the robot arm to be manually pulled into desired positions and/or poses. The joints move
with little resistance because the brakes are released.

An example to utilize the freedrive mode can be found in the `freedrive_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/freedrive_example.cpp>`_.

.. note:: For the following example to work on an e-Series (PolyScope 5) robot, the robot has to be
   in *remote control mode*.

At first, we create a ``ExampleRobotWrapper`` object in order to initialize communication with the
robot.

.. literalinclude:: ../../examples/freedrive_example.cpp
   :language: c++
   :caption: examples/freedrive_example.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-before: URCL_LOG_INFO("Starting freedrive mode");


Start freedrive mode
--------------------

The ``UrDriver`` provides a method to start freedrive mode directly:

.. literalinclude:: ../../examples/freedrive_example.cpp
   :language: c++
   :caption: examples/freedrive_example.cpp
   :linenos:
   :lineno-match:
   :start-at: URCL_LOG_INFO("Starting freedrive mode");
   :end-at: sendFreedriveMessageOrDie(control::FreedriveControlMessage::FREEDRIVE_START);

As it is potentially dangerous to leave the robot in freedrive mode, the robot program expect
frequent keepalive messages to verify that the remote connection is still available and freedrive
mode is being expected to be active.

Freedrive mode will be active from this point on until it is either stopped, or no keepalive
message is received by the robot anymore.

Therefore, we have to make sure to send regular keepalive messages to the robot. The following
section will keep freedrive mode active for a period of time defined in ``seconds_to_run``.

.. literalinclude:: ../../examples/freedrive_example.cpp
   :language: c++
   :caption: examples/freedrive_example.cpp
   :linenos:
   :lineno-match:
   :start-at: std::chrono::duration<double> time_done(0);
   :end-before: sendFreedriveMessageOrDie(control::FreedriveControlMessage::FREEDRIVE_STOP);

Stop freedrive Mode
-------------------

To stop freedrive mode either stop sending keepalive signals or request deactivating it explicitly:

.. literalinclude:: ../../examples/freedrive_example.cpp
   :language: c++
   :caption: examples/freedrive_example.cpp
   :linenos:
   :lineno-match:
   :start-at: sendFreedriveMessageOrDie(control::FreedriveControlMessage::FREEDRIVE_STOP);
   :end-at: sendFreedriveMessageOrDie(control::FreedriveControlMessage::FREEDRIVE_STOP);
