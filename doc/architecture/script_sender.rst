.. _script_sender:

ScriptSender
============

The ``ScriptSender`` class opens a tcp socket listens for "*request_program*" request. Upon such a
request, a predefined URScript code is sent to the caller. The script code itself has to be passed
to the ``ScriptSender``'s constructor.

Use this class in conjunction with the `External Control URCap
<https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap>`_ which will make the
corresponding request when starting a program on the robot that contains the **External Control**
program node. In order to work properly, make sure that the IP address and script sender port are
configured correctly on the robot.

The following example creates a ``ScriptSender`` listening on port ``12345`` and sends the script
``textmsg("Hello, World!")`` when requested. A fully compilable example can be found in `script_sender.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/script_sender.cpp>`_

.. literalinclude:: ../../examples/script_sender.cpp
   :language: c++
   :caption: examples/script_sender.cpp
   :linenos:
   :lineno-match:
   :start-at: constexpr uint32_t PORT

.. note::
   PolyScope X users cannot use the URCap linked above. There is a development version of a URCapX
   available at https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX.
