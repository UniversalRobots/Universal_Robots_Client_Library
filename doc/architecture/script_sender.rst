:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/script_sender.rst

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

An example of how to use the ``ScriptSender`` class can be found in the :ref:`script_sender_example`.

.. note::
   PolyScope X users cannot use the URCap linked above. There is a development version of a URCapX
   available at https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX.
