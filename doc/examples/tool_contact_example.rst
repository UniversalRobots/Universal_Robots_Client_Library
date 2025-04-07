:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/tool_contact_example.rst

.. _tool_contact_example:

Tool Contact example
====================

Univeral Robots' **tool contact mode** allows detecting collisions between the robot's tool and the
environment and interrupting motions when that happens. This example demonstrates how to use the
tool contact mode to detect collisions and stop the robot.

As a basic concept, we will move the robot linearly in the negative z axis until the tool hits
something or the program's timeout is hit.

At first, we create a initialize a driver as usual:

.. literalinclude:: ../../examples/tool_contact_example.cpp
   :language: c++
   :caption: examples/tool_contact_example.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-before: g_my_robot->getUrDriver()->registerToolContactResultCallback

We use a small helper function to make sure that the reverse interface is active and connected
before proceeding.

When a tool contact is detected, a callback will be triggered. For that we define a function to
handle this event:

.. literalinclude:: ../../examples/tool_contact_example.cpp
   :language: c++
   :caption: examples/tool_contact_example.cpp
   :linenos:
   :lineno-match:
   :start-at: void handleToolContactResult(control::ToolContactResult result)
   :end-at: }



This function is registered as a callback to the driver and then tool_contact mode is enabled:

.. literalinclude:: ../../examples/tool_contact_example.cpp
   :language: c++
   :caption: examples/tool_contact_example.cpp
   :linenos:
   :lineno-match:
   :start-at: g_my_robot->getUrDriver()->registerToolContactResultCallback(&handleToolContactResult);
   :end-at: g_my_robot->getUrDriver()->startToolContact()

Once everything is initialized, we can start a control loop commanding the robot to move in the
negative z direction until the program timeout is hit or a tool contact is detected:

.. literalinclude:: ../../examples/tool_contact_example.cpp
   :language: c++
   :caption: examples/tool_contact_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // This will move the robot
   :end-at: return 0;
