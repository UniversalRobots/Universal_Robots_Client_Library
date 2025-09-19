:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/instruction_executor.rst

.. _instruction_executor_example:

Instruction Executor example
============================

This example shows how to use the :ref:`instruction_executor` class. It can be used for easily
executing a sequence of instructions such as motions on the robot using the built-in URScript functions.

The `instruction_executor.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/instruction_executor.cpp>`_ shows how to use this class:

.. note:: For the instruciton executor to work there has to be an established connection to the
   :ref:`reverse_interface`. That means, the respective program has to be running on the robot. The
   example below will do that automatically, if the connected robot is in *remote_control* mode.


.. literalinclude:: ../../examples/instruction_executor.cpp
   :language: c++
   :caption: examples/instruction_executor.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-at: auto instruction_executor = std::make_shared<urcl::InstructionExecutor>(g_my_robot->getUrDriver());

At first, a ``InstructionExecutor`` object is created with the URDriver object as it needs that
for communication with the robot.

Currently, the ``InstructionExecutor`` can either be used to run sequences of motions or single motions.

Execute a sequence of motions
-----------------------------

To run a sequence of motions, create an
``std::vector<std::shared_ptr<urcl::cointrol::MotionPrimitive>>`` and pass it to the
``executeMotion`` function:

.. literalinclude:: ../../examples/instruction_executor.cpp
   :language: c++
   :caption: examples/instruction_executor.cpp
   :linenos:
   :lineno-match:
   :start-at: // Trajectory definition
   :end-at: instruction_executor->executeMotion(motion_sequence);

Each element in the motion sequence can be a different motion type. In the example, there are two
``MoveJ`` motions, a ``MoveL`` motion, a ``MoveP`` motion, a ``OptimiveJ`` motion and a
``OptimoveL`` motion. The primitives' parameters are directly forwarded to the underlying script
functions, so the parameter descriptions for them apply, as well. Particularly, you may want to
choose between either a time-based execution or an acceleration / velocity parametrization
for some move functions. The latter will be ignored if a time > 0 is given.

Please refer to the script manual for details.

Execute a single motion
-----------------------

To run a single motion, the ``InstructionExecutor`` provides the methods ``moveJ(...)`` and
``moveL(...)``:

.. literalinclude:: ../../examples/instruction_executor.cpp
   :language: c++
   :caption: examples/instruction_executor.cpp
   :linenos:
   :lineno-match:
   :start-at: double goal_time_sec = 2.0;
   :end-before: g_my_robot->getUrDriver()->stopControl();

Again, time parametrization has priority over acceleration / velocity parameters.
