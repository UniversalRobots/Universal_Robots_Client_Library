Instruction Executor
====================

The Instruction Executor is a convenience wrapper to make common robot instructions such as point
to point motions easily accessible. Currently, it supports the following instructions:

* Excecute MoveJ point to point motions
* Execute MoveL point to point motions
* Execute sequences consisting of MoveJ and MoveL instructions

The Instruction Executor uses the :ref:`trajectory_point_interface` and the
:ref:`reverse_interface`
for sending motion instructions to the robot. Hence, it requires a :ref:`ur_driver` object.

As a minimal working example, please see ``examples/instruction_executor.cpp`` example:

.. literalinclude:: ../../examples/instruction_executor.cpp
   :language: c++
   :caption: examples/instruction_executor.cpp
   :linenos:
   :lineno-match:
   :start-at: g_my_driver.reset
   :end-at: g_my_driver->stopControl();
