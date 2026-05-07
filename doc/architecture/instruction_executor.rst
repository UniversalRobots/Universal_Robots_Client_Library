:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/instruction_executor.rst

.. _instruction_executor:

Instruction Executor
====================

The Instruction Executor is a convenience wrapper to make common robot instructions such as point
to point motions easily accessible. Currently, it supports the following instructions:

* Execute MoveJ point to point motions
* Execute MoveL point to point motions
* Execute MoveP point to point motions
* Execute MoveC circular motions
* Execute OptimoveJ point to point motions (For PolyScope 5.21 / PolyScope 10.8 and later)
* Execute OptimoveL point to point motions (For PolyScope 5.21 / PolyScope 10.8 and later)
* Execute sequences consisting of the motion primitives above

Joint and Cartesian targets
---------------------------

Every motion function comes in two flavours:

* A "native" overload whose parameter is the target type that matches the underlying URScript
  command (``vector6d_t`` for ``moveJ`` / ``optimoveJ``, ``urcl::Pose`` for ``moveL``, ``moveP``,
  ``moveC`` and ``optimoveL``). Braced-initializer-list calls such as
  ``moveJ({ q1, q2, q3, q4, q5, q6 })`` bind to this overload and keep the behaviour from older
  releases.
* A ``urcl::MotionTarget`` overload that accepts either a ``urcl::Q`` (joint configuration) or a
  ``urcl::Pose`` (Cartesian pose). This lets the same function perform a motion whose target type
  does not match the URScript command's natural argument, e.g. ``moveJ(urcl::Pose{...})`` to reach
  a Cartesian target with a joint-interpolated motion, or ``moveL(urcl::Q{...})`` to perform a
  linear tool-space motion towards the pose implied by a joint configuration.

An ``urcl::Pose`` may optionally carry ``q_near`` (``std::optional<urcl::Q>``), a joint configuration
hint for inverse kinematic solver to select the desired joint angle solution,
when the pose is sent over the trajectory interface; see :ref:`trajectory_point_interface`.

The Instruction Executor uses the :ref:`trajectory_point_interface` and the
:ref:`reverse_interface`
for sending motion instructions to the robot. Hence, it requires a :ref:`ur_driver` object.

.. note::
   The ``InstructionExecutor`` simply forwards the trajectory points to the robot. Execution
   is done using respective URScript functions such as `movej
   <https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/movej_qa14v105t0r.htm>`_
   or `movel
   <https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/movel_posea12v025t.htm>`_.
   Therefore, all parameters and restrictions of these functions apply. For example, velocity and
   acceleration parameters will be ignored if there is a time > 0 given.

As a minimal working example, please see the :ref:`instruction_executor_example`.
