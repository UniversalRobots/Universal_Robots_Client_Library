:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/trajectory_point_interface.rst

.. _trajecotry_joint_interface_example:

Trajectory Joint Interface example
==================================



At first, we create a ``UrDriver`` object as usual:

.. literalinclude:: ../../examples/trajectory_point_interface.cpp
   :language: c++
   :caption: examples/trajectory_point_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-before: // --------------- INITIALIZATION END -------------------

We use a small helper function to make sure that the reverse interface is active and connected
before proceeding.


Initialization
--------------

As trajectory execution will be triggered asynchronously, we define a callback function to handle a
finished trajectory. A trajectory is considered finished when the robot is no longer executing it,
independent of whether it successfully reached its final point. The trajectory result will reflect
whether it was executed successfully, was canceled upon request or failed for some reason.

.. literalinclude:: ../../examples/trajectory_point_interface.cpp
   :language: c++
   :caption: examples/trajectory_point_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: void trajDoneCallback(const urcl::control::TrajectoryResult& result)
   :end-at: }

That callback can be registered at the ``UrDriver`` object:


.. literalinclude:: ../../examples/trajectory_point_interface.cpp
   :language: c++
   :caption: examples/trajectory_point_interface.cpp
   :linenos:
   :lineno-match:
   :start-at: g_my_robot->getUrDriver()->registerTrajectoryDoneCallback(&trajDoneCallback);
   :end-at: g_my_robot->getUrDriver()->registerTrajectoryDoneCallback(&trajDoneCallback);


MoveJ Trajectory
----------------

Then, in order to execute a trajectory, we need to define a trajectory as a sequence of points and
parameters. The following example shows execution of a 2-point trajectory using URScript's
``movej`` function:

.. literalinclude:: ../../examples/trajectory_point_interface.cpp
   :language: c++
   :caption: examples/trajectory_point_interface.cpp
   :linenos:
   :lineno-match:
   :start-after: // --------------- MOVEJ TRAJECTORY -------------------
   :end-before: // --------------- END MOVEJ TRAJECTORY -------------------

In fact, the path is followed twice, once parametrized by a segment duration and once using maximum
velocity / acceleration settings. If a duration > 0 is given for a segment, the velocity and
acceleration settings will be ignored as in the underlying URScript functions. In the example
above, each of the ``g_my_robot->getUrDriver()->writeTrajectoryPoint()`` calls will be translated into a
``movej`` command in URScript.

While the trajectory is running, we need to tell the robot program that connection is still active
and we expect the trajectory to be running. This is being done by the
``g_my_robot->getUrDriver()->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);``
call.

MoveL Trajectory
----------------

Similar to the ``movej``-based trajectory, execution can be done interpolating in joint space:

.. literalinclude:: ../../examples/trajectory_point_interface.cpp
   :language: c++
   :caption: examples/trajectory_point_interface.cpp
   :linenos:
   :lineno-match:
   :start-after: // --------------- MOVEL TRAJECTORY -------------------
   :end-before: // --------------- END MOVEL TRAJECTORY -------------------

Spline based interpolation
--------------------------

Similar to the :ref:`spline_example`, the trajectory point interface can be used to execute motions
using the spline interpolation:

.. literalinclude:: ../../examples/trajectory_point_interface.cpp
   :language: c++
   :caption: examples/trajectory_point_interface.cpp
   :linenos:
   :lineno-match:
   :start-after: // --------------- SPLINE TRAJECTORY -------------------
   :end-before: // --------------- END SPLINE TRAJECTORY -------------------
