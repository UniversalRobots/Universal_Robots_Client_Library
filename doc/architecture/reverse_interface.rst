:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/reverse_interface.rst

.. _reverse_interface:

ReverseInterface
================


The ``ReverseInterface`` opens a TCP port on which a custom protocol is implemented between the
robot and the control PC. The port can be specified in the class constructor.

It's basic functionality is to send a vector of floating point data together with a mode. It is
meant to send joint positions or velocities together with a mode that tells the robot how to
interpret those values (e.g. ``SERVOJ``, ``SPEEDJ``). Therefore, this interface can be used to do
motion command streaming to the robot.

In order to use this class in an application together with a robot, make sure that a corresponding
URScript is running on the robot that can interpret the commands sent. See `this example
script <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/resources/external_control.urscript>`_ for reference.

The ``ReverseInterface`` together with the :ref:`script_command_interface`, the
:ref:`trajectory_point_interface` and the
`external_control.urscript
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/resources/external_control.urscript>`_
creates a communication protocol in order to control many of the robot's functionalities.

Also see the :ref:`script_sender` for a way to define the corresponding URScript on the
control PC and sending it to the robot upon request.

Communication protocol
----------------------


The ``ReverseInterface``'s "reverse_socket" on the robot is expecting 32 bit integer
representations 8 datafields frequently. The timeout with which new data is expected can vary depending
on the current control mode and is set as part of the message. The data fields have the following
meaning:

.. table:: reverse_socket message format
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   0      read timeout in milliseconds. The next read from the socket will use this timeout.
   1-6    Current motion target. depending on the control mode, this can be interpreted as

           - joint positions (SERVOJ)
           - joint velocities (SPEEDJ)
           - trajectory instructions (FORWARD)

             - field 1: Trajectory control mode (1: TRAJECTORY_MODE_RECEIVE, 2: TRAJECTORY_MODE_STREAM_START, 3: TRAJECTORY_MODE_STREAM_END, -1: TRAJECTORY_MODE_CANCEL). See :ref:`streaming_trajectories` for the streaming modes.
             - field 2: Trajectory point count. Its interpretation depends on the control mode in field 1.

           - Cartesian velocities (SPEEDL)
           - Cartesian pose (POSE)
           - freedrive instruction (FREEDRIVE)

             - field 1: Freedrive mode (1: FREEDRIVE_MODE_START, -1: FREEDRIVE_MODE_STOP)
           - target joint torques (TORQUE, see :ref:`direct_torque_control_mode`).

   7      Control mode. Can be either of

           - -2: STOPPED -- status - not meant to be sent
           - -1: UNINITIALIZED -- status - not meant to be sent
           - 0: IDLE -- no motion
           - 1: SERVOJ -- High-frequent joint position control
           - 2: SPEEDJ -- High-frequent joint velocity control
           - 3: FORWARD -- Trajectory interpolation on the robot, see :ref:`trajectory_point_interface`
           - 4: SPEEDL -- High-frequenct Cartesian velocity control
           - 5: POSE -- High-frequent Cartesian pose control (servoj using inverse kinematics)
           - 6: FREEDRIVE -- Use the robot's freedrive mode
           - 7: TOOL_IN_CONTACT -- status - not meant to be sent.
             In tool contact mode this will
             encode whether tool contact has been established or not.
           - 8: TORQUE -- :ref:`direct_torque_control_mode` (since PolyScope 5.23.0 / 10.10.0)
   =====  =====

.. note::
   In URScript the ``socket_read_binary_integer()`` function is used to read the data from the
   reverse socket. The first index in that function's return value is the number of integers read,
   so the actual data starts at index 1. The indices in the table above are shifted by one when
   accessing the result array of the URScript function.

   The motion target is encoded into an integer representation and has to be divided by the
   ``MULT_JOINTSTATE`` constant to get the actual floating point value. This constant is defined in
   ``ReverseInterface`` class.

Depending on the control mode one can use the ``write()`` (SERVOJ, SPEEDJ, SPEEDL, POSE, TORQUE), ``writeTrajectoryControlMessage()`` (FORWARD) or ``writeFreedriveControlMessage()`` (FREEDRIVE) function to write a message to the "reverse_socket".

.. _streaming_trajectories:

Streaming trajectories
~~~~~~~~~~~~~~~~~~~~~~~~

A ``FORWARD``-mode trajectory can be executed in one of two ways.

A *finite* trajectory declares its total length up front:
``writeTrajectoryControlMessage(TRAJECTORY_START, n)`` promises exactly ``n`` points, and execution
completes once they have been consumed. This is the mode used by
:ref:`trajecotry_joint_interface_example`.

A *streaming* trajectory is open-ended. The producer does not commit to a point count in advance; it
opens the stream, writes points for as long as it likes, and signals end-of-stream explicitly. This
suits points produced on the fly -- from a planner or a teleoperation source -- where the total length
is unknown when motion begins. The message sequence is:

.. code-block:: c++

   writeTrajectoryControlMessage(TRAJECTORY_STREAM_START)   // open the stream
   // write a motion primitive, any number of times:
   // writeTrajectoryPoint(...) / writeTrajectorySplinePoint(...) / writeMotionPrimitive(...)
   writeTrajectoryControlMessage(TRAJECTORY_STREAM_END, n)  // close the stream

``TRAJECTORY_STREAM_START`` opens the trajectory; its point-count argument is unused. The producer then
writes motion primitives -- via ``writeTrajectoryPoint()``, ``writeTrajectorySplinePoint()`` or
``writeMotionPrimitive()`` -- for as long as it likes. ``TRAJECTORY_STREAM_END`` closes it.

.. important::
   The ``n`` passed to ``TRAJECTORY_STREAM_END`` must equal the total number of motion primitives the
   producer wrote since ``TRAJECTORY_STREAM_START``. The controller consumes primitives asynchronously
   as it executes them and cannot otherwise tell how many are still outstanding; it uses ``n`` to
   account for those still in flight before completing. Aborting a stream with ``TRAJECTORY_CANCEL``
   requires the same count, for the same reason. (For a finite ``TRAJECTORY_START`` trajectory the
   cancel argument is unused.)

Two obligations fall on the producer:

- **Do not starve the controller.** The robot consumes the stream as it executes it, so the producer
  must keep work available: the next primitive must arrive before the robot finishes the ones it
  already holds. The budget for delivering it is simply the duration of the work in hand -- a primitive
  that runs for five minutes gives five minutes to produce its successor, while a stream of
  eight-millisecond primitives demands one every eight milliseconds. If the robot runs out of submitted
  work before the next primitive arrives and before ``TRAJECTORY_STREAM_END`` is sent, the trajectory
  ends with ``TRAJECTORY_RESULT_FAILURE``. Delivering primitives as they are produced, without holding
  them back, is the natural way to satisfy this.

- **End at rest.** Streaming adds no wind-down deceleration of its own. Make the final point a
  controlled stop, with zero velocity and acceleration, so the spline interpolates smoothly to rest at
  the goal. The trajectory thread issues ``stopj`` on exit regardless, so a non-zero terminal point
  still stops the robot, but the transition will be less smooth.

On completion the callback registered with ``setTrajectoryEndCallback()`` reports
``TRAJECTORY_RESULT_SUCCESS`` for a clean end (``TRAJECTORY_STREAM_END`` processed, all points
consumed), ``TRAJECTORY_RESULT_FAILURE`` for a producer underrun, or ``TRAJECTORY_RESULT_CANCELED`` if
the stream was aborted with ``TRAJECTORY_CANCEL``.

See :ref:`trajectory_streaming_example` for a complete, working example.

.. _direct_torque_control_mode:

Direct torque control mode
~~~~~~~~~~~~~~~~~~~~~~~~~~

Direct torque control mode is available since PolyScope version 5.23.0 / 10.10.0. It allows to command
joint torques directly to the robot.

.. note:: Target torques are given **after** gravity compensation. A vector of zeros will hold the current position
   given that the payload is known to the controller.

.. warning:: Direct torque control is a very low-level command interface. Commanding high torques in
   free space can make the robot move very fast and hereby trigger a fault due to joint velocities
   or the TCP speed violating the safety settings. Keep that in mind when using this mode.
