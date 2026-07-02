:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/trajectory_streaming.rst

.. _trajectory_streaming_example:

Trajectory streaming example
============================

This example demonstrates open-ended *trajectory streaming*: a producer sends points to the
controller without declaring a total point count up front, then signals end-of-stream when it is
finished. Contrast this with the :ref:`trajecotry_joint_interface_example`, which uses the finite
trajectory path where the point count is fixed in the initial ``TRAJECTORY_START`` message.

The streaming communication contract -- the ``STREAM_START`` / ``STREAM_END`` handshake, the meaning
of the end-of-stream point count, the producer's obligation not to starve the controller, and the
trajectory result codes -- is described in :ref:`streaming_trajectories`. This page walks through the
example code and refers back to that section for the authoritative contract.

Setup
-----

As with the other examples, we create a ``UrDriver`` (here through ``ExampleRobotWrapper``) and
register a callback that fires when a trajectory finishes:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-at: const bool headless_mode = true;
   :end-before: // --------------- PRE-POSITION VIA FINITE TRAJECTORY

Trajectory execution is asynchronous, so completion is reported through a callback. The result
distinguishes a successful run from a cancellation or a failure:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-at: void trajDoneCallback
   :end-at: }

While a trajectory runs, the control PC must keep telling the robot program that the connection is
still alive. We do this by pumping ``TRAJECTORY_NOOP`` messages while we wait; without them the robot
program stops waiting for input and the ``external_control`` program exits:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-at: // Pump TRAJECTORY_NOOP
   :end-before: // Sample a quintic-Hermite

Generating a motion
-------------------

To have something to stream, we sample a quintic-Hermite blend between two joint configurations. The
blend has zero velocity and acceleration at both endpoints, which -- as the streaming contract
recommends -- gives the final point the controlled-stop property needed to bring the robot cleanly to
rest:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-at: // Sample a quintic-Hermite
   :end-before: int main(

Pre-positioning
---------------

Before streaming, we park the robot at a known start pose using an ordinary finite trajectory. This
doubles as a demonstration of the ``TRAJECTORY_START`` path for comparison:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-after: // --------------- PRE-POSITION VIA FINITE TRAJECTORY
   :end-before: // ----------------- STREAMING TRAJECTORY DEMO

Streaming the trajectory
------------------------

We precompute all of the points up front. The final point inherits the ``qd = 0``, ``qdd = 0``
boundary condition of the quintic blend, so it is a compliant controlled-stop terminal:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-after: // ----------------- STREAMING TRAJECTORY DEMO
   :end-before: URCL_LOG_INFO("Streaming %d points

The stream itself is the ``STREAM_START`` -> points -> ``STREAM_END`` handshake. We open the stream,
write every point back-to-back, then close it, passing the total number of points written so the
controller knows how many are still outstanding before it reports completion:

.. literalinclude:: ../../examples/trajectory_streaming.cpp
   :language: c++
   :caption: examples/trajectory_streaming.cpp
   :linenos:
   :lineno-match:
   :start-at: URCL_LOG_INFO("Streaming %d points
   :end-before: g_my_robot->getUrDriver()->stopControl();

Because this example delivers the whole motion up front, it can never starve the controller: all the
work is in hand before execution finishes, so the robot simply plays through it. The example also
measures the time from sending ``STREAM_END`` to the trajectory-done callback. That time is large
here, but not because ending a stream is costly: the producer sends every point and then
``STREAM_END`` within milliseconds, long before the robot has finished moving, so the callback cannot
fire until the robot has worked through the entire backlog. A producer that instead fed points in step
with the robot's motion would send ``STREAM_END`` just as the robot reached the final point, and the
callback would follow almost immediately.

For the general case, where points are produced on the fly and pacing does matter, see the producer
obligations in :ref:`streaming_trajectories`.
