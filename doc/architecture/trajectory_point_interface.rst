:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/trajectory_point_interface.rst

.. _trajectory_point_interface:

Trajectory Point Interface
==========================

The ``TrajectoryPointInterface`` allows sending trajectory points to the robot. It is intended to
be used in conjunction with the :ref:`reverse_interface` during trajectory forwarding.

Communication regarding trajectory point execution would usually look like this:

.. figure:: ../images/trajectory_interface.svg
   :width: 1200
   :alt: Trajectory interface

Basically, the ``TrajectoryPointInterface`` transfers the trajectory points from the external
control PC to the robot for execution. Execution isn't started, before a start command is sent via
the ``ReverseInterface``. Once trajectory execution is done (either successful, failed or canceled
externally), the robot will send a response back to the control PC via the trajectory socket.

.. note::
   The ``TrajectoryPointInterface`` simply forwards the trajectory points to the robot. Execution
   is done using respective URScript functions such as `movej
   <https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/movej_qa14v105t0r.htm>`_
   or `movel
   <https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/movel_posea12v025t.htm>`_.
   Therefore, all parameters and restrictions of these functions apply. For example, velocity and
   acceleration parameters will be ignored if there is a time > 0 given.


Communication protocol
----------------------

The ``TrajectoryPointInterface``'s "trajectory_socket" on the robot is expecting 32 bit integer
representations in 21 datafields. The data fields have the following meaning:

.. table:: trajectory_socket message format
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   0-5    trajectory point positions (multiplied by ``MULT_JOINTSTATE``).

          Interpreted as joint positions [rad] or as a Cartesian pose ([m, m, m, rad, rad, rad])
          depending on the motion type at index 20 (see below).

   6-11   Depending on the motion type, this represents either

          - Joint hint ``q_near`` (six joint positions in radians, ``MULT_JOINTSTATE``-scaled) when
            the motion target is a Cartesian ``urcl::Pose`` with ``Pose::q_near`` set on the library
            side. The bundled ``external_control.urscript`` applies this hint for ``MOVEJ_POSE`` and
            ``OPTIMOVEJ_POSE`` (``get_inverse_kin`` seed); other pose motion types still receive the
            block on the wire but ignore it in the default script.

          - For all MOVEC variants this field contains the via point (same
            joint-vs-pose interpretation as the target at indices 0-5, see the motion type at
            index 20).
          - trajectory point velocities (multiplied by ``MULT_JOINTSTATE``) for spline joint types

   12-17  Depending on the motion type, this represents either

          - trajectory point accelerations (multiplied by ``MULT_JOINTSTATE``) for spline joint
            types.

          - for all other motion types

            - 12: velocity (multiplied by ``MULT_JOINTSTATE``)
            - 13: acceleration (multiplied by ``MULT_JOINTSTATE``)
            - 14: Depending on motion type:

                - ``MOVEC_*``: arc mode (multiplied by ``MULT_JOINTSTATE``)
                - Other non-spline primitives with a Cartesian ``Pose`` target: ``1.0`` or ``0.0``
                  (scaled by ``MULT_JOINTSTATE``) indicating whether indices 6–11 carry ``q_near``.
                  The bundled URScript consumes ``q_near`` for ``MOVEJ_POSE`` and ``OPTIMOVEJ_POSE``
                  only; the flag and joint block are still defined this way for all such pose motions.

   18     segment duration (seconds, multiplied by ``MULT_TIME``; integer microseconds on the wire).

   19     depending on trajectory point type

          - All MOVE* and OPTIMOVE* variants: point blend radius (in meters, multiplied by
            ``MULT_TIME``)
          - SPLINE: spline type (1: CUBIC, 2: QUINTIC; raw integer, not ``MULT_TIME``-scaled)

   20     trajectory point type. The base values below use the URScript command's "natural"
          target type (joints for ``movej`` / ``optimovej``, Cartesian pose for ``movel`` /
          ``movep`` / ``movec`` / ``optimovel``). The ``*_POSE`` / ``*_JOINT`` variants indicate
          that the other target kind is being sent instead.

          - 0: MOVEJ – ``movej`` to a joint target
          - 1: MOVEL – ``movel`` to a pose target
          - 2: MOVEP – ``movep`` to a pose target
          - 3: MOVEC – ``movec`` with pose via and pose target
          - 4: OPTIMOVEJ – ``optimovej`` to a joint target
          - 5: OPTIMOVEL – ``optimovel`` to a pose target
          - 6: MOVEJ_POSE – ``movej`` to a pose target (IK on the robot controller)
          - 7: MOVEL_JOINT – ``movel`` to the pose implied by a joint target (FK on the
            robot controller)
          - 8: MOVEP_JOINT – ``movep`` to the pose implied by a joint target (FK on the
            robot controller)
          - 9: MOVEC_JOINT – ``movec`` with joint via and joint target
          - 10: MOVEC_JOINT_POSE – ``movec`` with pose via and joint target
          - 11: MOVEC_POSE_JOINT – ``movec`` with joint via and pose target
          - 12: OPTIMOVEJ_POSE – ``optimovej`` to a pose target
          - 13: OPTIMOVEL_JOINT – ``optimovel`` to the pose implied by a joint target
          - 51: SPLINE - Cubic or quintic spline.
   =====  =====

where

- ``MULT_JOINTSTATE``: 1000000
- ``MULT_TIME``: 1000000

.. note::
   With ``MULT_TIME`` being 1000000, the maximum duration that can be sent is 2147 seconds, while
   precision is cut off at 1 microsecond. (The same applies to the blend radius, respectively being
   max 2147 m and 1 μm precision.)

.. note::
   The ``*_POSE`` / ``*_JOINT`` motion-type variants let callers mix joint-space and Cartesian
   targets freely through the high-level APIs (see :ref:`instruction_executor` and the
   ``urcl::MotionTarget`` type). On the wire the positions are always packed as a 6-tuple of
   ``MULT_JOINTSTATE``-scaled integers; which physical quantity they represent (joint angles or
   Cartesian pose components) is determined solely by the motion type field at index 20. The
   corresponding mapping back to ``movej`` / ``movel`` / ``movep`` / ``movec`` / ``optimovej`` /
   ``optimovel`` calls with either ``q`` or ``p[...]`` arguments is performed on the robot side
   by ``resources/external_control.urscript``.
