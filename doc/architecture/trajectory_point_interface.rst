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
   0-5    trajectory point positions (Multiplied by ``MULT_JOINTSTATE``)
   6-11   trajectory point velocities (Multiplied by ``MULT_JOINTSTATE``). For MOVEC, this contains the "via pose".
   12-17  trajectory point accelerations (Multiplied by ``MULT_JOINTSTATE``).

          For MOVEC:

          - 12: velocity (Multiplied by ``MULT_JOINTSTATE``)
          - 13: acceleration (Multiplied by ``MULT_JOINTSTATE``)
          - 14: mode (Multiplied by ``MULT_JOINTSTATE``)

   18     trajectory point type

          - 0: MOVEJ
          - 1: MOVEL
          - 2: MOVEP
          - 3: MOVEC
          - 51: SPLINE)

   19     trajectory point time (in seconds, multiplied by ``MULT_TIME``)
   20     depending on trajectory point type

          - MOVEJ, MOVEL, MOVEP and MOVEC: point blend radius (in meters, multiplied by ``MULT_TIME``)
          - SPLINE: spline type (1: CUBIC, 2: QUINTIC)
   =====  =====

where

- ``MULT_JOINTSTATE``: 1000000
- ``MULT_TIME``: 1000
