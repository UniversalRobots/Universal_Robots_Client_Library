Migration notes
===============

This document contains notes on the migration of the ur_client_library between major versions.

It contains only breaking changes.

Migrating from 2.x.x to 3.x.x
-----------------------------

- Trajectory point records sent on the "trajectory_socket" have gained a trailing field, and the
  trajectory control message sent on the "reverse_socket" now carries a move identifier in a slot
  that was previously zero padding. See :ref:`trajectory_point_interface` and
  :ref:`reverse_interface` for the two layouts. The library and the ``external_control.urscript``
  that it sends to the robot must be upgraded together, because neither one can read the other's
  record layout. This only affects you if you supply your own copy of the script rather than
  using the one shipped alongside the library.

- ``urcl::control::ReverseInterface::writeTrajectoryControlMessage()`` takes a new ``move_id``
  argument, which sits before the existing ``robot_receive_timeout`` argument. Callers that
  passed a receive timeout positionally will no longer compile until they are updated. Callers
  that go through ``urcl::UrDriver`` need no change, since it assigns move identifiers on their
  behalf.

Migrating from 1.x.x to 2.x.x
-----------------------------

- In the ``urcl::ExampleRobotWrapper`` class the ``ur_driver_``, ``dashboard_client`` and
  ``primary_client`` members are now private. Use ``getUrDriver()``, ``getDashboardClient()`` and
  ``getPrimaryClient()`` to access them.

- In ``urcl::comm::ControlModeTypes`` two member functions have been renamed:

  - ``is_control_mode_realtime`` -> ``isControlModeRealtime``
  - ``is_control_mode_non_realtime`` -> ``isControlModeNonRealtime``

- In ``urcl::RobotReceiveTimeout`` the ``timeout_`` member is now private. Use
  ``getAsMilliseconds()`` to access it.

- In ``urcl::UrDriverConfiguration`` two members have been renamed:

  - ``rtde_initialization_attempts_`` -> ``rtde_initialization_attempts``
  - ``rtde_initialization_timeout_`` -> ``rtde_initialization_timeout``
