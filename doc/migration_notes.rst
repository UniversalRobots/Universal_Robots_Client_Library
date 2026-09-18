Migration notes
===============

This document contains notes on the migration of the ur_client_library between major versions.

It contains only breaking changes.

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
