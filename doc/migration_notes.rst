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


Script communication protocol changes
-------------------------------------

The communication between the client library and the script code is not necessarily backwards
compatible. We try to avoid wire-breaking changes, but they happen from time to time. If you write
your own script code that interfaces the client library, you might want to watch the list below for
changes that might affect you.

Trajectory point interface changes in 2.16.0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The trajectory point interface transfers 22 4-byte data fields instead of 21. The new field
holds an identifier for trajectories executed via the trajectory point interface. See `PR #552 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/552>`_ for details.

This is only relevant when using the ``TrajectoryPointInterface`` directly. If you use it through
the ``UrDriver`` class or the ``InstructionExecutor`` class, the driver will check whether the
script code supports motion IDs automatically.

If you want to revert to the old behavior, you can call
``TrajectoryPointInterface::setMoveIdSupport(false)`` after creating the
``TrajectoryPointInterface`` object.

The ``ReverseInterface`` also received a new slot in the TrajectoryInstructions field. However, this
will be ignored by older scripts and thus should be backwards-compatible.
