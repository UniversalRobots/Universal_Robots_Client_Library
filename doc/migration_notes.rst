Migration notes
===============

This document describes compatibility when upgrading the UR Client Library and lists breaking
changes that may require migration.

Compatibility when upgrading
-----------------------------

* **C++ source API:** The public API is intended to remain source-compatible within a major
  release series. An application using the public API should still build against a newer minor
  or patch release in that series.
* **C++ binary ABI:** Binary compatibility is not guaranteed, even within a major release
  series. Rebuild and relink applications against the new library instead of replacing the
  library under an existing binary.
* **Robot software:** Check :doc:`polyscope_compatibility` for supported PolyScope versions and
  feature-specific requirements. Library API compatibility does not imply that a feature is
  available on a given robot software version.
* **Custom URScript:** The wire protocols for interfaces such as the Reverse Interface and the
  Trajectory Point Interface may change independently of the public C++ API. If your URScript
  talks directly to these interfaces, review the protocol changes below and update the script
  as needed when upgrading the library.

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
