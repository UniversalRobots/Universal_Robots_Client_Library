Migration notes
===============

This document contains notes on the migration of the ur_client_library between major versions.

It contains only breaking changes.

RTDE field types come from the robot
------------------------------------

The data types of an RTDE recipe's fields are now taken from the robot's answer to the recipe setup,
instead of from a table of field names maintained inside the library. No application code has to
change for this: ``DataPackage`` is still constructed from a recipe, still allocates all of its
storage there, and is typed by the robot's answer afterwards, which costs no memory.

Three consequences are worth knowing about:

- **A field name the robot doesn't know is reported later.** Since the library no longer has its own
  list of field names, a typo is caught when the robot rejects the recipe during
  ``RTDEClient::init()`` rather than while constructing the ``RTDEClient``. It is still an
  ``RTDEInvalidKeyException``, and ``ignore_unavailable_outputs`` still strips such fields instead.
- **A wrongly typed input field is reported when the package is sent.** ``DataPackage::setData()``
  decides a field's type from the value passed to it, so it can no longer tell on its own that the
  robot expects something else. ``RTDEWriter::sendPackage()`` checks the package against the robot's
  answer and names the field and both types if they disagree.
- **Reading a field as the wrong type no longer throws.** ``DataPackage::getData()`` used to let a
  ``std::bad_variant_access`` escape when the passed variable didn't match the field's type. It now
  returns ``false`` and logs which type the robot reported for that field, matching what its
  documentation always promised. Code that caught that exception should check the return value
  instead.

On a ``DataPackage`` that hasn't been typed yet, meaning it has neither received data nor been
written to, ``getData()`` fails with an explanatory message instead of returning stale values, and
``getDataType()`` reports that the field has no type yet.

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
