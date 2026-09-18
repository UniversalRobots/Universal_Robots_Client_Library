Migration notes
===============

This document contains notes on the migration of the ur_client_library between major versions.

It contains only breaking changes.

RTDE field types come from the robot
------------------------------------

The data types of an RTDE recipe's fields are now taken from the robot's answer to the recipe setup,
instead of from a table of field names maintained inside the library. ``DataPackage`` can still be
constructed from a recipe and preallocates its field storage there. The first client read applies
the negotiated types without allocation when the recipe matches.

The following consequences are worth knowing about:

- **A field name the robot doesn't know is reported later.** Since the library no longer has its own
  list of field names, a typo is caught when the robot rejects the recipe during
  ``RTDEClient::init()`` rather than while constructing the ``RTDEClient``. It is still an
  ``RTDEInvalidKeyException``. ``ignore_unavailable_outputs`` now also strips a name no robot knows:
  without a list of its own, the library cannot tell a typo from a field of a newer robot.
- **A wrongly typed input field is reported when the package is sent.** ``DataPackage::setData()``
  decides a field's type from the value passed to it, so it can no longer tell on its own that the
  robot expects something else. ``RTDEWriter::sendPackage()`` checks the package against the robot's
  answer and returns ``false`` if they disagree. Unset fields in a same-recipe package are sent as
  typed zeros. A typed input package is now also
  available from ``RTDEClient::createInputDataPackage()`` after ``init()``, so filling several
  input fields no longer depends on guessing the field types correctly; ``setData()`` then rejects
  a mismatch immediately.
- **Client reads repair foreign recipes and allocate for null pointers.** Both
  ``RTDEClient::getDataPackage()`` and ``RTDEClient::getDataPackageBlocking()`` warn about these
  allocation-prone paths. Reuse a package built from ``getOutputRecipe()`` to avoid them.
  Direct ``RTDEParser`` users must register negotiated types with ``setExpectedDataPackage()``
  to enable null/non-data pointer replacement and deprecated vector allocation, or register only
  ``setExpectedLayoutHash()`` and supply a matching typed package. Layout mismatches return
  ``false`` rather than throwing.
- **Standalone writers need negotiated types before sending.** If using ``RTDEWriter`` directly,
  configure it while stopped with ``setProtocolVersion(negotiated_version)`` and
  ``setRecipeTypes(acknowledged_types)`` before ``init(recipe_id)``. Use the types, in recipe order,
  and the input recipe ID returned by the robot's setup reply. The constructor and
  ``init(recipe_id)`` alone no longer establish the field types. Repeat this setup if the recipe
  is replaced. ``RTDEClient::init()`` performs these steps automatically for its writer.
- **``getData()``/``setData()`` with a ``std::string`` is now a compile error.** That alternative
  was never a protocol type, so those calls used to compile and return ``false`` at runtime. Nothing
  could have relied on them working.

On a ``DataPackage`` that hasn't been typed yet, meaning it has neither received data nor been
written to, ``getData()`` throws ``std::bad_variant_access``, and ``getDataType()`` reports that
the field has no type yet.

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
