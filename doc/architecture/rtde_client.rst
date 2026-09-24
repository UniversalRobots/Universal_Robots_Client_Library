:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/rtde_client.rst

.. _rtde_client:

RTDEClient
==========

The Real Time Data Exchange Client, ``RTDEClient``, class serves as a standalone
`RTDE <https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/>`_
client. To use the RTDE-Client, you'll have to initialize and start it separately. When starting
it, it can be chosen whether data should be read in a background thread or if the user has to poll
data in each cycle.

- **Background read**: When background read is enabled (``start(true)``) (default), the RTDE client
  will start a background thread that continuously reads data from the robot. The latest data
  package can be fetched using the ``getDataPackage()`` method. This method returns immediately
  with the latest data package received from the robot. If no data has been received since last
  calling this function, it will block for a specified timeout waiting for new data to arrive.

- **Blocking synchronous read**: When background read is not enabled (``start(false)``), data can
  (and has to be) fetched using the ``getDataPackageBlocking()`` method. This call waits for a new
  data package to arrive and parses that into the passed ``DataPackage`` object. This has to be
  called with the RTDE control frequency, as the robot will shutdown RTDE communication if data is
  not read by the client.

The following example uses the background read method to fetch data from the RTDE interface. See
the :ref:`rtde_client_example` for an example of the blocking read method.

.. code-block:: c++

   rtde_interface::RTDEClient my_client(ROBOT_IP, notifier, OUTPUT_RECIPE_FILE, INPUT_RECIPE_FILE);
   my_client.init();
   my_client.start(true); // Start background read
   rtde_interface::DataPackage data_pkg(my_client.getOutputRecipe());
   while (true)
   {
     if (my_client.getDataPackage(data_pkg, READ_TIMEOUT))
     {
       std::cout << data_pkg.toString() << std::endl;
     }
   }

.. note::

  **Recommended:** Construct a ``DataPackage`` from ``getOutputRecipe()`` after ``init()`` and
  reuse it in your control loop. With a matching recipe, the normal data receive path of
  ``getDataPackage()`` and ``getDataPackageBlocking()`` does not allocate.

  **Still supported, but not recommended:** The older flow that lets the client allocate a
  package remains available for compatibility. The deprecated ``getDataPackage(timeout)``
  overload allocates a new package on each call, and passing a null unique pointer to either
  read method also allocates a package. Passing a package with a foreign recipe is supported
  through automatic repair, which may allocate. The null-pointer and foreign-recipe paths log
  warnings; these warn about allocation, not unsupported usage. Prefer a reusable, matching-recipe
  package for new code, especially in real-time loops.

  The allocation-free guarantee applies only to the normal data receive path with that reused
  package. It does not cover error handling, non-data messages or reconnection.

  A recipe only lists field names. The data types belonging to them are reported by the robot when
  it acknowledges the recipe, and the first read applies them to your ``DataPackage`` without
  allocation. Until that has happened ``getData()`` throws ``std::bad_variant_access``. See
  `Field data types`_ for how to ask a package what type it gave a field.

Upon construction, two recipe files have to be given, one for the RTDE inputs, one for the RTDE
outputs. Please refer to the `RTDE
guide <https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/>`_
on which elements are available.

The recipes can be either passed as a filename or as a list of strings directly. E.g. the
following will work

.. code-block:: c++

   rtde_interface::RTDEClient my_client(
     ROBOT_IP,
     notifier,
     {"timestamp", "actual_q"},
     {"speed_slider_mask", "speed_slider_fraction"}
   );

.. note::
   ``timestamp`` will always be a part of the output recipe and will be added afterwards, if not defined. As the ``timestamp`` used for verifying the connectivity.

Reading data
------------

After calling ``my_client.start()``, data can be read from the
``RTDEClient`` by calling ``getDataPackage()`` (with background thread running) or ``getDataPackageBlocking()`` (without background thread running) respectively.

Remember that, when not using a background thread, data has to be polled regularly, as the robot
will shutdown RTDE communication if the receiving side doesn't empty its buffer.

Both methods deliver their data into a ``DataPackage`` that the caller owns:
``getDataPackage()`` copies the background reader's latest package into it, while
``getDataPackageBlocking()`` parses the next package straight into it. Reusing a package with the
negotiated recipe keeps the normal data receive path free of memory allocations. The
older ``getDataPackage(timeout)`` overload, which returns a new package instead, is deprecated but
still supported. It allocates on every call by design and is not recommended for new code or
real-time use; prefer an overload that fills an existing, reusable package.

Always check the return value before using received data. A background read returns ``false`` on
timeout or when stopping or reconnecting cancels the pending read; restarting the reader does not
make a cancelled read succeed with stale data. Both unique-pointer overloads retain caller
ownership on failure and assign a previously null pointer only on success. A failed blocking read
can still partially update an existing package's values if the incoming data is malformed.

Pacing a loop with the robot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Both read modes can pace an application loop at the robot's negotiated RTDE output frequency,
without a separate fixed-period sleep. Choose the mode according to how directly the loop should
follow incoming packages:

- With ``start(false)``, ``getDataPackageBlocking()`` waits for the next package when no data is
  already buffered. Calling it at the start of each iteration lets packet arrival pace the loop,
  coupling it directly to the RTDE stream without a background-reader handoff. This is useful
  when each iteration should read data and then compute and submit a response. See the
  :ref:`rtde_roundtrip_example`.
- With ``start(true)``, ``getDataPackage(package, timeout)`` can also pace the loop: after the
  latest sample has been consumed, it waits for the background reader to publish another one,
  up to the timeout. If a newer sample is already available, it returns immediately. This mode
  decouples socket reading from application work and favors the latest sample; intermediate
  samples can be overwritten when the application is slower than the stream. It suits loops
  that need less direct synchronization and do not need to process every received sample.

For background reads used as a loop clock, allow enough timeout for the expected RTDE period
and scheduling jitter, and handle a ``false`` return instead of processing old data. In either
mode, the loop must keep up with the negotiated frequency for steady pacing. Buffered data can
make synchronous reads return immediately, and network or scheduling delays can make arrivals
irregular. Neither mode guarantees phase synchronization with the robot's internal control cycle
or receipt of a command in the next cycle. Use the output ``timestamp`` to track robot time and
detect gaps between samples.

Field data types
~~~~~~~~~~~~~~~~

``getData()`` has to be given a variable of the field's own type. A missing name returns
``false``; a type mismatch throws ``std::bad_variant_access``.
Rather than hardcoding which type a field has, ask the package: ``getDataType()`` reports the
``DataType`` a field currently holds. A successful client read applies the robot's negotiated
types to an output package; ``init()`` alone does not type application-owned packages.
``createInputDataPackage()`` returns an input package with the negotiated types already applied.
On a recipe-only package, ``setData()`` establishes an untyped field's type from the value written;
subsequent writes must match that type. An untouched field has no type. This is useful for code
that has to handle whatever recipe it is configured with, such as a bridge to another middleware:

.. code-block:: c++

   const std::optional<rtde_interface::DataType> type = data_pkg.getDataType(field_name);
   if (!type)
   {
     // Not part of the recipe, or the field has no type yet
     return;
   }

   // For "actual_q" this prints "VECTOR6D", the same spelling the RTDE guide uses
   std::cout << field_name << " is a " << rtde_interface::toString(*type) << std::endl;

   switch (*type)
   {
     case rtde_interface::DataType::DOUBLE:
     {
       double value;
       data_pkg.getData(field_name, value);
       break;
     }
     case rtde_interface::DataType::VECTOR6D:
     {
       vector6d_t value;
       data_pkg.getData(field_name, value);
       break;
     }
     // ... remaining types
   }

``DataType`` covers the complete set the protocol defines: ``BOOL``, ``UINT8``, ``UINT32``,
``UINT64``, ``INT32``, ``DOUBLE``, ``VECTOR3D``, ``VECTOR6D``, ``VECTOR6INT32`` and
``VECTOR6UINT32``. Switching over it exhaustively means the compiler will point out any case a
future protocol addition leaves unhandled.

Writing data
------------

For writing data to the RTDE interface, use the ``RTDEWriter`` member of the ``RTDEClient``. It can be
retrieved by calling ``getWriter()`` method. The ``RTDEWriter`` provides convenience methods to write
all data available at the RTDE interface. Make sure that the required keys are configured inside the
input recipe, as otherwise the send-methods will return ``false`` if the data field is not setup in
the recipe.

An example of a standalone RTDE-client can be found in the ``examples`` subfolder. To run it make
sure to

* have an instance of a robot controller / URSim running at the configured IP address (or adapt the
  address to your needs)
* run it from the package's main folder, as for simplicity reasons it doesn't use any sophisticated
  method to locate the required files.

.. note::
   The ``URDriver`` class creates a ``RTDEClient`` during initialization using the provided
   recipes and utilizing the robot model's maximum frequency. If you would like to use a different
   frequency, please use the ``resetRTDEClient()`` method after the ``UrDriver`` object has been
   created.

Read-Only RTDEClient
--------------------

While RTDE allows multiple clients to connect to the same robot, only one client is allowed to
write data to the robot. To create a read-only RTDE client, the ``RTDEClient`` can be created with
an empty input recipe, like this:

.. code-block:: c++

   rtde_interface::RTDEClient my_client(ROBOT_IP, notifier, OUTPUT_RECIPE, {});
   // Alternatively, pass an empty filename when using recipe files
   // rtde_interface::RTDEClient my_client(ROBOT_IP, notifier, OUTPUT_RECIPE_FILE, "");
   my_client.init();
   auto data_pkg = std::make_unique<rtde_interface::DataPackage>(my_client.getOutputRecipe());
   my_client.start();
   while (true)
   {
     if (my_client.getDataPackage(data_pkg, READ_TIMEOUT))
     {
       std::cout << data_pkg->toString() << std::endl;
     }
   }

RTDEWriter
----------

The ``RTDEWriter`` class provides an interface to write data to the RTDE interface. Data fields that
should be written have to be defined inside the ``INPUT_RECIPE`` as noted above.

.. important::

  **Use RTDEClient to initialize and access RTDEWriter (recommended).** Create an ``RTDEClient``
  with a non-empty input recipe, call ``init()``, then use its writer through ``getWriter()``.
  The client handles the connection, protocol negotiation, input recipe setup and writer
  initialization, including the field types and recipe ID reported by the robot.

  Constructing and using ``RTDEWriter`` directly is still supported, but is a lower-level option
  for applications that manage the RTDE connection and handshake themselves. It is not the
  recommended approach for normal application code.

The class offers convenience methods for common inputs and ``sendPackage()`` for a complete input
recipe.

Data is sent asynchronously to the RTDE interface. A successful ``sendPackage()`` or ``send...()``
call updates the pending send buffer and notifies the writer thread; it does not confirm delivery
or processing by the robot. This is not a FIFO queue of calls: multiple updates before the writer
consumes the pending buffer can be coalesced, and a later ``sendPackage()`` can replace an earlier
pending package. Separate helper calls may be transmitted separately or coalesced, depending on
when the writer runs. Use ``sendPackage()`` to submit related fields together in one buffer update,
not to guarantee a distinct transmission for every call.

To write several fields at once, ask the client for a package that already carries the data types
the robot reported for the input recipe. Call ``createInputDataPackage()`` after a successful
``init()`` with a non-empty input recipe, fill the fields you care about and pass it to
``sendPackage()``. The new package starts with zero values; when reusing it, fields retain their
previous values unless explicitly changed or reset. Because the package is already typed,
``setData()`` reports a value written with the wrong type immediately:

.. code-block:: c++

   rtde_interface::DataPackage input_pkg = my_client.createInputDataPackage();
   input_pkg.setData("speed_slider_mask", uint32_t{ 1 });
   input_pkg.setData("speed_slider_fraction", 0.5);
   my_client.getWriter().sendPackage(input_pkg);

A package constructed from ``getInputRecipe()`` still works. Its types are taken from the values
written to it and are checked when the package is submitted to ``sendPackage()``. The field names
and order must match the negotiated input recipe. Fields that remain untyped are sent as typed
zeros, while incompatible types cause ``sendPackage()`` to return ``false``. See the
:ref:`rtde_roundtrip_example` for a complete example.

If direct ``RTDEWriter`` use is required instead of the recommended ``RTDEClient`` flow, perform the RTDE handshake
and configure the stopped writer with ``setProtocolVersion(negotiated_version)`` and
``setRecipeTypes(acknowledged_types)`` before calling ``init(recipe_id)`` with the acknowledged
input recipe ID. Constructing the writer or calling ``init(recipe_id)`` alone does not establish
the field types. ``RTDEClient::init()`` handles these steps automatically.

.. note::

   The ``RTDEWriter`` will return ``false`` on any writing attempts for fields that have not been
   setup in the ``INPUT_RECIPE``. When no input recipe was provided, all write operations will
   return ``false``. No writer thread is started in that case, and ``createInputDataPackage()``
   throws ``UrException`` even after successful client initialization. The factory also throws
   before input recipe negotiation or while the writer is stopped.
