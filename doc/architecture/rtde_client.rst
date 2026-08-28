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
   Constructing the ``DataPackage`` is where its memory is allocated, so create it before entering
   your control loop and reuse it: ``getDataPackage()`` and ``getDataPackageBlocking()`` don't
   allocate.

   A recipe only lists field names. The data types belonging to them are reported by the robot when
   it acknowledges the recipe, and the first package received applies them to your ``DataPackage``,
   which costs no memory. Until that has happened ``getData()`` on the package fails. See `Field
   data types`_ for how to ask a package what type it gave a field.

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

Both methods parse into a ``DataPackage`` that the caller owns, which is what keeps the read path
free of memory allocations. The deprecated ``getDataPackage(timeout)`` overload, which returns a new
package instead, allocates on every call by design and is therefore not suited for real-time use.

Field data types
~~~~~~~~~~~~~~~~

``getData()`` has to be given a variable of the field's own type, and returns ``false`` if it isn't.
Rather than hardcoding which type a field has, ask the package: ``getDataType()`` reports the
``DataType`` the robot gave a field, or nothing at all if the recipe hasn't been acknowledged yet.
This is useful for code that has to handle whatever recipe it is configured with, such as a bridge
to another middleware:

.. code-block:: c++

   const std::optional<rtde_interface::DataType> type = data_pkg.getDataType(field_name);
   if (!type)
   {
     // Not part of the recipe, or the recipe hasn't been acknowledged yet
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

The class offers specific methods for every RTDE input possible to write.

Data is sent asynchronously to the RTDE interface.

To write several fields at once, construct a ``DataPackage`` from ``RTDEClient::getInputRecipe()``,
fill the fields you care about and pass it to ``sendPackage()``. Fields you leave alone are sent as
zeros. Since the robot decides what type each field has, ``sendPackage()`` is where a value written
with the wrong type is reported:

.. code-block:: c++

   rtde_interface::DataPackage input_pkg(my_client.getInputRecipe());
   input_pkg.setData<uint32_t>("speed_slider_mask", 1);
   input_pkg.setData("speed_slider_fraction", 0.5);
   my_client.getWriter().sendPackage(input_pkg);

.. note::

   The ``RTDEWriter`` will return ``false`` on any writing attempts for fields that have not been
   setup in the ``INPUT_RECIPE``. When no input recipe was provided, all write operations will
   return ``false``.
