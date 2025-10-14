:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/rtde_client.rst

.. _rtde_client:

RTDEClient
==========

The Real Time Data Exchange Client, ``RTDEClient``, class serves as a standalone
`RTDE <https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/>`_
client. To use the RTDE-Client, you'll have to initialize and start it separately:

.. code-block:: c++

   rtde_interface::RTDEClient my_client(ROBOT_IP, notifier, OUTPUT_RECIPE_FILE, INPUT_RECIPE_FILE);
   my_client.init();
   my_client.start();
   while (true)
   {
     std::unique_ptr<rtde_interface::DataPackage> data_pkg = my_client.getDataPackage(READ_TIMEOUT);
     if (data_pkg)
     {
       std::cout << data_pkg->toString() << std::endl;
     }
   }

Upon construction, two recipe files have to be given, one for the RTDE inputs, one for the RTDE
outputs. Please refer to the `RTDE
guide <https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/>`_
on which elements are available.

.. note::

   The recipes can be either passed as a filename or as a list of strings directly. E.g. the
   following will work

   .. code-block:: c++

      rtde_interface::RTDEClient my_client(
        ROBOT_IP,
        notifier,
        {"timestamp", "actual_q"},
        {"speed_slider_mask", "speed_slider_fraction"}
      );

Inside the ``RTDEclient`` data is received in a separate thread, parsed by the ``RTDEParser`` and
added to a pipeline queue.

Right after calling ``my_client.start()``, it should be made sure to read the buffer from the
``RTDEClient`` by calling ``getDataPackage()`` frequently. The Client's queue can only contain a
restricted number of items at a time, so a ``Pipeline producer overflowed!`` error will be raised
if the buffer isn't read frequently enough.

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
   my_client.start();
   while (true)
   {
     std::unique_ptr<rtde_interface::DataPackage> data_pkg = my_client.getDataPackage(READ_TIMEOUT);
     if (data_pkg)
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

.. note::

   The ``RTDEWriter`` will return ``false`` on any writing attempts for fields that have not been
   setup in the ``INPUT_RECIPE``. When no input recipe was provided, all write operations will
   return ``false``.
