:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/rtde_client.rst

.. _rtde_client_example:

RTDE Client example
===================

This example shows how to use the ``RTDEClient`` class for the robot's `Real-Time Data Exchange
(RTDE) interface
<https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/>`_.

The RTDE client has to be initialized with a list of keys that should be streamed from the robot
and a list of keys that should be sent to the robot. The client will then start a background thread
establishing communication.

In this example, those keys are stored in two text files relative to this repository's root:
``rtde_input_keys.txt`` and ``rtde_output_keys.txt``. The example will read those files and use them
to initialize the RTDE client.

.. literalinclude:: ../../examples/rtde_client.cpp
   :language: c++
   :caption: examples/rtde_client.cpp
   :linenos:
   :lineno-match:
   :start-at: const std::string OUTPUT_RECIPE
   :end-at: const std::string INPUT_RECIPE

Creating an RTDE Client
-----------------------

An RTDE client can be directly created passing the robot's IP address, a ``INotifier`` object, an
output and an input recipe. Optionally, a communication frequency can be passed as well. If that is
omitted, RTDE communication will be established at the robot's control frequency.

.. literalinclude:: ../../examples/rtde_client.cpp
   :language: c++
   :caption: examples/rtde_client.cpp
   :linenos:
   :lineno-match:
   :start-at: comm::INotifier notifier;
   :end-at: my_client.init();


Reading data from the RTDE client
---------------------------------

To read data received by the RTDE client, it has to be polled. For this two modes are available:

- **Background read**: When background read is enabled (default), the RTDE client will start a
  background thread that continuously reads data from the robot. The latest data package can be
  fetched using the ``getDataPackage()`` method. This method returns immediately with the latest
  data package received from the robot. If no data has been received since last calling this
  function, it will block for a specified timeout waiting for new data to arrive.

  .. note:: This methods allocates a new data package on each call. We recommend using the blocking
     read method explained below.
- **Blocking synchronous read**: When background read is not enabled, data can (and has to be)
  fetched using the ``getDataPackageBlocking()`` method. This call waits for a new data package to
  arrive and parses that into the passed ``DataPackage`` object. This has to be called with the
  RTDE control frequency, as the robot will shutdown RTDE communication if data is not read by the
  client.

Which of the above strategies is used can be specified when starting RTDE communication using the
``start()`` method. In our example, we do not use background read and instead fetch data
synchronously. Hence, we pass ``false`` to the ``start()`` method.

.. literalinclude:: ../../examples/rtde_client.cpp
   :language: c++
   :caption: examples/rtde_client.cpp
   :linenos:
   :lineno-match:
   :start-at: std::unique_ptr<rtde_interface::DataPackage> data_pkg =
   :end-before: // Change the speed slider

In our main loop, we wait for a new data package to arrive using the blocking read method. Once
received, data from the received package can be accessed using the ``getData()`` method of the
``DataPackage`` object. This method takes the key of the data to be accessed as a parameter and
returns the corresponding value.

.. note:: The key used to access data has to be part of the output recipe used to initialize the RTDE
   client. Passing a string literal, e.g. ``"actual_q"``, is possible but not recommended as it is
   converted to an ``std::string`` automatically, causing heap allocations which should be avoided
   in Real-Time contexts.

Writing Data to the RTDE client
-------------------------------

In this example, we use the RTDE client to oscillate the speed slider on the teach pendant between
0 and 1. While this doesn't bear any practical use it shows how sending data to the RTDE interface
works.

To send data to the RTDE client, we can use ``RTDEWriter`` object stored in the RTDE client. This
has methods implemented for each data type that can be sent to the robot. The input recipe used to
initialize the RTDE client has to contain the keys necessary to send that specific data.

.. literalinclude:: ../../examples/rtde_client.cpp
   :language: c++
   :caption: examples/rtde_client.cpp
   :linenos:
   :lineno-match:
   :start-at: my_client.getWriter().sendSpeedSlider
   :end-at: }


.. note:: Many RTDE inputs require setting up the data key and a mask key. That is done
   internally, but the mask keys have to be part of the input recipe, as well. See the `RTDE guide
   <https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/>`_
   for more information.

.. note:: Every ``send...`` call to the RTDEWriter triggers a package sent to the robot. If you
   want to modify more than one input at a time, it is recommended to use the ``sendPackage()``
   method. That allows setting up the complete data package with its input recipe and sending that
   to the robot at once.

