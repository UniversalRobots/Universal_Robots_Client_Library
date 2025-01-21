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


Internally, the RTDE client uses the same producer / consumer architecture as show in the
:ref:`primary_pipeline_example` example. However, it doesn't have a consumer thread, so data has to
be read by the user to avoid the pipeline's queue from overflowing.

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

An RTDE data package containing every key-value pair from the output recipe can be fetched using
the ``getDataPackage()`` method. This method will block until a new package is available.


Reading data from the RTDE client
---------------------------------

Once the RTDE client is initialized, we'll have to start communication separately. As mentioned
above, we'll have to read data from the client once communication is started, hence we start
communication right before a loop reading data.

.. literalinclude:: ../../examples/rtde_client.cpp
   :language: c++
   :caption: examples/rtde_client.cpp
   :linenos:
   :lineno-match:
   :start-at: // Once RTDE communication is started
   :end-before: // Change the speed slider

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


.. note:: Many RTDE inputs require setting up the data key and a mask key. See the `RTDE guide
   <https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/>`_
   for more information.
