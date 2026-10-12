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

To read data received by the RTDE client, it has to be polled. See the :ref:`rtde_client` section
for details on two possible strategies. In this example, we do not use background read and instead
fetch data synchronously. Hence, we pass ``false`` to the ``start()`` method.

.. literalinclude:: ../../examples/rtde_client.cpp
   :language: c++
   :caption: examples/rtde_client.cpp
   :linenos:
   :lineno-match:
   :start-at: auto data_pkg = std::make_unique<rtde_interface::DataPackage>(my_client.getOutputRecipe());
   :end-before: // Change the speed slider

The loop reuses a package built from the negotiated output recipe, keeping the normal data receive
path allocation-free. Null pointers allocate a package, and foreign-recipe repair may allocate;
error handling, non-data messages and reconnection are outside this guarantee. The recipe only
names the fields, so the first read applies their negotiated types in place without allocation.

In our main loop, we wait for a new data package to arrive using the blocking read method. Once
received, data from the received package can be accessed using the ``getData()`` method of the
``DataPackage`` object. This method takes the key of the data to be accessed as a parameter and
returns the corresponding value.

.. note:: The key used to access data has to be part of the output recipe used to initialize the RTDE
   client. ``getData()`` returns ``false`` for an unknown key. If the type of the passed
   variable doesn't match the type the robot reported for that field, it throws
   ``std::bad_variant_access``.

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

.. note:: Every successful ``send...`` call updates the pending buffer and notifies the writer
   thread. Calls may be coalesced before transmission; they are not queued as separate packages.
   To submit several inputs together, use ``createInputDataPackage()`` after ``init()``, fill the
   fields and pass the package to ``sendPackage()``. Separate helper calls can otherwise be
   transmitted between updates. Neither API confirms delivery to the robot; see the
   :ref:`rtde_roundtrip_example` for verification using robot outputs.
