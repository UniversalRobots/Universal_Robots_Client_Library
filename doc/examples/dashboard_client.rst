:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/dashboard_client.rst

Dashboard client example
========================

This example shows how to use the builtin `Dashboard server <https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/>`_ to communicate with a robot.

.. note::

   The Dashboard Server is only available on CB3 and e-Series robots. It is not available on
   PolyScope X.


The `dashboard_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/dashboard_example.cpp>`_ shows how to use this class:

.. note:: For the following example to work on an e-Series (PolyScope 5) robot, the robot has to be
   in *remote control mode*.

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: std::make_unique<DashboardClient>
   :end-at: my_dashboard->commandCloseSafetyPopup();

At first, a ``DashboardClient`` object is created with the IP address of the robot. Then, the
client is connected to the robot. After that, the client sends a command to the robot and receives
the answer.

Some commands support getting the response from the robot. For example, the
``commandPolyScopeVersion()`` function:

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // Get the PolyScope version
   :end-at: URCL_LOG_INFO(version.c_str());


The ``DashboardClient`` can easily be used to cycle through the robot's states, for example for
initialization:

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // Power it on
   :end-at: // Load existing program

All commands are blocking and will wait for the necessary action being done. The dashboard server's
response will be compared with an expected response. For example, when calling
``commandPowerOn(timeout)``, it is checked that the dashboard server is answering ``"Powering on"`` and
then it is queried until the robot reports ``"Robotmode: IDLE"`` or until the timeout is reached.
The example contains more commands that follow the same scheme.


If you want to send a query / command to the dashboard server and only want to receive the
response, you can use the ``sendAndReceive()`` function:

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // Make a raw request and save the response
   :end-at: URCL_LOG_INFO("Program state: %s", program_state.c_str());

For checking the response against an expected regular expression use ``sendRequest()``:

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // The response can be checked with a regular expression
   :end-at: URCL_LOG_INFO("Power off command success: %d", success);


