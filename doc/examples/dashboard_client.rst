:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/dashboard_client.rst

Dashboard client example
========================

This example shows how to use the builtin `Dashboard server <https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/>`_ to communicate with a robot.

.. note::

   The Dashboard Server euqivalent for PolyScope X is called the Robot API. In the client library
   it is accessed through the DashboardClient, as well, as it is meant as a replacement for the
   Dashboard Server. It doesn't offer full feature parity at the time of writing and is available
   only from Software 10.11.0 on.


The `dashboard_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/dashboard_example.cpp>`_ shows how to use this class:

.. note:: For the following example to work, the robot has to be in *remote control mode*. CB3
   robots don't require that.

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // Query the robot information
   :end-at: my_dashboard->commandCloseSafetyPopup();

At first, a ``DashboardClient`` object is created with the IP address of the robot and a
DashboardClient implementation policy. That policy can be either ``POLYSCOPE_X`` or ``G5``. To
generalize this, the robot's sofware version is queried using the primary interface beforehand.
Then, the client is connected to the robot. After that, the client sends a command to the robot and
receives the answer.

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
   :end-before: // Load existing program

