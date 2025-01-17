.. _dashboard_client:

DashboardClient
===============

The ``DashboardClient`` wraps the calls on the `Dashboard server <https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/>`_
directly into C++ functions.

After connecting to the dashboard server by using the ``connect()`` function, dashboard calls can be
sent using the ``sendAndReceive()`` function. Answers from the dashboard server will be returned as
string from this function. If no answer is received, a ``UrException`` is thrown.

Some functions are also wrapped into ``command...()`` functions such as
``commandCloseSafetyPopup()``. These functions are blocking and will wait for the necessary action
being done. This can involve querying another call to the dashboard server until the action is
done. For example, ``commandPowerOn()`` will block until the robot reports "Robotmode: RUNNING" or
the given timeout is reached.

The `dashboard_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/dashboard_example.cpp>`_ shows how to use this class:

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: std::make_unique<DashboardClient>
   :end-at: my_dashboard->commandCloseSafetyPopup();
