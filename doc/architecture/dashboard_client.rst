:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/dashboard_client.rst

.. _dashboard_client:

DashboardClient
===============

For CB3 robots and PolyScope 5 the ``DashboardClient`` wraps the calls on the `Dashboard server <https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/>`_
directly into C++ functions.

For PolyScope X the so-called Robot API `released with PolyScope X 10.11.0
<https://www.universal-robots.com/articles/ur/release-notes/release-note-software-version-1011x/>`_
is used. It offers a subset of the Dashboard Server from previous software versions.

After connecting to the dashboard server by using the ``connect()`` function, dashboard calls can
be done using the ``command...()`` functions such as ``commandCloseSafetyPopup()``. These functions
are blocking and will wait for the necessary action being done. This can involve querying another
call to the dashboard server until the action is done. For example, ``commandPowerOn()`` will block
until the robot reports "Robotmode: RUNNING" or the given timeout is reached.

The return value of those functions indicate whether or not the call was successful. If you want to
get the call's full response, use the ``command...WithResponse()`` calls, instead. They will return
a response struct

.. literalinclude:: ../../include/ur_client_library/ur/dashboard_client_implementation.h
   :language: c++
   :caption: urcl::DashboardResponse
   :start-at: struct DashboardResponse
   :end-at: };

The ``data`` dictionary of that response struct is populated by each command individually. See the
commands' docstrings for details.

The `dashboard_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/dashboard_example.cpp>`_ shows how to use the dashboard client:

.. literalinclude:: ../../examples/dashboard_example.cpp
   :language: c++
   :caption: examples/dashboard_example.cpp
   :linenos:
   :lineno-match:
   :start-at: std::make_unique<DashboardClient>
   :end-at: return 0

CB 3 and PolyScope 5 only
-------------------------

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

PolyScope X only
----------------

Internally, the dashboard client makes calls against a RESTful (**Re**\ presentational **S**\ tate **T**\ ransfer) API. Hence, all responses contain a JSON-encoded string of the received answer. For example, an answer to a pause request could be

.. code-block:: json

    {"state":"PAUSED","message":"Program state changed: PAUSED","details":"Pause successful"}
