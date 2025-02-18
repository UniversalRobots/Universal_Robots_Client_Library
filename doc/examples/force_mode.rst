:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/force_mode.rst

Force Mode example
==================

The ``ur_client_library`` supports leveraging the robot's force mode directly. An example on how to
use it can be found in `force_mode_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/force_mode_example.cpp>`_.

In order to utilize force mode, we'll have to create and initialize a driver object
first. We use the ``ExampleRobotWrapper`` class for this example. That starts a :ref:`ur_driver`
and a :ref:`dashboard_client` to communicate with the robot:

.. literalinclude:: ../../examples/force_mode_example.cpp
   :language: c++
   :caption: examples/force_mode_example.cpp
   :linenos:
   :lineno-match:
   :start-at: bool headless_mode = true;
   :end-at: // End of initialization

Start force mode
----------------

After that, we can start force mode by calling the ``startForceMode()`` function:

.. literalinclude:: ../../examples/force_mode_example.cpp
   :language: c++
   :caption: examples/force_mode_example.cpp
   :linenos:
   :lineno-match:
   :start-at: // Start force mode
   :end-at: if (!success)

All parameters for the force mode are included into the ``startForceMode()`` function call. If you
want to change the parameters, e.g. change the forces applied, you can simply call
``startForceMode()`` again with the new parameters.

.. note::
   CB3 robots don't support specifying force_mode's ``gain_scaling``, so there are two different
   functions available.

Once force mode is started successfully, we'll have to send keepalive messages to the robot in
order to keep the communication active:

.. literalinclude:: ../../examples/force_mode_example.cpp
   :language: c++
   :caption: examples/force_mode_example.cpp
   :linenos:
   :lineno-match:
   :start-at: std::chrono::duration<double> time_done(0);
   :end-at: URCL_LOG_INFO("Timeout reached.");

Stop force mode
---------------

Once finished, force_mode can be stopped by calling ``endForceMode()``.

.. literalinclude:: ../../examples/force_mode_example.cpp
   :language: c++
   :caption: examples/force_mode_example.cpp
   :linenos:
   :lineno-match:
   :start-at: endForceMode()
   :end-at: endForceMode()
