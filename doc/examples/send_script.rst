:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/send_script.rst

.. _send_script_example:

Send script example
===================

This example shows how to send arbitrary URScript code to the robot using the
:ref:`primary_client`. It demonstrates both the blocking variant (``sendScriptBlocking``), which
waits for execution feedback, and the non-blocking variant (``sendScript``), which only confirms
that the script has been forwarded to the robot.

The full source code can be found in `send_script.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/send_script.cpp>`_.

Setting up the primary client
-----------------------------

The example connects to the robot's primary interface by creating a ``PrimaryClient``. After
starting the client, the robot's brakes are released so that motion scripts can actually run, and
the safety state is checked before any script is sent:

.. literalinclude:: ../../examples/send_script.cpp
   :language: c++
   :caption: examples/send_script.cpp
   :linenos:
   :lineno-match:
   :start-at: auto notif = comm::INotifier();
   :end-at: }

Sending scripts with execution feedback
---------------------------------------

The ``sendScriptBlocking`` function uploads URScript code to the robot and waits until the robot
reports the result of the execution. The given code can be a fully defined script (with its own
``def ... end`` block) or a snippet that will automatically be wrapped into a function on the
client side. Comments and whitespace-only lines are stripped before the script is sent.

.. literalinclude:: ../../examples/send_script.cpp
   :language: c++
   :caption: examples/send_script.cpp
   :linenos:
   :lineno-match:
   :start-at: const std::string fully_defined_script
   :end-at: client.sendScriptBlocking(fully_defined_script)

If you don't provide a function definition, the library wraps the snippet in one for you. You can
optionally pass a ``script_name`` (used in log messages on both the client and the robot) and a
``start_timeout`` that limits how long the call waits for the robot to confirm that the script has
started. A timeout of ``0`` means "wait indefinitely":

.. literalinclude:: ../../examples/send_script.cpp
   :language: c++
   :caption: examples/send_script.cpp
   :linenos:
   :lineno-match:
   :start-at: client.sendScriptBlocking(R"(textmsg("Successful program execution"))");
   :end-at: client.sendScriptBlocking(R"(textmsg("hello"))", "cool_function_name", std::chrono::milliseconds(0));

Secondary programs can also be uploaded through ``sendScriptBlocking``. Since the robot does not
report execution feedback for secondary programs, the call returns as soon as the script has been
accepted. Note that secondary programs must be *fully defined* by the user (``sec ... end``):

.. literalinclude:: ../../examples/send_script.cpp
   :language: c++
   :caption: examples/send_script.cpp
   :linenos:
   :lineno-match:
   :start-at: std::string secondary_script
   :end-at: client.sendScriptBlocking(secondary_script);

Reporting bad script code
-------------------------

When a script contains errors (e.g. a typo or an undefined symbol), ``sendScriptBlocking`` will
report this back to the caller. The example sends a script that uses an undefined variable
``current_pos`` instead of ``current_pose``, and logs the result:

.. literalinclude:: ../../examples/send_script.cpp
   :language: c++
   :caption: examples/send_script.cpp
   :linenos:
   :lineno-match:
   :start-at: const std::string bad_script_code
   :end-before: // We can also send script code without any checks

Sending scripts without feedback
--------------------------------

For situations where execution feedback is not needed, ``sendScript`` can be
used. It returns ``true`` as soon as the script has been transferred to the robot. The library
performs no further checks, so faulty script code will *not* be reported back here:

.. literalinclude:: ../../examples/send_script.cpp
   :language: c++
   :caption: examples/send_script.cpp
   :linenos:
   :lineno-match:
   :start-at: // We can also send script code without any checks
   :end-at: }
