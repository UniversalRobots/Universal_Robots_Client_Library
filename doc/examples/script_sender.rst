:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/script_sender.rst

.. _script_sender_example:

Script Sender example
=====================

The following example creates a ``ScriptSender`` listening on port ``12345`` and sends the script
``textmsg("Hello, World!")`` when requested. A fully compilable example can be found in `script_sender.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/script_sender.cpp>`_

.. literalinclude:: ../../examples/script_sender.cpp
   :language: c++
   :caption: examples/script_sender.cpp
   :linenos:
   :lineno-match:
   :start-at: constexpr uint32_t PORT
