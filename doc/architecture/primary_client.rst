:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/primary_client.rst

.. _primary_client:

PrimaryClient
=============

The Primary Client serves as an interface to the robot's `primary interface <https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html>`_, present on port 30001.
The ``PrimaryClient`` class supports, among other things, sending URScript code for execution on the robot through the primary interface. Currently it offers two methods of script execution: ``sendScript`` and ``sendScriptBlocking``.

Script execution without feedback
---------------------------------
Method signature:

.. code-block:: c++

   bool sendScript(std::string program);

The ``sendScript`` method will accept valid URScript code, and send it to the robot through the primary interface. This is a non-blocking method, as it will return as soon as the program has been transferred to the robot. It returns true when the program is successfully transferred to the robot, and false otherwise.
There is no feedback on whether the program is actually executed on the robot.

Script execution with feedback
------------------------------
Method signature:

.. code-block:: c++

   bool sendScriptBlocking(
     const std::string& program,
     const std::string& script_name = "",
     const std::chrono::milliseconds start_timeout = std::chrono::seconds(1),
     const bool fail_on_warnings = true,
     const bool retry_on_readonly_interface = true
   );

| The ``sendScriptBlocking`` method will also accept valid URScript code, but blocks until the execution result of the given program is available.
| Prior to transferring the program it will first check that the robot is in a state where it can execute programs, otherwise an exception is thrown.
| If the robot is ready, the program is then transferred, and the method will wait for the robot to report that the program has either started, finished or encountered an error.
| If the program has not started within the given ``start_timeout``, the method throws an exception.
| If the robot encounters an error or runtime exception during program execution the method also throws an exception.
| If ``fail_on_warnings`` is true, it will also throw an exception, if the robot reports a warning during program execution. Note: protective stops are reported as warnings by the robot.
| If ``retry_on_readonly_interface`` is true, the method will restart the primary interface and retry sending the program, if the primary interface is read-only. It will retry once, and if the interface is still read-only, an exception will be thrown. If false, the exception will be thrown, without restarting/retrying.
| If no exceptions are thrown, the script has been executed successfully.
| This method also accepts secondary programs, but no feedback is available for those, so it will behave similarly to the ``sendScript`` method in those cases, except for the pre-transfer checks.
| The exact exceptions that are thrown in various cases can be seen in the `primary client header file <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/include/ur_client_library/primary/primary_client.h>`_.
| Note: This method clears all stored error codes in the client during execution.
