Library architecture
====================

The image below shows a rough architecture overview that should help developers to use the different
modules present in this library. Note that this is an incomplete view on the classes involved.

.. image:: images/dataflow.svg
  :width: 100%
  :alt: Data flow


The core of this library is the ``UrDriver`` class which creates a
fully functioning robot interface. For details on how to use it, please see the
:ref:`example-driver` section.

.. toctree::
   :maxdepth: 1

   architecture/script_sender

The ``UrDriver``'s modules will be explained in the following.


ReverseInterface
----------------

The ``ReverseInterface`` opens a TCP port on which a custom protocol is implemented between the
robot and the control PC. The port can be specified in the class constructor.

It's basic functionality is to send a vector of floating point data together with a mode. It is
meant to send joint positions or velocities together with a mode that tells the robot how to
interpret those values (e.g. ``SERVOJ``, ``SPEEDJ``). Therefore, this interface can be used to do
motion command streaming to the robot.

In order to use this class in an application together with a robot, make sure that a corresponding
URScript is running on the robot that can interpret the commands sent. See `this example
script <../resources/external_control.urscript>`_ for reference.

Also see the :ref:`ScriptSender` for a way to define the corresponding URScript on the
control PC and sending it to the robot upon request.

Other public interface functions
--------------------------------

This section shall explain the public interface functions that haven't been covered above


check_calibration()
^^^^^^^^^^^^^^^^^^^

This function opens a connection to the primary interface where it will receive a calibration
information as the first message. The checksum from this calibration info is compared to the one
given to this function. Connection to the primary interface is dropped afterwards.

sendScript()
^^^^^^^^^^^^

This function sends given URScript code directly to the secondary interface. The
``sendRobotProgram()`` function is a special case that will send the script code given in the
``RTDEClient`` constructor.

DashboardClient
---------------

The ``DashboardClient`` wraps the calls on the `Dashboard server <https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/>`_
directly into C++ functions.

After connecting to the dashboard server by using the ``connect()`` function, dashboard calls can be
sent using the ``sendAndReceive()`` function. Answers from the dashboard server will be returned as
string from this function. If no answer is received, a ``UrException`` is thrown.

Note: In order to make this more useful developers are expected to wrap this bare interface into
something that checks the returned string for something that is expected. See the
`DashboardClientROS <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/include/ur_robot_driver/dashboard_client_ros.h>`_ as an example.
