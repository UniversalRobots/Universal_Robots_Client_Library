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

   architecture/dashboard_client
   architecture/reverse_interface
   architecture/rtde_client
   architecture/script_command_interface
   architecture/script_sender
   architecture/trajectory_point_interface


The ``UrDriver``'s modules will be explained in the following.



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
