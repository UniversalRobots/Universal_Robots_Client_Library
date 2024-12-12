Library architecture
====================

This library implements the bare communication protocols used to communicate with the robot, as
well as a couple of standalone modules to directly use subsets of the library's functionality:

.. toctree::
   :maxdepth: 1

   architecture/dashboard_client
   architecture/reverse_interface
   architecture/rtde_client
   architecture/script_command_interface
   architecture/script_sender
   architecture/trajectory_point_interface
   architecture/ur_driver


Dataflow overview with UrDriver
-------------------------------

The image below shows a rough architecture overview that should help developers to use the different
modules present in this library. Note that this is an incomplete view on the classes involved.

.. image:: images/urcl_architecture.svg
  :width: 100%
  :alt: architecture overview
