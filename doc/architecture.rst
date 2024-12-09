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
   architecture/ur_driver
