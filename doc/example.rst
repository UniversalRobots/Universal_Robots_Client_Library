.. _example-driver:

Example driver
==============
In the ``examples`` subfolder you will find a minimal example of a running driver. It starts an
instance of the ``UrDriver`` class and prints the RTDE values read from the controller. To run it make
sure to

* have an instance of a robot controller / URSim running at the configured IP address (or adapt the
  address to your needs)
* run it from the package's main folder (the one where this README.md file is stored), as for
  simplicity reasons it doesn't use any sophisticated method to locate the required files.
