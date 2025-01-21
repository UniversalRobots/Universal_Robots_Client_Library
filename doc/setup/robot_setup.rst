.. _robot_setup:

Robot setup
===========

In order to use the client library with a robot, that robot has to be setup.

.. tabs::

   .. tab:: PolyScope 5

      **Enable remote control:**

      #. Go to the hamburger menu -> settings.
      #. Go to System -> Remote control.
      #. Unlock the menu using the admin password, and enable Remote Control.
      #. Press exit.
      #. The robot can now be toggled between local and remote control in the upper right hand corner.

      .. image:: ../images/initial_setup_images/remote_control.png
         :width: 600
         :alt: Screenshot showing remote control toggle.


      **If using PolyScope 5.10 or greater: Enable services**

      #. Go to the hamburger menu -> settings.
      #. Go to Security -> Services.
      #. Unlock the menu using the admin password.
      #. Enable the Dashboard Server, Primary Client Interface, Secondary Client Interface and Real-Time Data Exchange (RTDE) interfaces.
      #. Lock the menu and press exit.

      .. image:: ../images/initial_setup_images/services_polyscope5.png
            :width: 600
            :alt: Screenshot from PolyScope 5.xx services menu.


   .. tab:: PolyScope X

      **Enable services:**

      #. Go to the hamburger menu -> settings.
      #. Go to Security -> Services.
      #. Unlock the menu using the admin password.
      #. Enable the Primary Client Interface, Secondary Client Interface and Real-Time Data Exchange (RTDE) interfaces.
      #. Lock the menu and press exit.

      .. image:: ../images/initial_setup_images/services_polyscopex.png
         :width: 600
         :alt: Screenshot from PolyScope X screen.


URCap installation
------------------

To use the client library with a robot, you'll have to have the **External Control URCap**
installed. It allows a remote PC to control the robot externally. Generally, you will launch the
driver on the remote PC and then start a program from the tech pendant to connect to the remote
application.

.. tabs::

   .. tab:: CB3

      .. note::

         A minimal PolyScope version of 3.14.3 is required to use this URCap

      The latest release can be downloaded from `its own repository <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases>`_.

      For installing the necessary URCap and creating a program, please see the individual tutorials on
      how to :ref:`setup a CB3 robot <install-urcap-cb3>`.

   .. tab:: e-Series

      .. note::

         A minimal PolyScope version of 5.9.4 is required to use this URCap

      The latest release can be downloaded from `its own repository <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases>`_.

      For installing the necessary URCap and creating a program, please see the individual tutorials on
      how to :ref:`setup an e-Series robot <install-urcap-e-series>`.

   .. tab:: PolyScope X

      .. warning::

         Support for PolyScope X isn't fully developed, yet. Please consider using External Control
         with PolyScope X an open beta.

      For details on installing the External Control URCapX, please see https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX
