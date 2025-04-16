:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/setup/robot_setup.rst

.. _robot_setup:

Robot setup
===========

The robot needs to be prepared before it can be used with the client library.

.. tabs::

   .. group-tab:: CB3

      CB3 robots can directly be used with the ur_client_library. No special preparation is needed.


   .. group-tab:: PolyScope 5

      There are two ways the client library can be enabled to send command to the robot.

      - **Remote control**: There the full control is given to the client library and enables it to e.g. power on and off, brake release, load PolyScope programs and send URScript programs directly to the controller.
      - **External Control URCap**: There the control to power on, off and start programs etc. still remains on the teach pendant. The External Control URCap injects the needed URScript code from the client library. This also makes it possible to combine the use of the client library and other PolyScope program nodes, like standard moves or other third-party URCaps.

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
      #. Enable the Dashboard Server, Primary Client Interface and Real-Time Data Exchange (RTDE) interfaces.
      #. Lock the menu and press exit.

      .. image:: ../images/initial_setup_images/services_polyscope5.png
            :width: 600
            :alt: Screenshot from PolyScope 5.xx services menu.


   .. group-tab:: PolyScope X

      **Enable services:**

      #. Go to the hamburger menu -> settings.
      #. Go to Security -> Services.
      #. Unlock the menu using the admin password.
      #. Enable the Primary Client Interface and Real-Time Data Exchange (RTDE) interfaces.
      #. Lock the menu and press exit.

      .. image:: ../images/initial_setup_images/services_polyscopex.png
         :width: 600
         :alt: Screenshot from PolyScope X screen.

.. _install_urcap:

URCap installation
------------------

To use the client library with a robot, you'll have to have the **External Control URCap**
installed. It allows a remote PC to control the robot externally. Generally, you will launch the
driver on the remote PC and then start a program from the tech pendant to connect to the remote
application.

.. tabs::

   .. group-tab:: CB3

      .. note::

         A minimal PolyScope version of 3.14.3 is required to use this URCap

      The latest release can be downloaded from `its own repository <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases>`_.

      To install the URCap you first have to copy it to the robot's **programs** folder which can be done either
      via scp or using a USB stick.

      On the welcome screen select *Setup Robot* and then *URCaps* to enter the URCaps installation
      screen.


      .. image:: ../images/initial_setup_images/cb3_01_welcome.png
         :target: initial_setup_images/cb3_01_welcome.png
         :alt: Welcome screen of a CB3 robot


      There, click the little plus sign at the bottom to open the file selector. There you should see
      all urcap files stored inside the robot's programs folder or a plugged USB drive.  Select and open
      the **externalcontrol-X.Y.Z.urcap** file and click *open*. Your URCaps view should now show the
      **External Control** in the list of active URCaps and a notification to restart the robot. Do that
      now.


      .. image:: ../images/initial_setup_images/cb3_05_urcaps_installed.png
         :target: initial_setup_images/cb3_05_urcaps_installed.png
         :alt: URCaps screen with installed urcaps


      After the reboot you should find the **External Control** URCaps inside the *Installation* section.
      For this select *Program Robot* on the welcome screen, select the *Installation* tab and select
      **External Control** from the list.


      .. image:: ../images/initial_setup_images/cb3_07_installation_excontrol.png
         :target: initial_setup_images/cb3_07_installation_excontrol.png
         :alt: Installation screen of URCaps


      Here you'll have to setup the IP address of the external PC which will be running the remote
      application.
      Note that the robot and the external PC have to be in the same network, ideally in a direct
      connection with each other to minimize network disturbances. The custom port should be left
      untouched for now.


      .. image:: ../images/initial_setup_images/cb3_10_prog_structure_urcaps.png
         :target: initial_setup_images/cb3_10_prog_structure_urcaps.png
         :alt: Insert the external control node


      To use the new URCaps, create a new program and insert the **External Control** program node into
      the program tree


      .. image:: ../images/initial_setup_images/cb3_11_program_view_excontrol.png
         :target: initial_setup_images/cb3_11_program_view_excontrol.png
         :alt: Program view of external control


      If you click on the *command* tab again, you'll see the settings entered inside the *Installation*.
      Check that they are correct, then save the program. Your robot is now ready to be used together with
      this driver

   .. group-tab:: PolyScope 5

      .. note::

         A minimal PolyScope version of 5.9.4 is required to use this URCap

      The latest release can be downloaded from `its own repository <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases>`_.

      To install it you first have to copy it to the robot's **programs** folder which can be done either
      via scp or using a USB stick.

      On the welcome screen click on the hamburger menu in the top-right corner and select *Settings* to enter the robot's setup.  There select *System* and then *URCaps* to enter the URCaps installation screen.


      .. image:: ../images/initial_setup_images/es_01_welcome.png
         :target: initial_setup_images/es_01_welcome.png
         :alt: Welcome screen of an e-Series robot


      There, click the little plus sign at the bottom to open the file selector. There you should see
      all urcap files stored inside the robot's programs folder or a plugged USB drive.  Select and open
      the **externalcontrol-X.Y.Z.urcap** file and click *open*. Your URCaps view should now show the
      **External Control** in the list of active URCaps and a notification to restart the robot. Do that
      now.


      .. image:: ../images/initial_setup_images/es_05_urcaps_installed.png
         :target: initial_setup_images/es_05_urcaps_installed.png
         :alt: URCaps screen with installed urcaps


      After the reboot you should find the **External Control** URCaps inside the *Installation* section.
      For this select *Program Robot* on the welcome screen, select the *Installation* tab and select
      **External Control** from the list.


      .. image:: ../images/initial_setup_images/es_07_installation_excontrol.png
         :target: initial_setup_images/es_07_installation_excontrol.png
         :alt: Installation screen of URCaps


      Here you'll have to setup the IP address of the external PC which will be running the remote
      application. Note that the robot and the external PC have to be in the same network, ideally in a
      direct connection with each other to minimize network disturbances. The custom port should be left
      untouched for now.


      .. image:: ../images/initial_setup_images/es_10_prog_structure_urcaps.png
         :target: initial_setup_images/es_10_prog_structure_urcaps.png
         :alt: Insert the external control node


      To use the new URCaps, create a new program and insert the **External Control** program node into
      the program tree


      .. image:: ../images/initial_setup_images/es_11_program_view_excontrol.png
         :target: initial_setup_images/es_11_program_view_excontrol.png
         :alt: Program view of external control


      If you click on the *command* tab again, you'll see the settings entered inside the *Installation*.
      Check that they are correct, then save the program. Your robot is now ready to be used together with
      this driver.

   .. group-tab:: PolyScope X

      The latest release can be downloaded from `GitHub (URCapX)
      <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX/releases>`_.

      To install it you first have to copy it to a USB stick and plug that into the robot's teach
      pendant.

      On the welcome screen click on the hamburger menu in the top-left corner and select *System
      Manager* to enter the robot's setup.  There select *URCaps* to enter the URCaps installation
      screen. You'll have to press *unlock* and enter the admin password to be able to install new
      URCaps.


      .. image:: ../images/initial_setup_images/px_01_welcome.png
         :target: initial_setup_images/px_01_welcome.png
         :alt: Welcome screen of a PolyScope X robot


      There, click the "+URCap" button at the top-right to open the file selector. There you should see
      all urcap files stored on a plugged USB drive.  Select and open
      the **external-control-X.Y.Z.urcapx** file and click *Confirm*. A popup should ask you to
      confirm the installation.

      .. image:: ../images/initial_setup_images/px_05_urcaps_installed.png
         :target: initial_setup_images/px_05_urcaps_installed.png
         :alt: URCaps screen with installed urcaps

      Upon confirmation, the external-control URCap should be listed as installed.

      After the robot rebooted you should find the **External Control** URCapX on the *Application*
      Screen. If you open it, you will get to its configuration screen.

      .. image:: ../images/initial_setup_images/px_07_installation_excontrol.png
         :target: initial_setup_images/px_07_installation_excontrol.png
         :alt: Configuration screen of External Control URCapX


      Here you'll have to setup the IP address of the external PC which will be running the remote
      application. Note that the robot and the external PC have to be in the same network, ideally in a
      direct connection with each other to minimize network disturbances. The custom port should be left
      untouched for now.

      To use the new URCaps, create a new program and insert the **External Control** program node into
      the program tree

      .. image:: ../images/initial_setup_images/px_10_prog_structure_urcaps.png
         :target: initial_setup_images/px_10_prog_structure_urcaps.png
         :alt: Insert the external control node


      .. image:: ../images/initial_setup_images/px_11_program_view_excontrol.png
         :target: initial_setup_images/px_11_program_view_excontrol.png
         :alt: Program view of external control


      If you select the External Control program node, you'll see a button "Update program"
      appearing next to it. With the external application started, press it to receive the script
      code.

      .. warning::

         Support for PolyScope X isn't fully developed, yet. Please consider using External Control
         with PolyScope X as an open beta.
