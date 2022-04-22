^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_client_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-04-22)
------------------
* Support starting the driver, before the robot is booted (`#98 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/98>`_)
* Clear the queue when consumer reads from it (`#96 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/96>`_)
* Fix build with newer glibc
* Doxygen check (`#77 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/77>`_)
* Added target_frequency to RTDEClient (`#85 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/85>`_)
* Removed console_bridge dependency (`#74 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/74>`_)
* Added "On behalf of Universal Robots A/S" notice (`#81 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/81>`_)
  to all files that have been created by FZI
* Always install package.xml file (`#78 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/78>`_)
* register package with ament index
* Corrected smaller doxygen errors
* Added rosdoc_lite check
* Contributors: Cory Crean, Felix Exner, Jørn Bersvendsen, Mads Holm Peters, Martin Jansa, Stefan Scherzinger

1.0.0 (2021-06-18)
------------------
* Added Cartesian streaming interface `#75 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/75>`_
* Added trajectory forwarding interface `#72 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/72>`_
* Refactored Reverse interface `#70 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/70>`_ from fmauch/refactor_reverse_interface
* Added option for robot_ip as runtime argument for rtde_test (`#71 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/71>`_)
* Added reverse_ip parameter (`#52 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/52>`_)
* Move calibration check out of constructor. `#65 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/65>`_ from fmauch/calibration_check_optional
* Install the resources folder instead of the script file directly (`#62 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/62>`_)
* Use a non-blocking tcp server for the `ReverseInterface` and `ScriptSender`. `#46 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/46>`_ from fmauch/tcp_server
* Added LogHandler `#40 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/40>`_ from urmahp/logging_feature
* Fixed links in README (`#35 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/35>`_)
* Contributors: Felix Exner, G.A. vd. Hoorn, JS00000, Lennart Puck, Mads Holm Peters, Tristan Schnell

0.1.1 (2020-09-15)
------------------
* readme: missing whitespace
* Further elaborated license statements in README
* Install package.xml when built with catkin support
* Contributors: Felix Exner, G.A. vd. Hoorn

0.1.0 (2020-09-11)
------------------
* initial standalone release
