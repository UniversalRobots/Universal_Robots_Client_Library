^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_client_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
