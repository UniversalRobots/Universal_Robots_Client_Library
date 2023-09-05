^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_client_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2023-09-05)
------------------
* Add support for setting socket max num tries and reconnect timeout (`#172 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/172>`_)
* Unify socket open (`#174 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/174>`_)
* Added handling of spline interpolation with end point velocities (`#169 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/169>`_)
* Throws exception if the URScript file doesn't exists (`#173 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/173>`_)
* Added check to ensure receive timeout isn't overwritten (`#171 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/171>`_)
* Added RTDEClient constructor with vector recipes (`#143 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/143>`_)
* Only warn if system is not setup for FIFO scheduling (`#170 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/170>`_)
* Ensuring that the Timestamp is always in the output recipe (`#168 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/168>`_)
* CI: Add Iron to CI tests (`#167 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/167>`_)
* Add issue templates for bugs and features (`#166 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/166>`_)
* Updated license (`#164 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/164>`_)
* Bugfixes for spline interpolation (`#162 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/162>`_)
   * Add separate rounding in the conversion from float to int32
   * Add more debug printout for splines
   * Add Copying flight reports if CI fails
   * Update ursim mininum version in start_ursim.sh
* Fix the prerelease ci for Melodic (`#163 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/163>`_)
* Contributors: Dag-Are Trydal, Felix Exner, Felix Exner (fexner), Mads Holm Peters, Michael Eriksen, RobertWilbrandt, Rune Søe-Knudsen, urmahp, urrsk

1.3.2 (2023-07-13)
------------------
* Add a cmake option to activate address sanitizers (`#146 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/146>`_)
* Install start ursim (`#155 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/155>`_)
* Add spline interpolation on robot (`#151 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/151>`_)
* Add codecov.yml to exclude test and examples folders (`#152 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/152>`_)
* Make URSim log files available as artifacts also for the CI-industrial (`#153 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/153>`_)
* Remove Foxy from CI
* Add a script to run the examples instead of run-parts
* Add SaveLog command to the Dashboard client
* Make URSim log files available as artifacts
* Specifically set RTDE pipeline producer to FIFO scheduling (`#139 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/139>`_)
* Added support for force_mode, freedrive and tool contact (`#138 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/138>`_)
* Docs: Update link to ros_industrial_cmake_boilerplate
* Added tests for the comm classes (`#129 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/129>`_)
* Changed num_retries from static to an unsigned int (`#136 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/136>`_)
* Build downstream humble version from humble branch (`#132 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/132>`_)
* Contributors: Felix Exner, Mads Holm Peters, Rune Søe-Knudsen, Robert Wilbrandt

1.3.1 (2022-11-30)
------------------
* CI: Add a prerelease check that calls `bloom-generate` (`#134 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/134>`_)
* Contributors: Felix Exner

1.3.0 (2022-11-28)
------------------
* Dashboard commands, Docker Image and CI step for running the examples `#127 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/127>`_
* Added tests for the rtde interface clasess (`#125 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/125>`_)
* Fix unique_ptr type (`#124 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/124>`_)
* Fix 'BEGIN_REPLACE' - used in tool_communication (copy `#101 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/101>`_) (`#120 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/120>`_)
  (cherry picked from commit f7ce9f73181848f3957c660647fac0e5325862b9)
  Co-authored-by: rxjia <60809735+rxjia@users.noreply.github.com>
* Contributors: Felix Exner, Mads Holm Peters, RobertWilbrandt, Rune Søe-Knudsen, mergify[bot], urmarp, urrsk

1.2.0 (2022-10-04)
------------------
* Initialized receive timeout and changed exception to warning (`#118 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/118>`_)
* Added tests for the control interface classes (`#112 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/112>`_)
* Added note about Polyscope version requirement
* Added tcp_offset
* Added interface for forwarding script commands to the robot, that is … (`#111 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/111>`_)
* Fixed parsing of incomming packages when using rtde protocol v1 (`#114 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/114>`_)
  The received rtde packages should be parsed slightly different whether we use protocol v1 or v2.
* Add codecov step (`#116 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/116>`_)
* Added humble build
* Fixed downstream test instructions
* Update atomicops.h (`#117 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/117>`_)
  Fix the url in the comment regarding POSIX semaphores to fix error in the CI
* Make the read during boot depend on the frequency of the robot controller (`#102 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/102>`_)
* Ignore debian folder in check_links (`#100 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/100>`_)
  Otherwise this job raises an error in the release repository.
* Contributors: Felix Exner, Mads Holm Peters, Rune Søe-Knudsen, urmahp, urmarp

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
