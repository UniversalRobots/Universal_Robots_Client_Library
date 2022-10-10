^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_client_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2022-10-10)
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
* Contributors: Cory Crean, Felix Exner, Jørn Bersvendsen, Mads Holm Peters, Martin Jansa, Stefan Scherzinger, Rune Søe-Knudsen, urmahp, urmarp

0.3.2 (2021-09-15)
------------------
* Removed console_bridge dependency (`#74 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/74>`_)
  As log handlers for the client library has been created in the drivers, the console bridge dependency is no longer needed.
* Added "On behalf of Universal Robots A/S" notice (`#81 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/81>`_)
  to all files that have been created by FZI
* Contributors: Felix Exner, Mads Holm Peters

0.3.1 (2021-06-22)
------------------
* Always install package.xml file
* Contributors: Felix Exner

0.3.0 (2021-06-18)
------------------
* Added Cartesian streaming interface `#75 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/75>`_ from UniversalRobots/cartesian_interface
* Add trajectory interface to library `#72 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/72>`_ from fmauch/trajectory_interface
* Refactor reverse interface `#70 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/70>`_ from fmauch/refactor_reverse_interface
* Contributors: Felix Exner, Mads Holm Peters, Tristan Schnell

0.2.2 (2021-05-31)
------------------
* Add reverse_ip parameter to UrDriver (`#52 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/52>`_)
* Make calibration check optionally callable
* Use file= fields for license tags in package.xml (`#63 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/63>`_)
* Install the resources folder instead of the script file directly (`#62 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/62>`_)
* Contributors: Felix Exner, JS00000

0.2.1 (2021-05-06)
------------------
* Run ci also for ROS2 foxy
* Prepare package.xml and cmakelists for ROS2
* Add downstream workspace
* Contributors: Felix Exner, Lennart Puck

0.2.0 (2021-05-03)
------------------
* Add function to set keepalive counter
* Use a non-blocking tcp server for the `ReverseInterface` and `ScriptSender`.
* Added header and control loop definitions.
* Update documentation on ReverseInterface
* Testing improvements:
  + Run coverage on tests
  + Added unit test for tcp server
  + Remove the special boost include dir from the ci pipeline
  + Install boost in test container
* Implement a TCPServer class
* Move script file to resources folder instead of examples/resources
* Add interface function to receive configured RTDE output recipe from driver
* Added loghandler, this enables the possibility for the driver to change the behavior when logging messages with this library.
* Fixed links in README (`#35 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/35>`_)
* Documentation improvements:
  + Added Compiler minimum version table
  + Corrected links to driver's files
  + Add a requirements section to the README
  + Removed unstable warning
  + Added a requirement note about Linux
  + Further elaborated license statements in README
  + readme: missing whitespace
* Merge changes from boost branch
  Changes
  * Added changelog from boost branch
  * fixed build warnings
* Reduce build warnings
  + Removing unused code
  + Bumping the minimum required cmake version to 3.0.2
  + Using const qualifiers for size_t variables used to initialize arrays
* Use const qualifier for array size_t variables
* Removed unused and incomplete code from example
* Bump minimum required cmake version to 3.0.2
* Install package.xml when built with catkin support
* Contributors: Felix Exner, G.A. vd. Hoorn, urmahp

0.1.1 (2020-09-15)
------------------
* readme: missing whitespace
* Further elaborated license statements in README
* Install package.xml when built with catkin support
* Contributors: Felix Exner, G.A. vd. Hoorn

0.1.0 (2020-09-11)
------------------
* initial standalone release
