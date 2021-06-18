^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_client_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
