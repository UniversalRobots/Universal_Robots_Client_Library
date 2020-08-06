#ur_lib

A C++ library for accessing Universal Robots interfaces. With this library C++-based drivers can be
implemented in order to create external applications leveraging the versatility of Universal Robots
robotic manipulators.

## Library contents
Currently, this library contains the following components:
 * **Basic primary interface:** The primary interface isn't fully implemented at the current state
   and provides only basic functionality. See **TODO** for further information about the primary
   interface.
 * **RTDE interface:** The [RTDE interface](https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/)
   is fully supported by this library. See **TODO** for further information on how to use this
   library as an RTDE client.
 * **Dashboard interface:** The [Dashboard server](https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/) can be accessed directly from C++ through helper functions using this library.
 * **Custom motion streaming:** This library was initially developed as part of the [Universal
   Robots ROS driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). Therefore, it
   also contains a mechanism to do data streaming through a custom socket, e.g. to perform motion
   command streaming.
