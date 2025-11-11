# Universal Robots Client Library

A C++ library for accessing Universal Robots interfaces. With this library C++-based drivers can be
implemented in order to create external applications leveraging the versatility of Universal Robots
robotic manipulators.

The library has no external dependencies besides the standard C++ libraries such as ROS, or boost
to make it easy to integrate and maintain. It also serves as the foundation for the ROS and ROS 2
drivers.

---

<!-- markdownlint-disable MD033 -->
<div align="center">
  <img src="doc/resources/family_photo.png" alt="Universal Robot family" style="width: 90%;"/>
 </div>

## Requirements

* **Polyscope** (The software running on the robot controller) version **3.14.3** (for CB3-Series),
  or **5.9.4** (for e-Series) or **10.7.0** (For PolyScope X) or higher. If you use an older
  Polyscope version it is suggested to update your robot. If for some reason (please tell us in the
  issues why) you cannot upgrade your robot, please see the [version compatibility
  table](doc/polyscope_compatibility.rst) for a compatible tag.
* The library requires an implementation of **POSIX threads** such as the `pthread` library

## Build instructions

See [Build / installation](doc/installation.rst)

## License

The majority of this library is licensed under the Apache-2.0 licensed. However, certain parts are
licensed under different licenses:

* The queue used inside the communication structures is originally written by Cameron Desrochers
  and is released under the BSD-2-Clause license.
* The semaphore implementation used inside the queue implementation is written by Jeff Preshing and
  licensed under the zlib license

While the main `LICENSE` file in this repository contains the Apache-2.0 license used for the
majority of the work, the respective libraries of third-party components reside together with the
code imported from those third parties.

## Library contents

Currently, this library contains the following components:

* **Basic primary interface:** The primary interface isn't fully implemented at the current state
  and provides only basic functionality. See [A word on the primary / secondary
  interface](#a-word-on-the-primary--secondary-interface) for further information about the primary
  interface.
* **RTDE interface:** The [RTDE interface](https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/)
  is fully supported by this library. See
  [RTDEClient](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/architecture.html#rtdeclient)
  for further information on how
  to use this library as an RTDE client.
* **Dashboard interface:** The [Dashboard
  server](https://www.universal-robots.com/articles/ur-articles/dashboard-server-e-series-port-29999/)
  can be accessed directly from C++ through helper functions using this library.
* **Custom motion streaming:** This library was initially developed as part of the [Universal
  Robots ROS driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). Therefore, it
  also contains a mechanism to do data streaming through a custom socket, e.g. to perform motion
  command streaming.

## Example driver

In the `examples` subfolder you will find a minimal example of a running driver. It starts an
instance of the `UrDriver` class and prints the RTDE values read from the controller. To run it make
sure to

* have an instance of a robot controller / URSim running at the configured IP address (or adapt the
  address to your needs)
* run it from the package's main folder (the one where this README.md file is stored), as for
  simplicity reasons it doesn't use any sophisticated method to locate the required files.

## Architecture

See [Architecture documentation](doc/architecture.rst)

## A word on the primary / secondary interface

Currently, this library doesn't support the primary interface very well, as the [Universal Robots
ROS driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) was built mainly upon
the RTDE interface. The `PrimaryClient` for directly accessing the primary
interface doesn't support all features of the primary interface.

The `comm::URStream` class can be used to open a connection to the primary / secondary interface
and send data to it. The [producer/consumer](#producer--consumer-architecture) pipeline structure
can also be used together with the primary / secondary interface. However, package parsing isn't
implemented for most packages currently. See the [`primary_pipeline`
example](examples/primary_pipeline.cpp) on details how to set this up. Note that when running this
example, most packages will just be printed as their raw byte streams in a hex notation, as they
aren't implemented in the library, yet.

## A word on Real-Time scheduling

As mentioned above, for a clean operation it is quite critical that arriving RTDE messages are read
before the next message arrives. Due to this, both, the RTDE receive thread and the thread calling
`getDataPackage()` should be scheduled with real-time priority. See [this guide](doc/real_time.rst)
for details on how to set this up.

The RTDE receive thread will be scheduled to real-time priority automatically, if applicable. If
this doesn't work, an error is raised at startup. The main thread calling `getDataPackage` should be
scheduled to real-time priority by the application. See the
[ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/src/hardware_interface_node.cpp)
as an example.

## Producer / Consumer architecture

Communication with the primary / secondary and RTDE interfaces is designed to use a
consumer/producer pattern. The Producer reads data from the socket whenever it comes in, parses the
contents and stores the parsed packages into a pipeline queue.
You can write your own consumers that use the packages coming from the producer. See the
[`comm::ShellConsumer`](include/ur_client_library/comm/shell_consumer.h) as an example.

## Logging configuration

As this library was originally designed to be included into a ROS driver but also to be used as a
standalone library, it uses custom logging macros instead of direct `printf` or `std::cout`
statements.

The macro based interface is by default using the [`DefaultLogHandler`](include/ur_client_library/default_log_handler.h)
to print the logging messages as `printf` statements. It is possible to define your own log handler
to change the behavior, [see create new log handler](#create-new-log-handler) on how to.

### Change logging level

Make sure to set the logging level in your application, as by default only messages of level
WARNING or higher will be printed. See below for an example:

```c++
#include "ur_client_library/log.h"

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::DEBUG);

  URCL_LOG_DEBUG("Logging debug message");
  return 0;
}
```

### Create new log handler

The logger comes with an interface [`LogHandler`](include/ur_client_library/log.h), which can be
used to implement your own log handler for messages logged with this library. This can be done by
inheriting from the `LogHandler class`.

If you want to create a new log handler in your application, you can use below example as
inspiration:

```c++
#include "ur_client_library/log.h"
#include <iostream>

class MyLogHandler : public urcl::LogHandler
{
public:
  MyLogHandler() = default;

  void log(const char* file, int line, urcl::LogLevel loglevel, const char* log) override
  {
    switch (loglevel)
    {
      case urcl::LogLevel::INFO:
        std::cout << "INFO " << file << " " << line << ": " << log << std::endl;
        break;
      case urcl::LogLevel::DEBUG:
        std::cout << "DEBUG " << file << " " << line << ": " << log << std::endl;
        break;
      case urcl::LogLevel::WARN:
        std::cout << "WARN " << file << " " << line << ": " << log << std::endl;
        break;
      case urcl::LogLevel::ERROR:
        std::cout << "ERROR " << file << " " << line << ": " << log << std::endl;
        break;
      case urcl::LogLevel::FATAL:
        std::cout << "ERROR " << file << " " << line << ": " << log << std::endl;
        break;
      default:
        break;
    }
  }
};

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::DEBUG);
  std::unique_ptr<MyLogHandler> log_handler(new MyLogHandler);
  urcl::registerLogHandler(std::move(log_handler));

  URCL_LOG_DEBUG("logging debug message");
  URCL_LOG_INFO("logging info message");
  return 0;
}
```

## Contributor Guidelines

* This repo supports [pre-commit](https://pre-commit.com/) e.g. for automatic code formatting. TLDR:
  This will prevent you from committing falsely formatted code:

  ``` bash
  pipx install pre-commit
  pre-commit install
  ```

* Succeeding pipelines are a must on Pull Requests (unless there is a reason, e.g. when there have
been upstream changes).
* We try to increase and keep our code coverage high, so PRs with new
features should also have tests covering them.
* Parameters of public methods must all be documented.

## Acknowledgment

Many parts of this library are forked from the [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver).

Developed in collaboration between:

[<img height="60" alt="Universal Robots A/S" src="doc/resources/ur_logo.jpg">](https://www.universal-robots.com/)
&nbsp; and &nbsp;
[<img height="60" alt="FZI Research Center for Information Technology" src="doc/resources/fzi-logo_transparenz.png">](https://www.fzi.de).

<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="https://raw.githubusercontent.com/rosin-project/press_kit/master/img/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="https://raw.githubusercontent.com/rosin-project/press_kit/master/img/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
