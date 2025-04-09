:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/installation.rst

Build / installation
====================

Plain cmake
-----------
To build this library standalone so that you can build you own
applications using this library, follow the usual cmake procedure:

.. code:: console

   $ git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library ur_client_library
   $ cd ur_client_library
   $ mkdir build && cd build
   $ cmake ..
   $ make
   $ sudo make install

This will install the library into your system so that it can be used by
other cmake projects directly.


Inside a ROS / ROS 2 workspace
------------------------------

The ``ur_client_library`` is available in all maintained ROS distribution and can be installed
using

.. code:: console

   $ sudo apt install ros-$ROS_DISTRO-ur-client-library

Unless you explicitly want to contribute to this library we recommend using the binary installation
instead of building from source as explained below.

ROS noetic
^^^^^^^^^^

If you want to build this library inside a ROS workspace, e.g. because
you want to build the `Universal Robots ROS
driver <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>`__
from source, you cannot use ``catkin_make`` directly, as this library is
not a catkin package. Instead, you will have to use
`catkin_make_isolated <http://docs.ros.org/independent/api/rep/html/rep-0134.html>`_
or `catkin
build <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html>`__
to build your workspace.

ROS 2
^^^^^

If you want to build this library inside a ROS 2 workspace, e.g. because
you want to build the `Universal Robots ROS2
driver <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>`__
from source, simply clone this project into your workspace and build your workspace as usual.

Use this library in other projects
----------------------------------

When you want to use this library in other cmake projects, make sure to

* Add ``find_package(ur_client_library REQUIRED)`` to your ``CMakeLists.txt`` (Requires to have the
  client library installed and your PATH setup done correctly. As an alternative, fetch and build
  it as part of your project as shown below.)
* add ``ur_client_library::urcl`` to the list of ``target_link_libraries(...)`` commands inside your
  ``CMakeLists.txt`` file.

As a minimal example, take the following “project”:

As a minimal executable, we'll create a client for the dashboard server and ask for the
PolyScope version.

.. note:: This will not work on PolyScope X versions, as the dashboard server is not available
   there.

.. code-block:: cpp
   :caption: main.cpp
   :linenos:

   #include <iostream>
   #include <ur_client_library/ur/dashboard_client.h>

   int main(int argc, char* argv[])
   {
     urcl::DashboardClient my_client("192.168.56.101");
     bool connected = my_client.connect();
     if (connected)
     {
       std::string answer = my_client.sendAndReceive("PolyscopeVersion\n");
       std::cout << answer << std::endl;
       my_client.disconnect();
     }
     return 0;
   }

In this example, we'll fetch the client library as part of the project and build it together with
our application:

.. code-block:: cmake
   :caption: CMakeLists.txt
   :linenos:

   cmake_minimum_required(VERSION 3.11.0) # That's the minimum required version for FetchContent
   project(minimal_example)

   include(FetchContent)
   FetchContent_Declare(
     ur_client_library
     GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
     GIT_TAG        master
   )

   # This will download the ur_client_library and replace the `find_package(ur_client_library)` call.
   FetchContent_MakeAvailable(ur_client_library)

   add_executable(db_client main.cpp)
   target_link_libraries(db_client ur_client_library::urcl)

To build the project, create a build directory and run cmake:

.. code:: console

   $ mkdir build && cd build
   $ cmake ..
   $ cmake --build .

Then (with a robot switched on and available on 192.168.56.101), you can test the minimal example application:

.. code:: console

   $ ./db_client
   INFO /<...>/ur/dashboard_client.cpp 72: Connected: Universal Robots Dashboard Server

   URSoftware 5.19.0.1210631 (Oct 23 2024)
   INFO /<...>/ur/dashboard_client.cpp 98: Disconnecting from Dashboard server on 192.168.56.101:29999
