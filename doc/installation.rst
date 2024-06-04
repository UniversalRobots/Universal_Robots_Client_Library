Build / installation
====================

Plain cmake
-----------
To build this library standalone so that you can build you own
applications using this library, follow the usual cmake procedure:

.. code:: console

   $ cd <clone of this repository>
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

.. code-block:: console

   $ sudo apt install ros-<distro>-ur-client-library

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

* Add ``find_package(ur_client_library REQUIRED)`` to your ``CMakeLists.txt``
* add ``ur_client_library::urcl`` to the list of ``target_link_libraries(...)`` commands inside your
  ``CMakeLists.txt`` file

As a minimal example, take the following “project”:

.. code:: cpp

   /*main.cpp*/

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

.. code:: cmake

   # CMakeLists.txt

   cmake_minimum_required(VERSION 3.0.2)
   project(minimal_example)

   find_package(ur_client_library REQUIRED)
   add_executable(db_client main.cpp)
   target_link_libraries(db_client ur_client_library::urcl)
