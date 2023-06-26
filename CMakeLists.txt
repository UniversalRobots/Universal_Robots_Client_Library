cmake_minimum_required(VERSION 3.0.2)
project(ur_client_library)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/" ${CMAKE_MODULE_PATH})

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()


##
## Check C++11 support / enable global pedantic and Wall
##
include(DefineCXX17CompilerFlag)
DEFINE_CXX_17_COMPILER_FLAG(CXX17_FLAG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

add_library(urcl SHARED
    src/comm/tcp_socket.cpp
    src/comm/tcp_server.cpp
    src/control/reverse_interface.cpp
    src/control/script_sender.cpp
    src/control/trajectory_point_interface.cpp
    src/control/script_command_interface.cpp
    src/primary/primary_package.cpp
    src/primary/robot_message.cpp
    src/primary/robot_state.cpp
    src/primary/robot_message/version_message.cpp
    src/primary/robot_state/kinematics_info.cpp
    src/rtde/control_package_pause.cpp
    src/rtde/control_package_setup_inputs.cpp
    src/rtde/control_package_setup_outputs.cpp
    src/rtde/control_package_start.cpp
    src/rtde/data_package.cpp
    src/rtde/get_urcontrol_version.cpp
    src/rtde/request_protocol_version.cpp
    src/rtde/rtde_package.cpp
    src/rtde/text_message.cpp
    src/rtde/rtde_client.cpp
    src/ur/ur_driver.cpp
    src/ur/calibration_checker.cpp
    src/ur/dashboard_client.cpp
    src/ur/tool_communication.cpp
    src/ur/version_information.cpp
    src/rtde/rtde_writer.cpp
    src/default_log_handler.cpp
    src/log.cpp
    src/helpers.cpp
)
add_library(ur_client_library::urcl ALIAS urcl)
target_compile_options(urcl PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_compile_options(urcl PUBLIC ${CXX17_FLAG})
target_include_directories( urcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(urcl PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(urcl PUBLIC "${CMAKE_THREAD_LIBS_INIT}")
endif()

##
## Build testing if enabled by option
##
if (BUILDING_TESTS)
  enable_testing()
  add_subdirectory(tests)
else()
  message(STATUS "Building tests disabled.")
endif()


add_subdirectory(examples)

include(GNUInstallDirs)
install(TARGETS urcl EXPORT urcl_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION include)

install(EXPORT urcl_targets
  DESTINATION lib/cmake/ur_client_library
  FILE urclTargets.cmake
  NAMESPACE ur_client_library::)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/ur_client_libraryConfigVersion.cmake VERSION 0.0.3
  COMPATIBILITY SameMajorVersion)
install(FILES ur_client_libraryConfig.cmake
  DESTINATION lib/cmake/ur_client_library)

# Install package.xml file so this package can be processed by ROS toolings
# See REP 136 for details
# Installing this in non-ROS environments won't have any effect, but it won't harm, either.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

#### Begin import ####
# Imported from ros-industrial/ros_industrial_cmake_boilerplate
# https://github.com/ros-industrial/ros_industrial_cmake_boilerplate/blob/59b86df662/ros_industrial_cmake_boilerplate/cmake/cmake_tools.cmake
# Copyright (C) 2018 by George Cave - gcave@stablecoder.ca
# Copyright (c) 2020, Southwest Research Institute
# Licensed under Apache-2.0 license

# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv "prepend-non-duplicate;ROS_PACKAGE_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
#### End iport ####

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
