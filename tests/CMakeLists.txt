cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/" ${CMAKE_MODULE_PATH})

project(ur_client_library_tests)

# This will allow cmake 3.0 and above to fail on missing link targets (when specified with an double colon syntax)
# Note that not all packages are support the colon syntax, but if they do it is recomended to use this syntax, since it is less error prone.
if(POLICY CMP0028)
  cmake_policy(SET CMP0028 NEW)
endif()

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

# Find dependencies for testing
include(FindGTestPackage)
find_gtest_package(GTEST_LIBRARIES)
if (NOT TARGET ur_client_library::urcl)
  find_package(ur_client_library REQUIRED)
endif()

# Check C++11 support
include(DefineCXX17CompilerFlag)
DEFINE_CXX_17_COMPILER_FLAG(CXX17_FLAG)


# Build Tests
if (INTEGRATION_TESTS)
  # Integration tests require a robot reachable at 192.168.56.101. Therefore, they have to be
  # activated separately.
  add_executable(rtde_tests test_rtde_client.cpp)
  target_compile_options(rtde_tests PRIVATE ${CXX17_FLAG})
  target_include_directories(rtde_tests PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(rtde_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
  gtest_add_tests(TARGET      rtde_tests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_executable(dashboard_client_tests test_dashboard_client.cpp)
  target_compile_options(dashboard_client_tests PRIVATE ${CXX17_FLAG})
  target_include_directories(dashboard_client_tests PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(dashboard_client_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
  gtest_add_tests(TARGET      dashboard_client_tests
  )

  add_executable(spline_tests test_spline_interpolation.cpp)
  target_compile_options(spline_tests PRIVATE ${CXX17_FLAG})
  target_include_directories(spline_tests PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(spline_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
  gtest_add_tests(TARGET      spline_tests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
else()
  message(STATUS "Skipping integration tests.")
endif()


add_executable(primary_parser_tests test_primary_parser.cpp)
target_compile_options(primary_parser_tests PRIVATE ${CXX17_FLAG})
target_include_directories(primary_parser_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(primary_parser_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      primary_parser_tests
)

add_executable(rtde_data_package_tests test_rtde_data_package.cpp)
target_compile_options(rtde_data_package_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_data_package_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_data_package_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      rtde_data_package_tests
)

add_executable(rtde_parser_tests test_rtde_parser.cpp)
target_compile_options(rtde_parser_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_parser_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_parser_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      rtde_parser_tests
)

add_executable(tcp_server_tests test_tcp_server.cpp)
target_compile_options(tcp_server_tests PRIVATE ${CXX17_FLAG})
target_include_directories(tcp_server_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(tcp_server_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      tcp_server_tests
)

add_executable(script_sender_tests test_script_sender.cpp)
target_compile_options(script_sender_tests PRIVATE ${CXX17_FLAG})
target_include_directories(script_sender_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(script_sender_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      script_sender_tests
)

add_executable(reverse_interface_tests test_reverse_interface.cpp)
target_compile_options(reverse_interface_tests PRIVATE ${CXX17_FLAG})
target_include_directories(reverse_interface_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(reverse_interface_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET reverse_interface_tests
)

add_executable(trajectory_point_interface_tests test_trajectory_point_interface.cpp)
target_compile_options(trajectory_point_interface_tests PRIVATE ${CXX17_FLAG})
target_include_directories(trajectory_point_interface_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(trajectory_point_interface_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET trajectory_point_interface_tests
)

add_executable(rtde_control_package_pause_tests test_rtde_control_package_pause.cpp)
target_compile_options(rtde_control_package_pause_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_control_package_pause_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_control_package_pause_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_control_package_pause_tests
)

add_executable(rtde_control_package_start_tests test_rtde_control_package_start.cpp)
target_compile_options(rtde_control_package_start_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_control_package_start_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_control_package_start_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_control_package_start_tests
)

add_executable(rtde_control_package_setup_outputs_tests test_rtde_control_package_setup_outputs.cpp)
target_compile_options(rtde_control_package_setup_outputs_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_control_package_setup_outputs_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_control_package_setup_outputs_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_control_package_setup_outputs_tests
)

add_executable(rtde_control_package_setup_inputs_tests test_rtde_control_package_setup_inputs.cpp)
target_compile_options(rtde_control_package_setup_inputs_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_control_package_setup_inputs_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_control_package_setup_inputs_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_control_package_setup_inputs_tests
)

add_executable(rtde_get_urcontrol_version_tests test_rtde_get_urcontrol_version.cpp)
target_compile_options(rtde_get_urcontrol_version_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_get_urcontrol_version_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_get_urcontrol_version_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_get_urcontrol_version_tests
)

add_executable(rtde_request_protocol_version_tests test_rtde_request_protocol_version.cpp)
target_compile_options(rtde_request_protocol_version_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_request_protocol_version_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_request_protocol_version_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_request_protocol_version_tests
)

add_executable(rtde_writer_tests test_rtde_writer.cpp)
target_compile_options(rtde_writer_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_writer_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_writer_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET rtde_writer_tests
)

add_executable(version_information_tests test_version_information.cpp)
target_compile_options(version_information_tests PRIVATE ${CXX17_FLAG})
target_include_directories(version_information_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(version_information_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET version_information_tests
)

add_executable(bin_parser_tests test_bin_parser.cpp)
target_compile_options(bin_parser_tests PRIVATE ${CXX17_FLAG})
target_include_directories(bin_parser_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(bin_parser_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET bin_parser_tests
)

add_executable(package_serializer_tests test_package_serializer.cpp)
target_compile_options(package_serializer_tests PRIVATE ${CXX17_FLAG})
target_include_directories(package_serializer_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(package_serializer_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET package_serializer_tests
)

add_executable(tcp_socket_tests test_tcp_socket.cpp)
target_compile_options(tcp_socket_tests PRIVATE ${CXX17_FLAG})
target_include_directories(tcp_socket_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(tcp_socket_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET tcp_socket_tests
)

add_executable(stream_tests test_stream.cpp)
target_compile_options(stream_tests PRIVATE ${CXX17_FLAG})
target_include_directories(stream_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(stream_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET stream_tests
)

add_executable(producer_tests test_producer.cpp)
target_compile_options(producer_tests PRIVATE ${CXX17_FLAG})
target_include_directories(producer_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(producer_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET producer_tests
)

add_executable(pipeline_tests test_pipeline.cpp)
target_compile_options(pipeline_tests PRIVATE ${CXX17_FLAG})
target_include_directories(pipeline_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(pipeline_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET pipeline_tests
)

add_executable(script_command_interface_tests test_script_command_interface.cpp)
target_compile_options(script_command_interface_tests PRIVATE ${CXX17_FLAG})
target_include_directories(script_command_interface_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(script_command_interface_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET script_command_interface_tests
)

add_executable(ur_driver_tests test_ur_driver.cpp)
target_compile_options(ur_driver_tests PRIVATE ${CXX17_FLAG})
target_include_directories(ur_driver_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(ur_driver_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      ur_driver_tests
)
