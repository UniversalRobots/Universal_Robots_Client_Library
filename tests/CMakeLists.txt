cmake_minimum_required(VERSION 3.14.0)


project(ur_client_library_tests)


include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

option(INTEGRATION_TESTS "Build the integration tests that require a running robot / URSim" OFF)
# Build Tests
if (INTEGRATION_TESTS)
  # Integration tests require a robot reachable at 192.168.56.101. Therefore, they have to be
  # activated separately.
  
  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  add_custom_target(generate_outputs ALL COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/resources/generate_rtde_outputs.py)
  add_executable(rtde_tests test_rtde_client.cpp)
  add_dependencies(rtde_tests generate_outputs)
  target_link_libraries(rtde_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      rtde_tests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_executable(dashboard_client_g5_tests test_dashboard_client_g5.cpp)
  target_link_libraries(dashboard_client_g5_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      dashboard_client_g5_tests
  )

  add_executable(dashboard_client_x_tests test_dashboard_client_x.cpp)
  target_link_libraries(dashboard_client_x_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      dashboard_client_x_tests
  )

  # Spline tests
  add_executable(spline_tests_urcap test_spline_interpolation.cpp)
  target_link_libraries(spline_tests_urcap PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      spline_tests_urcap
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless false
                  TEST_SUFFIX _urcap
  )
  add_executable(spline_tests_headless test_spline_interpolation.cpp)
  target_link_libraries(spline_tests_headless PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      spline_tests_headless
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless true
                  TEST_SUFFIX _headless
  )

  # UrDriver tests
  add_executable(ur_driver_tests_urcap test_ur_driver.cpp)
  target_link_libraries(ur_driver_tests_urcap PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      ur_driver_tests_urcap
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless false
                  TEST_SUFFIX _urcap
  )
  add_executable(ur_driver_tests_headless test_ur_driver.cpp)
  target_link_libraries(ur_driver_tests_headless PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET      ur_driver_tests_headless
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless true
                  TEST_SUFFIX _headless
  )

  # InstructionExecutor tests
  add_executable(instruction_executor_test_urcap test_instruction_executor.cpp)
  target_link_libraries(instruction_executor_test_urcap PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET instruction_executor_test_urcap
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless false
                  TEST_SUFFIX _urcap
  )
  add_executable(instruction_executor_test_headless test_instruction_executor.cpp)
  target_link_libraries(instruction_executor_test_headless PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET instruction_executor_test_headless
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless true
                  TEST_SUFFIX _headless
  )

  # PrimaryClient tests
  add_executable(primary_client_test_urcap test_primary_client.cpp)
  target_link_libraries(primary_client_test_urcap PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET primary_client_test_urcap
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless false
                  TEST_SUFFIX _urcap
  )
  add_executable(primary_client_test_headless test_primary_client.cpp)
  target_link_libraries(primary_client_test_headless PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET primary_client_test_headless
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  EXTRA_ARGS --headless true
                  TEST_SUFFIX _headless
  )

  add_executable(ur_driver_deprecated_constructor_test test_deprecated_ur_driver_construction.cpp)
  target_link_libraries(ur_driver_deprecated_constructor_test PRIVATE ur_client_library::urcl GTest::gtest_main)
  gtest_add_tests(TARGET ur_driver_deprecated_constructor_test
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
else()
  message(STATUS "Skipping integration tests.")
endif()


add_executable(dashboard_client_mock_tests test_dashboard_client.cpp)
target_link_libraries(dashboard_client_mock_tests PRIVATE ur_client_library::urcl GTest::gmock_main)
gtest_add_tests(TARGET      dashboard_client_mock_tests
)

add_executable(primary_parser_tests test_primary_parser.cpp)
target_link_libraries(primary_parser_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET      primary_parser_tests
)



add_executable(rtde_data_package_tests test_rtde_data_package.cpp)
target_link_libraries(rtde_data_package_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET      rtde_data_package_tests
)

add_executable(rtde_parser_tests test_rtde_parser.cpp)
target_link_libraries(rtde_parser_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET      rtde_parser_tests
)

add_executable(tcp_server_tests test_tcp_server.cpp)
target_link_libraries(tcp_server_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET      tcp_server_tests
)

add_executable(script_sender_tests test_script_sender.cpp)
target_link_libraries(script_sender_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET      script_sender_tests
)

add_executable(script_reader_tests test_script_reader.cpp)
target_link_libraries(script_reader_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(
        TARGET script_reader_tests
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(reverse_interface_tests test_reverse_interface.cpp)
target_link_libraries(reverse_interface_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET reverse_interface_tests
)

add_executable(trajectory_point_interface_tests test_trajectory_point_interface.cpp)
target_link_libraries(trajectory_point_interface_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET trajectory_point_interface_tests
)

add_executable(rtde_control_package_pause_tests test_rtde_control_package_pause.cpp)
target_link_libraries(rtde_control_package_pause_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_control_package_pause_tests
)

add_executable(rtde_control_package_start_tests test_rtde_control_package_start.cpp)
target_link_libraries(rtde_control_package_start_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_control_package_start_tests
)

add_executable(rtde_control_package_setup_outputs_tests test_rtde_control_package_setup_outputs.cpp)
target_link_libraries(rtde_control_package_setup_outputs_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_control_package_setup_outputs_tests
)

add_executable(rtde_control_package_setup_inputs_tests test_rtde_control_package_setup_inputs.cpp)
target_link_libraries(rtde_control_package_setup_inputs_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_control_package_setup_inputs_tests
)

add_executable(rtde_get_urcontrol_version_tests test_rtde_get_urcontrol_version.cpp)
target_link_libraries(rtde_get_urcontrol_version_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_get_urcontrol_version_tests
)

add_executable(rtde_request_protocol_version_tests test_rtde_request_protocol_version.cpp)
target_link_libraries(rtde_request_protocol_version_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_request_protocol_version_tests
)

add_executable(rtde_writer_tests test_rtde_writer.cpp)
target_link_libraries(rtde_writer_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET rtde_writer_tests
)

add_executable(version_information_tests test_version_information.cpp)
target_link_libraries(version_information_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET version_information_tests
)

add_executable(bin_parser_tests test_bin_parser.cpp)
target_link_libraries(bin_parser_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET bin_parser_tests
)

add_executable(package_serializer_tests test_package_serializer.cpp)
target_link_libraries(package_serializer_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET package_serializer_tests
)

add_executable(tcp_socket_tests test_tcp_socket.cpp)
target_link_libraries(tcp_socket_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET tcp_socket_tests
)

add_executable(stream_tests test_stream.cpp)
target_link_libraries(stream_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET stream_tests
)

add_executable(producer_tests test_producer.cpp)
target_link_libraries(producer_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET producer_tests
)

add_executable(pipeline_tests test_pipeline.cpp)
target_link_libraries(pipeline_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET pipeline_tests
)

add_executable(script_command_interface_tests test_script_command_interface.cpp)
target_link_libraries(script_command_interface_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET script_command_interface_tests
)

add_executable(robot_receive_timeout_tests test_robot_receive_timeout.cpp)
target_link_libraries(robot_receive_timeout_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET robot_receive_timeout_tests
)

add_executable(control_mode_tests test_control_mode.cpp)
target_link_libraries(control_mode_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET control_mode_tests
)

add_executable(helpers_tests test_helpers.cpp)
target_link_libraries(helpers_tests PRIVATE ur_client_library::urcl GTest::gtest_main)
gtest_add_tests(TARGET helpers_tests
)