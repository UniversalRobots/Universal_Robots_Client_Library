cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/" ${CMAKE_MODULE_PATH})

project(ur_lib_tests)

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
if (NOT TARGET ur_lib::ur_lib)
  find_package(ur_lib REQUIRED)
endif()

# Check C++11 support
include(DefineCXX11CompilerFlag)
DEFINE_CXX_11_COMPILER_FLAG(CXX11_FLAG)


# Build Tests
add_executable(rtde_tests test_rtde_client.cpp)
target_compile_options(rtde_tests PRIVATE ${CXX11_FLAG})
target_include_directories(rtde_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_tests PRIVATE ur_lib::ur_lib ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      rtde_tests
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

