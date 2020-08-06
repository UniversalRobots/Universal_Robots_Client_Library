cmake_minimum_required(VERSION 2.8.12)
project(ur_driver_examples)

#find_package(ur_lib REQUIRED)

##
## Check C++11 support / enable global pedantic and Wall
##
include(DefineCXX11CompilerFlag)
DEFINE_CXX_11_COMPILER_FLAG(CXX11_FLAG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
add_executable(driver_example
  full_driver.cpp)
target_compile_options(driver_example PUBLIC ${CXX11_FLAG})
target_link_libraries(driver_example ur_lib::ur_lib)

