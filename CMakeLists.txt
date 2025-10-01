cmake_minimum_required(VERSION 3.10)
project(quadruped)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
include_directories(websocketpp)

file(GLOB_RECURSE SOURCES src/*.cpp)

set(CMAKE_BUILD_TYPE Debug) 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

find_package(Boost REQUIRED system thread)

add_executable(main ${SOURCES})
target_link_libraries(main Boost::system Boost::thread pthread)
