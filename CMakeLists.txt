cmake_minimum_required(VERSION 3.3)
project(quadruped)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    include/
)

set(SOURCES
    src/main.cpp
    src/pca9685.cpp
    src/joint.cpp
    src/leg.cpp
    src/robot.cpp
    src/kinematics.cpp
)

set(HEADERS
    include/pca9685.hpp
    include/joint.hpp
    include/leg.hpp
    include/robot.hpp
    include/kinematics.hpp
)

set(CMAKE_BUILD_TYPE Debug) 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")  

add_executable( main ${SOURCES} ${HEADERS} )
target_link_libraries( main )