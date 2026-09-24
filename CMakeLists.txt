cmake_minimum_required(VERSION 3.16)
project(hitl_bridge CXX)

# Gazebo Harmonic. The ModalAI document targets Jetty and uses unversioned
# package names; the compatibility table in that document gives these
# versioned names for Harmonic, which is what this machine runs.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(hitl_bridge SHARED src/hitl_bridge.cpp)

target_link_libraries(hitl_bridge
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::register
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)
