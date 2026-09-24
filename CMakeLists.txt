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

# MAVLink is header only and is generated rather than vendored, by
# scripts/gen_mavlink_headers.sh. Point this elsewhere to use headers that
# are already installed.
set(MAVLINK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/mavlink"
    CACHE PATH "Directory holding common/mavlink.h")
if(NOT EXISTS "${MAVLINK_INCLUDE_DIR}/common/mavlink.h")
  message(FATAL_ERROR
    "MAVLink headers not found in ${MAVLINK_INCLUDE_DIR}. "
    "Run scripts/gen_mavlink_headers.sh first.")
endif()

add_library(hitl_bridge SHARED src/hitl_bridge.cpp)

target_include_directories(hitl_bridge PRIVATE "${MAVLINK_INCLUDE_DIR}")

# The generated MAVLink headers trip these on GCC without saying anything
# about our own code.
target_compile_options(hitl_bridge PRIVATE
  -Wno-address-of-packed-member
  -Wno-cast-align
)

target_link_libraries(hitl_bridge
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::register
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)
