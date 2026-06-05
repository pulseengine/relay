cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(FalconGroundEffect)

# Gazebo Harmonic (gz-sim8) System plugin. Build:
#   cmake -B build && cmake --build build
# then point gz at it:
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(FalconGroundEffect SHARED GroundEffect.cc)
target_link_libraries(FalconGroundEffect PRIVATE gz-sim8::gz-sim8)
