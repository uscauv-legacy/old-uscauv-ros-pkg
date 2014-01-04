cmake_minimum_required(VERSION 2.8.3)
project(uscauv_build)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# Searches in the cmake/ directory
catkin_package(
  CFG_EXTRAS generate_common_header.cmake
)
