cmake_minimum_required(VERSION 2.8.3)
project(cartographer_ros_pr2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
google_enable_testing()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cartographer_ros
  tf_conversions
)

include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cartographer_ros_pr2
#  CATKIN_DEPENDS cartographer_ros
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
)

add_executable(initpose_node src/initpose_node.cpp)
target_link_libraries(initpose_node ${catkin_LIBRARIES} cartographer ${LUA_LIBRARIES})

