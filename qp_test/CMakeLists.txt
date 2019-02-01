cmake_minimum_required(VERSION 3.2)
project(qp_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)

# QP OASES
add_subdirectory(qpOASES)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/bin)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/include)
add_definitions(-DUSE_QPOASES)

# OSQP + OSQP-eigen
add_subdirectory(osqp)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp/osqp/build/out)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp/osqp/include)

#add_subdirectory(osqp-eigen)
add_subdirectory(osqp-eigen)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp-eigen/osqp-eigen/build)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp-eigen/osqp-eigen/include)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qp_test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ library
add_executable(mpc_example_osqp src/mpc_example_osqp.cpp)
target_link_libraries(mpc_example_osqp OsqpEigen osqp)
add_executable(mpc_example_qpoases src/mpc_example_qpoases.cpp)
target_link_libraries(mpc_example_qpoases qpOASES)


#############
## Install ##
#############


#############
## Testing ##
#############

