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

include(ExternalProject)

set(QPOASES_INSTALL_DIR ${PROJECT_SOURCE_DIR}/qpoases)
message(WARNING "${QPOASES_INSTALL_DIR}")
# QP OASES
ExternalProject_Add(qpoases
  SVN_REPOSITORY https://projects.coin-or.org/svn/qpOASES/stable/3.1
  SVN_TRUST_CERT 1
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${QPOASES_INSTALL_DIR}
  TIMEOUT 30
  PATCH_COMMAND patch -s -p0 < ${PROJECT_SOURCE_DIR}/patch/qpoases.patch
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${QPOASES_INSTALL_DIR}/include)
link_directories(${QPOASES_INSTALL_DIR}/lib)

## Alternative Method
#ExternalProject_Get_Property(qpoases SOURCE_DIR)
#include_directories(${SOURCE_DIR}/include)
#link_directories(${SOURCE_DIR}/libs)

set(OSQP_INSTALL_DIR ${PROJECT_SOURCE_DIR}/osqp)
# QP OASES
ExternalProject_Add(OSQP
  DOWNLOAD_COMMAND git clone --recursive https://github.com/oxfordcontrol/osqp OSQP
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OSQP_INSTALL_DIR}
  TIMEOUT 30
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${OSQP_INSTALL_DIR}/include/osqp)
link_directories(${OSQP_INSTALL_DIR}/lib)

set(OSQP_EIGEN_INSTALL_DIR ${PROJECT_SOURCE_DIR}/osqp_eigen)
# QP OASES
ExternalProject_Add(osqpeigen
  DEPENDS OSQP
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OSQP_EIGEN_INSTALL_DIR} -DCMAKE_PROJECT_OsqpEigen_INCLUDE=${OSQP_INSTALL_DIR}/osqp_path.cmake
  TIMEOUT 30
  PATCH_COMMAND patch -s -p0 < ${PROJECT_SOURCE_DIR}/patch/osqp_eigen.patch
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${OSQP_EIGEN_INSTALL_DIR}/include)
link_directories(${OSQP_EIGEN_INSTALL_DIR}/lib)

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
add_executable(mpc_example_qpoases src/mpc_example_qpoases.cpp)
target_link_libraries(mpc_example_qpoases qpOASES)
add_dependencies(mpc_example_qpoases qpoases)

add_executable(mpc_example_osqp src/mpc_example_osqp.cpp)
target_link_libraries(mpc_example_osqp osqp OsqpEigen)
add_dependencies(mpc_example_osqp OSQP osqpeigen)

#############
## Install ##
#############


#############
## Testing ##
#############

