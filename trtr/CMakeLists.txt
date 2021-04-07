cmake_minimum_required(VERSION 3.0.2)
project(trtr)


add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")


# CUDA
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  HINTS "/usr/local/cuda"
  PATH_SUFFIXES include)
MESSAGE(STATUS "Found CUDA headers at ${CUDA_INCLUDE_DIR}")
find_library(CUDA_LIBRARY cudart
  HINTS "/usr/local/cuda"
  PATH_SUFFIXES lib lib64 lib/x64)
MESSAGE(STATUS "CUDA_LIBRARY at ${CUDA_LIBRARY}")

# TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES include)
MESSAGE(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")
find_path(TENSORRT_SAMPLE_INCLUDE_DIR common.h
  HINTS "/usr/src/tensorrt/samples/common")
MESSAGE(STATUS "Found TensorRT sample headers at ${TENSORRT_SAMPLE_INCLUDE_DIR}")
set(TENSORRT_SAMPLE_SRC_DIR ${TENSORRT_SAMPLE_INCLUDE_DIR})
find_library(TENSORRT_LIBRARY_INFER nvinfer
  HINTS ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
MESSAGE(STATUS "TENSORRT_LIBRARY_INFER at ${TENSORRT_LIBRARY_INFER}")
find_library(TENSORRT_LIBRARY_INFER_PLUGIN nvinfer_plugin
  HINTS  ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(TENSORRT_LIBRARY_MYELIN myelin
  HINTS  ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
set(TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER} ${TENSORRT_LIBRARY_INFER_PLUGIN} ${TENSORRT_LIBRARY_MYELIN})
MESSAGE(Status "Find TensorRT libs at ${TENSORRT_LIBRARY}")
find_package_handle_standard_args(
  TENSORRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBRARY)
if(NOT TENSORRT_FOUND)
  MESSAGE(WARNING "Cannot find TensorRT library.")
else()
  # check tensor version
  file(STRINGS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h" VERSION_STRINGS REGEX "#define NV_TENSORRT_.*")
  foreach(TYPE MAJOR MINOR PATCH)
    string(REGEX MATCH "NV_TENSORRT_${TYPE} [0-9]" TRT_TYPE_STRING ${VERSION_STRINGS})
    string(REGEX MATCH "[0-9]" TRT_${TYPE} ${TRT_TYPE_STRING})
  endforeach(TYPE)
  set(TRT_VERSION "${TRT_MAJOR}.${TRT_MINOR}.${TRT_PATCH}" CACHE STRING "TensorRT project version")
  MESSAGE(SATUS "Building for TensorRT version: ${TRT_VERSION}")
  if(TRT_VERSION VERSION_LESS 7.2.2)
    message(FATAL_ERROR "We find a TensorRT of ${TRT_VERSION} . please install version >= 7.2.2")
  endif()
  add_definitions(-DTENSORRT)
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES trtr
#  CATKIN_DEPENDS cv_bridge roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
${TENSORRT_INCLUDE_DIR}
${TENSORRT_SAMPLE_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${CUDA_INCLUDE_DIR}
)

add_executable(trtr_tracker_node src/trtr_tracker.cpp ${TENSORRT_SAMPLE_INCLUDE_DIR}/logger.cpp)
target_link_libraries (trtr_tracker_node ${catkin_LIBRARIES} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
