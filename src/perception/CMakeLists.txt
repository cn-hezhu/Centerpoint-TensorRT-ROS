# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.7)
project(perception)
find_package(yaml-cpp REQUIRED)
find_package(CUDA REQUIRED)
find_package(Threads)
find_package(OpenMP REQUIRED)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")
  set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
  set(CUDA_INSTALL_TARGET_DIR targets/aarch64-linux)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CMAKE_C_COMPILER /usr/bin/gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
  set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
endif()

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
add_compile_options(-W)
add_compile_options(-std=c++14)

set( SMS 30 32 35 37 50 52 53 60 61 62 70 72 75 87)
foreach(sm ${SMS})
	set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
endforeach()
set(HIGHEST_SM 87)
set(GENCODE ${GENCODE} -gencode arch=compute_${HIGHEST_SM},code=compute_${HIGHEST_SM})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
  -ccbin ${CMAKE_CXX_COMPILER}
  -Xcompiler -DWIN_INTERFACE_CUSTOM
  -Xcompiler -I/usr/aarch64-linux-gnu/include/
  -Xlinker -lsocket
  -Xlinker -rpath=/usr/lib/aarch64-linux-gnu/
  -Xlinker -rpath=/usr/aarch64-linux-gnu/lib/
  -Xlinker -L/usr/lib/aarch64-linux-gnu/
  -Xlinker -L/usr/aarch64-linux-gnu/lib/
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)
# set(TENSORRT_INCLUDE_DIRS /usr/local/TensorRT-8.2.3/include)
# set(TENSORRT_LIBRARY_DIRS /usr/local/TensorRT-8.2.3/lib)

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(
  ${TENSORRT_LIBRARY_DIRS}
  /usr/lib/aarch64-linux-gnu
  /usr/aarch64-linux-gnu/lib/
)
############################################################
# Create an executable
############################################################
# file(GLOB_RECURSE SOURCE_FILES
#   centerpoint/*.cu
#   centerpoint/*.cc
# )

# cuda_add_executable(${PROJECT_NAME} demo.cc ${SOURCE_FILES})

# target_link_libraries(${PROJECT_NAME}
#   libnvinfer.so
#   libnvonnxparser.so
#   yaml-cpp
# )

############################################################
# Create a library in subdirectory
############################################################
set(COMMON_INCLUDES ${CMAKE_CURRENT_BINARY_DIR})
set(PROJECT_LIB_NAME perception_lib)
include_directories(${COMMON_INCLUDES})
add_subdirectory(${PROJECT_SOURCE_DIR}/centerpoint)
message(STATUS " PROJECT_SOURCE_DIR : ${PROJECT_SOURCE_DIR}")
message(STATUS " PROJECT_LIB_NAME : ${PROJECT_LIB_NAME}")
############################################################
# ROS
############################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roslib
    pcl_ros
    autoware_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
  nodes/mmdet_node.cc
  nodes/mmdet_ros.cc
)

add_executable(perception
  ${SOURCE_FILES}
)

add_dependencies(perception
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(perception
  ${catkin_LIBRARIES}
  ${PROJECT_LIB_NAME}
  libnvinfer.so
  libnvonnxparser.so
  yaml-cpp
)

install(
  TARGETS
    ${PROJECT_LIB_NAME}
    perception
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
