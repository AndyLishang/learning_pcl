cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-matrix_transform)

find_package(PCL REQUIRED)

message(STATUS "PCL Version: ${PCL_VERSION}")   # 打印查找到的PCL版本

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})