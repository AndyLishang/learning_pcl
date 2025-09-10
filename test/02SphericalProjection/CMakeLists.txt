cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sp)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

set(
    THIRD_PARTY_LIB
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_executable (sp spherical_projection.cpp)
target_link_libraries (sp ${THIRD_PARTY_LIB})