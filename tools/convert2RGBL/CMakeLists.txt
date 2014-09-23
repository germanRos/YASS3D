cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(convert2RGBL)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC_FILES convert2RGBL.cpp)

add_executable (convert2RGBL ${SRC_FILES})
target_link_libraries (convert2RGBL ${PCL_LIBRARIES})
