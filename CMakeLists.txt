cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(YASS3D)

find_package(PCL 1.7 REQUIRED)

set(LIBLINEAR_INCLUDE_DIRS src/liblinear src/liblinear/blas)
set(BLAS_LIBRARY_DIRS  src/liblinear/blas)
set(BLAS_LIBRARY blas.a)

include_directories(${PCL_INCLUDE_DIRS} ${LIBLINEAR_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${BLAS_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -std=c++11)

set(SRC_FILES_TRAIN	src/train.cpp
		src/yass.cpp
		src/FeatureManager.cpp
		src/CurvatureWrapper.cpp
		src/PositionWrapper.cpp
		src/ColorNWrapper.cpp
		src/DONWrapper.cpp
		src/FPFHWrapper.cpp
		src/liblinear/linear.cpp
		src/liblinear/tron.cpp
)

set(SRC_FILES_TEST	src/test.cpp
		src/yass.cpp
		src/FeatureManager.cpp
		src/CurvatureWrapper.cpp
		src/PositionWrapper.cpp
		src/ColorNWrapper.cpp
		src/DONWrapper.cpp
		src/FPFHWrapper.cpp
		src/liblinear/linear.cpp
		src/liblinear/tron.cpp
)

add_executable (YASStrain ${SRC_FILES_TRAIN})
add_executable (YASStest ${SRC_FILES_TEST})
target_link_libraries (YASStrain ${PCL_LIBRARIES} ${BLAS_LIBRARY})
target_link_libraries (YASStest ${PCL_LIBRARIES} ${BLAS_LIBRARY})
