cmake_minimum_required(VERSION 2.8.0)
cmake_policy(VERSION 2.8)

cmake_policy(SET CMP0003 NEW)
if (POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()

##################
# Build Settings #
##################

# Set project name
project(pose-graph-optimization C CXX)

# Set cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/module)

# Disable in-source build
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)

# Set Runtime output path
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Check if the compiler supports C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++11 COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support!")
endif()

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -march=native -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -DNDEBUG")

#########################################
# Set include directories and libraries #
#########################################

# core module
include_directories(${PROJECT_SOURCE_DIR}/src)

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Ceres Solver
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIR})

# Lapack for Ceres Solver
find_package(LAPACK)
if (LAPACK_FOUND)
    include_directories(${LAPACK_INCLUDE_DIRS})
endif()

# CXSparse for Ceres Solver
find_package(CXSparse)
if (CXSPARSE_FOUND)
    include_directories(${CXSPARSE_INCLUDE_DIRS})
endif()

# SuiteSparse for Ceres Solver
find_package(SuiteSparse)
if (SUITESPARSE_FOUND)
    include_directories(${SUITESPARSE_INCLUDE_DIRS})
endif()

# Gflags
find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIR})
message("-- Found Google Flags header in: ${GFLAGS_INCLUDE_DIRS}, in namespace: ${GFLAGS_NAMESPACE}")
add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(src)
