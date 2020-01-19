CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

PROJECT(ppfplane)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

# ROOTs
SET(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/eigen")
SET(CERES_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/build-ceres-solver/")
SET(PANGOLIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/build-Pangolin/src")

########## INCLUDED LIBRARIES ##########

# Eigen
SET(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
FIND_PACKAGE(Eigen3 3.3.4 REQUIRED)
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN_ROOT})
MESSAGE(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")

# Ceres Solver
FIND_PACKAGE(Ceres REQUIRED HINTS "${CERES_ROOT}")
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE(STATUS "Found Ceres headers in: ${CERES_INCLUDE_DIRS}")

# Pangolin viewer
SET(Pangolin_DIR ${PANGOLIN_ROOT})
FIND_PACKAGE(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIR})
MESSAGE(STATUS "Found Pangolin headers in: ${Pangolin_INCLUDE_DIR}")

# Sophus
INCLUDE_DIRECTORIES( 3rdParty/Sophus/ )

# CLI, command line parser
INCLUDE_DIRECTORIES( 3rdParty/CLI11/include/ )

# nanoflann
INCLUDE_DIRECTORIES( 3rdParty/nanoflann/ )

# tinyply
INCLUDE_DIRECTORIES( 3rdParty/tinyply/ )
ADD_LIBRARY( tinyply ../3rdParty/tinyply/tinyply/tinyply.cpp )
TARGET_COMPILE_OPTIONS( tinyply PRIVATE -std=c++11 )

########## CUSTOM LIBRARIES ##########

INCLUDE_DIRECTORIES( include/ )

## visualize OpenCV point cloud
SET( LIB_NAME vis_lib )
ADD_LIBRARY( ${LIB_NAME} ../include/visualize/pcshow.cpp
                         ../include/visualize/pcshow_lines.cpp
                         ../include/visualize/pcshow_corners.cpp
                          )
TARGET_LINK_LIBRARIES( ${LIB_NAME} ${Pangolin_LIBRARIES})
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

## graph
SET( LIB_NAME graph_lib )
ADD_LIBRARY( ${LIB_NAME} ../include/graph/PlaneGraph.cpp
                         ../include/graph/ParallelPlaneGraph_filtering.cpp
                         ../include/graph/ParallelPlaneGraph_clustering.cpp
                         ../include/graph/ParallelPlaneGraph_optimization.cpp
                          )
TARGET_LINK_LIBRARIES( ${LIB_NAME} ${CERES_LIBRARIES})
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

## corners
SET( LIB_NAME corner_lib )
ADD_LIBRARY( ${LIB_NAME} ../include/corners/Corner.cpp
                          )
TARGET_LINK_LIBRARIES( ${LIB_NAME} ${CERES_LIBRARIES})
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

## PPF
SET( LIB_NAME ppf_lib )
ADD_LIBRARY( ${LIB_NAME} ../include/PPF/PairDetector.cpp
                          )
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

########## DIRECTORIES ##########

ADD_SUBDIRECTORY(ply_detect_refine)
ADD_SUBDIRECTORY(ply_corners)
