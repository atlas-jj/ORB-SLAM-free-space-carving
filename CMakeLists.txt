cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_Pub)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# set carv needed flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__STDC_CONSTANT_MACROS -DGL_GLEXT_PROTOTYPES -D_LINUX -D_REENTRANT -DCGAL_USE_F2C -DCGAL_DISABLE_ROUNDING_MATH_CHECK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STDC_CONSTANT_MACROS -DGL_GLEXT_PROTOTYPES -D_LINUX -D_REENTRANT -DCGAL_USE_F2C -DCGAL_DISABLE_ROUNDING_MATH_CHECK")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Find CGAL
set(CMAKE_BUILD_TYPE_BACKUP ${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_BACKUP})
# Find Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
# BLAS and LAPACK
find_package(LAPACK REQUIRED)
link_directories(${LAPACK_LIBRARY_DIR})


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/Line3Dpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/Line3Dpp/build
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# added carv source file
add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/Modeler/FreespaceDelaunayAlgorithm.cc
        src/Modeler/GraphWrapper_Boost.cc
        src/Modeler/lovimath.cc
        src/Modeler/SFMTranscript.cpp
        src/Modeler/SFMTranscriptInterface_Delaunay.cpp
        src/Modeler/Matrix.cc
        src/Modeler/StringFunctions.cpp
        src/Modeler/Exception.cpp
        src/Modeler/SFMTranscriptInterface_ORBSLAM.cpp
        src/Modeler/Modeler.cc
        src/Modeler/ModelDrawer.cc
        src/Modeler/TextureFrame.cc
        )

# add lapack blas link
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/EDLines/EDLinesLib.a
        lapack blas
        )

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
        Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_mono
        Examples/Monocular/mono_mono.cc)
target_link_libraries(mono_mono ${PROJECT_NAME})

# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
        tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})
