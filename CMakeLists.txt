cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
set(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" TRUE)

project(stereo_slam)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
  sensor_msgs
  geometry_msgs
  image_transport
  message_filters
  message_generation
  cv_bridge
  image_geometry
  pcl_ros
  cmake_modules
  tf_conversions
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg 
                  FILES GraphPoses.msg
                  TimeTracking.msg
                  TimeGraph.msg
                  TimeLoopClosing.msg
                  SubTimeLoopClosing.msg)
                  
generate_messages(DEPENDENCIES std_msgs)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++20, C++17, C++14 or C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
  add_definitions(-DCOMPILEDWITHC20)
  message(status " Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(status " Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(status " Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(status " Using flag -std=c++11.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20, C++17, C++14 or C++11 support. Please use a different C++ compiler.")
endif()

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

# Dependencies - PCL:
find_package(PCL REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Localization node
add_executable(localization
  src/node.cpp
  src/frame.cpp
  src/publisher.cpp
  src/tracking.cpp
  src/graph.cpp
  src/loop_closing.cpp
  src/cluster.cpp
  src/hash.cpp)
target_link_libraries(localization
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  cholmod
  ${catkin_LIBRARIES})
add_dependencies(localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})