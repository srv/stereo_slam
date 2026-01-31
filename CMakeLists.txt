cmake_minimum_required(VERSION 3.0.2)
project(stereo_slam)

# Add custom cmake modules path only for SuiteSparse if needed.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")


##############
## Compiler ##
##############

# C++17 Standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


#######################
## Required packages ##
#######################

find_package(
  catkin REQUIRED COMPONENTS
    tf2
    roscpp
    tf2_ros
    pcl_ros
    nav_msgs
    std_msgs
    cv_bridge
    sensor_msgs
    rosgraph_msgs
    cmake_modules
    geometry_msgs
    image_geometry
    image_transport
    message_filters
    tf2_geometry_msgs
    message_generation
)

# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

# Dependencies - PCL:
find_package(PCL REQUIRED)

# Dependencies - OpenCV:
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4") # Change path if OpenCV-4.2.0 with the contributors module is installed in another location.
find_package(OpenCV 4.2.0 REQUIRED)

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)


###################################
## Catkin specific configuration ##
###################################

## Declare ROS messages and services
add_message_files(
  DIRECTORY 
    msg 
  FILES 
    GraphPoses.msg
    TimeTracking.msg
    TimeGraph.msg
    TimeLoopClosing.msg
)
                  
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

# Executables
add_executable(clock_extender src/clock_extender.cpp)
add_executable(localization
  src/node.cpp
  src/frame.cpp
  src/publisher.cpp
  src/tracking.cpp
  src/graph.cpp
  src/loop_closing.cpp
  src/cluster.cpp
  src/hash.cpp)

# Dependencies
add_dependencies(clock_extender ${catkin_EXPORTED_TARGETS})
add_dependencies(localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Linking
target_link_libraries(clock_extender ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(localization ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} cholmod ${catkin_LIBRARIES})