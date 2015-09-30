cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

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
  std_srvs
  pcl_ros
  cmake_modules
  tf_conversions)

## Declare ROS messages and services
add_message_files(FILES GraphPoses.msg)
generate_messages(DEPENDENCIES std_msgs)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen REQUIRED)

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

# Dependencies - Vtk
find_package(VTK REQUIRED)

# Dependencies - MySql
find_package(mysqlclient REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${libhaloc_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${MYSQL_INCLUDE_DIRS})

add_definitions(${MYSQL_DEFINITIONS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Localization node
add_executable(localization
  src/node.cpp
  src/frame.cpp
  src/publisher.cpp
  src/tracking.cpp
  src/ldb.cpp
  src/graph.cpp
  src/hash.cpp
  src/loop_closing.cpp
  src/cluster.cpp)

target_link_libraries(localization
  ${eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  cholmod
  ${catkin_LIBRARIES})

add_dependencies(localization stereo_slam_gencpp)

add_executable(save_clouds
  src/reconstruction/save_clouds.cpp)

target_link_libraries(save_clouds
  ${eigen_LIBRARIES}
  ${MYSQL_LIBRARIES}
  ${catkin_LIBRARIES})
