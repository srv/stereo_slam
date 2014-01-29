cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

project(stereo_slam)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	nav_msgs
	tf
	sensor_msgs
	image_transport
	message_filters
	pcl_ros
	cv_bridge
	image_geometry
	std_srvs
	sql_database)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Boost:
find_package(Boost REQUIRED COMPONENTS signals filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)
include_directories(${SUITESPARSE_INCLUDE_DIRS})

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually 
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

catkin_package()

#get the libpq include and lib directories
execute_process( 
  COMMAND pg_config --includedir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_INCLUDE_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process( 
  COMMAND pg_config --libdir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_LIB_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(CheckIncludeFiles)
set(CMAKE_REQUIRED_INCLUDES ${PQ_INCLUDE_DIR})
check_include_files(libpq-fe.h HAVE_LIBPQ)
link_directories(${PQ_LIB_DIR})
include_directories(${PQ_INCLUDE_DIR})

include_directories(${Boost_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

add_executable(stereo_slam 
			   src/stereo_slam_node.cpp
			   src/stereo_slam_base.cpp
			   src/vertex_insertion.cpp
			   src/graph_updater.cpp)

target_link_libraries(stereo_slam pq ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${catkin_LIBRARIES})