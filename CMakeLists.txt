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
	cv_bridge
	image_geometry
	std_srvs)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen REQUIRED)

# Dependencies - Libhaloc:
find_package(libhaloc REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually 
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)

catkin_package()

include_directories(include 
					${catkin_INCLUDE_DIRS}
					${Eigen_INCLUDE_DIRS}
					${libhaloc_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS}
					${SUITESPARSE_INCLUDE_DIRS}
					${G2O_INCLUDE_DIRS})


add_executable(slam 
			   src/slam/node.cpp
			   src/slam/base.cpp
			   src/slam/graph.cpp
			   src/slam/pose.cpp)

target_link_libraries(slam 
					  ${eigen_LIBRARIES} 
					  ${libhaloc_LIBRARIES} 
					  ${OpenCV_LIBRARIES} 
					  ${SUITESPARSE_LIBRARIES} 
					  ${G2O_LIBRARIES} 
					  ${catkin_LIBRARIES})