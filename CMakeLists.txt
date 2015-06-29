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

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen REQUIRED)

# Dependencies - PCL:
find_package(PCL REQUIRED)

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

# Dependencies - Vtk
find_package(VTK REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include
						${catkin_INCLUDE_DIRS}
						${Eigen_INCLUDE_DIRS}
						${libhaloc_INCLUDE_DIRS}
						${OpenCV_INCLUDE_DIRS}
						${SUITESPARSE_INCLUDE_DIRS}
						${G2O_INCLUDE_DIRS}
						${PCL_INCLUDE_DIRS}
						${VTK_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Localization node
add_executable(localization
						src/localization/node.cpp
						src/localization/frame.cpp
						src/localization/frame_publisher.cpp
						src/localization/tracking.cpp
						src/localization/ldb.cpp
						src/localization/graph.cpp
						src/localization/hash.cpp
						src/localization/loop_closing.cpp
						src/localization/map.cpp)
target_link_libraries(	localization
						${eigen_LIBRARIES}
						${OpenCV_LIBRARIES}
						${SUITESPARSE_LIBRARIES}
						${G2O_LIBRARIES}
						cholmod
						${catkin_LIBRARIES})
add_dependencies(localization stereo_slam_gencpp)