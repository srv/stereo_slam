cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#g2o library
find_package(libg2o REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS} ${libg2o_INCLUDE_DIRS})
include_directories(/usr/include/suitesparse/)
set(G2O_LIBRARIES cholmod 
									g2o_cli 
									g2o_core 
									g2o_incremental 
									g2o_interactive 
									g2o_interface 
									g2o_parser 
									g2o_solver_cholmod 
									g2o_solver_csparse 
									g2o_solver_dense 
									g2o_solver_pcg 
									g2o_stuff 
									g2o_types_icp 
									g2o_types_sba 
									g2o_types_sim3 
									g2o_types_slam2d 
									g2o_types_slam3d)

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

#add bost directories
rosbuild_add_boost_directories()

#executables
rosbuild_add_executable(stereo_localization 
												src/stereo_localization.cpp)

#link with libraries
target_link_libraries(stereo_localization pq ${G2O_LIBRARIES})
rosbuild_link_boost(stereo_localization signals)
