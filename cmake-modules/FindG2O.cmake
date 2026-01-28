# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# G2O_FOUND, if false, do not try to link against g2o
# G2O_LIBRARIES, path to the libg2o
# G2O_INCLUDE_DIR, where to find the g2o header files

IF(UNIX)

  IF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)
    # in cache already
    SET(G2O_FIND_QUIETLY TRUE)
  ENDIF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)

  MESSAGE(STATUS "Searching for g2o ...")

  FIND_PATH(G2O_INCLUDE_DIR
    NAMES core math_groups types
    PATHS
    /opt/ros/noetic
    /usr/local
    /usr
    PATH_SUFFIXES include/g2o include
  )
  IF (G2O_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  ENDIF (G2O_INCLUDE_DIR)

  # Macro for easily searching libraries in the ROS path as well
  MACRO(FIND_G2O_LIBRARY MYLIB MYNAME)
    FIND_LIBRARY(${MYLIB}
      NAMES ${MYNAME}
      PATHS
      /opt/ros/noetic
      /usr/local
      /usr
      PATH_SUFFIXES lib
    )
  ENDMACRO()

  FIND_G2O_LIBRARY(G2O_CORE_LIBRARIES g2o_core)
  FIND_G2O_LIBRARY(G2O_CLI_LIBRARIES g2o_cli)
  FIND_G2O_LIBRARY(G2O_INCREMENTAL g2o_incremental)
  FIND_G2O_LIBRARY(G2O_INTERACTIVE g2o_interactive)
  FIND_G2O_LIBRARY(G2O_INTERFACE g2o_interface)
  FIND_G2O_LIBRARY(G2O_PARSER g2o_parser)
  FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD g2o_solver_cholmod)
  FIND_G2O_LIBRARY(G2O_SOLVER_DENSE g2o_solver_dense)
  FIND_G2O_LIBRARY(G2O_SOLVER_PCG g2o_solver_pcg)
  FIND_G2O_LIBRARY(G2O_STUFF g2o_stuff)
  FIND_G2O_LIBRARY(G2O_SLAM2D_LIBRARIES g2o_types_slam2d)
  FIND_G2O_LIBRARY(G2O_SLAM3D_LIBRARIES g2o_types_slam3d)
  FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE g2o_solver_csparse)
  FIND_G2O_LIBRARY(G2O_CSPARSE_EXTENSION g2o_csparse_extension)
  FIND_G2O_LIBRARY(G2O_TYPES_ICP g2o_types_icp)
  FIND_G2O_LIBRARY(G2O_TYPES_SBA g2o_types_sba)
  FIND_G2O_LIBRARY(G2O_TYPES_SIM3 g2o_types_sim3)
  FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D g2o_types_slam2d)
  FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D g2o_types_slam3d)
  FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN g2o_solver_eigen)

  SET(G2O_LIBRARIES ${G2O_CORE_LIBRARIES}
                    ${G2O_CLI_LIBRARIES}
                    ${G2O_INCREMENTAL}
                    ${G2O_INTERACTIVE}
                    ${G2O_INTERFACE}
                    ${G2O_PARSER}
                    ${G2O_SOLVER_CHOLMOD}
                    ${G2O_SOLVER_DENSE}
                    ${G2O_SOLVER_PCG}
                    ${G2O_STUFF}
                    ${G2O_SLAM2D_LIBRARIES}
                    ${G2O_SLAM3D_LIBRARIES}
                    ${G2O_SOLVER_CSPARSE}
                    ${G2O_CSPARSE_EXTENSION}
                    ${G2O_TYPES_ICP}
                    ${G2O_TYPES_SBA}
                    ${G2O_TYPES_SIM3}
                    ${G2O_TYPES_SLAM2D}
                    ${G2O_TYPES_SLAM3D}
                    ${G2O_SOLVER_EIGEN}
                    )

  IF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    SET(G2O_FOUND "YES")
    IF(NOT G2O_FIND_QUIETLY)
      MESSAGE(STATUS "Found libg2o: ${G2O_LIBRARIES}")
    ENDIF(NOT G2O_FIND_QUIETLY)
  ELSE(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    IF(NOT G2O_LIBRARIES)
      IF(G2O_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find libg2o!")
      ENDIF(G2O_FIND_REQUIRED)
    ENDIF(NOT G2O_LIBRARIES)

    IF(NOT G2O_INCLUDE_DIR)
      IF(G2O_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find g2o include directory!")
      ENDIF(G2O_FIND_REQUIRED)
    ENDIF(NOT G2O_INCLUDE_DIR)
  ENDIF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)

ENDIF(UNIX)