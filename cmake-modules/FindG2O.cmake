# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# G2O_FOUND, if false, do not try to link against g2o
# G2O_LIBRARIES, path to the libg2o
# G2O_INCLUDE_DIR, where to find the g2o header files
#
# Niko Suenderhauf <niko@etit.tu-chemnitz.de>

IF(UNIX)

  IF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)
    # in cache already
    SET(G2O_FIND_QUIETLY TRUE)
  ENDIF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)

  MESSAGE(STATUS "Searching for g2o ...")

  FIND_PATH(G2O_INCLUDE_DIR
    NAMES core math_groups types
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES include/g2o include
  )
  IF (G2O_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  ENDIF (G2O_INCLUDE_DIR)

  FIND_LIBRARY(G2O_CORE_LIBRARIES
    NAMES g2o_core
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_CLI_LIBRARIES
    NAMES g2o_cli
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_INCREMENTAL
    NAMES g2o_incremental
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_INTERACTIVE
    NAMES g2o_interactive
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_INTERFACE
    NAMES g2o_interface
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_PARSER
    NAMES g2o_parser
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_SOLVER_CHOLMOD
    NAMES g2o_solver_cholmod
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_SOLVER_DENSE
    NAMES g2o_solver_dense
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_SOLVER_PCG
    NAMES g2o_solver_pcg
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_STUFF
    NAMES g2o_stuff
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_SLAM2D_LIBRARIES
    NAMES g2o_types_slam2d
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_SLAM3D_LIBRARIES
    NAMES g2o_types_slam3d
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_SOLVER_CSPARSE
    NAMES g2o_solver_csparse
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_CSPARSE_EXTENSION
    NAMES g2o_csparse_extension
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_TYPES_ICP
    NAMES g2o_types_icp
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_TYPES_SBA
    NAMES g2o_types_sba
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_TYPES_SIM3
    NAMES g2o_types_sim3
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_TYPES_SLAM2D
    NAMES g2o_types_slam2d
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  FIND_LIBRARY(G2O_TYPES_SLAM3D
    NAMES g2o_types_slam3d
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

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
                    ${G2O_TYPES_SLAM3D})

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

