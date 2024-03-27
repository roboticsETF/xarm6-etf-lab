# message("Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

set(XARM6_ETF_LAB_PATH ${CMAKE_CURRENT_SOURCE_DIR})
get_filename_component(XARM6_ETF_LAB_PATH ${XARM6_ETF_LAB_PATH} DIRECTORY)
get_filename_component(XARM6_ETF_LAB_PATH ${XARM6_ETF_LAB_PATH} DIRECTORY)
get_filename_component(XARM6_ETF_LAB_PATH ${XARM6_ETF_LAB_PATH} DIRECTORY)

# message("xarm6-etf-lab path: ${XARM6_ETF_LAB_PATH}")
find_library(RPMPL_LIBRARY
  NAMES rpmpl_library
  PATHS ${XARM6_ETF_LAB_PATH}/build/rpmpl_library/src
)

set(RPMPL_PATH "${XARM6_ETF_LAB_PATH}/src/etf_modules/RPMPLv2")

# message("RPMPLv2 path: ${RPMPL_PATH}")
set(RPMPL_LIBRARY_INCLUDE_DIRS
  ${RPMPL_PATH}/include
  ${RPMPL_PATH}/include/state_spaces
  ${RPMPL_PATH}/include/state_spaces/real_vector_space
  ${RPMPL_PATH}/include/planners
  ${RPMPL_PATH}/include/planners/rrt
  ${RPMPL_PATH}/include/planners/rbt
  ${RPMPL_PATH}/include/planners/drbt
  ${RPMPL_PATH}/include/planners/rbt_star
  ${RPMPL_PATH}/include/planners/trajectory
  ${RPMPL_PATH}/include/robots
  ${RPMPL_PATH}/include/environments
  ${RPMPL_PATH}/include/environments/objects
  ${RPMPL_PATH}/include/scenario
  ${RPMPL_PATH}/include/configurations
)

if(RPMPL_LIBRARY)
  message(STATUS "Found rpmpl_library at ${RPMPL_LIBRARY}")
else(RPMPL_LIBRARY)
  message(STATUS "Not found rpmpl_library")
endif(RPMPL_LIBRARY)

find_package(Flann REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
include_directories(${OROCOS_KDL_INCLUDE_DIR})
find_package(yaml-cpp REQUIRED)
find_package(fcl 0.7 REQUIRED)

# RPMPL_LIBRARIES are libraries needed in order to use rpmpl_library
set(RPMPL_LIBRARIES gtest glog nanoflann::nanoflann kdl_parser orocos-kdl fcl ccd yaml-cpp gflags)
