# message("Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

set(XARM6_ETF_LAB_PATH ${CMAKE_CURRENT_SOURCE_DIR})
get_filename_component(XARM6_ETF_LAB_PATH ${XARM6_ETF_LAB_PATH} DIRECTORY)
get_filename_component(XARM6_ETF_LAB_PATH ${XARM6_ETF_LAB_PATH} DIRECTORY)
get_filename_component(XARM6_ETF_LAB_PATH ${XARM6_ETF_LAB_PATH} DIRECTORY)

# message("xarm6-etf-lab path: ${XARM6_ETF_LAB_PATH}")
find_library(SIM_BRINGUP_LIBRARY
  NAMES sim_bringup_library
  PATHS ${XARM6_ETF_LAB_PATH}/build/sim_bringup/src
)

set(SIM_BRINGUP_PATH "${XARM6_ETF_LAB_PATH}/src/etf_modules/sim_bringup")

# message("sim_bringup path: ${SIM_BRINGUP_PATH}")
set(SIM_BRINGUP_LIBRARY_INCLUDE_DIRS
  ${SIM_BRINGUP_PATH}/include
  ${SIM_BRINGUP_PATH}/include/base
  ${SIM_BRINGUP_PATH}/include/demos
  ${SIM_BRINGUP_PATH}/include/environments
)

if(SIM_BRINGUP_LIBRARY)
  message(STATUS "Found sim_bringup_library at ${SIM_BRINGUP_LIBRARY}")
else(SIM_BRINGUP_LIBRARY)
  message(STATUS "Not found sim_bringup_library")
endif(SIM_BRINGUP_LIBRARY)
