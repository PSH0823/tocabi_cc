cmake_minimum_required(VERSION 2.8.3)
project(tocabi_cc)

add_compile_options(-std=c++17 -Ofast)

find_package(catkin REQUIRED
roscpp
tocabi_lib
tocabi_msgs  
eigen_conversions
)

# find_package(casadi REQUIRED)   # KW add (FOR CASADI)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tocabi_lib tocabi_msgs
  # DEPENDS system_lib
)

# set(LIBRARY_DIRS /usr/local/lib)  # KW add (FOR CASADI)
# set(CASADI_INCLUDE_DIR /usr/local/include/casadi) # KW add (FOR CASADI)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)

string(FIND ${UNAME_RES} "xenomai" wsloc)

if(${wsloc} EQUAL -1 )
  message(" Controller : simulation only compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_SIMULATION)
else()
  message(" xenomai detected realrobot compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
endif() 


###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cc.cpp
  src/control_manager.cpp
  src/task_manager.cpp
  src/walking_manager.cpp
  src/teleop_manager.cpp
  src/cbf_manager.cpp
  src/collision_manager.cpp
  src/kin_wbc.cpp
  src/dyn_wbc.cpp
  src/utils.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  # ${LIBRARY_DIRS}/libcasadi.so  # KW add (FOR CASADI)
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
