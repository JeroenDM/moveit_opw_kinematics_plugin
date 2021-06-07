cmake_minimum_required(VERSION 2.8.3)
project(moveit_opw_kinematics_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_core
  roscpp
  pluginlib
)

set(MOVEIT_LIB_NAME moveit_opw_kinematics_plugin)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(opw_kinematics REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${MOVEIT_LIB_NAME}
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${MOVEIT_LIB_NAME}
  src/moveit_opw_kinematics_plugin.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  opw_kinematics::opw_kinematics
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    moveit_opw_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_ros_planning REQUIRED)
  find_package(rostest REQUIRED)

  include_directories(test ${moveit_ros_planning_INCLUDE_DIRS})
  
  add_rostest_gtest(${PROJECT_NAME}_test_kuka
    test/test_kuka.rostest test/test_plugin.cpp
    test/test_kuka_specific.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_kuka
    gtest
    ${moveit_ros_planning_LIBRARIES}
    ${catkin_LIBRARIES}
    ${MOVEIT_LIB_NAME}
  )

  add_rostest_gtest(${PROJECT_NAME}_test_fanuc
    test/test_fanuc.rostest
    test/test_plugin.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_fanuc
    gtest
    ${moveit_ros_planning_LIBRARIES}
    ${catkin_LIBRARIES}
    ${MOVEIT_LIB_NAME}
  )

  add_rostest_gtest(${PROJECT_NAME}_test_fanuc_faulty
    test/test_fanuc_faulty_params.rostest
    test/test_faulty_params.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_fanuc_faulty
    gtest
    ${moveit_ros_planning_LIBRARIES}
    ${catkin_LIBRARIES}
    ${MOVEIT_LIB_NAME}
  )

  add_rostest_gtest(${PROJECT_NAME}_test_kuka_faulty
    test/test_kuka_faulty_params.rostest
    test/test_faulty_params.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_kuka_faulty
    gtest
    ${moveit_ros_planning_LIBRARIES}
    ${catkin_LIBRARIES}
    ${MOVEIT_LIB_NAME}
  )
endif()
