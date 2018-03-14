cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  base_local_planner
  angles
  tf
  nav_core
  costmap_2d
)

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
  INCLUDE_DIRS include
  LIBRARIES carrot_planner
  CATKIN_DEPENDS
      roscpp
      pluginlib
      costmap_2d
      base_local_planner
      nav_core
      tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 include/${PROJECT_NAME}
 ${catkin_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})


## CARROT PLANNER
# a very basic global planner plugin
 add_library(${PROJECT_NAME}_carrot src/carrot.cpp)
 add_dependencies(${PROJECT_NAME}_carrot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_carrot ${catkin_LIBRARIES}
)

## RRTS PLANNER
# The RRTS global planner plugin
 add_library(${PROJECT_NAME}_rrt src/rrts_ros.cpp src/system_ros.cpp src/kdtree.c)
 add_dependencies(${PROJECT_NAME}_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_rrt ${catkin_LIBRARIES}
 )
 
 ## RRT PLANNER TESTER
 # The tester node  for the RRTS_PLANNER library
 add_executable(${PROJECT_NAME}_tester src/test_ros.cpp src/rrts_ros.cpp src/system_ros.cpp src/kdtree.c)
 add_dependencies(${PROJECT_NAME}_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_tester ${catkin_LIBRARIES})
 

## RRTS_STANDALONE      [Optional - disabled by default]
# Standalone RRTS program (no use of ROS)
# This building method is provided as a bonus : a simple 'make rrtstar' has the same result
add_executable(${PROJECT_NAME}_standalone src/rrts_standalone.cpp src/kdtree.c src/system_single_integrator.cpp)
set_target_properties(${PROJECT_NAME}_standalone PROPERTIES OUTPUT_NAME rrts_standalone PREFIX "")
add_dependencies(${PROJECT_NAME}_standalone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_standalone
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS ${PROJECT_NAME}_carrot
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_rrt
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES rrt_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES carrot_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




