cmake_minimum_required(VERSION 2.8.3)
project(configlib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure roscpp xmlrpcpp
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  LIBRARIES configlib
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
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

add_library(${PROJECT_NAME}
  src/config.cpp
)

add_library(${PROJECT_NAME}_ros
  src/config_server_ros.cpp
  src/dynamic_reconfigure_conversions.cpp
)

target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_example_node src/example_node.cpp)
set_target_properties(${PROJECT_NAME}_example_node PROPERTIES OUTPUT_NAME example_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_example_node ${PROJECT_NAME}_ros ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros ${PROJECT_NAME}_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(${PROJECT_NAME}_config_test test/test_config.cpp)
  target_link_libraries(${PROJECT_NAME}_config_test ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_parameter_test test/test_parameter.cpp)
  target_link_libraries(${PROJECT_NAME}_parameter_test ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_dynamic_reconfigure_conversions_test test/test_dynamic_reconfigure_conversions.cpp)
  target_link_libraries(${PROJECT_NAME}_dynamic_reconfigure_conversions_test ${PROJECT_NAME}_ros ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_config_server_ros_test test/config_server_ros.test test/test_config_server_ros.cpp)
  target_link_libraries(${PROJECT_NAME}_config_server_ros_test ${PROJECT_NAME}_ros ${catkin_LIBRARIES})
endif()

