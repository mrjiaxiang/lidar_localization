cmake_minimum_required(VERSION 3.10.2)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++17)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
  cv_bridge
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/geographic.cmake)
include(cmake/opencv.cmake)
include(cmake/ceres.cmake)
include(cmake/g2o.cmake)
include(cmake/YAML.cmake)
include(cmake/sophus.cmake)

# find_package(G2O REQUIRED)
# include_directories(${G2O_INCLUDE_DIRS})
# add_executable(g2oCurveFitting g2oCurveFitting.cpp)
# target_link_libraries(g2oCurveFitting g2o_core g2o_stuff)


include_directories(include ${catkin_INCLUDE_DIRS})

include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")

list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(hello_kitti_node src/apps/hello_kitti_node.cpp ${ALL_SRCS})
target_link_libraries(hello_kitti_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(orange_data_callback_node src/apps/orange_data_callback_node.cpp ${ALL_SRCS})
target_link_libraries(orange_data_callback_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

# add_executable(eigen_dynamic_node src/test/Eigen/eigen_dynamic.cpp ${ALL_SRCS})
# target_link_libraries(eigen_dynamic_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

install(
    TARGETS 
        hello_kitti_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
    DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(
    DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(
    DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(
    DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)