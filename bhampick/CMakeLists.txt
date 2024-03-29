cmake_minimum_required(VERSION 3.0.2)
project(bhampick)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    tf2_msgs
    std_msgs
    message_generation
    gpd_ros
    vision_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pub.msg
  posefortf.msg
  dimmsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  boxpose.srv
  tf.srv
  graspit.srv
  posedim.srv
  objectlist.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES bhampick
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS
        # moveit_core
        message_runtime
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        
    DEPENDS
        EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/bhampick.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/bhampick_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/client.py
  scripts/client2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bhampick.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(pick src/pick.cpp)
target_link_libraries(pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(tfb src/tfb.cpp)
target_link_libraries(tfb ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS tfb DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(broadcast src/broadcast.cpp)
target_link_libraries(broadcast ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS broadcast DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(testclient src/testclient.cpp)
target_link_libraries(testclient ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS testclient DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasptest src/grasptest.cpp)
target_link_libraries(grasptest ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS grasptest DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(GPDobjtf src/GPDobjtf.cpp)
target_link_libraries(GPDobjtf ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS GPDobjtf DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(markerpub src/markerpub.cpp)
target_link_libraries(markerpub ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS markerpub DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(colmain src/colmain.cpp)
target_link_libraries(colmain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS colmain DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(colmain2nd src/colmain2nd.cpp)
target_link_libraries(colmain2nd ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS colmain2nd DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(colmain3rd src/colmain3rd.cpp)
target_link_libraries(colmain3rd ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS colmain3rd DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(client2 src/client2.cpp)
target_link_libraries(client2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS client2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(colmain3rdnoz src/colmain3rdnoz.cpp)
target_link_libraries(colmain3rdnoz ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS colmain3rdnoz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(splitcolmainplace src/splitcolmainplace.cpp)
target_link_libraries(splitcolmainplace ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS splitcolmainplace DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(splitcolmainpick src/splitcolmainpick.cpp)
target_link_libraries(splitcolmainpick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS splitcolmainpick DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

