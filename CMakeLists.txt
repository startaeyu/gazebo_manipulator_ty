cmake_minimum_required(VERSION 3.0.2)
project(manipulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_ros
  message_generation
  roscpp
  std_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
  FILES
#   Message1.msg
#   Message2.msg
three.msg
nine.msg
 )



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES manipulator
  CATKIN_DEPENDS gazebo_ros roscpp std_msgs
  DEPENDS system_lib
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(sub_manipulator src/sub_manipulator.cpp)
target_link_libraries(sub_manipulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(sub_manipulator manipulator_generate_messages_cpp)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(controller manipulator_generate_messages_cpp)
