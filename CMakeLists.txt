cmake_minimum_required(VERSION 3.0.2)
project(Rendezvous)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PX4Vision_AutonomousLanding
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs mavros_msgs message_runtime
#  DEPENDS system_lib
)

add_executable(mocap_odometry_to_fcu
  src/mocap_odometry_to_fcu.cpp
)
add_executable(simulator_node
  src/simulator_node.cpp
)
add_executable(UAVoffboard_node
  src/UAVoffboard_node.cpp
)
add_executable(UGVoffboard_node
  src/UGVoffboard_node.cpp
)
add_executable(centralized_rendezvous
  src/Centralized_Rendezvous.cpp
)

add_library(MPC SHARED IMPORTED)
set_target_properties(MPC PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/SharedLibs/MPC.so)
add_library(RK4 SHARED IMPORTED)
set_target_properties(RK4 PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/SharedLibs/rk4.so)
add_library(MPC_UGV SHARED IMPORTED)
set_target_properties(MPC_UGV PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/SharedLibs/MPC_UGV.so)
add_library(RK4_UGV SHARED IMPORTED)
set_target_properties(RK4_UGV PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/SharedLibs/rk4_UGV.so)

target_link_libraries(mocap_odometry_to_fcu
  ${catkin_LIBRARIES}
)
target_link_libraries(simulator_node
  RK4
  RK4_UGV
  ${CMAKE_DL_LIBS}
  ${catkin_LIBRARIES} 
)
target_link_libraries(UAVoffboard_node
   MPC
   ${CMAKE_DL_LIBS}
   ${catkin_LIBRARIES} 
)
target_link_libraries(UGVoffboard_node
   MPC_UGV
   ${CMAKE_DL_LIBS}
   ${catkin_LIBRARIES} 
)
target_link_libraries(centralized_rendezvous
  ${catkin_LIBRARIES}
)
