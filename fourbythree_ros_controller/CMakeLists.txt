cmake_minimum_required(VERSION 2.8.3)
project(fourbythree_ros_controller)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  nodelet
  roscpp
  itia_futils
  itia_nodelet_hw_interface
  itia_msgs
  control_msgs
  actionlib
  itia_dynamics_core
  diagnostic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fourbythree_ros_controller
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface nodelet roscpp itia_futils itia_nodelet_hw_interface itia_msgs control_msgs actionlib itia_dynamics_core diagnostic_msgs
  DEPENDS 
)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(fourbythree_ros_controller
  src/${PROJECT_NAME}/fourbythree_controller_manager.cpp
  src/${PROJECT_NAME}/fourbythree_driver_manager.cpp
)
add_dependencies(fourbythree_ros_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fourbythree_ros_controller
  fourbythree_ros_controller
  ${catkin_LIBRARIES}
)

add_executable(fourbythree_ros_controller_gui src/fourbythree_start_gui.cpp)
add_dependencies(fourbythree_ros_controller_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fourbythree_ros_controller_gui
  fourbythree_ros_controller
  ${catkin_LIBRARIES}
)

add_executable(fourbythree_controller_manager src/fourbythree_controller_manager.cpp)
add_dependencies(fourbythree_controller_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fourbythree_controller_manager
  fourbythree_ros_controller
  ${catkin_LIBRARIES}
)


add_executable(tekniker_itia_wrapper_node src/tekniker_itia_wrapper_node.cpp)
add_dependencies(tekniker_itia_wrapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tekniker_itia_wrapper_node
  fourbythree_ros_controller
  ${catkin_LIBRARIES}
)

add_executable(gain_scheduling src/gain_scheduling.cpp)
add_dependencies(gain_scheduling ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gain_scheduling
  fourbythree_ros_controller
  ${catkin_LIBRARIES}
)
