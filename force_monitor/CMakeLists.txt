cmake_minimum_required(VERSION 2.8.3)
project(force_monitor)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  itia_rutils
  diagnostic_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES force_monitor
  CATKIN_DEPENDS geometry_msgs nodelet roscpp sensor_msgs std_msgs itia_rutils diagnostic_msgs
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/force_monitor.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node src/force_monitor_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

