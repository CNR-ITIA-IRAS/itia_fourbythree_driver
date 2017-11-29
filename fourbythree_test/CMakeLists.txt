cmake_minimum_required(VERSION 2.8.3)
project(fourbythree_test)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  itia_controllers_and_filters
  itia_futils
  itia_rutils
  roscpp
  sensor_msgs
  std_msgs
  rosbag
  nodelet
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fourbythree_test
 CATKIN_DEPENDS itia_controllers_and_filters itia_futils itia_rutils roscpp sensor_msgs std_msgs rosbag nodelet
 DEPENDS  mutex
)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(fourbythree_test  src/fourbythree_test/multisine_nodelet.cpp
                              src/fourbythree_test/feedforward_action.cpp
)
add_dependencies(fourbythree_test  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fourbythree_test 
  ${catkin_LIBRARIES}
)


add_executable(low_frequency_filter_test src/low_frequency_filter_test.cpp)
add_dependencies(low_frequency_filter_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(low_frequency_filter_test
  ${catkin_LIBRARIES}
)

add_executable(calibration_test src/trj_test.cpp)
add_dependencies(calibration_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibration_test
  ${catkin_LIBRARIES}
)

add_executable(test_multisine_single_axis src/test_multisine.cpp)
add_dependencies(test_multisine_single_axis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_multisine_single_axis
  ${catkin_LIBRARIES}
)


add_executable(log_spfb src/log_to_binary.cpp)
add_dependencies(log_spfb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(log_spfb
  ${catkin_LIBRARIES}
)

