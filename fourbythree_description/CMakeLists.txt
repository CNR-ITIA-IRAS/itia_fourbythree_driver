cmake_minimum_required(VERSION 2.8.3)
project(fourbythree_description)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_description
#  CATKIN_DEPENDS urdf
#  DEPENDS 
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

