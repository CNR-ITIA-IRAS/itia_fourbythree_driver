cmake_minimum_required(VERSION 2.8.3)
project(ndlcom_driver)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast -fPIC -Dlinux -D_LOGCNR)
set(CMAKE_BUILD_TYPE Release)

set(myPath "/usr/local/include/ndlcom_device_driver")

if(EXISTS "${myPath}")

    find_package(catkin REQUIRED COMPONENTS
      roscpp std_msgs sensor_msgs std_srvs itia_rutils itia_msgs nodelet itia_controllers_and_filters diagnostic_msgs urdf
    )

    set(NDLCOM_DEVICE_DRIVER_VERBOSITY_FLAGS "")
    add_definitions(${NDLCOM_DEVICE_DRIVER_VERBOSITY_FLAGS})

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ndlcom_driver
      CATKIN_DEPENDS roscpp std_msgs sensor_msgs std_srvs itia_rutils itia_msgs nodelet itia_controllers_and_filters diagnostic_msgs urdf
      DEPENDS 
    )

    include_directories(include)
    include_directories(
      ${catkin_INCLUDE_DIRS}
      /usr/local/include/ndlcom_device_driver/extras
    )

    set( ndlcom_libs
        ndlcom_serialelasticjoints
        ndlcom_device_driver
        RobotConfig
        representations
        ndlcom
      )

      add_library(ndlcom_driver
        src/ndlcom_driver/ndlcom_utils.cpp
        src/ndlcom_driver/ndlcom_nodelet.cpp
        src/ndlcom_driver/ndlcom_nodelet_fake.cpp
        src/ndlcom_driver/ndlcom_threads.cpp
        src/ndlcom_driver/serial_communication.cpp
      )

      add_dependencies(ndlcom_driver ${catkin_EXPORTED_TARGETS})

      target_link_libraries( ndlcom_driver
        ${ndlcom_libs}
        ${catkin_LIBRARIES}
      )

else()
    message(WARNING "Cannot compile ndlcom_driver package because the ndlcom_device_driver is not installed.") 
endif()


