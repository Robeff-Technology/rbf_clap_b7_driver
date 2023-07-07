cmake_minimum_required(VERSION 3.8)
project(rbf_clap_b7_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

include_directories(include/rbf_clap_b7_driver)
include_directories(rbf_clap_b7_msgs)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rbf_clap_b7_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
  )

set(CLAP_B7_DRIVER_LIB_SRC
        src/TermiosSerial.cpp
        src/ClapB7DriverNode.cpp
        src/ClapB7BinaryParser.cpp
        src/ClapB7Driver.cpp
        src/AsyncSerial.cpp
        )

set(CLAP_B7_DRIVER_LIB_HEADERS
        include/rbf_clap_b7_driver/TermiosSerial.h
        include/rbf_clap_b7_driver/clap_b7.h
        include/rbf_clap_b7_driver/ClapB7Driver.hpp
        include/rbf_clap_b7_driver/AgricInfoInterface.h
        )

set(NTRIP_LIB_SRC
        src/ntrip_client.cpp
        src/ntrip_util.cpp)

set(NTRIP_LIB_HEADER
        include/rbf_clap_b7_driver/ntrip/cmake_definition.h
        include/rbf_clap_b7_driver/ntrip/ntrip_client.h
        include/rbf_clap_b7_driver/ntrip/ntrip_util.h
        include/rbf_clap_b7_driver/ntrip/thread_raii.h
        )

add_executable(rbf_clap_b7_driver_exe
        ${CLAP_B7_DRIVER_LIB_SRC}
        ${CLAP_B7_DRIVER_LIB_HEADER}
        ${NTRIP_LIB_SRC}
        ${NTRIP_LIB_HEADER}
        ${GeographicLib_LIBRARIES}
        )

target_link_libraries(rbf_clap_b7_driver_exe
  ${GeographicLib_LIBRARIES}
  )
ament_target_dependencies(rbf_clap_b7_driver_exe
        rclcpp
        std_msgs
        rbf_clap_b7_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        nmea_msgs
        tf2
        autoware_sensing_msgs
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
        mavros_msgs

        )

install(TARGETS
        rbf_clap_b7_driver_exe
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
