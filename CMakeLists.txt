cmake_minimum_required(VERSION 3.0.2)
project(choreonoid_tactile_sensor_plugin)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS)
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(CnoidTactileSensorPlugin SHARED
  src/CnoidTactileSensorPlugin.cpp
  src/tactile_shm.cpp
  )
target_link_libraries(CnoidTactileSensorPlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )

set_target_properties(CnoidTactileSensorPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})

## sample
configure_file(${PROJECT_SOURCE_DIR}/sample/TactileSensor.cnoid.in ${PROJECT_SOURCE_DIR}/sample/TactileSensor.cnoid @ONLY)
