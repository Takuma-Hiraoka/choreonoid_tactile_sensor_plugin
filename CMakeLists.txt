cmake_minimum_required(VERSION 3.0.2)
project(choreonoid_tactile_sensor_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS)
find_package(choreonoid REQUIRED)
find_package(Boost 1.54.0 REQUIRED COMPONENTS program_options)

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
  src/TactileSensorItem.cpp
  )
target_link_libraries(CnoidTactileSensorPlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )

set_target_properties(CnoidTactileSensorPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(config_writer src/config_writer.cpp)
target_link_libraries(config_writer ${CHOREONOID_BODY_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})

# add_custom_target(make_SAMPLE_CONFIG ALL
#   COMMAND ${CATKIN_DEVEL_PREFIX}/lib/choreonoid_tactile_sensor_plugin/config_writer -i ${PROJECT_SOURCE_DIR}/sample/sample.yaml -o ${PROJECT_SOURCE_DIR}/sample/sample_config.yaml)

## sample
configure_file(${PROJECT_SOURCE_DIR}/sample/TactileSensor.cnoid.in ${PROJECT_SOURCE_DIR}/sample/TactileSensor.cnoid @ONLY)
