cmake_minimum_required(VERSION 2.8.3)
project(dumbo_ft_sensor_frequency_check)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    dumbo_force_torque_sensor
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS  roscpp dumbo_force_torque_sensor
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/dumbo_ft_sensor_frequency_check.cpp)
target_link_libraries(${PROJECT_NAME} rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
