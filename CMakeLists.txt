cmake_minimum_required(VERSION 2.8.3)
project(turbot_imc_broker)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  ros_imc_broker
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS auv_msgs ros_imc_broker roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/turbot_imc_broker_node.cpp
                                    src/turbot_imc_broker.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
