cmake_minimum_required(VERSION 2.8.3)
project(turbot_imc_broker)

set(UIB ON)
set(UDG OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  auv_msgs
  cyclops_rhodamine_ros
  ros_imc_broker
  tf
)

if(UIB)
  message("------------------")
  message("UIB Mode selected!")
  message("------------------")
  # find_package(catkin COMPONENTS
  #   safety
  # )
  add_definitions(-DUIB)
elseif(UDG)
  message("------------------")
  message("UdG Mode selected!")
  message("------------------")
  find_package(catkin COMPONENTS
    cola2_msgs
  )
  add_definitions(-DUDG)
endif(UIB)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS auv_msgs cyclops_rhodamine_ros ros_imc_broker roscpp tf
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

