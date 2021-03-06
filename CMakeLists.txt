cmake_minimum_required(VERSION 2.8.3)
project(turbot_imc_broker)

set(UIB ON)
set(UDG OFF)

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  auv_msgs
#  cyclops_rhodamine_ros
#  ros_imc_broker
#  tf
#)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time)

if(UIB)
  message("------------------")
  message("UIB Mode selected!")
  message("------------------")
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     auv_msgs
     cyclops_rhodamine_ros
     ros_imc_broker
     tf
     safety
     control
     std_srvs
   )
  add_definitions(-DUIB)
elseif(UDG)
  message("------------------")
  message("UdG Mode selected!")
  message("------------------")
  find_package(catkin COMPONENTS
    cola2_msgs
    roscpp
    auv_msgs
    cyclops_rhodamine_ros
    ros_imc_broker
    tf
    utils
  )
  add_definitions(-DUDG)
endif(UIB)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(BOOST_THREAD -lboost_thread)

catkin_package(
  LIBRARIES ned
  INCLUDE_DIRS include
  CATKIN_DEPENDS auv_msgs safety cyclops_rhodamine_ros ros_imc_broker roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(turbot_imc_ned src/ned.cpp)

add_executable(${PROJECT_NAME}_node src/turbot_imc_broker_node.cpp
                                    src/auv_base.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  turbot_imc_ned
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS turbot_imc_ned ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
