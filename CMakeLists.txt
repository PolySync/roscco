cmake_minimum_required(VERSION 2.8.3)
project(roscco)

## Compile as C++11, supported in ROS Kinetic and newer
## C++11 is required for rapidcheck
if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

if(KIA_SOUL)
    add_definitions(-DKIA_SOUL)
elseif(KIA_SOUL_EV)
    add_definitions(-DKIA_SOUL_EV)
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
   FILES
   BrakeReportData.msg
   BrakeReport.msg
   BrakeCommand.msg
   SteeringReportData.msg
   SteeringReport.msg
   SteeringCommand.msg
   ThrottleReportData.msg
   ThrottleReport.msg
   ThrottleCommand.msg
   FaultReportData.msg
   FaultReport.msg
   CanFrame.msg
   CanFrameData.msg
   EnableDisable.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscco oscc_api
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

###########
## Build ##
###########

add_library(
  oscc_api
  oscc/api/src/oscc.c
)

target_include_directories(
  oscc_api PUBLIC
  include
  oscc/api/include
)

add_library(
  ${PROJECT_NAME}_ros_to_oscc
  src/ros_to_oscc.cpp
)

target_include_directories(
  ${PROJECT_NAME}_ros_to_oscc PUBLIC
  include
  oscc/api/include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_oscc_to_ros
  src/oscc_to_ros.cpp
)

target_include_directories(
  ${PROJECT_NAME}_oscc_to_ros PUBLIC
  include
  oscc/api/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}_oscc_to_ros
  oscc_api
)

add_dependencies(
  ${PROJECT_NAME}_ros_to_oscc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  ${PROJECT_NAME}_oscc_to_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/roscco_node.cpp)

target_include_directories(
  ${PROJECT_NAME}_node PUBLIC
  include
  oscc/api/include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  oscc_api
  ${PROJECT_NAME}_ros_to_oscc
  ${PROJECT_NAME}_oscc_to_ros
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_subdirectory(test/rapidcheck)

  add_library(
    oscc_fixture
    test/oscc_fixture.c
  )


  target_include_directories(
    oscc_fixture PUBLIC
    test/include
    oscc/api/include/can_protocols
    ${catkin_INCLUDE_DIRS}
  )

  add_rostest_gtest(
    test_ros_oscc_api
    test/test_ros_to_oscc.launch
    test/test_ros_to_oscc.cpp
  )

  target_include_directories(
    test_ros_oscc_api PUBLIC
    test/include
    test/rapidcheck/include
    test/rapidcheck/extras/gtest/include
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(
    test_ros_oscc_api
    rapidcheck
    ${PROJECT_NAME}_ros_to_oscc
    oscc_fixture
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(
    test_oscc_ros_api
    test/test_oscc_to_ros.launch
    test/test_oscc_to_ros.cpp
  )

  target_include_directories(
    test_oscc_ros_api PUBLIC
    include
    test/include
    test/rapidcheck/include
    test/rapidcheck/extras/gtest/include
    oscc/api/include/can_protocols
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(
    test_oscc_ros_api
    rapidcheck
    ${PROJECT_NAME}_oscc_to_ros
    oscc_fixture
    ${catkin_LIBRARIES}
  )

endif()
