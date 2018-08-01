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

include(oscc/api/OsccConfig.cmake)

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
   BrakeCommand.msg
   BrakeReport.msg
   BrakeReportData.msg
   CanFrame.msg
   CanFrameData.msg
   EnableDisable.msg
   FaultReport.msg
   FaultReportData.msg
   SteeringCommand.msg
   SteeringReport.msg
   SteeringReportData.msg
   ThrottleCommand.msg
   ThrottleReport.msg
   ThrottleReportData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES oscc_api ${PROJECT_NAME}_ros_to_oscc ${PROJECT_NAME}_oscc_to_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

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
)

target_link_libraries(
  ${PROJECT_NAME}_oscc_to_ros
  oscc_api
)

target_link_libraries(
  ${PROJECT_NAME}_ros_to_oscc
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

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_include_directories(
  ${PROJECT_NAME}_node PUBLIC
  include
  oscc/api/include
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

if(EXAMPLE)

add_executable(
  ${PROJECT_NAME}_teleop
  example/${PROJECT_NAME}_teleop.cpp
)

target_include_directories(
  ${PROJECT_NAME}_teleop PUBLIC
  include
)

add_dependencies(
  ${PROJECT_NAME}_teleop
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_teleop
  ${catkin_LIBRARIES}
)

endif()

if(APOLLO AND KIA_SOUL_EV)

find_package(catkin 
             REQUIRED COMPONENTS 
             roscpp 
             rospy 
             std_msgs 
             genmsg)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    /apollo/bazel-apollo/bazel-out/local-dbg/genfiles/
)

add_library(
  ${PROJECT_NAME}_pid_control
  src/pid_control.cpp
)


link_directories(
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/canbus/
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/control/
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/localization/
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/common/proto
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/drivers/canbus/common/
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/drivers/canbus/proto/
    /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/drivers/canbus/ )    

add_executable(
  ${PROJECT_NAME}_apollo
  example/${PROJECT_NAME}_apollo.cpp
)

target_include_directories(
  ${PROJECT_NAME}_apollo PUBLIC
  include
  oscc/api/include
)

add_dependencies(
  ${PROJECT_NAME}_apollo
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_apollo_generate_messages_cpp
)

target_link_libraries(
  ${PROJECT_NAME}_apollo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_pid_control
  /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/canbus/proto/libcanbus_proto_lib.a
  /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/control/proto/libcontrol_proto_lib.a
  /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/localization/proto/liblocalization_proto_lib.a
  /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/localization/proto/libpose_proto_lib.a
  canbus_proto_lib
  protobuf
  canbus_lib
  control_lib
  localization_lib
  canbus_common
  vehicle_signal_proto_lib
  pnc_point_proto_lib
  gnss_status_proto_lib
  car_sound_proto_lib
  header_proto_lib
  common_proto_lib
  error_code_proto_lib
  vehicle_state_proto_lib 
)

endif()

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
  )

  target_link_libraries(
    test_oscc_ros_api
    rapidcheck
    ${PROJECT_NAME}_oscc_to_ros
    oscc_fixture
    ${catkin_LIBRARIES}
  )

endif()

#############
## Install ##
#############

install(
  TARGETS
  oscc_api
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_oscc_to_ros
  ${PROJECT_NAME}_ros_to_oscc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  include/${PROJECT_NAME}
  oscc/api/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
