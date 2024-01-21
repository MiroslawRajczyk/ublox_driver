cmake_minimum_required(VERSION 3.0.2)
project(ublox_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tools
  std_msgs
  rostest
  code_coverage
)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ublox_driver
  src/main.cpp
)
add_dependencies(ublox_driver
  tools_generate_messages_cpp
  tools_generate_messages
  tools_generate_messages_py
  tools_generate_messages_lisp)
target_link_libraries(ublox_driver ${catkin_LIBRARIES})

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  # Add your tests here
  # Add coverage excludes separated with spaces - THIS CANNOT BE EMPTY
  # set(COVERAGE_EXCLUDES "*${PROJECT_NAME}/test*")
  # add_code_coverage(NAME ${PROJECT_NAME}_coverage DEPENDENCIES tests)
endif()
