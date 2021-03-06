cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation tf rostest)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(FILES change_string.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher beginner_tutorials_generate_messages_cpp)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber beginner_tutorials_generate_messages_cpp)

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(testpublisher launch/nodetest.launch test/main.cpp test/testpublisher.cpp)
  target_link_libraries(testpublisher ${catkin_LIBRARIES})
  add_dependencies(testpublisher publisher beginner_tutorials_generate_messages_cpp)
endif()