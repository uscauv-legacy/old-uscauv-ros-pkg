# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(seabee3_actions)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation seabee3_msgs)

add_action_files(DIRECTORY action FILES
  Configure.action
  FollowTrajectory.action
  MakeTrajectory.action
  MatchContours.action
  )

generate_messages(DEPENDENCIES actionlib_msgs seabee3_msgs)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS actionlib seabee3_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)