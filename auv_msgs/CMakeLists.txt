cmake_minimum_required(VERSION 2.4.6)
project(auv_msgs)
find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs sensor_msgs visualization_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

add_message_files(FILES
  ColorEncodedImage.msg	
  MaskedTwist.msg	
  MatchedShapeArray.msg	
  MatchedShape.msg	
  MotorPowerArray.msg	
  MotorPower.msg	
  TrackedObjectArray.msg
  TrackedObject.msg
  )

generate_messages(DEPENDENCIES
  std_msgs sensor_msgs visualization_msgs )

catkin_package(CATKIN_DEPENDS
  message_runtime std_msgs sensor_msgs visualization_msgs)