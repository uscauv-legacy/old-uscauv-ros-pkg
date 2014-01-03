# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(seabee3_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs)

# uncomment if you have defined messages
add_message_files(FILES
  ContourArray.msg	
  Contour.msg		
  Depth.msg		
  Imu.msg		
  KillSwitch.msg	
  LandmarkArray.msg	
  Landmark.msg		
  MatchQualityArray.msg	
  MatchQuality.msg	
  MotorVals.msg		
  NamedImageArray.msg	
  NamedImage.msg	
  Point2D.msg		
  Pressure.msg		
  TrajectoryInterval.msg
  Trajectory.msg	
  TrajectoryWaypoint.msg
  )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
  )

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
  DEPENDS 
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
  INCLUDE_DIRS
  LIBRARIES
  )