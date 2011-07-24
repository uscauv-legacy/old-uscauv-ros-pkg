#!/bin/bash

echo ""
echo -n 'Install compiler tools? (y/n) '
read res
if [ "$res" == "y" ]; then
	sudo apt-get update && sudo apt-get install build-essential python-yaml cmake subversion wget
#	wget --no-check-certificate http://ros.org/rosinstall -O ~/rosinstall
#	chmod 755 ~/rosinstall
fi
echo "----------"

echo ""
echo -n 'Install ros-cturtle-base? (y/n) '
read res
if [ "$res" == "y" ]; then
	sudo sh -c 'echo "deb http://code.ros.org/packages/ros/ubuntu lucid main" > /etc/apt/sources.list.d/ros-latest.list'
	wget http://code.ros.org/packages/ros.key -O - | sudo apt-key add -
	sudo apt-get update
	sudo apt-get install ros-cturtle-base
fi
echo "----------"

echo ""
echo -n 'Install pr2-controllers? (y/n) '
read res
if [ "$res" == "y" ]; then
	sudo apt-get install ros-cturtle-pr2-controllers
fi
echo "----------"

echo ""
echo -n 'Configure environment variables? (y/n) '
read res
if [ "$res" == "y" ]; then
	sudo sh -c 'echo "source /opt/ros/cturtle/setup.sh" >> ~/.bashrc'
	sudo sh -c 'echo "\nexport ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:~/workspace" >> /opt/ros/cturtle/setup.sh'
	. ~/.bashrc
fi
echo "----------"

echo ""
echo -n 'Download seabee3-ros-pkg? (y/n) '
read res
if [ "$res" == "y" ]; then
	svn co https://seabee3-ros-pkg.googlecode.com/svn/trunk/cturtle/ ~/workspace/seabee3-ros-pkg
	rosstack profile && rospack profile
fi
echo "----------"

echo ""
echo -n 'Compile seabee3-ros-pkg? (y/n) '
read res
if [ "$res" == "y" ]; then
	rosdep install seabee3-ros-pkg
	rosmake seabee3-ros-pkg
fi
echo "----------"

echo ""
echo -n 'Update permissions for video devices? (y/n) '
read res
if [ "$res" == "y" ]; then
	sudo sh ~/workspace/seabee3-ros-pkg/seabee3_util/config-1394.sh
fi
echo "----------"

echo ""
echo 'Done'
