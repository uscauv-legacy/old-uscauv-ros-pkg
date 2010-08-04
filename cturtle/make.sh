#!/bin/bash
source /home/edward/ros/setup.sh

opt=$1
num=$#
shift

if [ "$1" = "all" ]; then
	if [ "$opt" = "strip-svn" ]; then
		find -type d -name ".svn" | while read file; do
			rm -rf $file
		done
	else
		find -maxdepth 1 -type d ! -name ".*" | while read file; do
			cd $file
			make $opt
			if [ "$opt" = "clean" ]; then
				rm -rf bin
			fi
			cd ..
		done
	fi
else
	for ((i = 1; i < num; i++))
	do
		roscd $1
		make $opt
		shift
	done
fi
