#!/bin/bash

###########################################################################
#  scripts/create_pkg.sh
#  --------------------
#
#  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of usc-ros-pkg nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

usage()
{
        echo ""
        echo "Usage: create_pkg package [-u user1, -u user2] [-d dependency1, -d dependency2] [-p project] [-n node1, -n node2] [-l nodelet1, -l nodelet2] [-s source1, -s source2]"
        echo ""
}

if [ $# -le 0 ]; then
        usage
        exit
fi

manifest_file="manifest.xml"
makefile_file="Makefile"
cmakelists_file="CMakeLists.txt"
doxygen_file="mainpage.dox"
nodelet_plugins_file="nodelets/nodelet_plugins.xml"

default_user=$ROS_USER;
if [ "$default_user" == "" ]; then default_user=`whoami`; fi

package=$1; shift

while [ "$1" != "" ]; do
	case $1 in
		-u )    	shift
					if [ "$users" == "" ]; then
						users="$1"
						users_cmd="-u $1"
					else
						users="$users $1"
						users_cmd="$users -u $1"
					fi
					shift
					;;
		-d )     	shift
					if [ "$deps" == "" ]; then
						deps="$1"
					else
						deps="$deps $1"
					fi
					shift
					;;
		-n )    	shift
					if [ "$nodes" == "" ]; then
						nodes="$1"
					else
						nodes="$nodes $1"
					fi
					shift
					;;
		-l )    	shift
					if [ "$nodelets" == "" ]; then
						nodelets="$1"
					else
						nodelets="$nodelets $1"
					fi
					shift
					;;
		-s )    	shift
					if [ "$sources" == "" ]; then
						sources="$1"
					else
						sources="$sources $1"
					fi
					shift
					;;
		--help )    usage
					exit
					;;
		-p ) shift
					project=$1
					shift
					;;
	esac
done

if [ "$package" == "" ]; then usage; exit; fi
if [ "$project" == "" ]; then project=$package; fi
if [ "$users" == "" ]; then users=$default_user; fi

echo ""
echo "Creating package $package with authors { $users }, dependencies { $deps }, sources { $sources }, nodes { $nodes }, and nodelets { $nodelets }."
echo ""

mkdir $package
cd $package

echo "<package>
  <description brief=\"$package\">

     $package

  </description>
  <author>$users</author>" >> $manifest_file

#for user in $users; do
#echo "
#  <author>$user</author>" >> $manifest_file;
#done

echo "  <license>BSD</license>
  <review status=\"unreviewed\" notes=\"\"/>
  <url>http://ros.org/wiki/$package</url>" >> $manifest_file

if [ "$nodelets" != "" ]; then deps="nodelet $deps"; fi
if [ "$sources" != "" ]; then deps="base_libs $deps"; fi

for dep in $deps; do
  echo "  <depend package=\"$dep\"/>" >> $manifest_file
done

if [ "$sources" != "" ] || [ "$nodelets" != "" ];
then
  echo "  <export>" >> $manifest_file

  if [ "$sources" != "" ]; then echo "    <cpp cflags=\"-I\${prefix}/include\" lflags=\"-Wl,-rpath,\${prefix}/lib -L\${prefix}/lib -l$package\"/>" >> $manifest_file; fi
  if [ "$nodelets" != "" ]; then echo "    <nodelet plugin=\"\${prefix}/$nodelet_plugins_file\"/>" >> $manifest_file; fi

  echo "  </export>" >> $manifest_file;
fi

echo "</package>" >> $manifest_file

echo 'include $(shell rospack find mk)/cmake.mk' >> $makefile_file

echo 'cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()' >> $cmakelists_file

if [ "$sources" != "" ]; then echo 'rosbuild_include( base_libs add_library_auto )' >> $cmakelists_file; fi

echo '

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
' >> $cmakelists_file

if [ "$sources" != "" ]; then echo 'add_subdirectory( src )' >> $cmakelists_file; fi
if [ "$nodes" != "" ]; then echo 'add_subdirectory( nodes )' >> $cmakelists_file; fi
if [ "$nodelets" != "" ]; then echo 'add_subdirectory( nodelets )' >> $cmakelists_file; fi

echo "/**
\mainpage
\htmlinclude manifest.html

\b $package is ...

<!--
Provide an overview of your package.
-->


\section codeapi Code API

<!--
Provide links to specific auto-generated API documentation within your
package that is of particular interest to a reader. Doxygen will
document pretty much every part of your code, so do your best here to
point the reader to the actual API.

If your codebase is fairly large or has different sets of APIs, you
should use the doxygen 'group' tag to keep these APIs together. For
example, the roscpp documentation has 'libros' group.
-->


*/" >> $doxygen_file

if [ "$sources" != "" ] || [ "$nodes" != "" ]; then mkdir -p include/$package; fi

if [ "$sources" != "" ]; then

mkdir src
echo 'add_library_auto( ${PROJECT_NAME} *.cpp *.cc *.c )' >> src/$cmakelists_file

fi

if [ "$nodes" != "" ]; then

mkdir nodes
echo "# gather all sources in current dir using relative filenames
file( GLOB ALL_SOURCES RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )

foreach( source \${ALL_SOURCES} )
	# ALL_SOURCES = foo.cpp;bar.cpp
	# source = foo.cpp
	# source_name_base = foo
	# source_src = foo.cpp
	# source_name = foo
	get_filename_component( source_name_base \${source} NAME_WE )
	set( source_name \${source_name_base} )
	set( source_src \${source} )

	# rosbuild_add_executable( foo foo.cpp )
	rosbuild_add_executable( \${source_name} \${source_src} )
endforeach( source )" >> nodes/$cmakelists_file

fi

if [ "$nodelets" != "" ]; then

mkdir nodelets
echo "# gather all sources in current dir using relative filenames
file( GLOB NODELET_SOURCES RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )
rosbuild_add_library( \${PROJECT_NAME}_nodelets \${NODELET_SOURCES} )" >> nodelets/$cmakelists_file

echo "<library path=\"lib/lib$package""_nodelets\">" >> $nodelet_plugins_file

fi

for source in $sources; do
  source_h_relpath="include/$package/$source.h"
  source_cpp_relpath="src/$source.cpp"

  touch $source_h_relpath
  echo "#include <$package/$source.h>" >> $source_cpp_relpath

  license_files="$license_files $source_h_relpath $source_cpp_relpath"
done

for node in $nodes; do
  node_h_relpath="include/$package/$node.h"
  node_cpp_relpath="nodes/$node.cpp"

  echo "#ifndef $package""_$package""_$node""_H_
#define $package""_$package""_$node""_H_

#include <base_libs/node.h>

BASE_LIBS_DECLARE_NODE( $node )

BASE_LIBS_DECLARE_NODE_CLASS( $node )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( $node )
	{
		//
	}

	BASE_LIBS_SPIN_FIRST
	{
		initAll();
	}

	BASE_LIBS_SPIN_ONCE
	{
		//
	}
};

#endif // $package""_$package""_$node""_H_" >> $node_h_relpath

  echo "#include <$package/$node.h>
BASE_LIBS_INST_NODE( $node""Node, \"$node""_node\" )" >> $node_cpp_relpath

  license_files="$license_files $node_h_relpath $node_cpp_relpath"
done

for nodelet in $nodelets; do
  nodelet_cpp_relpath="nodelets/$nodelet""_nodelet.cpp"

  echo "#include <base_libs/nodelet.h>
#include <$package/$nodelet.h>

BASE_LIBS_DECLARE_NODELET( $package, $nodelet )

BASE_LIBS_INST_NODELET( $package, $nodelet, $nodelet )" >> $nodelet_cpp_relpath

echo "  <class name=\"$package/$nodelet\" type=\"$package::$nodelet""Nodelet\" base_class_type=\"nodelet::Nodelet\">
    <description>
      todo: fill this in
    </description>
  </class>" >> $nodelet_plugins_file

  license_files="$license_files $nodelet_cpp_relpath"
done

if [ "$nodelets" != "" ]; then echo "</library>" >> $nodelet_plugins_file; fi

if [ "$license_files" != "" ]; then

echo ""
echo "Using `which add_license`"
echo ""

add_license -p $project $users_cmd $license_files

fi

echo "done"
