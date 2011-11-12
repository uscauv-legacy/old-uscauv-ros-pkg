#!/bin/bash

###########################################################################
#  scripts/create_lib.sh
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
        echo "Usage: create_lib package [-u user] [-d dependency] [-p project] [-i include_dir] Class1 Class2 ..."
        echo ""
}

if [ $# -le 0 ]; then
        usage
        exit
fi

package=$1; shift

while [ "$1" != "" ]; do
	case $1 in
		-u )        shift
					if [ "$users_cmd" == "" ]; then
						users_cmd="-u $1"
					else
						users_cmd="$users_cmd -u $1"
					fi
					shift
					;;
		-d )        shift
					if [ "$deps_cmd" == "" ]; then
						deps_cmd="-d $1"
					else
						deps_cmd="$deps_cmd -d $1"
					fi
					shift
					;;
		--help )    usage
					exit
					;;
		-p )        shift
					project_cmd="-p $1"
					shift
					;;
		-i )        shift
					include_dir_cmd="-i $1"
					shift
					;;
		* )         if [ "$1" != "" ] && [ "$1" != "-d" ] && [ "$1" != "--help" ] && [ "$1" != "-u" ] && [ "$1" != "-p" ]; then
						sources_cmd="$sources_cmd -s $1"
						shift
					fi
					;;
	esac
done

if [ "$package" == "" ]; then usage; exit; fi

create_pkg $package $users_cmd $deps_cmd $project_cmd $include_dir_cmd $sources_cmd
