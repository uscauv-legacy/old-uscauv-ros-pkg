###########################################################################
 #  uscauv.bash
 #  --------------------
 #
 #  Copyright (c) 2013, Dylan Foster, Rand Voorhies
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions are
 #  met:
 #
 #  # Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  # Redistributions in binary form must reproduce the above
 #    copyright notice, this list of conditions and the following disclaimer
 #    in the documentation and/or other materials provided with the
 #    distribution.
 #  # Neither the name of USC AUV nor the names of its
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
 ##########################################################################

PATH=$PATH:`rospack find uscauv_scripts`/bash

######################################################################
# Randolph Voorhies' ROS Bash Utilities 
#
# The following are various command-line utility functions and bash
# autocompletions for ROS.
######################################################################

######################################################################
# _rostopic_list_cache: Just like 'rostopic list -v', except the
#                       results are cached for 10 seconds. Nice for
#                       <tab> completion!
_rostopic_list_cache()
{
  local cache_file="/tmp/rosbash.cache"

  if [ -f ${cache_file} ]; then
    local mod_time=$(date --utc --reference=${cache_file} +%s)
    local now_time=$(date +%s)
    local del_time=$((${now_time}-${mod_time}))

    if [ ${del_time} -lt 10 ]; then
      cat ${cache_file}
      return 0
    fi
  fi

  rostopic list -v > ${cache_file}
  cat ${cache_file}
  return 0
}

######################################################################
# rosiv: Launch an image_viewer node. Use tab completion to see all
#        available image topics
rosiv (){
  if [ -z "$1" ]
  then
    echo "Usage: rosiv TopicName"
    echo " (Press [tab] for TopicName autocompletion)"
  else
    rosrun image_view image_view image:=$1
  fi
}
_rosiv()
{
  topics=`_rostopic_list_cache | grep "sensor_msgs/.*Image" | cut -d' ' -f 3 | uniq`
  local cur=${COMP_WORDS[COMP_CWORD]}
  COMPREPLY=( $(compgen -W "${topics}" -- $cur) )
}
complete -F _rosiv rosiv

######################################################################
# rosdv: Launch an disparity_viewer node. Use tab completion to see all
#        available disparity topics
rosdv (){
  if [ -z "$1" ]
  then
    echo "Usage: rosdv TopicName"
    echo " (Press [tab] for TopicName autocompletion)"
  else
    rosrun image_view disparity_view image:=$1
  fi
}
_rosdv()
{
  topics=`_rostopic_list_cache | grep "stereo_msgs/DisparityImage" | cut -d' ' -f 3 | uniq`
  local cur=${COMP_WORDS[COMP_CWORD]}
  COMPREPLY=( $(compgen -W "${topics}" -- $cur) )
}
complete -F _rosdv rosdv

######################################################################
# rosbag completion
_rosbag()
{
  local cur=${COMP_WORDS[COMP_CWORD]}
  local command=${COMP_WORDS[1]}

  case "${command}" in
    record)
      topics=`_rostopic_list_cache | cut -d' ' -f 3 | sed '/^$/d'`
      COMPREPLY=( $(compgen -W "${topics}" -- $cur) )
      return 0
      ;;
    play)
      if [ "${COMP_CWORD}" == "2" ]; then
        COMPREPLY=( $(compgen -o plusdirs -o nospace -o filenames -f -X "!*.bag" -- $cur) )
        return 0
      else
        return 0
      fi
      ;;
  esac

    COMPREPLY=( $(compgen -W "check compress decompress filter fix help info play record reindex" -- $cur) )
}
complete -F _rosbag -o filenames rosbag

######################################################################
# rxplot completion
_rxplot()
{
  local cur=${COMP_WORDS[COMP_CWORD]}
  topics=`rostopic list`

  #if echo "$topics" | grep -x "$cur" ; then
  #  COMPREPLY=( $(compgen -W "" -- $cur) )
  #else
    COMPREPLY=( $(compgen -W "${topics}" -- $cur) )
  #fi

}
complete -F _rxplot rxplot

######################################################################
# rosconfig: just a shortcut for dynamic_reconfigure reconfigure_gui
alias rosconfig='rosrun dynamic_reconfigure reconfigure_gui'

######################################################################
# rviz: just a shortcut for rviz rviz
alias rviz='rosrun rviz rviz'

######################################################################
# dynparam: just a shortcut for dynamic_reconfigure dynparam
alias dynparam='rosrun dynamic_reconfigure dynparam'

######################################################################
# roslaunch completion: Now with .xml completion!
function _roslaunch {
  _roscomplete_search_dir "-type f -regex .*\.launch$\|.*\.test$\|.*\.xml$"
   if [[ $COMP_CWORD == 1 ]]; then
       arg="${COMP_WORDS[COMP_CWORD]}"
       COMPREPLY=($(compgen -o plusdirs -f -X "!*.launch" -- ${arg}) \
                  $(compgen -o plusdirs -f -X "!*.xml" -- ${arg}) \
                  ${COMPREPLY[@]} \
                  $(compgen -o plusdirs -f -X "!*.test" -- ${arg})
                  )
  fi
}
complete -F _roslaunch roslaunch

######################################################################
# Dylan Foster's ROS Bash Utilities 

######################################################################

dev_path="/dev/seabee /dev/uscauv"

# Check for device
function cfd()
{
    if [ -z $1 ]; then
	echo -e "Usage: $0 [device name]\n"
	return -- -1
    fi

    if [ -e "/dev/seabee/$1" ]; then
	return 0
    else
	return 1
    fi
    
}

# Wait for device
function wfd()
{
    if [ -z $1 ]; then
	echo -e "Usage: $0 [device name]\n"
	return -- -1
    fi
                                                                             
    while true; do
	
	sleep 1s
	
	if cfd $1; then                     
            return 0
	fi
	
    done
}

# Poll for device forever
function pfd()
{
    if [ -z $1 ]; then
	echo -e "Usage: $0 [device name]\n"
	return -- -1
    fi
                                                                             
    while true; do
	
	sleep 1s
	
	if cfd $1; then                     
            echo "$1 is connected."                                     
	else                                                                
	    echo "$1 is not connected." 
	fi                                
                                                                           
    done

}

# Wait for camera, the launch the driver
function waitcam()
{

    if [ -z $1 ]; then
	echo "Usage: $0 [camera id]"
	return
    fi

    camera="camera$1"

    wfd $camera && roslaunch uscauv_camera monocular_camera.launch camera:=$camera

}

alias wcam="waitcam"

# A descendent of ls /dev/seabee
function lsb()
{
    for dir in $dev_path; do
	echo "Checking $dir:"
	if [ ! -d "$dir" ]; then
	    echo "No devices found at $dir."
	else
	    ls -lh "$dir"
	fi
    done
}

alias get_image_header="grep header -A 10"

alias uscauv-calibrate-depth="rosservice call /seabee3/calibrate_surface_pressure 10"
alias uscauv-calibrate-ori="rosservice call /xsens_driver/calibrate_rpy_ori 100"
alias uscauv-calibrate-all="uscauv-calibrate-ori && uscauv-calibrate-depth"
alias uscauv-joy="rosrun joy joy_node"
alias uscauv-reconfigure="rosrun rqt_reconfigure rqt_reconfigure"
alias uscauv-observe-mission="roslaunch auv_missions mission_observer.launch"

function uscauv-lock-frames()
{
    rosrun tf static_transform_publisher 0 0 0 0 0 0 $1 $2 100
}

function uscauv-plot-pid()
{
    rqt_plot /control_server/pid/$1/feedback/x:y:e
}

function uscauv-run-motors()
{
    rostopic pub /seabee3/motor_vals seabee3_msgs/MotorVals '{motors: [50, 50, 50, 50, 50, 50, 0, 0, 0], mask: [1, 1, 1, 1, 1, 1, 0, 0, 0] }' -r 10
}

function uscauv-publish-depth()
{
    rostopic pub /robot/sensors/depth seabee3_msgs/Depth $@
}

function uscauv-publish-killswitch()
{
    rostopic pub /robot/sensors/kill_switch seabee3_msgs/KillSwitch $@
}



## The greatest idea ###########################################
alias roslaunch="uscauv-print-logo && roslaunch $@"

################################################################
alias uscauv-master-local="export ROS_MASTER_URI=http://localhost:11311"
alias uscauv-master-seabee="export ROS_MASTER_URI=http://seabee:11311"

alias uscauv-ssh-seabee="ssh -Xv janetkim@seabee"
alias uscauv-arp-scan="sudo arp-scan --interface=eth0 --localnet"

alias uscauv-echo-pressure="rostopic echo /robot/sensors/internal_pressure"
alias uscauv-echo-depth="rostopic echo /robot/sensors/depth"