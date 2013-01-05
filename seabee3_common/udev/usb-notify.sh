#!/bin/sh

icon_path="/usr/local/seabee/seabee_logo_transparent.png"

device=$1

if [ -z $device ]; then
    device="device"
fi

# Discard the first argument
shift

# Assume that bitrate is specified as second argument (we want to append Mbit/s label)
speed=$1
if [ ! -z $speed ]; then
    info="[ $speed Mbit/s ]"
    shift
fi

# Give each additional argument its own set of brackets
for var in "$@"; do
    if [ -z $var ]; then
	continue
    else
	info="$info[ $var ] "
    fi
done

# We don't know which user connected the device, so we will notify all users
users=`users`

for user in $users; do

    if [ "$user" = "$last_user" ]; then
    	continue;
    fi

    last_user=$user
    
    echo "Using user $user"

    
    pids=`pgrep -u $user gnome-panel`
    
# Have to set DBUS_SESSION_BUS_ADDRESS. notify-send uses it but it is not set when we run as root.
# Also need to run notify-send as the user for which the notification will be displayed
    for pid in $pids; do
	
	DBUS_SESSION_BUS_ADDRESS=`grep -z DBUS_SESSION_BUS_ADDRESS /proc/$pid/environ | sed -e 's/DBUS_SESSION_BUS_ADDRESS=//'`
	
	su "$user" -s /bin/sh -c "DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS notify-send -u low -t 100 -i $icon_path 'Seabee' '""Mounted $device $info""'"
	
    done
    
done