#!/bin/sh

PDIR=${0%`basename $0`}

USB_PATH=/usr/local/seabee/
RULES_PATH=/etc/udev/rules.d/

if [ ! -d $USB_PATH ]; then
    mkdir $USB_PATH
fi

cp "$PDIR/seabee_logo_transparent.png" -t $USB_PATH
cp "$PDIR/usb-notify.sh"               -t $USB_PATH

cp "$PDIR/81-seabee.rules" -t $RULES_PATH