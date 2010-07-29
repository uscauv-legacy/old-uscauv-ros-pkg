#!/bin/bash

MY_PROMPT='$ '
MY_YESNO_PROMPT='(y/n)$ '

grpname="pgrimaging"


 
if [ "$(id -u)" = "0" ]
then
    echo
    echo "This script will assist users in configuring their udev rules to allow";
    echo "access to 1394 and usb devices.  The script will create a udev rule which";
    echo "will added the 1394 cards and usb devices to a user group of your choosing.";
    echo "You can then also add yourself as a member of that group.  As a last step";
    echo "the user may choose to restart the udev daemon.  All of this can be done";
    echo "manually as well.  Please note that this script will change the permissions";
    echo "for all ieee1394 devices including hard drives and web cams.  It will allow";
    echo "the user to read and modify data on any IEEE1394 device.";
    echo
    echo
else
    echo
    echo "This script needs to be run as root.";
    echo "eg.";
    echo "sudo sh config-1394.sh";
    echo
    exit 0
fi
 

#while :
#do
#    echo "Please enter the name of the user group you'd like to add the 1394";
#    echo "and usb devices to:";

#    echo -n "$MY_PROMPT"
#    read grpname
#    echo "Is this group name ok?: $grpname";
#    echo -n "$MY_YESNO_PROMPT"
#    read confirm

#    if [ $confirm = "y" ] || [ $confirm = "Y" ] || [ $confirm = "yes" ] || [ $confirm = "Yes" ]
#    then
#        break
#    fi
#    done
    
while :
do
    echo "Please enter the name of the user to add to this user group.";

    echo -n "$MY_PROMPT"
    read usrname
    echo "Is this user name ok?: $usrname";
    echo -n "$MY_YESNO_PROMPT"
    read confirm

    if [ $confirm = "y" ] || [ $confirm = "Y" ] || [ $confirm = "yes" ] || [ $confirm = "Yes" ]
    then
        break
    fi
    done

echo
echo "About to add user $usrname to group $grpname.";
echo "Is this ok?:";
echo -n "$MY_YESNO_PROMPT"
read confirm

if [ $confirm = "y" ] || [ $confirm = "Y" ] || [ $confirm = "yes" ] || [ $confirm = "Yes" ]
then
    groupadd -f $grpname
    usermod -a -G $grpname $usrname
else
    echo
    echo "$usrname was not added to group $grpname.  Please configure your devices manually";
    echo "or re-run this script.";
    exit 0
fi

UdevFile="/etc/udev/rules.d/40-pgr.rules";
echo
echo "Writing the udev rules file.";
echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"2000\", MODE=\"0664\", GROUP=\"$grpname\"" 1>$UdevFile
echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"2001\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile
echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"2002\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile
echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"2003\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile
echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"2004\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile
echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"2005\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile
echo "KERNEL==\"raw1394\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile
echo "KERNEL==\"video1394*\", MODE=\"0664\", GROUP=\"$grpname\"" 1>>$UdevFile


echo
echo "Almost Done.  Do you want to restart the udev daemon?";
echo -n "$MY_YESNO_PROMPT"
read confirm

if [ $confirm = "y" ] || [ $confirm = "Y" ] || [ $confirm = "yes" ] || [ $confirm = "Yes" ]
then
    /etc/init.d/udev restart
else
    echo
    echo "Udev was not restarted.  Please reboot the computer for the rules to take effect.";
    exit 0
fi


echo
echo "Everything is done.  On some machines you will need to restart";
echo "for the new udev rules to take effect.";
echo



exit 0
