#!/usr/bin/env bash
# STUDENTS: DO NOT MODIFY THIS FILE, DOING SO MAY BRICK YOUR PYNQ BOARD

function fn()
{
	echo "TU/e PYNQ platform, (C) 2023"
	echo -e "\a"
	echo "Board Info:"
	echo -e "\x0e"

	echo "IPs:"
	echo -e "\x0e"
	IPS=( $(hostname -I) )
	for var in "${IPS[@]}"
	do
		  echo "* ${var}"
        done
	echo -e "\x0f"

	if [ -e /boot/read-version ]
	then
		echo "HW "$(/boot/read-version)
	fi
#	echo "SW version: "$(git rev-parse --short HEAD)
	echo -e "\x0f"
	echo -e "\a"
	echo "Distro:"
	echo -e "\x0e"
	echo "id:      "$(lsb_release  -i -s)
	echo "version: "$(lsb_release  -r -s)
	echo -e "\x0f"
	echo -e "\a"
	echo "Release 5EWC0-2023-v0.2.0a"
}

cd /boot/
chmod 666 /dev/uio* /dev/mem

if [ -e /boot/cat-display ]
then
	(( fn ) 2>&1 | /boot/cat-display)
fi
if [ -e /boot/scpi ]
then
#	/boot/scpi -x
	true
fi
if [ -f /home/student/startup.sh ] ; then
	su student /home/student/startup.sh
fi
