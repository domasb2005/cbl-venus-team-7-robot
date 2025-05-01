#!/usr/bin/env bash
if [ ! -d bin ] ; then
	echo run this script in the libpynq directory
	exit 1
fi
cd bin || exit 1
if [ ! -e cat-display -o ! -f boot-startup.sh -o ! -f student-startup.sh ] ; then
	echo cannot upgrade: files missing
	exit 1
fi
if [ `grep 'SD-card image release 5EWC0-2023 version 0.2.0' /etc/motd | wc -l` == 1 ] &&
	[ `grep 'TU/e image version 0.2.0' /boot/startup.sh | wc -l` == 1 ] ; then
	if ! cmp -s cat-display /boot/cat-display ; then
		sudo cp cat-display /boot
		sudo chmod +x /boot/cat-display
	fi
	if ! cmp -s boot-startup.sh /boot/startup.sh ; then
		sudo cp boot-startup.sh /boot/startup.sh
	fi
	if [ -f /home/student/startup.sh ] ; then
		echo not overwriting existing /home/student/startup.sh
	else
		cp student-startup.sh /home/student/startup.sh
		# doesn't need to be executable
	fi
	echo upgraded SD card
else
	echo "SD card doesn't require upgrade"
fi

