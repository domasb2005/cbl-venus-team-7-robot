echo SCPI update script
if (( $EUID != 0 )); then
    echo "Please run as root"
    exit
fi

make -C /home/student/libpynq/applications/scpi-bridge/
rm /boot/scpi
rm /boot/version.txt
pkill scpi
cp /home/student/libpynq/applications/scpi-bridge/main /boot/scpi
cp /home/student/libpynq/applications/scpi-bridge/version.txt /boot
echo Starting up SCPI app
/boot/scpi