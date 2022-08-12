#!/bin/bash
### BEGIN INIT INFO
# Provides:          bbzhh.com
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: tomcat service
# Description:       tomcat service daemon
### END INIT INFO

sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

exit 0
