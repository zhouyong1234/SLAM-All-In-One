#!/bin/bash

# enable kernel module: gs_usb
sudo modprobe gs_usb

# bring up can interface
sudo ip link set can0 up type can bitrate 500000

# install can utils
sudo apt install -y can-utils