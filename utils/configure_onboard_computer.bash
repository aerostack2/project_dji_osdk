#!/bin/bash
sudo chmod 777 /dev/ttyTHS0
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyUSB0

sudo bash -c "echo 'SUBSYSTEM==\"usb\", ATTRS{id_vendor}==\"2ca3\", MODE=\"0666\"' > /etc/udev/rules.d/DJIDevice.rules"



