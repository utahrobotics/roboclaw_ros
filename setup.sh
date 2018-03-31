#!/usr/bin/env bash

# run this to make the make the SparkFun FTDI chip be called /dev/roboclaw
sudo cp 10-local.rules /etc/udev/rules.d
sudo udevadm trigger

echo ""
echo "unplug and replug the device"
echo ""
echo "test with (it should not show an error):"
echo "ls /dev/roboclaw"
echo ""

