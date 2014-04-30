#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : start.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-04-19, 14:08:41
# Description   : 
# 

gnome-terminal -e "source ~/.bashrc; echo $ROS_WORKSPACE"
gnome-terminal -e "$ROS_WORKSPACE/src/furoc_launch/scripts/core.sh"
read -p "请在 roscore 启动成功后按回车键..."

gnome-terminal -e "./openni.sh"

gnome-terminal -e "./speech.sh"

# usb, pl2303
line=$(dmesg | grep pl2303 | grep ttyUSB)
usbname=${line##*attached to }
usbname=${usbname% *}
# chmod
echo 修改 $usbname 的权限为777
sudo chmod 777 /dev/$usbname

# serial node
gnome-terminal -e "./serial.sh"
read -p "请在 serial 节点启动成功后按回车键..."

# parser
gnome-terminal -e "./parser.sh"
gnome-terminal -e "./parser_decision.sh"

read -p "请在 openni 节点启动成功后按回车键..."

