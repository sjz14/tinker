#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : start.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-04-30, 13:32:09
# Description   : 
# 

echo "提示：按 Alt+数字 切换标签页"

export CUR_DIR=~/catkin_ws/src/furoc_launch/scripts
cd $CUR_DIR

# core
export FU_="bash $CUR_DIR/core.sh"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"
read -p "请在 roscore 启动成功后按回车键..."

# openni
export FU_="rosrun openni2_camera openni2_camera_node"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"

# pocketsphinx
export FU_="roslaunch pocketsphinx furoc.launch"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"

# usb, pl2303
line=$(dmesg | grep pl2303 | grep ttyUSB)
usbname=${line##*attached to }
usbname=${usbname% *}
# chmod
echo 修改 $usbname 的权限为777
xdotool key alt+1
sudo chmod 777 /dev/$usbname

# serial node
export FU_="rosrun serial serial_node"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"
read -p "请在 serial 节点启动成功后按回车键..."

# parser
export FU_="rosrun parser parser_node"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"
export FU_="rosrun parser_decision parser_decision_node"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"

xdotool key alt+3
read -p "请检查 openni 节点启动成功后按回车键..."

xdotool key alt+4
read -p "请检查 pocketsphinx 节点启动成功后按回车键..."

