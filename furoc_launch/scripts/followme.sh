#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : followme.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-04-30, 13:31:00
# Description   : 
# 

echo "followme 的启动脚本，按 Ctrl+C 终止"

export CUR_DIR=~/catkin_ws/src/furoc_launch/scripts
cd $CUR_DIR
source $CUR_DIR/start.sh

# followme
export FU_="rosrun followme followme_decision"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"

