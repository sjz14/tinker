#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : answer_questions.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-05-09
#  Last modified: 2014-05-09, 18:01:25
# Description   : 
# 

echo "answer_questions 的启动脚本，按 Ctrl+C 终止"
echo "提示：按 Alt+数字 切换标签页"

export CUR_DIR=~/catkin_ws/src/furoc_launch/scripts
cd $CUR_DIR

# core
export FU_="bash $CUR_DIR/core.sh"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"
read -p "请在 roscore 启动成功后按回车键..."

# pocketsphinx
export FU_="roslaunch pocketsphinx answer.launch"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"

xdotool key alt+3
read -p "请检查 pocketsphinx 节点启动成功后按回车键..."

# answer_questions
export FU_="rosrun answer_questions answer_node.py"
xdotool key ctrl+shift+t; sleep 2; xdotool type "$FU_"; xdotool key "Return"

