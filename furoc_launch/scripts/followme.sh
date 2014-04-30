#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : followme.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-04-11, 18:00:12
# Description   : 
# 

echo "followme 的启动脚本，按 Ctrl+C 终止"
echo "提示：按 Super+W 切换窗口"

source ./start.sh

gnome-terminal -e "./followme_decision.sh"

