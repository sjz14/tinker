#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : core.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-05-01, 10:58:57
# Description   : Start roscore.
# 

# Find IP address, default wlan0
ipaddr=$(ifconfig | grep 192 | grep inet\ )
ipaddr=${ipaddr#*inet 地址:}
ipaddr=${ipaddr%%\ *}

# Start roscore
export ROS_HOSTNAME=$ipaddr
roscore

