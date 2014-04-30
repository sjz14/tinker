#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : core.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-04-30, 13:17:15
# Description   : Start roscore.
# 

# Find IP address, default wlan0
ipaddr=$(ifconfig eth0 | grep inet\ )
ipaddr=${ipaddr#*inet 地址:}
ipaddr=${ipaddr%%\ *}

# Start roscore
export ROS_HOSTNAME=$ipaddr
roscore

