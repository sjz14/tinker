#!/bin/bash
# -*- coding: utf-8 -*-
# Module        : core.sh
# Author        : bss
# Package       : furoc_launch
# Creation date : 2014-04-11
#  Last modified: 2014-04-11, 17:21:23
# Description   : Start roscore.
# 

# Find IP address, default wlan0
ipaddr=$(ifconfig eth0 | grep inet\ )
ipaddr=${ipaddr#*inet 地址:}
ipaddr=${ipaddr%%\ *}

# Start roscore
ROS_HOSTNAME=$ipaddr
roscore

