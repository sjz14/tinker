#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : say.py
# Author        : bss
# Creation date : 2014-07-21
#  Last modified: 2014-07-21, 21:17:19
# Description   : 
#

import sys
import os
import time
import getopt
import rospkg
import rospy
from std_msgs.msg import String

def getSpeechCallback(data):
    print(data.data)
    playSound(data.data)

def Usage():
    print('say.py usage:')
    print('speak')


def playSound(sentense):
    os.system("espeak -s 130 --stdout '" + sentense + "' | aplay")

def main(argv):
    rcdir = rospkg.RosPack().get_path('say') + '/resource/'
    # Listen to /say
    rospy.init_node('say_node', anonymous=True)
    rospy.Subscriber('/say', String, getSpeechCallback)
    print('say ok.')
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

