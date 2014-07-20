#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : answer_node.py
# Author        : bss
# Creation date : 2014-07-19
#  Last modified: 2014-07-20, 02:11:01
# Description   : Decision node of WhatDidYouSay.
#

import sys
import os
import rospkg
import rospy
from std_msgs.msg import String
from std_srvs.srv import *


class answer_handler:
    def __init__(self):
        self.answer_num = 0
    def answer_once(self, req):
        # stop when answered 3 questions
        self.answer_num += 1
        if self.answer_num >= 3:
            print('answered 3 questions.')
            stop_answer()
        return EmptyResponse()

def stop_answer():
    print('stopping')
    try:
        stop = rospy.ServiceProxy('/answer/stop', Empty)
        stop()
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def playSound(answer):
    mp3dir = rospkg.RosPack().get_path('whatdidyousay') + '/resource/sounds/'
    if os.path.exists(mp3dir + answer + '.mp3'):
        os.system('mplayer "' + mp3dir + answer + '.mp3"')
    else:
        ans_speak = answer.replace("'", '')
        os.system("espeak -s 130 --stdout '" + ans_speak + "' | aplay")

def main(argv):
    pkgdir = rospkg.RosPack().get_path('whatdidyousay')
    
    rospy.init_node('answer_node', anonymous=True)
    rospy.on_shutdown(stop_answer)

    playSound("I can't find you. Please come to me.")

    print('starting')
    rospy.wait_for_service('/answer/start')
    try:
        stop = rospy.ServiceProxy('/answer/start', Empty)
        stop()
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

    ah = answer_handler()
    rospy.Service("/answer/answer_once", Empty, ah.answer_once)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

