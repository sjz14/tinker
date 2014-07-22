#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : answer_node.py
# Author        : bss
# Creation date : 2014-07-19
#  Last modified: 2014-07-22, 06:44:33
# Description   : Decision node of WhatDidYouSay.
#

import sys
import os
import rospkg
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_srvs.srv import *


fin_pub = rospy.Publisher('/answer/finished', Int32)

class answer_handler:
    def __init__(self):
        self.answer_num = 0
    def answer_once(self, req):
        # stop when answered 3 questions
        self.answer_num += 1
        if self.answer_num >= 3:
            time.sleep(14)
            print('answered 3 questions.')
            stop_answer()
            fin_pub.publish(1)
        return EmptyResponse()

def initCb(data):
    print('/answer/init is ' + str(data.data))
    if data.data == 1:
        # the robot don't have the ability to find people.
        playSound("I can't find you. Please come to me.")
        # start answer question
        start_answer()

def stop_answer():
    print('stopping')
    try:
        stop = rospy.ServiceProxy('/answer/stop', Empty)
        stop()
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def start_answer():
    print('starting')
    rospy.wait_for_service('/answer/start')
    try:
        start = rospy.ServiceProxy('/answer/start', Empty)
        start()
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

    ah = answer_handler()
    # from answer_questions
    rospy.Service("/answer/answer_once", Empty, ah.answer_once)
    # wait until robot is in position
    rospy.Subscriber('/answer/init', Int32, initCb)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

