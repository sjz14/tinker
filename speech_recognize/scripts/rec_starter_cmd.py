#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : rec_starter_cmd.py
# Project       : speech_recognize
# Author        : bss
# created by bss at 2014-07-16
#  Last modified: 2014-07-16, 02:02:04

import sys
import os
import getopt
import roslib
import rospy
from std_msgs.msg import String
from frmsg.msg import starter_state


pub = rospy.Publisher('/starter/cmd', starter_state)
specified = False
task = ""
state = starter_state()


def Usage():
    print('rec_starter_cmd.py usage:')
    print('用语音识别启动机器人')
    print('-h,--help: print help message.')
    print('-t,--task: only work for specified task.')
    print('')
    print('example:')
    print('rosrun speech_recognize rec_starter_cmd.py')
    print('rosrun speech_recognize rec_starter_cmd.py -t "follow me"')


def publish_state(state):
    rate = rospy.Rate(100)
    print('pub')
    for i in range(0, 5):
        pub.publish(state)
        rate.sleep()


def callback(data):
    cmd = data.data
    cmd = cmd.lower().strip()
    if specified:
        if cmd != task:
            return

    if cmd == "tinker please follow me":
        print(cmd)
        state.state = starter_state.FOLLOWME
        print(state)
        publish_state(state)


def listener():
    rospy.init_node('rec_starter_cmd', anonymous=True)
    rospy.Subscriber("/recognizer/cmd_starter", String, callback)
    rospy.spin()


def main(argv):
    try:
        opts, argv = getopt.getopt(argv[1:], 'ht:', ['help', 'task='])
    except getopt.GetoptError, err:
        print(str(err))
        Usage()
        sys.exit(2)
    except:
        Usage()
        sys.exit(1)

    for o, a in opts:
        if o in ('-h', '--help'):
            Usage()
            sys.exit(0)
        if o in ('-t', '--task'):
            specified = True
            task = a.lower().strip()
    listener()


if __name__ == '__main__':
    main(sys.argv)

