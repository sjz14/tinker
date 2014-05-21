#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : select_left_eye.py
# Author        : bss
# Creation date : 2014-05-21
#  Last modified: 2014-05-21, 23:43:04
# Description   : Select the left eye, binocular.
#

import sys
import os
import rospkg
import rospy
from std_msgs.msg import String
import cv2
import yaml

def main(argv):
    rcdir = rospkg.RosPack().get_path('easycap_bridge') + '/'
    paramdir = rcdir + 'param.yml'
    f = open(paramdir, 'r')
    f.readline()
    data = yaml.safe_load(f)
    f.close()

    param = data['Parameters']
    a_is_left = (param['a_is_left'] != 0)
    video_id = 0
    if a_is_left:
        video_id = param['video_a']
    else:
        video_id = param['video_b']
    cap = cv2.VideoCapture(video_id)
    ch = ''

    while (True):
        ret, frame = cap.read()
        cv2.imshow('Is it from left eye? (Y(es)/N(o)/C(ancel))', frame)
        try:
            ch = cv2.waitKey(60) & 0xFF
        except:
            print('error')
            break
        ch = chr(ch)
        if 'Y' == ch or 'y' == ch or 'C' == ch or 'c' == ch:
            break
        elif 'N' == ch or 'n' == ch:
            cap.release()
            a_is_left = not a_is_left
            if a_is_left:
                video_id = param['video_a']
                data['Parameters']['a_is_left'] = 1
            else:
                video_id = param['video_b']
                data['Parameters']['a_is_left'] = 0
            cap = cv2.VideoCapture(video_id)

    cap.release()
    cv2.destroyAllWindows()

    if not ('y' == ch or 'Y' == ch):
        return
    
    f = open(paramdir)
    lines = f.readlines()
    f.close()
    f = open(paramdir, 'wt')
    for line in lines:
        if line.strip().startswith('a_is_left'):
            if a_is_left:
                f.write('    a_is_left: 1\n')
            else:
                f.write('    a_is_left: 0\n')
        else:
            f.write(line)
    f.close()


if __name__ == '__main__':
    main(sys.argv)
