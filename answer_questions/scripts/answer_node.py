#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : answer_node.py
# Author        : bss
# Creation date : 2014-05-09
#  Last modified: 2014-05-10, 22:17:20
# Description   : Answer question listed in resource/
#

import sys
import os
import rospkg
import rospy
from std_msgs.msg import String

ANS = {}

def getQuestionCallback(data):
    ques = str(data.data).strip()
    try:
        ans = ANS[ques.upper()]
    except:
        print("I can't answer your question: " + ques)
        print('Make sure you use the correct launch file in pocketsphinx.')
        sys.exit(1)
    print(ques + '?')
    print('-' + ans)
    if ques.upper() == "WHO ARE YOU":
        os.system("espeak -s 130 -vzh --stdout '我的名字叫 Tinker' | aplay")
        sys.exit(0)
    playSound(ans)

def playSound(answer):
    mp3dir = rospkg.RosPack().get_path('answer_questions') + '/resource/sounds/'
    if os.path.exists(mp3dir + answer + '.mp3'):
        os.system('mplayer "' + mp3dir + answer + '.mp3"')
    else:
        ans_speak = answer.replace("'", '')
        os.system("espeak -s 130 --stdout '" + ans_speak + "' | aplay")

def main(argv):
    rcdir = rospkg.RosPack().get_path('answer_questions') + '/resource/'
    # question
    fp = open(rcdir + 'questions.txt', 'r')
    ques = []
    for line in fp.readlines():
        sentence = line.strip().upper()
        if sentence != '':
            ques.append(str(sentence))
    fp.close()
    # answer
    fp = open(rcdir + 'answers.txt', 'r')
    ans = []
    for line in fp.readlines():
        sentence = line.strip()
        if sentence != '':
            ans.append(str(sentence))
    fp.close()
    
    for i in range(0, min(len(ques), len(ans))):
        ANS[ques[i]] = ans[i]
    print(str(len(ANS)) + ' q&a find.')
    
    # Listen to /recognizer/output from pocketsphinx, task:answer
    rospy.init_node('answer_node', anonymous=True)
    rospy.Subscriber('/recognizer/output', String, getQuestionCallback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
