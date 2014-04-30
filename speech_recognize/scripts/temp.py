#!/usr/bin/env python
# created by bss at 2014-03-13
# Last modified: 2014-04-12, 14:45:16

import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import os
ans = False

def callback(data):
    global ans
    if ans==False:
        tmp = data.data
        os.system('espeak --stdout \'The temperature in the room is'+"%.2f"%tmp+'degree centigrade, it is too hot, do I need to turn on the air conditioner\' | aplay')
        print '1'
        ans = True
def callback2(data):
    global ans
    if ans==True:
        cmd = data.data
        s = 'Ok, I will go'
        if cmd=='go':
            os.system('espeak --stdout \''+s+'\' | aplay')
def listener():
    rospy.init_node('inquere', anonymous=True)
    rospy.Subscriber("/building1/room1/sensors/temperature/temp", Float32, callback)
    rospy.Subscriber("/recognizer/output", String, callback2)
    rospy.spin()


if __name__ == '__main__':
    listener()

