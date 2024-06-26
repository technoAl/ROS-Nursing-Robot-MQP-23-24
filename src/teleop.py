#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
from std_msgs.msg import Header
import termios
import tty
import sys
from tf.transformations import quaternion_from_euler, quaternion_from_matrix

from select import select

""" 
Based on the TeleOp Turtlebot Script

Utilized to move adjust frames around to perfectly align digital twins
There are three frames that can be adjusted
- adjust
- adjust objects
- robot adjust
"""
def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

if __name__ == '__main__':
    settings = saveTerminalSettings()
    rospy.init_node('teleop')
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    x = 0
    y = 0
    z = 0
    table_z = 0
    rx = 0
    ry = 0
    rz = 0
    while not rospy.is_shutdown():
        key = getKey(settings, 0.01)
        if key == 'w':
            x += 0.001
        elif key == 's':
            x -= 0.001
        elif key == 'a':
            y += 0.001
        elif key == 'd':
            y -= 0.001
        elif key == 'q':
            z += 0.001
        elif key == 'e':
            z -= 0.001
        elif key == 'i':
            rx += 0.001
        elif key == 'k':
            rx -= 0.001
        elif key == 'j':
            ry += 0.001
        elif key == 'l':
            ry -= 0.001
        elif key == 'u':
            rz += 0.001
        elif key == 'o':
            rz -= 0.001
        elif key == 'r':
            table_z += 0.001
        elif key == 'f':
            table_z -= 0.001
        elif key == ' ':
            rospy.signal_shutdown("Interrupt")

        br.sendTransform(
            (0, 0, table_z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(), "adjust", "world")
        br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(), "adjust_objects", "calibration_tag")
        br.sendTransform(
            (rx, ry, rz), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(), "robot_adjust", "robot")


