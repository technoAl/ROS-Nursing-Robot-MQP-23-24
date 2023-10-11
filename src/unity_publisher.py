#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.sleep(5)
    rospy.init_node('unity_publisher')
    listener = tf.TransformListener()

    #adjust_pub = rospy.Publisher('/objects/adjust', TransformStamped, queue_size=1)
    camera_pub = rospy.Publisher('/objects/camera', TransformStamped, queue_size=1)
    cube_pub = rospy.Publisher('/objects/grey_cube', TransformStamped, queue_size=1)
    can_pub = rospy.Publisher('/objects/corn_can', TransformStamped, queue_size=1)

    rate = rospy.Rate(2.5)
    while not rospy.is_shutdown():

        tag_msg = TransformStamped()

        # Header
        generic_header = Header()
        generic_header.stamp = rospy.Time.now()
        generic_header.frame_id = "world"
        tag_msg.header = generic_header

        #(adjust_trans, adjust_rot) = listener.lookupTransform('/world', '/adjust', rospy.Time(0))

        try:
            (camera_trans, camera_rot) = listener.lookupTransform('/world', '/camera', rospy.Time(0))
            transform.translation = Vector3(camera_trans[0], camera_trans[1], camera_trans[2])
            transform.rotation = Quaternion(camera_rot[0], camera_rot[1], camera_rot[2], camera_rot[3])
            tag_msg.transform = transform
            camera_pub.publish(tag_msg)
        except:
            #rospy.logwarn("No Camera Transform")
            pass

        try:
            (can_trans, can_rot) = listener.lookupTransform('/world', '/corn_can_center', rospy.Time(0))
            transform.translation = Vector3(can_trans[0], can_trans[1], can_trans[2])
            transform.rotation = Quaternion(can_rot[0], can_rot[1], can_rot[2], can_rot[3])
            tag_msg.transform = transform
            can_pub.publish(tag_msg)
        except:
            #rospy.logwarn("No Corn Can Transform")
            pass

        try:
            (cube_trans, cube_rot) = listener.lookupTransform('/world', '/calibration_box_center', rospy.Time(0))
            transform = Transform()
            transform.translation = Vector3(cube_trans[0], cube_trans[1], cube_trans[2])
            transform.rotation = Quaternion(cube_rot[0], cube_rot[1], cube_rot[2], cube_rot[3])
            tag_msg.transform = transform
            cube_pub.publish(tag_msg)
        except:
            #rospy.logwarn("No Grey Cube")
            pass


