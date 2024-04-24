#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
from tf2_msgs.msg import TFMessage
from pyquaternion import Quaternion as pyQuaternion


def broadcast_object(lookup_name, child_frame_id, generic_header):
    """
    Broadcast An Object by looking it up in the TF Tree
    Grabs the transform relative to origin (/world)
    """

    (trans, rot) = listener.lookupTransform('/world', lookup_name, rospy.Time(0))
    transform_stamped = TransformStamped()
    transform = Transform()
    transform.translation = Vector3(trans[0], trans[1], trans[2])
    transform.rotation = Quaternion(rot[0], rot[1], rot[2], rot[3])
    transform_stamped.transform = transform
    transform_stamped.header = generic_header
    transform_stamped.child_frame_id = child_frame_id
    return transform_stamped

def broadcast_object_gen(lookup_name, child_frame_id, frame_id, generic_header):
    """
    Broadcast An Object by looking it up in the TF Tree
    Grabs the transform relative to a give frame_id
    """

    (trans, rot) = listener.lookupTransform(frame_id, lookup_name, rospy.Time(0))
    transform_stamped = TransformStamped()
    transform = Transform()
    transform.translation = Vector3(trans[0], trans[1], trans[2])
    transform.rotation = Quaternion(rot[0], rot[1], rot[2], rot[3])
    transform_stamped.transform = transform
    transform_stamped.header = generic_header
    transform_stamped.child_frame_id = child_frame_id
    return transform_stamped

if __name__ == '__main__':
    rospy.sleep(2)
    rospy.init_node('unity_publisher')
    listener = tf.TransformListener(cache_time=rospy.Duration(0.5))
    broadcaster = tf.TransformBroadcaster()

    tf_pub = rospy.Publisher('/objects', TFMessage, queue_size=1)

    rate = rospy.Rate(30)
    # Broadcast all objects to unity specified here
    while not rospy.is_shutdown():
        tag_msg = TFMessage()

        # Header
        generic_header = Header()
        generic_header.stamp = rospy.Time.now()
        generic_header.frame_id = "world"

        try:
            can_transform = broadcast_object('/corn_can_center', 'corn_can', generic_header)
            tag_msg.transforms.append(can_transform)
        except:
            #rospy.logwarn("No Corn Can Transform")
            pass

        try:
            cube_transform = broadcast_object('/grey_cube_center', 'grey_cube', generic_header)
            tag_msg.transforms.append(cube_transform)

        except Exception as e:
            pass

        try:
            bot_transform = broadcast_object('/bottle_2_center', 'bottle_2', generic_header)
            tag_msg.transforms.append(bot_transform)
        except:
            # rospy.logwarn("No Bottle 2")
            pass

        try:
            white_transform = broadcast_object('/white_cup_center', 'white_cup', generic_header)
            tag_msg.transforms.append(white_transform)
        except:
            pass
        
        try:
            blue_transform = broadcast_object('/blue_cup_center', 'blue_cup', generic_header)
            tag_msg.transforms.append(blue_transform)
        except:
            pass

        try:
            blue_transform = broadcast_object_gen('/blue_cup_handle', 'blue_cup_handle', '/arm_relative', generic_header)
            tag_msg.transforms.append(blue_transform)
        except:
            pass

        try:
            white_transform = broadcast_object_gen('/white_cup_bowl', 'white_cup_bowl', '/arm_relative', generic_header)
            tag_msg.transforms.append(white_transform)
        except:
            pass

        try:
            box_transform = broadcast_object('/box', 'box', generic_header)
            tag_msg.transforms.append(box_transform)
        except:
            pass

        try:
            box_transform = broadcast_object_gen('/box', 'box_robo', '/arm_relative', generic_header)
            tag_msg.transforms.append(box_transform)
        except:
            pass

        try:
            robot_transform = broadcast_object('/robot_center', 'robot', generic_header)
            tag_msg.transforms.append(robot_transform)
        except:
            pass

        try:
            cam_transform = broadcast_object('/camera_green', 'camera_green', generic_header)
            tag_msg.transforms.append(cam_transform)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass
        try:
            cam_transform = broadcast_object('/camera_purple', 'camera_purple', generic_header)
            tag_msg.transforms.append(cam_transform)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass
        try:
            tag_transform = broadcast_object('/calibration_tag', 'table', generic_header)
            tag_msg.transforms.append(tag_transform)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass

        try:
            arm_transform0 = broadcast_object('/arm_origin', 'arm_origin', generic_header)
            tag_msg.transforms.append(arm_transform0)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass

        try:
            arm_transform0 = broadcast_object('/arm_relative', 'arm_relative', generic_header)
            tag_msg.transforms.append(arm_transform0)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass

        try:
            arm_transform1 = broadcast_object('/shoulder_link', 'arm_1', generic_header)
            tag_msg.transforms.append(arm_transform1)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass
        try:
            arm_transform2 = broadcast_object('/elbow_link', 'arm_2', generic_header)
            tag_msg.transforms.append(arm_transform2)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass
        try:
            arm_transform3 = broadcast_object('/forearm_link', 'arm_3', generic_header)
            tag_msg.transforms.append(arm_transform3)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass
        try:
            arm_transform4 = broadcast_object('/wrist_link', 'arm_4', generic_header)
            tag_msg.transforms.append(arm_transform4)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass
        try:
            arm_gripper = broadcast_object('/gripper_link', 'arm_gripper', generic_header)
            tag_msg.transforms.append(arm_gripper)
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass

        #append all arm transforms
        
        tf_pub.publish(tag_msg)

        rate.sleep()

