#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
from tf.msg import tfMessage
from pyquaternion import Quaternion as pyQuaternion


def cumulative_distance(prev_trans, trans, trans_tolerance, prev_rot, rot, rot_tolerance):
    q1 = pyQuaternion(axis=[prev_rot.x, prev_rot.y, prev_rot.z], angle=prev_rot.w)
    q2 = pyQuaternion(axis=[rot.x, rot.y, rot.z], angle=rot.w)
    if math.sqrt(math.pow(prev_trans.x - trans.x, 2) + math.pow(prev_trans.y - trans.y, 2) + math.pow(prev_trans.z - trans.z, 2)) > trans_tolerance:
        return True
    elif abs(pyQuaternion.distance(q1, q2)) > rot_tolerance:
        return True
    else:
        return False

if __name__ == '__main__':
    rospy.sleep(2)
    rospy.init_node('unity_publisher')
    listener = tf.TransformListener()

    #adjust_pub = rospy.Publisher('/objects/adjust', TransformStamped, queue_size=1)
    # camera_pub = rospy.Publisher('/objects/camera', TransformStamped, queue_size=1)
    # cube_pub = rospy.Publisher('/objects/grey_cube', TransformStamped, queue_size=1)
    # can_pub = rospy.Publisher('/objects/corn_can', TransformStamped, queue_size=1)
    # bottle_2_pub = rospy.Publisher('/objects/bottle_2', TransformStamped, queue_size=1)

    tf_pub = rospy.Publisher('/objects', tfMessage, queue_size=1)

    base_quat = quaternion_from_euler(0.01, 0.01, 0.01)
    prev_cube_trans = Vector3(0, 0, 0)
    prev_cube_rot = Quaternion(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
    prev_can_trans = Vector3(0, 0, 0)
    prev_can_rot = Quaternion(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
    prev_bottle_trans = Vector3(0, 0, 0)
    prev_bottle_rot = Quaternion(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
    rot_tolerance = 0.0
    trans_tolerance = 0.0

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        tag_msg = tfMessage()

        # Header
        generic_header = Header()
        generic_header.stamp = rospy.Time.now()
        generic_header.frame_id = "world"

        #(adjust_trans, adjust_rot) = listener.lookupTransform('/world', '/adjust', rospy.Time(0))

        # try:
        #     (camera_trans, camera_rot) = listener.lookupTransform('/world', '/camera', rospy.Time(0))
        #     transform.translation = Vector3(camera_trans[0], camera_trans[1], camera_trans[2])
        #     transform.rotation = Quaternion(camera_rot[0], camera_rot[1], camera_rot[2], camera_rot[3])
        #     tag_msg.transform = transform
        #     camera_pub.publish(tag_msg)
        # except:
        #     #rospy.logwarn("No Camera Transform")
        #     pass

        try:
            (can_trans, can_rot) = listener.lookupTransform('/world', '/corn_can_center', rospy.Time(0))
            can_transform = TransformStamped()
            transform.translation = Vector3(can_trans[0], can_trans[1], can_trans[2])
            transform.rotation = Quaternion(can_rot[0], can_rot[1], can_rot[2], can_rot[3])
            can_transform.transform = transform
            can_tranform.header = generic_header
            can_transform.child_frame_id = "/corn_can"
            if cumulative_distance(prev_can_trans, transform.translation, trans_tolerance, prev_can_rot, transform.rotation, rot_tolerance) or True:
                #can_pub.publish(tag_msg)
                tag_message.transforms.append(can_transform)
                prev_can_trans = transform.translation
                prev_can_rot = transform.rotation

        except:
            #rospy.logwarn("No Corn Can Transform")
            pass

        try:
            (cube_trans, cube_rot) = listener.lookupTransform('/world', '/grey_cube_center', rospy.Time(0))
            cube_transform = TransformStamped()
            transform.translation = Vector3(cube_trans[0], cube_trans[1], cube_trans[2])
            transform.rotation = Quaternion(cube_rot[0], cube_rot[1], cube_rot[2], cube_rot[3])
            cube_transform.transform = transform
            cube_transform.header = generic_header
            cube_transform.child_frame_id = "grey_cube"
            if cumulative_distance(prev_cube_trans, transform.translation, trans_tolerance, prev_cube_rot, transform.rotation, rot_tolerance) or True:
                #cube_pub.publish(tag_msg)
                tag_message.transforms.append(cube_transform)
                prev_cube_trans = transform.translation
                prev_cube_rot = transform.rotation

        except:
            #rospy.logwarn(error)
            pass

        try:
            (bot_2_trans, bot_2_rot) = listener.lookupTransform('/world', '/bottle_2_center', rospy.Time(0))
            bot_transform = TransformStamped()
            transform.translation = Vector3(bot_2_trans[0], bot_2_trans[1], bot_2_trans[2])
            transform.rotation = Quaternion(bot_2_rot[0], bot_2_rot[1], bot_2_rot[2], bot_2_rot[3])
            bot_transform.transform = transform
            bot_transform.header = generic_header
            bot_transform.child_frame_id = "bottle_2"
            if cumulative_distance(prev_bottle_trans, transform.translation, trans_tolerance, prev_bottle_rot, transform.rotation, rot_tolerance) or True:
                #bottle_2_pub.publish(tag_msg)
                tag_message.transforms.append(bot_transform)
                prev_bottle_trans = transform.translation
                prev_bottle_rot = transform.rotation
        except:
            # rospy.logwarn("No Bottle 2")
            pass

        rate.sleep()

