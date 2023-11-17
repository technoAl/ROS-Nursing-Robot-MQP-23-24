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


def cumulative_distance(prev_trans, trans, trans_tolerance, prev_rot, rot, rot_tolerance):
    q1 = pyQuaternion(axis=[prev_rot.x, prev_rot.y, prev_rot.z], angle=prev_rot.w)
    q2 = pyQuaternion(axis=[rot.x, rot.y, rot.z], angle=rot.w)
    if math.sqrt(math.pow(prev_trans.x - trans.x, 2) + math.pow(prev_trans.y - trans.y, 2) + math.pow(prev_trans.z - trans.z, 2)) > trans_tolerance:
        return True
    elif abs(pyQuaternion.distance(q1, q2)) > rot_tolerance:
        return True
    else:
        return False

def broadcast_object(lookup_name, child_frame_id, generic_header):
    (trans, rot) = listener.lookupTransform('/world', lookup_name, rospy.Time(0))
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

    #adjust_pub = rospy.Publisher('/objects/adjust', TransformStamped, queue_size=1)
    # camera_pub = rospy.Publisher('/objects/camera', TransformStamped, queue_size=1)
    # cube_pub = rospy.Publisher('/objects/grey_cube', TransformStamped, queue_size=1)
    # can_pub = rospy.Publisher('/objects/corn_can', TransformStamped, queue_size=1)
    # bottle_2_pub = rospy.Publisher('/objects/bottle_2', TransformStamped, queue_size=1)

    tf_pub = rospy.Publisher('/objects', TFMessage, queue_size=1)

    base_quat = quaternion_from_euler(0.01, 0.01, 0.01)
    prev_cube_trans = Vector3(0, 0, 0)
    prev_cube_rot = Quaternion(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
    prev_can_trans = Vector3(0, 0, 0)
    prev_can_rot = Quaternion(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
    prev_bottle_trans = Vector3(0, 0, 0)
    prev_bottle_rot = Quaternion(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
    rot_tolerance = 0.0
    trans_tolerance = 0.0

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tag_msg = TFMessage()

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
            can_transform = broadcast_object('/corn_can_center', 'corn_can', generic_header)
            tag_msg.transforms.append(can_transform)
        except:
            #rospy.logwarn("No Corn Can Transform")
            pass

        try:
            cube_transform = broadcast_object('/grey_cube_center', 'grey_cube', generic_header)
            tag_msg.transforms.append(cube_transform)

        except:
            #rospy.logwarn(e)
            pass

        try:
            cube_transform = broadcast_object('/bottle_2_center', 'bottle_2', generic_header)
            tag_msg.transforms.append(cube_transform)

        except:
            # rospy.logwarn("No Bottle 2")
            pass
        
        tf_pub.publish(tag_msg)

        rate.sleep()


