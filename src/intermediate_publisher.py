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
from splines.quaternion import Quaternion as SQuaternion
import numpy as np

def lerp(one, two, t):
    """Linear intERPolation."""
    one = np.asarray(one)
    two = np.asarray(two)
    return (1 - t) * one + t * two

def nlerp(one, two, t):
    """Normalized Linear intERPolation.

    Linear interpolation in 4D quaternion space,
    normalizing the result.

    """
    if not np.isscalar(t):
        # If t is a list, return a list of unit quaternions
        return [nlerp(one, two, t) for t in t]
    *vector, scalar = lerp(one.xyzw, two.xyzw, t)
    return SQuaternion(scalar, vector).normalized()

def average_positions(t1, r1, t2, r2):
    transformation = [0.0, 0.0, 0.0]
    rotation = [0.0, 0.0, 0.0, 0.0]

    # t1 = [t1.x, t1.y, t1.z]
    # r1 = [r1.x, r1.y, r1.z, r1.w]
    #
    # t2 = [t2.x, t2.y, t2.z]
    # r2 = [r2.x, r2.y, r2.z, r2.w]

    nr2 = [-r2[0], -r2[1], -r2[2], -r2[3]]
    q1 = SQuaternion(r1[3], r1[0:3])
    q2 = SQuaternion(r2[3], r2[0:3])
    nq2 = SQuaternion(nr2[3], nr2[0:3])

    qint = nlerp(q1, q2, 0.5)
    nqint = nlerp(q1, nq2, 0.5)

    rotation = [qint.vector[0], qint.vector[1], qint.vector[2], qint.scalar]
    if (q1.dot(qint) < q1.dot(nqint)):
        rotation = [nqint.vector[0], nqint.vector[1], nqint.vector[2], nqint.scalar]
    for i in range(len(t1)):
        transformation[i] = (t1[i] + t2[i]) / 2

    # x = (r1[0] * r1[3] + r2[0] * r2[3]) / 2
    # y = (r1[1] * r1[3] + r2[1] * r2[3]) / 2
    # z = (r1[2] * r1[3] + r2[2] * r2[3]) / 2
    # w = pow((pow(x, 2) + pow(y, 2) + pow(z, 2)), 0.5)
    #
    # new_x = x / w
    # new_y = y / w
    # new_z = z / w
    #
    # d = pow((pow(new_x, 2) + pow(new_y, 2) + pow(new_z, 2) + pow(w, 2)), 0.5)
    # q = (new_x / d, new_y / d, new_z / d, w / d)


    # for i in range(len(rotation)):
    #     rotation[i] = q[i]
    return transformation, rotation


if __name__ == '__main__':
    rospy.sleep(2)
    rospy.init_node('intermediate_publisher')
    listener = tf.TransformListener(cache_time=rospy.Duration(0.5))
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(5)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        tag_msg = TFMessage()

        # Header
        generic_header = Header()
        generic_header.stamp = rospy.Time.now()
        generic_header.frame_id = "world"

        # (adjust_trans, adjust_rot) = listener.lookupTransform('/world', '/adjust', rospy.Time(0))

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
            (trans1, rot1) = listener.lookupTransform('/world', '/corn_can_green', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/world', '/corn_can_purple', rospy.Time(0))

            corn_trans, corn_rotation = average_positions(trans1, rot1, trans2, rot2)

            corn_transform = Transform()

            corn_transform.translation = Vector3(corn_trans[0], corn_trans[1], corn_trans[2])
            corn_transform.rotation = Quaternion(corn_rotation[0], corn_rotation[1], corn_rotation[2], corn_rotation[3])

            broadcaster.sendTransform(
                (corn_transform.translation.x, corn_transform.translation.y, corn_transform.translation.z), (
                corn_transform.rotation.x, corn_transform.rotation.y, corn_transform.rotation.z,
                corn_transform.rotation.w), rospy.Time.now(), "/corn_can", "/world")
        except:
            # rospy.logwarn("No Corn Can Transform")
            pass

        try:
            (trans1, rot1) = listener.lookupTransform('/world', '/grey_cube_green', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/world', '/grey_cube_purple', rospy.Time(0))
            # rospy.loginfo(
            #     "Cam 1 rotation: X: " + str(rot1[0]) + " Y: " + str(rot1[1]) + " Z: " + str(
            #         rot1[2]) + " w: " + str(rot1[3]))
            #
            # rospy.loginfo(
            #     "Cam 2 rotation: X: " + str(rot2[0]) + " Y: " + str(rot2[1]) + " Z: " + str(
            #         rot2[2]) + " w: " + str(rot2[3]))

            # rospy.loginfo('KMS1')
            grey_trans, grey_rotation = average_positions(trans1, rot1, trans2, rot2)
            # rospy.loginfo('KMS2')
            grey_transform = Transform()

            grey_transform.translation = Vector3(grey_trans[0], grey_trans[1], grey_trans[2])
            grey_transform.rotation = Quaternion(grey_rotation[0], grey_rotation[1], grey_rotation[2], grey_rotation[3])

            broadcaster.sendTransform(
                (grey_transform.translation.x, grey_transform.translation.y, grey_transform.translation.z), (
                    grey_transform.rotation.x, grey_transform.rotation.y, grey_transform.rotation.z,
                    grey_transform.rotation.w), rospy.Time.now(), "grey_cube", "world")
            # rospy.loginfo('KMS3')

        except Exception as e:
            pass

        try:
            (trans1, rot1) = listener.lookupTransform('/world', '/bottle_2_green', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/world', '/bottle_2_purple', rospy.Time(0))

            bottle_trans, bottle_rot = average_positions(trans1, rot1, trans2, rot2)

            bottle_transform = Transform()

            bottle_transform.translation = Vector3(bottle_trans[0], bottle_trans[1], bottle_trans[2])
            bottle_transform.rotation = Quaternion(bottle_rot[0], bottle_rot[1], bottle_rot[2], bottle_rot[3])

            broadcaster.sendTransform(
                (bottle_transform.translation.x, bottle_transform.translation.y, bottle_transform.translation.z), (
                    bottle_transform.rotation.x, bottle_transform.rotation.y, bottle_transform.rotation.z,
                    bottle_transform.rotation.w), rospy.Time.now(), "/bottle_2", "/world")
        except:
            # rospy.logwarn("No Bottle 2")
            pass

        rate.sleep()

