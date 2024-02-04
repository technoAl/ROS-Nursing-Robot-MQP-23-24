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

def broadcast_object(object_name):
    green_correct = False
    trans1, rot1, trans2, rot2 = None, None, None, None
    try:
        (trans1, rot1) = listener.lookupTransform('/world', '/' + object_name + '_green', rospy.Time(0))
        green_correct = True
    except Exception as e:
        # rospy.logwarn(e)
        # rospy.loginfo("GRENN HERE")
        pass

    purple_correct = False
    try:
        (trans2, rot2) = listener.lookupTransform('/world', '/' + object_name + '_purple', rospy.Time(0))
        purple_correct = True
    except Exception as e:
        # rospy.logwarn(e)
        # rospy.loginfo("PURPLE HERE")
        pass

    trans, rot = None, None


    if green_correct and purple_correct:
        trans, rot = average_positions(trans1, rot1, trans2, rot2)
    elif green_correct:
        trans, rot = trans1, rot1
    elif purple_correct:
        trans, rot = trans2, rot2
    else:
        return

    transform = Transform()

    transform.translation = Vector3(trans[0], trans[1], trans[2])
    transform.rotation = Quaternion(rot[0], rot[1], rot[2], rot[3])

    broadcaster.sendTransform(
        (transform.translation.x, transform.translation.y, transform.translation.z), (
            transform.rotation.x, transform.rotation.y, transform.rotation.z,
            transform.rotation.w), rospy.Time.now(), object_name, "/world")


if __name__ == '__main__':
    rospy.sleep(2)
    rospy.init_node('intermediate_publisher')
    listener = tf.TransformListener(cache_time=rospy.Duration(1.00))
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(5)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        broadcast_object('corn_can')
        broadcast_object('grey_cube')
        broadcast_object('bottle_2')
        broadcast_object('white_cup')
        broadcast_object('blue_cup')
        broadcast_object('robot')
        rate.sleep()

