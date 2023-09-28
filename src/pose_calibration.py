#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
import rospy
from std_msgs.msg import Header
import tf

class Pose_Calibration:
    def __init__(self):
        """
        Class constructor
        """

        ### Initialize node, name it 'lab2'
        rospy.init_node('Pose_Calibration')

        self.trans_pub = rospy.Publisher('/calibration/blue_lighthouse', TransformStamped, queue_size=1)
        rospy.sleep(1)

    def publish_message(self):
        trans_msg = TransformStamped()
        # Header
        generic_header = Header()
        generic_header.stamp = rospy.Time.now()
        generic_header.frame_id = "map"
        trans_msg.header = generic_header
        trans_msg.child_frame_id = "calib_trans"

        # handle rotation
        transform = Transform()
        transform.translation = Vector3(0.2, 0.0, 0.0)
        #transform.translation = Vector3(0.71, 0.0, 0.0)
        transform.rotation = Quaternion(0,0, 0,
                                        0)
        trans_msg.transform = transform
        br = tf.TransformBroadcaster()
        br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z), (
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                         rospy.Time.now(), "blue_lighthouse", "camera")
        self.trans_pub.publish(trans_msg)

    def run(self):
        self.publish_message()

if __name__ == '__main__':
    Pose_Calibration().run()
