#!/usr/bin/env python3

import rospy
import time
import math
from std_msgs.msg import Header, String
import numpy as np
import cv2
from apriltag import apriltag
from PIL import Image as im
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
import time
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
import tf
from pyquaternion import Quaternion as pyQuaternion
from std_msgs.msg import Float32
import cv2
import threading
import statistics


def object_points(tag_size):
    return [[-tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, -tag_size / 2, 0.0],
            [-tag_size / 2, -tag_size / 2, 0.0]]


class Image_Processing:
    def __init__(self):

        fx_green = 2102.0722
        fy_green = 2109.2954
        cx_green = 1829.1608
        cy_green = 1058.9215

        fx_purple = 1169.9125
        fy_purple = 1168.2444
        cx_purple = 1284.8136
        cy_purple = 1015.6015

        self.cam_green_intrinsics = np.array([[fx_green, 0, cx_green],
                                         [0, fy_green, cy_green],
                                         [0, 0, 1]])
        self.cam_purple_intrinsics = np.array([[fx_purple, 0, cx_purple],
                                         [0, fy_purple, cy_purple],
                                         [0, 0, 1]])
        self.br = tf.TransformBroadcaster()
        self.cam1_rot = np.zeros((4, 4))
        self.cam2_rot = np.zeros((4, 4))

    def pipeline(self, image, camera):
        if camera == 'green':
            intrinsics_mat = self.cam_green_intrinsics
            image = cv2.undistort(image, intrinsics_mat, np.array([-0.349, 0.0844, 0, 0, 0]), intrinsics_mat)
        else:
            intrinsics_mat = self.cam_purple_intrinsics


        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detector = apriltag(family="tag36h11")
        detections = detector.detect(gray_image)  # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        # rospy.loginfo("Detector Time " + str(time2 - time1))

        found_objects = {}

        tag_size = 0
        if len(detections) > 0:
            for tag in detections:

                ID = tag['id']
                object_name = ""
                if ID == 33:
                    object_name = "tag"
                    tag_size = 0.110
                elif ID == 0:
                    # rospy.loginfo("Grey Cube")
                    object_name = "grey_cube"
                    tag_size = 0.025
                elif ID == 1:
                    # rospy.loginfo("Corn Can")
                    object_name = "corn_can"
                    tag_size = 0.025
                elif ID == 2:
                    # rospy.loginfo("Bottle 2")
                    object_name = "bottle_2"
                    tag_size = 0.025
                elif ID == 10:
                    # rospy.loginfo("White Cup")
                    object_name = "white_cup"
                    tag_size = 0.04
                elif ID == 12:
                    # rospy.loginfo("Blue Cup")
                    object_name = "blue_cup"
                    tag_size = 0.04
                elif ID == 13:
                    object_name = "robot"
                    tag_size = 0.04
                else:
                    continue

                obj_pts = np.array(object_points(tag_size))

                center = tag['center']
                lb_rb_rt_lt = tag['lb-rb-rt-lt']
                lt_rt_rb_lb = np.zeros((4, 2))
                for i in range(4):
                    lt_rt_rb_lb[i] = lb_rb_rt_lt[3 - i]

                good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (),
                                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if good:
                    rot_matrix, _ = cv2.Rodrigues(prvecs)
                    mat = np.zeros((4, 4), np.float32)
                    for i in range(3):
                        for j in range(3):
                            mat[i][j] = rot_matrix[i][j]

                    mat[3, 3] = 1

                    mat[0, 3] = ptvecs[0][0]
                    mat[1, 3] = ptvecs[1][0]
                    mat[2, 3] = ptvecs[2][0]

                    # handle pos

                    if ID == 33:
                        mat = np.linalg.inv(mat)

                    orientation = quaternion_from_matrix(mat)

                    translation = [mat[0, 3], mat[1, 3], mat[2, 3]]

                    found_objects[object_name] = (translation, orientation)

        return found_objects


class Pipeline:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('pipeline')

        self.br = tf.TransformBroadcaster()

        self.pipeline_rate = 0

        self.current_image = 0
        self.intrinsics = 0
        self.plot_publisher = rospy.Publisher('/plot/value', Float32, queue_size=10)

        ### Making robot go 10Hz
        self.rate = rospy.Rate(30)
        self.count = 0

        self.br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adjust",
                              "world")

        self.publish_ready = rospy.Publisher('/status', String, queue_size=10)

        self.t1 = [0.0, 0.0, 0.0]
        self.q1 = [0.0, 0.0, 0.0, 0.0]

        self.t2 = [0.0, 0.0, 0.0]
        self.q2 = [0.0, 0.0, 0.0, 0.0]

        self.listener = tf.TransformListener()

        self.sample1_done = False
        self.sample2_done = False
        self.image_count = 0

        self.processor = Image_Processing()

        self.median_filter = [TransformStamped(), TransformStamped(), TransformStamped()]
        self.median_count = 0
        self.still_count = 5

        self.br = tf.TransformBroadcaster()
        self.cam1_rot = np.zeros((4, 4))
        self.cam2_rot = np.zeros((4, 4))

        self.ready = ''

        rospy.Subscriber('/status', String, self.callback)
        self.listener = tf.TransformListener()
        rospy.sleep(2)
        self.br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adjust",
                              "world")
        self.object_name_list = ["grey_cube", "corn_can", "bottle_2", "white_cup", "blue_cup", "robot"]
        rospy.sleep(1)


    def callback(self, data):
        self.ready = data.data

    def calibration(self, image, cam_name):

        while not (self.sample1_done and self.sample2_done):
            found_tag = self.processor.pipeline(image, cam_name)

            cam_translation = found_tag['tag'][0]
            cam_rotation = found_tag['tag'][1]

            if(cam_name == "green"):
                self.t1[0] = cam_translation[0]
                self.t1[1] = cam_translation[1]
                self.t1[2] = cam_translation[2]

                self.q1[0] = cam_rotation[0]
                self.q1[1] = cam_rotation[1]
                self.q1[2] = cam_rotation[2]
                self.q1[3] = cam_rotation[3]

                self.sample1_done = True
            else:
                self.t2[0] = cam_translation[0]
                self.t2[1] = cam_translation[1]
                self.t2[2] = cam_translation[2]

                self.q2[0] = cam_rotation[0]
                self.q2[1] = cam_rotation[1]
                self.q2[2] = cam_rotation[2]
                self.q2[3] = cam_rotation[3]
                self.sample2_done = True

            
        transform = Transform()
        transform.translation = Vector3(self.t1[0], self.t1[1], self.t1[2])
        transform.rotation = Quaternion(self.q1[0], self.q1[1], self.q1[2], self.q1[3])

        self.br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z), (
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                            rospy.Time.now(), "camera_green", "adjust_objects")
            
        transform = Transform()
        transform.translation = Vector3(self.t2[0], self.t2[1], self.t2[2])
        transform.rotation = Quaternion(self.q2[0], self.q2[1], self.q2[2], self.q2[3])

        self.br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z), (
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                                rospy.Time.now(), "camera_purple", "adjust_objects")

    def update_current_image(self, camera, cam_name):
            
            while not rospy.is_shutdown():
                # Show images
                ret, image = camera.read()

                if ret:
                    # cv2.imshow(cam_name, image)
                    # cv2.waitKey(1)
                    self.calibration(image, cam_name)
                    self.publish(image, cam_name)

                self.rate.sleep()
    
    def publish(self, image, cam_name):

        self.pipeline_rate += 1
        tags = self.processor.pipeline(image, cam_name)
        for object_name in self.object_name_list:
            if object_name in tags.keys():

                transform = Transform()
                translation = tags[object_name][0]
                quaternion = tags[object_name][1]

                transform.translation = Vector3(translation[0], translation[1], translation[2])
                transform.rotation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                self.br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z), (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w), rospy.Time.now(), object_name + "_" +  cam_name, "camera_" + cam_name)

    def current_milli_time(self):
        return round(time.time() * 1000)

    def run(self):
        self.rate = rospy.Rate(30)
        # # Start streaming
        camera_green = cv2.VideoCapture(2)
        camera_purple = cv2.VideoCapture(4)

        camera_green.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        camera_green.set(cv2.CAP_PROP_FPS, 30)
        camera_green.set(3, 3840)
        camera_green.set(4, 2160)

        camera_purple.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        camera_purple.set(cv2.CAP_PROP_FPS, 30)
        camera_purple.set(3, 2500)
        camera_purple.set(4, 1900)

        green_cam = threading.Thread(target=self.update_current_image, args=[camera_green, "green"])
        
        purple_cam = threading.Thread(target=self.update_current_image, args=[camera_purple, "purple"])


        green_cam.start()
        purple_cam.start()

        while not rospy.is_shutdown():
            pass

        green_cam.join()
        purple_cam.join()
        
        rospy.spin()

if __name__ == '__main__':
    Pipeline().run()
