#!/usr/bin/env python3

import rospy
import time
import math
from std_msgs.msg import Header
import numpy as np
import cv2
from apriltag import apriltag
from PIL import Image as im
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Vector3, Transform
import pyrealsense2 as rs
import time
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
import tf
from pyquaternion import Quaternion as pyQuaternion
from std_msgs.msg import Float32
import statistics



def object_points(tag_size):
    return [[-tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, -tag_size / 2, 0.0],
            [-tag_size / 2, -tag_size / 2, 0.0]]

class previousReadings:
    def __init__(self):
        self.initialized = False
        self.previousQuat = [0, 0, 0, 0]
        self.previousFour = [0, 0, 0, 0]
        self.prevCounter = 0
        self.inMotion = False
        self.prevTrans = 0
        self.threshold = 0.03

        # use for normal distribution
        # self.rolling_values = np.zeros(20)
        # self.rolling_index = 0

        # use for entropy
        self.rolling_values = []
    def filter_readings(self, orientation):
        self.rolling_values.append(orientation)

        if len(self.rolling_values) > 16:
            self.rolling_values.pop(0)

        sum_x, sum_y, sum_z = 0, 0, 0
        diff = 0
        avgs = []
        for i in range(len(self.rolling_values)): # Calc distance amongst every quaternion to see if we are moving as well as average out quaternions for filter

            sum = 0

            for j in range(len(self.rolling_values)):
                q = pyQuaternion(axis=[self.rolling_values[i][0], self.rolling_values[i][1], self.rolling_values[i][2]], angle=self.rolling_values[i][3])
                r = pyQuaternion(axis=[self.rolling_values[j][0], self.rolling_values[j][1], self.rolling_values[j][2]], angle=self.rolling_values[j][3])
                sum = sum + pyQuaternion.distance(q, r)

            avg = round(sum / len(self.rolling_values), 5)
            avgs.append(avg)
            diff = diff + avg

        avg_diff = diff / len(self.rolling_values) # Average difference among quaternions

        sorted_avgs = sorted(avgs)
        if len(avgs) < 7:
            return orientation
        else:
            if (avg_diff > 0.035): # If moving use median filter
                #rospy.loginfo("Moving")
                #self.still_count = 0
                return orientation
            else: # If we are still, average out the quaternions
                #rospy.loginfo("Still")
                #rospy.loginfo(avg_diff)
                for k in range(len(sorted_avgs) - 6):
                    index = avgs.index(sorted_avgs[k])
                    x = self.rolling_values[index][0]
                    y = self.rolling_values[index][1]
                    z = self.rolling_values[index][2]
                    w = self.rolling_values[index][3]
                    sum_x = sum_x + w * x
                    sum_y = sum_y + w * y
                    sum_z = sum_z + w * z

                avg_x = sum_x / (len(self.rolling_values) - 6)
                avg_y = sum_y / (len(self.rolling_values) - 6)
                avg_z = sum_z / (len(self.rolling_values) - 6)

                new_w = pow((pow(avg_x , 2) + pow(avg_y, 2) + pow(avg_z, 2)) , 0.5)

                new_x = avg_x / new_w
                new_y = avg_y / new_w
                new_z = avg_z / new_w

                d = pow((pow(new_x , 2) + pow(new_y, 2) + pow(new_z, 2) + pow(new_w, 2)) , 0.5)

                return (new_x, new_y, new_z, new_w)/d
class Pipeline:

    def __init__(self):
        """
        Class constructor
        """

        ### Initialize node, name it 'lab2'
        rospy.init_node('pipeline')

        self.cam1serial = '936322072225'
        self.cam2serial = '825412070317'
        self.br = tf.TransformBroadcaster()

        self.pipeline_rate = 0

        self.current_image = 0
        self.intrinsics = 0
        self.plot_publisher = rospy.Publisher('/plot/value', Float32, queue_size=10)
        # camera setup through pyrealsense2
        self.rspipeline1 = rs.pipeline()
        self.rspipeline2 = rs.pipeline()

        self.rsconfig1 = rs.config()
        self.rsconfig2 = rs.config()

        self.rspipe_wrapper1 = rs.pipeline_wrapper(self.rspipeline1)
        self.pipeline_profile1 = self.rsconfig1.resolve(self.rspipe_wrapper1)
        self.device1 = self.pipeline_profile1.get_device()
        self.device_product_line1 = str(self.device1.get_info(rs.camera_info.product_line))

        self.rspipe_wrapper2 = rs.pipeline_wrapper(self.rspipeline2)
        self.pipeline_profile2 = self.rsconfig2.resolve(self.rspipe_wrapper2)
        self.device2 = self.pipeline_profile2.get_device()
        self.device_product_line2 = str(self.device2.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device1.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            rospy.loginfo("The demo requires Depth camera with Color sensor")
            exit(0)
        for s in self.device2.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            rospy.loginfo("The demo requires Depth camera with Color sensor")
            exit(0)
        self.rsconfig1.enable_device(self.cam1serial)
        self.rsconfig1.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.rsconfig1.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.rsconfig2.enable_device(self.cam2serial)
        self.rsconfig2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.rsconfig2.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.profile1 = self.rsconfig1.resolve(self.rspipeline1)
        self.profile2 = self.rsconfig2.resolve(self.rspipeline2)

        ### Making robot go 10Hz
        self.rate = rospy.Rate(60)
        self.count = 0

        self.tagIDs = [0, 1]
        self.boxVal = previousReadings()
        self.canVal = previousReadings()

        self.median_filter = [TransformStamped(), TransformStamped(), TransformStamped()]
        self.median_count = 0
        self.still_count = 5

        self.listener = tf.TransformListener()
        rospy.sleep(2)
        self.br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adjust",
                              "world")

        rospy.sleep(1)

    def update_current_image(self):
        # # Start streaming
        rospy.loginfo("Starting cam 1")
        self.rspipeline1.start(self.rsconfig1)
        time.sleep(1)
        rospy.loginfo("Starting cam 2")
        self.rspipeline2.start(self.rsconfig2)

        try:
            while True:

                # Wait for a coherent pair of frames: depth and color
                # lines 98 and 100 are used to test frame rate
                # self.tim = self.current_milli_time()
                frames2 = self.rspipeline2.wait_for_frames()
                #rospy.loginfo("Frame 2 acquired")
                frames1 = self.rspipeline1.wait_for_frames()
                #rospy.loginfo("Frame 1 acquired")


                # rospy.loginfo("Cam Time: " + str(self.current_milli_time() - self.tim))
                depth_frame1 = frames1.get_depth_frame()
                color_frame1 = frames1.get_color_frame()
                depth_frame2 = frames2.get_depth_frame()
                color_frame2 = frames2.get_color_frame()
                if not depth_frame1 or not color_frame1:
                    continue

                # Convert images to numpy arrays
                depth_image1 = np.asanyarray(depth_frame1.get_data())
                color_image1 = np.asanyarray(color_frame1.get_data())

                depth_image2 = np.asanyarray(depth_frame2.get_data())
                color_image2 = np.asanyarray(color_frame2.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image1, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap_dim1 = depth_colormap1.shape
                color_colormap_dim1 = color_image1.shape

                depth_colormap2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image2, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap_dim2 = depth_colormap2.shape
                color_colormap_dim2 = color_image2.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim1 != color_colormap_dim1:
                    resized_color_image1 = cv2.resize(color_image1, dsize=(depth_colormap_dim1[1], depth_colormap_dim1[0]),
                                                     interpolation=cv2.INTER_AREA)
                    images1 = np.hstack((resized_color_image1, depth_colormap1))
                else:
                    images1 = np.hstack((color_image1, depth_colormap1))

                if depth_colormap_dim2 != color_colormap_dim2:
                    resized_color_image2 = cv2.resize(color_image2, dsize=(depth_colormap_dim2[1], depth_colormap_dim2[0]),
                                                     interpolation=cv2.INTER_AREA)
                    images2 = np.hstack((resized_color_image2, depth_colormap2))
                else:
                    images2 = np.hstack((color_image2, depth_colormap2))

                # Show images
                self.pipeline(color_image1, color_image2)
                #self.record_images(color_image)
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('RealSense', images)
                # cv2.waitKey(1)
        finally:
            self.rspipeline1.stop()
            self.rspipeline2.stop()

    def record_images(self, image):
        # height = image.height
        # width = image.width
        #
        # # Loop through each pixel of the map and convert it to pixel data
        # new_image = np.zeros((height, width, 3), dtype=np.uint8)
        #
        # for i in range(height):
        #     for j in range(width):
        #         for k in range(3):
        #             # BGR encoding for opencv
        #
        #             mult = 2 if k == 0 else 0 if k == 2 else 1
        #             cell = image.data[(i * width * 3 + j * 3 + k)]
        #             if cell >= 0:
        #                 new_image[i][j][mult] = cell
        cv2.imwrite(str(self.count) + ".jpg", image)
        print("done writing image")
        print(self.count)
        self.count += 1
        rospy.sleep(2)

        # for q in self.rolling_values:
        #     x = q[0]
        #     y = q[1]
        #     z = q[2]
        #     w = q[3]
        #
        #     sum_x = sum_x + w*x
        #     sum_y = sum_y + w*y
        #     sum_z = sum_z + w*z
    # image as image message
    def pipeline(self, image1, image2):
        # height = image.height
        # width = image.width

        # # Loop through each pixel of the map and convert it to pixel data
        # new_image = np.zeros((height, width, 3), dtype=np.uint8)

        # for i in range(height):
        #     for j in range(width):
        #         for k in range(3):
        #             # BGR encoding for opencv
        #             mult = 2 if k == 0 else 0 if k == 2 else 1
        #             cell = image.data[(i * width * 3 + j * 3 + k)]
        #             if cell >= 0:
        #                 new_image[i][j][mult] = cell

        gray_image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray_image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        # rospy.loginfo("Convert Time " + str(time2 - time1))

        # cv2.imshow("header", new_image)
        # cv2.waitKey(0)

        # # closing all open windows
        # cv2.destroyAllWindows()

        # 1080 by 1920
        #   FocalLength: [1380.4628 1379.4309]
        #   PrincipalPoint: [956.5579 542.9203]
        # fx = 1380.4628
        # fy = 1379.4309
        # cx = 956.5579
        # cy = 542.9203

        # 480 by 640
        # fx = 629.0741
        # fy = 615.1736
        # cx = 325.2477
        # cy = 251.2810

        # 1280 by 720
        # FocalLength: [908.3491 906.5133]
        # PrincipalPoint: [632.3028 343.9200]
        # fx = 908.3491
        # fy = 906.5133
        # cx = 632.3028
        # cy = 343.9200
        # Camera 2
        fx = 900.1325
        fy = 900.2865
        cx = 631.4351
        cy = 342.4242

        intrinsics_mat = np.array([[fx, 0, cx],
                                   [0, fy, cy],
                                   [0, 0, 1]])  # elements from the K matrix

        TAG_SIZE = 0.025 #0.062 # Tag size from Step 1 in meters
        obj_pts = np.array(object_points(TAG_SIZE))
        detector = apriltag(family="tag36h11")
        detections1 = detector.detect(gray_image1)
        detections2 = detector.detect(gray_image2)

        for ID in self.tagIDs:
            for tag in detections1:
                if tag['id'] == ID:
                    center = tag['center']
                    lb_rb_rt_lt = tag['lb-rb-rt-lt']
                    lt_rt_rb_lb = np.zeros((4, 2))
                    for i in range(4):
                        lt_rt_rb_lb[i] = lb_rb_rt_lt[3 - i]
                    # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
                    # rospy.loginfo("Detector Time " + str(time2 - time1))
                    # rospy.loginfo(lt_rt_rb_lb)

                    # time1 = self.current_milli_time()
                    good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (),
                                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    # time2 = self.current_milli_time()
                    # rospy.loginfo("Solver Time " + str(time2 - time1))

                    if good:
                        # Make 2 Pose w/ vectors

                        # handle rotation
                        transform = Transform()
                        transform.translation = Vector3(ptvecs[0][0], ptvecs[1][0], ptvecs[2][0])

                        rot_matrix, _ = cv2.Rodrigues(prvecs)

                        new_mat = np.zeros((4, 4), np.float32)
                        for i in range(3):
                            for j in range(3):
                                new_mat[i][j] = rot_matrix[i][j]
                        new_mat[3, 3] = 1
                        # rospy.loginfo(rot_matrix)
                        # handle pos
                        rot_matrix, _ = cv2.Rodrigues(prvecs)

                        orientation = quaternion_from_matrix(new_mat)
                        # orientation = self.boxVal.filter_readings(orientation)

                        transform.rotation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
                        if ID == 0:
                            object_name = "calibration_box"
                        elif ID == 1:
                            object_name = "corn_can"
                        self.br.sendTransform(
                            (transform.translation.x, transform.translation.y, transform.translation.z), (
                                transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                            rospy.Time.now(), object_name, "camera")
            for tag in detections2:
                if tag['id'] == ID:
                    center = tag['center']
                    lb_rb_rt_lt = tag['lb-rb-rt-lt']
                    lt_rt_rb_lb = np.zeros((4, 2))
                    for i in range(4):
                        lt_rt_rb_lb[i] = lb_rb_rt_lt[3 - i]
                    # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)

                    # time1 = self.current_milli_time()
                    good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (),
                                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if good:
                        transform = Transform()
                        transform.translation = Vector3(ptvecs[0][0], ptvecs[1][0], ptvecs[2][0])
                        rot_matrix, _ = cv2.Rodrigues(prvecs)
                        new_mat = np.zeros((4, 4), np.float32)
                        for i in range(3):
                            for j in range(3):
                                new_mat[i][j] = rot_matrix[i][j]
                        new_mat[3, 3] = 1
                        orientation2 = quaternion_from_matrix(new_mat)



    def current_milli_time(self):
        return round(time.time() * 1000)

    def run(self):
        r = rospy.Rate(60)

        while not rospy.is_shutdown():
            self.update_current_image()
        rospy.spin()


if __name__ == '__main__':
    Pipeline().run()
