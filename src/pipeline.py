#!/usr/bin/env python3

import rospy
import cam_transform
import time
import math
from std_msgs.msg import Header, String
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

class Image_Processing:
    def __init__(self):

        self.tag_size = 0.079  # tag size in meters
        fx_green= 1357.7098
        fy_green = 1364.4257
        cx_green = 951.6751
        cy_green = 511.0647


        self.cam1_intrinsics = np.array([[fx_green, 0, cx_green],
                                         [0, fy_green, cy_green],
                                         [0, 0, 1]])
        self.cam2_intrinsics = np.zeros((3, 3))
        self.br = tf.TransformBroadcaster()
        self.cam1_rot = np.zeros((4, 4))
        self.cam2_rot = np.zeros((4, 4))


    def processor(self, image, in_init):


        intrinsics_mat = self.cam1_intrinsics
        '''if camera == 'cam1':
            intrinsics_mat = self.cam1_intrinsics
        else:
            intrinsics_mat = self.cam2_intrinsics
        '''

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if in_init:
            self.tag_size = 0.079
        else:
            self.tag_size = 0.025
        obj_pts = np.array(object_points(self.tag_size))
        detector = apriltag(family="tag36h11")
        detections = detector.detect(gray_image)  # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        # rospy.loginfo("Detector Time " + str(time2 - time1))

        found_objects = {}

        if len(detections) > 0:
            for tag in detections:
                # rospy.loginfo("DETECTED")
                center = tag['center']
                lb_rb_rt_lt = tag['lb-rb-rt-lt']
                lt_rt_rb_lb = np.zeros((4, 2))
                for i in range(4):
                    lt_rt_rb_lb[i] = lb_rb_rt_lt[3 - i]

                # rospy.loginfo(lt_rt_rb_lb)

                # time1 = self.current_milli_time()
                good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (),
                                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                # time2 = self.current_milli_time()
                # rospy.loginfo("Solver Time " + str(time2 - time1))

                if good:

                    # pt = lt_rt_rb_lb[0]
                    # print(tuple(pt))

                    # p1 = (int(lt_rt_rb_lb[0][0]), int(lt_rt_rb_lb[0][1]))
                    # p2 = (int(lt_rt_rb_lb[1][0]), int(lt_rt_rb_lb[1][1]))
                    # p3 = (int(lt_rt_rb_lb[2][0]), int(lt_rt_rb_lb[2][1]))
                    # p4 = (int(lt_rt_rb_lb[3][0]), int(lt_rt_rb_lb[3][1]))
                    #
                    # image = cv2.line(image, p1, p2, (0, 255, 0), 2)
                    # image = cv2.line(image, p2, p3, (0, 255, 0), 2)
                    # #new_image = cv2.line(new_image, p3, p4, (0, 255, 0), 2)
                    # #new_image = cv2.line(new_image, p4, p1, (0, 255, 0), 2)
                    #
                    # cv2.imshow("max range", image)
                    # cv2.waitKey(0)
                    # time1 = self.current_milli_time()
                    ID = tag['id']
                    object_name = ""
                    if in_init and ID == 47:
                        object_name = "tag"
                    elif not in_init:
                        if ID == 0:
                            object_name = "grey_cube"
                        if ID == 1:
                            object_name = "corn_can"
                        if ID == 2:
                            object_name = "bottle_2"
                    else:
                        continue

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

                    if in_init and ID == 47:
                        mat = np.linalg.inv(mat)

                    orientation = quaternion_from_matrix(mat)

                    translation = [mat[0, 3], mat[1, 3], mat[2, 3]]

                    found_objects[object_name] = (translation, orientation)

        return found_objects


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
        self.rsconfig1.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        self.rsconfig2.enable_device(self.cam2serial)
        self.rsconfig2.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        self.profile1 = self.rsconfig1.resolve(self.rspipeline1)
        self.profile2 = self.rsconfig2.resolve(self.rspipeline2)

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

        self.sample_done = False
        self.image_count = 0

        self.tagIDs = [0, 1, 2]
        self.boxVal = previousReadings()
        self.canVal = previousReadings()

        self.processor = Image_Processing()

        self.median_filter = [TransformStamped(), TransformStamped(), TransformStamped()]
        self.median_count = 0
        self.still_count = 5

        self.tag_size = 0.079  # tag size in meters
        self.br = tf.TransformBroadcaster()
        self.cam1_rot = np.zeros((4, 4))
        self.cam2_rot = np.zeros((4, 4))

        self.ready = ''

        rospy.Subscriber('/status', String, self.callback)
        self.listener = tf.TransformListener()
        rospy.sleep(2)
        self.br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adjust",
                              "world")


        self.object_name_list = ["grey_cube", "corn_can", "bottle_2"]
        rospy.sleep(1)

    def callback(self,data):
        self.ready = data.data

    def broadcaster(self, image1, image2):

        while self.sample_done == False:

            sum_cam1 = [0] * 6
            sum_cam2 = [0] * 6

            rospy.loginfo("Initiating Sampling")
            # while True:
            while self.image_count < 10:
                self.image_count = self.image_count + 1
            found_tag = self.processor.processor(image1, True)
            cam1_translation = found_tag['tag'][0]
            cam1_rotation = found_tag['tag'][1]

            found_tag = self.processor.processor(image2, True)
            cam2_translation = found_tag['tag'][0]
            cam2_rotation = found_tag['tag'][1]
                #
                # sum_cam1[0] = sum_cam1[0] + cam1_translation[0]
                # sum_cam1[1] = sum_cam1[1] + cam1_translation[1]
                # sum_cam1[2] = sum_cam1[2] + cam1_translation[2]
                #
                # sum_cam1[3] = sum_cam1[3] + cam1_rotation[0] * cam1_rotation[3]
                # sum_cam1[4] = sum_cam1[4] + cam1_rotation[1] * cam1_rotation[3]
                # sum_cam1[5] = sum_cam1[5] + cam1_rotation[2] * cam1_rotation[3]
                #
                # sum_cam2[0] = sum_cam2[0] + cam2_translation[0]
                # sum_cam2[1] = sum_cam2[1] + cam2_translation[1]
                # sum_cam2[2] = sum_cam2[2] + cam2_translation[2]
                #
                # sum_cam2[3] = sum_cam2[3] + cam2_rotation[0] * cam2_rotation[3]
                # sum_cam2[4] = sum_cam2[4] + cam2_rotation[1] * cam2_rotation[3]
                # sum_cam2[5] = sum_cam2[5] + cam2_rotation[2] * cam2_rotation[3]

                #self.image_count = self.image_count + 1

            rospy.loginfo("Finishing Sampling")

            # avg_x1 = sum_cam1[0] / 100.0
            # avg_y1 = sum_cam1[1] / 100.0
            # avg_z1 = sum_cam1[2] / 100.0
            #
            # avg_qx1 = sum_cam1[3] / 100.0
            # avg_qy1 = sum_cam1[4] / 100.0
            # avg_qz1 = sum_cam1[5] / 100.0
            #
            # avg_x2 = sum_cam2[0] / 100.0
            # avg_y2 = sum_cam2[1] / 100.0
            # avg_z2 = sum_cam2[2] / 100.0
            #
            # avg_qx2 = sum_cam2[3] / 100.0
            # avg_qy2 = sum_cam2[4] / 100.0
            # avg_qz2 = sum_cam2[5] / 100.0
            #
            # w1 = pow((pow(avg_qx1, 2) + pow(avg_qy1, 2) + pow(avg_qz1, 2)), 0.5)
            # w2 = pow((pow(avg_qx2, 2) + pow(avg_qy2, 2) + pow(avg_qz2, 2)), 0.5)
            #
            # avg_qx1 = avg_qx1 / w1
            # avg_qy1 = avg_qy1 / w1
            # avg_qz1 = avg_qz1 / w1
            #
            # avg_qx2 = avg_qx2 / w2
            # avg_qy2 = avg_qy2 / w2
            # avg_qz2 = avg_qz2 / w2
            #
            # d1 = pow((pow(avg_qx1, 2) + pow(avg_qy1, 2) + pow(avg_qz1, 2) + pow(w1, 2)), 0.5)
            # d2 = pow((pow(avg_qx2, 2) + pow(avg_qy2, 2) + pow(avg_qz2, 2) + pow(w2, 2)), 0.5)


            self.t1[0] = cam1_translation[0]
            self.t1[1] = cam1_translation[1]
            self.t1[2] = cam1_translation[2]

            # quat_1 = (avg_qx1, avg_qy1, avg_qz1, w1)/d1

            self.q1[0] = cam1_rotation[0]
            self.q1[1] = cam1_rotation[1]
            self.q1[2] = cam1_rotation[2]
            self.q1[3] = cam1_rotation[3]

            self.t2[0] = cam2_translation[0]
            self.t2[1] = cam2_translation[1]
            self.t2[2] = cam2_translation[2]

            # quat_2 = (avg_qx2, avg_qy2, avg_qz2, w2)/d2

            self.q2[0] = cam2_rotation[0]
            self.q2[1] = cam2_rotation[1]
            self.q2[2] = cam2_rotation[2]
            self.q2[3] = cam2_rotation[3]

            self.sample_done = True


        transform1 = Transform()
        transform1.translation = Vector3(self.t1[0], self.t1[1], self.t1[2])
        transform1.rotation = Quaternion(self.q1[0], self.q1[1], self.q1[2], self.q1[3])

        transform2 = Transform()
        transform2.translation = Vector3(self.t2[0], self.t2[1], self.t2[2])
        transform2.rotation = Quaternion(self.q2[0], self.q2[1], self.q2[2], self.q2[3])

        self.br.sendTransform((transform1.translation.x, transform1.translation.y, transform1.translation.z), (
            transform1.rotation.x, transform1.rotation.y, transform1.rotation.z, transform1.rotation.w),
                              rospy.Time.now(), "camera_green", "calibration_tag")

        self.br.sendTransform((transform2.translation.x, transform2.translation.y, transform2.translation.z), (
            transform2.rotation.x, transform2.rotation.y, transform2.rotation.z, transform2.rotation.w),
                              rospy.Time.now(), "camera_purple", "calibration_tag")

    def update_current_image(self):

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
                # depth_frame1 = frames1.get_depth_frame()
                color_frame1 = frames1.get_color_frame()
                # depth_frame2 = frames2.get_depth_frame()
                color_frame2 = frames2.get_color_frame()
                # if not depth_frame1 or not color_frame1:
                #     continue

                # Convert images to numpy arrays
                # depth_image1 = np.asanyarray(depth_frame1.get_data())
                color_image1 = np.asanyarray(color_frame1.get_data())

                # depth_image2 = np.asanyarray(depth_frame2.get_data())
                color_image2 = np.asanyarray(color_frame2.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                # depth_colormap1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image1, alpha=0.03), cv2.COLORMAP_JET)
                # depth_colormap_dim1 = depth_colormap1.shape
                # color_colormap_dim1 = color_image1.shape
                #
                # depth_colormap2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image2, alpha=0.03), cv2.COLORMAP_JET)
                # depth_colormap_dim2 = depth_colormap2.shape
                # color_colormap_dim2 = color_image2.shape
                #
                # # If depth and color resolutions are different, resize color image to match depth image for display
                # if depth_colormap_dim1 != color_colormap_dim1:
                #     resized_color_image1 = cv2.resize(color_image1, dsize=(depth_colormap_dim1[1], depth_colormap_dim1[0]),
                #                                      interpolation=cv2.INTER_AREA)
                #     images1 = np.hstack((resized_color_image1, depth_colormap1))
                # else:
                #     images1 = np.hstack((color_image1, depth_colormap1))
                #
                # if depth_colormap_dim2 != color_colormap_dim2:
                #     resized_color_image2 = cv2.resize(color_image2, dsize=(depth_colormap_dim2[1], depth_colormap_dim2[0]),
                #                                      interpolation=cv2.INTER_AREA)
                #     images2 = np.hstack((resized_color_image2, depth_colormap2))
                # else:
                #     images2 = np.hstack((color_image2, depth_colormap2))

                # Show images
                self.broadcaster(color_image1, color_image2)
                self.pipeline(color_image1, color_image2)
                #self.record_images(color_image)
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                #cv2.imshow('RealSense', images1)
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
        tags = self.processor.processor(image1, False)

        for object_name in self.object_name_list:
            if object_name in tags.keys():

                transform = Transform()
                translation = tags[object_name][0]
                quaternion = tags[object_name][1]

                transform.translation = Vector3(translation[0], translation[1], translation[2])
                transform.rotation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                self.br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z), (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w), rospy.Time.now(), object_name + "_green", "camera_green")

        tags = self.processor.processor(image2, False)
        for object_name in self.object_name_list:
            if object_name in tags.keys():
                transform = Transform()
                translation = tags[object_name][0]
                quaternion = tags[object_name][1]

                transform.translation = Vector3(translation[0], translation[1], translation[2])
                transform.rotation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

                self.br.sendTransform((transform.translation.x, transform.translation.y, transform.translation.z), (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w), rospy.Time.now(), object_name + "_purple", "camera_purple")



        # gray_image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        # gray_image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        #
        # # 1080 by 1920
        # #   FocalLength: [1380.4628 1379.4309]
        # #   PrincipalPoint: [956.5579 542.9203]
        # # fx = 1380.4628
        # # fy = 1379.4309
        # # cx = 956.5579
        # # cy = 542.9203
        #
        # # 480 by 640
        # # fx = 629.0741
        # # fy = 615.1736
        # # cx = 325.2477
        # # cy = 251.2810
        #
        # # 1280 by 720
        # # FocalLength: [908.3491 906.5133]
        # # PrincipalPoint: [632.3028 343.9200]
        # # fx = 908.3491
        # # fy = 906.5133
        # # cx = 632.3028
        # # cy = 343.9200
        # # Camera 1
        # fx = 1357.7098
        # fy = 1364.4257
        # cx = 951.6751
        # cy = 511.0647
        #
        # intrinsics_mat = np.array([[fx, 0, cx],
        #                            [0, fy, cy],
        #                            [0, 0, 1]])  # elements from the K matrix
        #
        # TAG_SIZE = 0.025 #0.062 # Tag size from Step 1 in meters
        # obj_pts = np.array(object_points(TAG_SIZE))
        # detector = apriltag(family="tag36h11")
        # detections1 = detector.detect(gray_image1)
        # detections2 = detector.detect(gray_image2)
        #
        # for ID in self.tagIDs:
        #     for tag in detections1:
        #         if tag['id'] == ID:
        #             center = tag['center']
        #             lb_rb_rt_lt = tag['lb-rb-rt-lt']
        #             lt_rt_rb_lb = np.zeros((4, 2))
        #             for i in range(4):
        #                 lt_rt_rb_lb[i] = lb_rb_rt_lt[3 - i]
        #             # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        #             # rospy.loginfo("Detector Time " + str(time2 - time1))
        #             # rospy.loginfo(lt_rt_rb_lb)
        #
        #             # time1 = self.current_milli_time()
        #             good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (),
        #                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
        #             # time2 = self.current_milli_time()
        #             # rospy.loginfo("Solver Time " + str(time2 - time1))
        #
        #             if good:
        #                 # Make 2 Pose w/ vectors
        #
        #                 # handle rotation
        #                 transform = Transform()
        #                 transform.translation = Vector3(ptvecs[0][0], ptvecs[1][0], ptvecs[2][0])
        #
        #                 rot_matrix, _ = cv2.Rodrigues(prvecs)
        #
        #                 new_mat = np.zeros((4, 4), np.float32)
        #                 for i in range(3):
        #                     for j in range(3):
        #                         new_mat[i][j] = rot_matrix[i][j]
        #                 new_mat[3, 3] = 1
        #                 # rospy.loginfo(rot_matrix)
        #                 # handle pos
        #                 rot_matrix, _ = cv2.Rodrigues(prvecs)
        #
        #                 orientation = quaternion_from_matrix(new_mat)
        #                 orientation = self.boxVal.filter_readings(orientation)
        #
        #
        #
        #                 transform.rotation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        #                 if ID == 0:
        #                     object_name = "calibration_box"
        #                 elif ID == 1:
        #                     object_name = "corn_can"
        #                 self.br.sendTransform(
        #                     (transform.translation.x, transform.translation.y, transform.translation.z), (
        #                         transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
        #                     rospy.Time.now(), object_name, "camera_green")
        #     for tag in detections2:
        #         if tag['id'] == ID:
        #             center = tag['center']
        #             lb_rb_rt_lt = tag['lb-rb-rt-lt']
        #             lt_rt_rb_lb = np.zeros((4, 2))
        #             for i in range(4):
        #                 lt_rt_rb_lb[i] = lb_rb_rt_lt[3 - i]
        #             # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        #
        #             # time1 = self.current_milli_time()
        #             good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (),
        #                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
        #             if good:
        #                 transform = Transform()
        #                 transform.translation = Vector3(ptvecs[0][0], ptvecs[1][0], ptvecs[2][0])
        #                 rot_matrix, _ = cv2.Rodrigues(prvecs)
        #                 new_mat = np.zeros((4, 4), np.float32)
        #                 for i in range(3):
        #                     for j in range(3):
        #                         new_mat[i][j] = rot_matrix[i][j]
        #                 new_mat[3, 3] = 1
        #                 orientation2 = quaternion_from_matrix(new_mat)
        #                 orientation2 = self.boxVal.filter_readings(orientation2)
        #
        #                 transform.rotation = Quaternion(orientation2[0], orientation2[1], orientation2[2], orientation2[3])
        #                 if ID == 0:
        #                     object_name = "calibration_box"
        #                 elif ID == 1:
        #                     object_name = "corn_can"
        #                 self.br.sendTransform(
        #                     (transform.translation.x, transform.translation.y, transform.translation.z), (
        #                         transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
        #                     rospy.Time.now(), object_name, "camera_purple")
    def current_milli_time(self):
        return round(time.time() * 1000)

    def run(self):
        r = rospy.Rate(60)
        # # Start streaming
        rospy.loginfo("Starting cam 1")
        self.rspipeline1.start(self.rsconfig1)
        time.sleep(1)
        rospy.loginfo("Starting cam 2")
        self.rspipeline2.start(self.rsconfig2)

        while not rospy.is_shutdown():
            self.update_current_image()
        rospy.spin()

if __name__ == '__main__':
    Pipeline().run()
