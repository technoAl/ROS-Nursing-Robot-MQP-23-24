#!/usr/bin/env python3

import rospy
import time 
import math
from std_msgs.msg import Header
import numpy as np 
import cv2
from apriltag import apriltag
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3, Transform
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
import pyrealsense2 as rs


def object_points(tag_size):
    return [[-tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, -tag_size / 2, 0.0],
            [-tag_size / 2, -tag_size / 2, 0.0]]
class Image_Pipeline:
    def __init__(self):
        
        self.tag_size = 0.079 #tag size in meters
        self.cam1_intrinsics = np.array([[900.1325, 0 , 631.4351],
                                         [0, 900.2865, 342.4242],
                                         [0, 0, 1]])
        self.cam2_intrinsics = np.zeros((3,3))
        self.br = tf.TransformBroadcaster()
        self.cam1_rot = np.zeros((3,3))
        self.cam2_rot = np.zeros((3, 3))
        
    def pipeline(self, image, camera):
        
        intrinsics_mat = self.cam1_intrinsics
        '''if camera == 'cam1':
            intrinsics_mat = self.cam1_intrinsics
        else:
            intrinsics_mat = self.cam2_intrinsics
        '''


        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        obj_pts = np.array(object_points(self.tag_size))
        detector = apriltag(family="tag36h11")
        detections = detector.detect(gray_image)  # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        # rospy.loginfo("Detector Time " + str(time2 - time1))

        if len(detections) > 0:
            for tag in detections:
                if tag['id'] == 47:

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

                        rot_matrix,_ = cv2.Rodrigues(prvecs)



                        new_mat = np.zeros((4,4), np.float32)
                        for i in range(3):
                            for j in range(3):
                                if camera == 'cam1':
                                    self.cam1_rot[i][j] = rot_matrix[i][j]
                                else:
                                    self.cam2_rot[i][j] = rot_matrix[i][j]
                                new_mat[i][j] = rot_matrix[i][j]


                        new_mat[3,3] = 1

                        new_mat[0, 3] = ptvecs[0][0]
                        new_mat[1, 3] = ptvecs[1][0]
                        new_mat[2, 3] = ptvecs[2][0]

                        mat = np.linalg.inv(new_mat)
                        #rospy.loginfo(rot_matrix)
                        # handle pos
                        orientation = quaternion_from_matrix(mat)
                        
                        translation = [ptvecs[0][0], ptvecs[1][0], ptvecs[2][0]]
                        return translation, orientation


class Cam_Transform:
    
      
    def __init__(self):
        ### Initialize node, name it 'lab2'
        rospy.init_node('cam_transform')
        self.cam1serial = '936322072225'
        self.cam2serial = '825412070317'
        self.br = tf.TransformBroadcaster()

        self.pipeline_rate = 0

        self.image_count = 0

        self.current_image = 0
        self.intrinsics = 0
        self.plot_publisher = rospy.Publisher('/plot/value', Float32, queue_size=10)
        
        self.image_pipeline = Image_Pipeline()

         
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
        self.br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adjust",
                              "world")

        rospy.sleep(1)

    def get_current_image(self):

        # # Start streaming
        rospy.loginfo("Starting cam 1")
        self.rspipeline1.start(self.rsconfig1)
        time.sleep(1)
        rospy.loginfo("Starting cam 2")
        self.rspipeline2.start(self.rsconfig2)

        try:
            while True:

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
                self.broadcaster(color_image1, color_image2)
                #self.record_images(color_image)
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('RealSense', images)
                # cv2.waitKey(1)
        finally:
            self.rspipeline1.stop()
            self.rspipeline2.stop()

    def broadcaster(self, image1, image2):

        try:
            cam1_translation, cam1_rotation = self.image_pipeline.pipeline(image1, 'cam1')
            cam2_translation, cam2_rotation = self.image_pipeline.pipeline(image2, 'cam2')



            # while True:
            # if self.image_count >= 10:
                # while True:
            # rospy.loginfo("Camera 1 -> X: " + str(cam2_translation[0]) + "Y: " + str(cam2_translation[1]) + "Z: " + str(cam2_translation[2]))
            # rospy.loginfo("Camera 2 -> X: " + str(cam1_translation[0]) + "Y: " + str(cam1_translation[1]) + "Z: " + str(
            #     cam1_translation[2]))
            transform1 = Transform()
            transform1.translation = Vector3(cam1_translation[0], cam1_translation[1], cam1_translation[2])

            transform1.rotation = Quaternion(cam1_rotation[0], cam1_rotation[1], cam1_rotation[2], cam1_rotation[3])

            self.br.sendTransform((transform1.translation.x, transform1.translation.y, transform1.translation.z), (
                                transform1.rotation.x, transform1.rotation.y, transform1.rotation.z, transform1.rotation.w), rospy.Time.now(), "camera1", "calibration_tag")

            transform2 = Transform()
            transform2.translation = Vector3(cam2_translation[1], cam2_translation[0], cam2_translation[2])

            transform2.rotation = Quaternion(cam2_rotation[0], cam2_rotation[1], cam2_rotation[2], cam2_rotation[3])

            self.br.sendTransform((transform2.translation.x, transform2.translation.y, transform2.translation.z), (
                                transform2.rotation.x, transform2.rotation.y, transform2.rotation.z, transform2.rotation.w), rospy.Time.now(), "camera2", "calibration_tag")
            # else:
            #     self.image_count = self.image_count + 1

        except Exception as e:
            rospy.loginfo(e)
            pass

    def run(self):
        r = rospy.Rate(60)

        while not rospy.is_shutdown():
            self.get_current_image()
        rospy.spin()

if __name__ == '__main__':
    Cam_Transform().run()
    
        
        
        






