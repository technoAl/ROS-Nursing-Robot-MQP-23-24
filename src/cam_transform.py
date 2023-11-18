#!/usr/bin/env python3

import rospy
import time 
import math
from std_msgs.msg import Header, String
import numpy as np 
import cv2
from apriltag import apriltag
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3, Transform
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix
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
        self.cam1_rot = np.zeros((4, 4))
        self.cam2_rot = np.zeros((4, 4))
        
    def cam_pipeline(self, image, camera):
        
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
                    #rospy.loginfo("DETECTED")
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



                        mat = np.zeros((4,4), np.float32)
                        for i in range(3):
                            for j in range(3):
                                if camera == 'cam1':
                                    self.cam1_rot[i][j] = rot_matrix[i][j]
                                    self.cam1_rot[0,3] = ptvecs[0][0]
                                    self.cam1_rot[1,3] = ptvecs[1][0]
                                    self.cam1_rot[2,3] = ptvecs[2][0]

                                else:
                                    self.cam2_rot[i][j] = rot_matrix[i][j]
                                    self.cam2_rot[0,3] = ptvecs[0][0]
                                    self.cam2_rot[1,3] = ptvecs[1][0]
                                    self.cam2_rot[2,3] = ptvecs[2][0]

                                mat[i][j] = rot_matrix[i][j]

                
                        mat[3,3] = 1

                        mat[0, 3] = ptvecs[0][0]
                        mat[1, 3] = ptvecs[1][0]
                        mat[2, 3] = ptvecs[2][0]

                        mat = np.linalg.inv(mat)

                        # handle pos
                        orientation = quaternion_from_matrix(mat)
                        
                        translation = [mat[0, 3], mat[1, 3], mat[2, 3]]
                        return translation, orientation


class Cam_Transform:
    
      
    def __init__(self):
        ### Initialize node, name it 'lab2'
        #rospy.init_node('cam_transform')
        self.cam1serial = '936322072225'
        self.cam2serial = '825412070317'
        self.br = tf.TransformBroadcaster()

        self.pipeline_rate = 0

        self.image_count = 0

        self.current_image = 0
        self.intrinsics = 0

        
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
        self.br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adjust",
                              "world")

        self.publish_ready = rospy.Publisher('/status', String, queue_size=10)

        self.t1 = [0.0, 0.0, 0.0]
        self.q1 = [0.0, 0.0, 0.0, 0.0]

        self.t2 = [0.0, 0.0, 0.0]
        self.q2 = [0.0, 0.0, 0.0, 0.0]


        self.listener = tf.TransformListener()

        self.sample_done = False

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

    # def cam_matricies(self, t1, q1, t2, q2):
    #
    #     matrix1 = quaternion_matrix(q1)
    #
    #     matrix1[0,3] = t1[0]
    #     matrix1[1,3] = t1[1]
    #     matrix1[2,3] = t1[2]
    #
    #     cam1_adjust_t, cam1_adjust_r = self.listener.lookupTransform('/adjust', 'camera_purple', rospy.Time(0))
    #
    #     cam1_adjust = quaternion_matrix(cam1_adjust_r)
    #
    #     cam1_adjust[0,3] = cam1_adjust_t[0]
    #     cam1_adjust[1,3] = cam1_adjust_t[1]
    #     cam1_adjust[2,3] = cam1_adjust_t[2]
    #
    #     cam1_matrix = matrix * np.linalg.inv(cam1_adjust)
    #
    #     new_t1 = [cam1_matrix[0,3], cam1_matrix[1,3], cam1_matrix[2,3]]
    #
    #     orientation1 = quaternion_from_matrix(cam1_matrix)
    #
    #     matrix2 = quaternion_matrix(q2)
    #
    #     matrix2[0,3] = t2[0]
    #     matrix2[1,3] = t2[1]
    #     matrix2[2,3] = t2[2]
    #
    #
    #     cam2_adjust_t, cam2_adjust_r = self.listener.lookupTransform('/adjust', 'camera_green', rospy.Time(0))
    #
    #     cam2_adjust = quaternion_matrix(cam2_adjust_r)
    #
    #     cam2_adjust[0,3] = cam2_adjust_t[0]
    #     cam2_adjust[1,3] = cam2_adjust_t[1]
    #     cam2_adjust[2,3] = cam2_adjust_t[2]
    #
    #     cam2_matrix = matrix2 * np.linalg.inv(cam2_adjust)
    #
    #     new_t2 = [cam2_matrix[0,3], cam2_matrix[1,3], cam2_matrix[2,3]]
    #
    #     orientation2 = quaternion_from_matrix(cam2_matrix)
    #
    #     p2 = Pose()
    #
    #     p2.position.x = new_t2[0]
    #     p2.position.y = new_t2[1]
    #     p2.position.z = new_t2[2]
    #
    #     p2.orientation.x = orientation2[0]
    #     p2.orientation.y = orientation2[1]
    #     p2.orientation.z = orientation2[2]
    #     p2.orientation.w = orientation2[3]
    #
    #
    #     self.cam1_publish.publish(p1)
    #     self.cam2_publish.publish(p2)
    #     self.status.publish("Ready")
    #
    #
    #     return new_t1, orientation1, new_t2, orientation2

    def broadcaster(self, image1, image2):

        try:

            if self.sample_done == False:

                sum_cam1 = [0] * 6
                sum_cam2 = [0] * 6

                rospy.loginfo("Initiating Sampling")
                # while True:
                while self.image_count < 100:
                    cam1_translation, cam1_rotation = self.image_pipeline.cam_pipeline(image1, 'cam1')
                    cam2_translation, cam2_rotation = self.image_pipeline.cam_pipeline(image2, 'cam2')

                    sum_cam1[0] = sum_cam1[0] + cam1_translation[0]
                    sum_cam1[1] = sum_cam1[1] + cam1_translation[1]
                    sum_cam1[2] = sum_cam1[2] + cam1_translation[2]

                    sum_cam1[3] = sum_cam1[3] + cam1_rotation[0] * cam1_rotation[3]
                    sum_cam1[4] = sum_cam1[4] + cam1_rotation[1] * cam1_rotation[3]
                    sum_cam1[5] = sum_cam1[5] + cam1_rotation[2] * cam1_rotation[3]

                    sum_cam2[0] = sum_cam2[0] + cam2_translation[0]
                    sum_cam2[1] = sum_cam2[1] + cam2_translation[1]
                    sum_cam2[2] = sum_cam2[2] + cam2_translation[2]

                    sum_cam2[3] = sum_cam2[3] + cam2_rotation[0] * cam2_rotation[3]
                    sum_cam2[4] = sum_cam2[4] + cam2_rotation[1] * cam2_rotation[3]
                    sum_cam2[5] = sum_cam2[5] + cam2_rotation[2] * cam2_rotation[3]

                    self.image_count = self.image_count + 1

                rospy.loginfo("Finishing Sampling")

                avg_x1 = sum_cam1[0] / 100.0
                avg_y1 = sum_cam1[1] / 100.0
                avg_z1 = sum_cam1[2] / 100.0

                avg_qx1 = sum_cam1[3] / 100.0
                avg_qy1 = sum_cam1[4] / 100.0
                avg_qz1 = sum_cam1[5] / 100.0

                avg_x2 = sum_cam2[0] / 100.0
                avg_y2 = sum_cam2[1] / 100.0
                avg_z2 = sum_cam2[2] / 100.0

                avg_qx2 = sum_cam2[3] / 100.0
                avg_qy2 = sum_cam2[4] / 100.0
                avg_qz2 = sum_cam2[5] / 100.0

                w1 = pow((pow(avg_qx1, 2) + pow(avg_qy1, 2) + pow(avg_qz1, 2)), 0.5)
                w2 = pow((pow(avg_qx2, 2) + pow(avg_qy2, 2) + pow(avg_qz2, 2)), 0.5)

                avg_qx1 = avg_qx1 / w1
                avg_qy1 = avg_qy1 / w1
                avg_qz1 = avg_qz1 / w1

                avg_qx2 = avg_qx2 / w2
                avg_qy2 = avg_qy2 / w2
                avg_qz2 = avg_qz2 / w2

                d1 = pow((pow(avg_qx1, 2) + pow(avg_qy1, 2) + pow(avg_qz1, 2) + pow(w1, 2)), 0.5)
                d2 = pow((pow(avg_qx2, 2) + pow(avg_qy2, 2) + pow(avg_qz2, 2) + pow(w2, 2)), 0.5)


                self.t1[0] = avg_x1
                self.t1[1] = avg_y1
                self.t1[2] = avg_z1

                quat_1 = (avg_qx1, avg_qy1, avg_qz1, w1)/d1

                self.q1[0] = quat_1[0]
                self.q1[1] = quat_1[1]
                self.q1[2] = quat_1[2]
                self.q1[3] = quat_1[3]

                self.t2[0] = avg_x2
                self.t2[1] = avg_y2
                self.t2[2] = avg_z2

                quat_2 = (avg_qx2, avg_qy2, avg_qz2, w2)/d2

                self.q2[0] = quat_2[0]
                self.q2[1] = quat_2[1]
                self.q2[2] = quat_2[2]
                self.q2[3] = quat_2[3]

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

            self.rspipeline1.stop()
            self.rspipeline2.stop()
            self.publish_ready.publish("Ready")

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
    
        
        
        






