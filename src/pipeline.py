#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Header
import numpy as np
import cv2
from apriltag import apriltag
from PIL import Image as im
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
import pyrealsense2 as rs
import time
from tf.transformations import quaternion_from_euler


def object_points(tag_size):
    return [[-tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, tag_size / 2, 0.0],
            [tag_size / 2, -tag_size / 2, 0.0],
            [-tag_size / 2, tag_size / 2, 0.0]]


class Pipeline:

    def __init__(self):
        """
        Class constructor
        """
        rospy.sleep(5)

        ### Initialize node, name it 'lab2'
        rospy.init_node('pipeline')

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.tag_pub = rospy.Publisher('/april/calibration_box', PoseStamped, queue_size=1)

        # rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.update_intrinsics)

        # rospy.Subscriber('/camera/color/image_raw', Image, self.update_current_image, queue_size=1)

        self.pipeline_rate = 0 

        self.current_image = 0
        self.intrinsics = 0

        # camera setup through pyrealsense2
        self.rspipeline = rs.pipeline()
        self.rsconfig = rs.config()

        self.rspipe_wrapper = rs.pipeline_wrapper(self.rspipeline)
        self.pipeline_profile = self.rsconfig.resolve(self.rspipe_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            rospy.loginfo("The demo requires Depth camera with Color sensor")
            exit(0)
        
        self.rsconfig.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        self.rsconfig.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        ### Making robot go 10Hz
        self.rate = rospy.Rate(60)
        self.count = 0
        #

    def update_current_image(self):

        # # Start streaming
        self.rspipeline.start(self.rsconfig)

        try:
            while True:

                # Wait for a coherent pair of frames: depth and color
                # lines 98 and 100 are used to test frame rate
                # self.tim = self.current_milli_time()
                frames = self.rspipeline.wait_for_frames()
                # rospy.loginfo("Cam Time: " + str(self.current_milli_time() - self.tim))
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))

                # Show images
                self.pipeline(color_image)
                # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('RealSense', images)
                # cv2.waitKey(1)
        finally:
            self.rspipeline.stop()

    def record_images(self, image):
        height = image.height
        width = image.width

        # Loop through each pixel of the map and convert it to pixel data
        new_image = np.zeros((height, width, 3), dtype=np.uint8)

        for i in range(height):
            for j in range(width):
                for k in range(3):
                    # BGR encoding for opencv

                    mult = 2 if k == 0 else 0 if k == 2 else 1
                    cell = image.data[(i * width * 3 + j * 3 + k)]
                    if cell >= 0:
                        new_image[i][j][mult] = cell
        cv2.imwrite(str(self.count)+".jpg", new_image)
        print("done writing image")
        self.count += 1
        rospy.sleep(2)

    # image as image message
    def pipeline(self, image):
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

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # rospy.loginfo("Convert Time " + str(time2 - time1))
        

        # cv2.imshow("header", new_image)
        # cv2.waitKey(0)
        
        # # closing all open windows
        # cv2.destroyAllWindows()

        #   FocalLength: [1380.4628 1379.4309]
        #   PrincipalPoint: [956.5579 542.9203]
        fx = 1380.4628
        fy = 1379.4309
        cx = 956.5579
        cy = 542.9203
        # fx = 629.0741
        # fy = 615.1736
        # cx = 325.2477
        # cy = 251.2810
        intrinsics_mat = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0,  0,  1]])# elements from the K matrix

        TAG_SIZE = 0.076  # Tag size from Step 1 in meters
        obj_pts = np.array(object_points(TAG_SIZE))
        detector = apriltag(family="tag36h11")
        detections = detector.detect(gray_image) #, estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        # rospy.loginfo("Detector Time " + str(time2 - time1))
        if len(detections) > 0:
            for tag in detections:
                center = tag['center']
                lb_rb_rt_lt = tag['lb-rb-rt-lt']
                lt_rt_rb_lb = np.zeros((4, 2))
                for i in range(4):
                    lt_rt_rb_lb[i] = lb_rb_rt_lt[3-i]

                # time1 = self.current_milli_time()
                good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, (), flags=cv2.SOLVEPNP_IPPE_SQUARE)
                # time2 = self.current_milli_time()
                # rospy.loginfo("Solver Time " + str(time2 - time1))

                
                 
                if good:

                    # pt = lt_rt_rb_lb[0]
                    # print(tuple(pt))

                    # p1 = (int(lt_rt_rb_lb[0][0]), int(lt_rt_rb_lb[0][1]))
                    # p2 = (int(lt_rt_rb_lb[1][0]), int(lt_rt_rb_lb[1][1]))
                    # p3 = (int(lt_rt_rb_lb[2][0]), int(lt_rt_rb_lb[2][1]))
                    # p4 = (int(lt_rt_rb_lb[3][0]), int(lt_rt_rb_lb[3][1]))
                    
                    # new_image = cv2.line(new_image, p1, p2, (0, 255, 0), 2)
                    # new_image = cv2.line(new_image, p2, p3, (0, 255, 0), 2)
                    # new_image = cv2.line(new_image, p3, p4, (0, 255, 0), 2)
                    # new_image = cv2.line(new_image, p4, p1, (0, 255, 0), 2)
                    
                    # cv2.imshow("max range", new_image)
                    # cv2.waitKey(0)
                    # time1 = self.current_milli_time()
                    tag_msg = PoseStamped()

                    # Header
                    generic_header = Header()
                    generic_header.stamp = rospy.Time.now()
                    generic_header.frame_id = "none"
                    tag_msg.header = generic_header

                    # Make 2 Pose w/ vectors

                    # handle rotation
                    pose = Pose()
                    pose.position = Point(ptvecs[0][0], ptvecs[1][0], ptvecs[2][0])

                    # handle pos
                    orientation = quaternion_from_euler(prvecs[0][0], prvecs[1][0], prvecs[2][0])
                    pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
                    tag_msg.pose = pose


                    # time2 = self.current_milli_time()
                    # rospy.loginfo("Make Time " + str(time2 - time1))
                    if ptvecs[2][0] > 0 and ptvecs[2][0] < 2.5:
                        #rospy.loginfo(str(ptvecs[0][0]) + " " + str(ptvecs[1][0]) + " " + str(ptvecs[2][0]))
                        #rospy.loginfo(str(ptvecs[0]) + " " + str(ptvecs[1]) + " " + ptvecs[2])
                        # time1 = self.current_milli_time()
                        self.tag_pub.publish(tag_msg)
                        # time2 = self.current_milli_time()
                        # rospy.loginfo("Publish Time " + str(time2 - time1))
                        self.pipeline_rate += 1
                        # rospy.loginfo(self.current_milli_time()-self.tim)

                        

                # imgpts, jac = cv2.projectPoints(opoints, prvecs, ptvecs, intrinsics_mat)
                # draw_boxes(new_image, imgpts, edges)
    

    def current_milli_time(self):
        return round(time.time() * 1000)

    def run(self):
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.update_current_image()
        rospy.spin()


if __name__ == '__main__':
    Pipeline().run()