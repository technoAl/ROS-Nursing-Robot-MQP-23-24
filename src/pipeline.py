#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import numpy as np
import cv2
from apriltag import apriltag
from PIL import Image as im
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Pose


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
        self.tag_pub = rospy.Publisher('/tag', Path, queue_size=1)

        # rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.update_intrinsics)

        # rospy.Subscriber('/camera/color/image_raw', Image, self.update_current_image, queue_size=1)

        self.current_image = 0
        self.intrinsics = 0

        ### Making robot go 10Hz
        self.rate = rospy.Rate(20)
        self.count = 0

    def update_intrinsics(self, msg):
        self.intrinsics = msg
        # focal length (fx, fy)
        # principal point(cx, cy)
        # Matlab
        #   FocalLength: [629.0741 615.1736]
        #   PrincipalPoint: [325.2477 251.2810]
        # Realsense says: (615.615, 615.5727)
        # (320.305, 242.789)
        #

    def update_current_image(self):
        cam_port = 4
        # time1 = rospy.get_time()
        cam = cv2.VideoCapture(cam_port)
        result, image = cam.read()
        self.pipeline(image);
        rospy.loginfo(image.shape)
        # time2 = rospy.get_time()
        # rospy.loginfo(time2 - time1)
        # if result:
        #     cv2.imshow("bah", image)
        #     cv2.waitKey(0)
            
        #     # closing all open windows
        #     cv2.destroyAllWindows()
        # else:
        #     rospy.loginfo("You're stupid")
        # if result:
        #     # self.current_image = msg
        #     self.pipeline(image)
            # self.record_images(msg)

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

        # cv2.imshow("header", new_image)
        # cv2.waitKey(0)
        
        # # closing all open windows
        # cv2.destroyAllWindows()

        #   FocalLength: [629.0741 615.1736]
        #   PrincipalPoint: [325.2477 251.2810]
        fx = 629.0741
        fy = 615.1736
        cx = 325.2477
        cy = 251.2810
        intrinsics_mat = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0,  0,  1]])# elements from the K matrix

        TAG_SIZE = 0.028  # Tag size from Step 1 in meters
        obj_pts = np.array(object_points(TAG_SIZE))

        detector = apriltag(family="tag36h11")
        detections = detector.detect(gray_image) #, estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        if len(detections) > 0:
            rospy.loginfo("Discovered Tag")
            for tag in detections:
                center = tag['center']
                lb_rb_rt_lt = tag['lb-rb-rt-lt']
                lt_rt_rb_lb = np.zeros((4, 2))
                for i in range(4):
                    lt_rt_rb_lb[i] = lb_rb_rt_lt[3-i]

                good, prvecs, ptvecs = cv2.solvePnP(obj_pts, lt_rt_rb_lb, intrinsics_mat, ()) #, flags=cv2.SOLVEPNP_IPPE_SQUARE
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
                    tag_msg = Path()

                    # Header
                    generic_header = Header()
                    generic_header.stamp = rospy.Time.now()
                    generic_header.frame_id = "none"
                    tag_msg.header = generic_header

                    # Make 2 Pose w/ vectors
                    rotation_stamped = PoseStamped()
                    rotation_stamped.header = generic_header
                    rotation_stamped.header.stamp = rospy.Time.now()

                    # handle rotation
                    rotation = Pose()
                    rotation.position = Point(prvecs[0][0], prvecs[1][0], prvecs[2][0])
                    rotation_stamped.pose = rotation
                    tag_msg.poses.append(rotation_stamped)

                    position_stamped = PoseStamped()
                    position_stamped.header = generic_header
                    position_stamped.header.stamp = rospy.Time.now()

                    # handle pos
                    position = Pose()
                    position.position = Point(ptvecs[0][0], ptvecs[1][0], ptvecs[2][0])
                    position_stamped.pose = position
                    tag_msg.poses.append(position_stamped)
                    rospy.loginfo(str(ptvecs[0][0]) + " " + str(ptvecs[1][0]) + " " + str(ptvecs[2][0]))
                    #rospy.loginfo(str(ptvecs[0]) + " " + str(ptvecs[1]) + " " + ptvecs[2])
                    self.tag_pub.publish(tag_msg)

                # imgpts, jac = cv2.projectPoints(opoints, prvecs, ptvecs, intrinsics_mat)
                # draw_boxes(new_image, imgpts, edges)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_current_image()
        rospy.spin()


if __name__ == '__main__':
    Pipeline().run()