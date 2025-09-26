#!/usr/bin/env python3
"""
This Python file runs a ROS 2 node named localization_node which publishes the position of crates and a holonomic drive robot.
This node subscribes to the following topics:
 SUBSCRIPTIONS
 /camera/image_raw
 /camera/camera_info
 /crates_pose   
 /bot_pose
"""
import math
import cv2 as cv 
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D
from eyantrasim_msgs.msg import Pose
import matplotlib.pyplot as plt 

class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.get_logger().info('PoseDetector initialized')
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        self.aruco_dict_name = cv.aruco.DICT_4X4_50  # Choose ArUco dictionary

        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = camera_matrix = np.array([
                                [1030.4890823364258,    0.0,    960.0],
                                [   0.0,    1030.489103794098, 540.0],
                                [   0.0,       0.0,      1.0]
                            ], dtype=np.float64)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = [[446.0, 27.0],[1474.0,26.0],[445.0,1055.0],[1475.0, 1055.0]]  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = [[0,0],[2438.4, 0],[0, 2438.4],[2438.4, 2438.4]]  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        # self.world_coords_dict = {
        #     1: [[0, 0], [50, 0], [50, 50], [0, 50]],
        #     3: [[2438.4-50.0, 0], [2438.4, 0], [2438.4, 50], [2438.4-50, 50]],
        #     5: [[0, 2438.4-50], [50, 2438.4-50], [50, 2438.4], [0, 2438.4]],
        #     7: [[2438.4-50, 2438.4-50], [2438.4, 2438.4-50],[2438.4-50, 2438.4], [2438.4, 2438.4]]
        # }
        self.world_coords_dict = {
            1: [0, 50],
            3: [2438.4, 0],
            5: [0, 2438.4],
            7: [2438.4, 2438.4]
        }

        self.yaw = None
        self.y = None
 
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        self.aruco_dict = cv.aruco.getPredefinedDictionary(self.aruco_dict_name)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.aruco_dict,self.aruco_params)

        self.detected_crates = {}
        self.detected_bots = {}
        

    def pixel_to_world(self, pixel_x, pixel_y):

        center_pixel = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)  # shape (1,1,2)
        world_pt = cv.perspectiveTransform(center_pixel, self.H_matrix)
        x_world, y_world = world_pt[0][0]
        return x_world, y_world

    def image_callback(self, msg):

        try:
            image = self.bridge.imgmsg_to_cv2(msg)
            # cv.namedWindow('arucos')
            # cv.resizeWindow('arucos',480,480)
            undistorted_image = cv.undistort(image,self.camera_matrix,self.dist_coeffs)
            image_gray = cv.cvtColor(undistorted_image,cv.COLOR_BGR2GRAY)

            corners , ids ,rejected = self.detector.detectMarkers(image_gray)
            # print(corners,ids)
            cv.aruco.drawDetectedMarkers(undistorted_image,corners,ids)
            for i ,marker_id in enumerate(ids.flatten()):
                # if marker_id in [1,3,5,7]:
                    # print(f'{marker_id} {corners[i][0]}')
                    pass

            
            # for i, marker_id in enumerate(ids.flatten()):
            #     if marker_id in self.world_coords_dict:
            #         for j,corner in enumerate(corners[i][0]):     # each marker has 4 corners
            #             # print(corner[0])
            #             self.pixel_matrix.append([corner[0], corner[1]])
            #             self.world_matrix.append(self.world_coords_dict[marker_id][j])
                    
            self.pixel_matrix = np.array(self.pixel_matrix, dtype=np.float32)
            self.world_matrix = np.array(self.world_matrix, dtype=np.float32)
            
            self.H_matrix, _ = cv.findHomography(self.pixel_matrix, self.world_matrix)

            # print(self.H_matrix)
            rvecs , tvecs , _ = cv.aruco.estimatePoseSingleMarkers(corners,self.bots_marker_length,self.camera_matrix,self.dist_coeffs)

            for i ,marker_id in enumerate(ids.flatten()):
                if marker_id in [1,3,5,7]:
                    continue

                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:,0])
                center_y = np.mean(marker_corners[:,1])

                x_world, y_world = self.pixel_to_world(center_x, center_y)
                # print(marker_id,x_world,y_world)
                
                rmat,jac = cv.Rodrigues(rvecs[i])
                self.yaw = math.atan2(rmat[1,0],rmat[0,0])
                # self.yaw = self.yaw % (2 * math.pi)
                # print(self.yaw)
                # self.yaw = math.degrees(self.yaw) % 360.0
                # if self.yaw < 0:
                #     self.yaw += math.pi
                #     print(self.yaw)
                # self.y = self.yaw
                self.yaw = math.degrees(self.yaw)
                self.yaw = int(self.yaw)

                # if self.yaw < 0:
                #     if abs(self.yaw) < 3:
                #         self.yaw = 0
                #     else: 
                #         self.yaw += 360
                # if self.yaw < 0:    
                #     self.yaw += 360
                print(self.yaw)



                cv.putText(undistorted_image,
                           (f'x:{x_world:.2f},y:{y_world:.2f},yaw:{self.yaw:.2f}'),
                           (int(center_x+20),int(center_y-20)),
                           cv.FONT_HERSHEY_COMPLEX,
                           0.5,
                           (0,0,255),2)
                
                # print(self.yaw)
                # if marker_id == 9 :
                #     self.detected_bots[marker_id] = (x_world, y_world, self.yaw)
                # else:
                #     self.detected_crates[marker_id] = (x_world, y_world, self.yaw)
                self.detected_bots[marker_id] = (x_world, y_world, self.yaw)

            # print(self.detected_crates)
            # self.publish_crate_poses()
            self.publish_bot_poses()
                # print(self.detected_bots)


            # # for corner in corners

            # self.world_matrix = []  
            # self.pixel_matrix = []
            # cv.imshow('arucos', undistorted_image)
            # plt.imshow(undistorted_image)
            # plt.show()
            # cv.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_crate_poses(self):

        poses_msg = Poses2D()

        # Publish all detected crates
        for detected_id, (x, y, yaw) in self.detected_crates.items():
            crate_pose = Pose2D()
            # print(detected_id,x,y,yaw)

            crate_pose.id = int(detected_id)
            crate_pose.x = float(x)
            crate_pose.y = float(y)
            crate_pose.w = float(yaw)
            poses_msg.poses.append(crate_pose)

        self.crate_poses_pub.publish(poses_msg)

    def publish_bot_poses(self):

        poses_msg = Poses2D()

        # # Publish all detected crates
        for detected_id, (x, y, yaw) in self.detected_bots.items():
            bot_pose = Pose2D()
            # print(detected_id,x,y,yaw)
            bot_pose.id = int(detected_id)
            bot_pose.x = float(x)
            bot_pose.y = float(y)
            bot_pose.w = float(yaw)
            poses_msg.poses.append(bot_pose)

        self.bot_poses_pub.publish(poses_msg)

def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()