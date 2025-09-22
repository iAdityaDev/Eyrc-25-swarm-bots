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
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crates_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = camera_matrix = np.array([
                                [1030.4891,    0.0,    960.0],
                                [   0.0,    1030.4891, 540.0],
                                [   0.0,       0.0,      1.0]
                            ], dtype=np.float32)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = []  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = []  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        self.world_coords_dict = {
            1: [[0, 0], [50, 0], [50, 50], [0, 50]],
            3: [[2438.4-50, 0], [2438.4, 0], [2438.4, 50], [2438.4-50, 50]],
            5: [[0, 2438.4-50], [50, 2438.4-50], [50, 2438.4], [0, 2438.4]],
            7: [[2438.4-50, 2438.4-50], [2438.4-50, 2438.4],[2438.4-50, 2438.4], [2438.4, 2438.4]]
        }
 
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        self.aruco_dict = cv.aruco.getPredefinedDictionary(self.aruco_dict_name)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.aruco_dict,self.aruco_params)

        self.detected_crates = {}
        self.detected_bots = {}
        

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        - Calculate the H_matrix using: use cv2.findHomography
        - Convert the pixel coordinates into real world coordinates using: cv2.perspectiveTransform(src_pts, self.H_matrix)
        """
        # Implement pixel to world coordinate conversion
        # Step 1: Ensure H_matrix is computed
        # Step 2: Create pixel point in correct format for cv2.perspectiveTransform
        # Step 3: Apply transformation and return world coordinates
        center_pixel = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)  # shape (1,1,2)
        world_pt = cv.perspectiveTransform(center_pixel, self.H_matrix)
        x_world, y_world = world_pt[0][0]
        return x_world, y_world

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Main Steps:
        1) Convert ROS Image -> cv image using CvBridge
        2) Undistort the image using camera intrinsics
        3) Detect all the markers in the world (cv2.aruco.drawDetectedM arkers)
        4) Derive the Pixel Matrix and the World Matrix using Corner Markers
        5) Compute the Homography Matrix (cv2.findHomography)
        5) Convert center pixel of crates marker and bot markers to world coordinates
        6) Using OpenCV calculate the yaw angle of each marker (cv2.aruco.estimatePoseSingleMarkers)
        7) Convert the yaw angle as per the new coordinate system
        8) Publish the bot pose and crate poses using the given custom message type
        """
        # try:
            # Step 1: Convert ROS Image -> cv image using CvBridge
            # Use self.bridge.imgmsg_to_cv2() to convert ROS image to OpenCV format
            
            # Step 2: Undistort the image using camera intrinsics
            # Use cv2.undistort() with camera_matrix and dist_coeffs
            # Convert to grayscale for marker detection
            
            # Step 3: Detect all the markers in the world
            # Use self.detector.detectMarkers() to find ArUco markers
            # Use cv2.aruco.drawDetectedMarkers() to visualize detected markers
            
            # Step 4: Derive the Pixel Matrix and the World Matrix using Corner Markers
            # Identify corner markers (IDs 1, 3, 5, 7)
            # Extract their pixel coordinates and map to known world coordinates
            
            # Step 5: Compute the Homography Matrix
            # Use cv2.findHomography() with pixel and world points
            
            # Step 6: Convert center pixel of markers to world coordinates
            # For each detected marker (excluding corner markers):
            #       - Calculate center pixel coordinate
            #       - Use pixel_to_world() to convert to world coordinates
            
            # Step 7: Calculate yaw angle of each marker
            # Use cv2.aruco.estimatePoseSingleMarkers() or any other method to get rotation vectors
            # If you are going ahead with it, convert rotation vector to rotation matrix using cv2.Rodrigues()
            # Extract yaw angle from rotation matrix
            
            # Step 8: Separate and publish poses
            # Create separate dictionaries for bot_poses and crate_poses
            # Call publish_crate_poses() and publish_bot_poses()
            
            # Display the image with detected markers
            # cv2.imshow('Detected Markers', undistorted_image)
            # cv2.waitKey(1)
            
        #     pass
            
        # except Exception as e:
        #     self.get_logger().error(f'Error processing image: {str(e)}')

        try:
            image = self.bridge.imgmsg_to_cv2(msg)

            undistorted_image = cv.undistort(image,self.camera_matrix,self.dist_coeffs)
            image_gray = cv.cvtColor(undistorted_image,cv.COLOR_BGR2GRAY)

            corners , ids ,rejected = self.detector.detectMarkers(image_gray)
            # print(corners,ids)
            cv.aruco.drawDetectedMarkers(undistorted_image,corners,ids)

            
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.world_coords_dict:
                    for j,corner in enumerate(corners[i][0]):  # each marker has 4 corners
                        self.pixel_matrix.append([corner[0], corner[1]])
                        self.world_matrix.append(self.world_coords_dict[marker_id][j])
                    
            self.pixel_matrix = np.array(self.pixel_matrix, dtype=np.float32)
            self.world_matrix = np.array(self.world_matrix, dtype=np.float32)
            
            self.H_matrix, _ = cv.findHomography(self.pixel_matrix, self.world_matrix)

            # print(self.H_matrix)

            for i ,marker_id in enumerate(ids.flatten()):
                if marker_id in [1,3,5,7]:
                    continue

                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:,0])
                center_y = np.mean(marker_corners[:,1])

                x_world, y_world = self.pixel_to_world(center_x, center_y)
                print(marker_id,x_world,y_world)

            cv.imshow('Detected Markers', undistorted_image)
            cv.waitKey(1)

            self.pixel_matrix = []
            self.world_matrix = []  

            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_crate_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.crate_poses_pub.publish(msg)
        """
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message
        poses_msg = Poses2D()

        # Publish all detected crates
        for detected_id, (x, y, yaw) in self.detected_crates.items():
            crate_pose = Pose2D()
            crate_pose.id = detected_id
            crate_pose.x = x
            crate_pose.y = y
            crate_pose.yaw = yaw
            poses_msg.poses.append(crate_pose)

        self.crate_poses_pub.publish(poses_msg)

    def publish_bot_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.bot_poses_pub.publish(msg)
        """
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message
        poses_msg = Poses2D()

        # Publish all detected crates
        for detected_id, (x, y, yaw) in self.detected_bot.items():
            bot_pose = Pose2D()
            bot_pose.id = detected_id
            bot_pose.x = x
            bot_pose.y = y
            bot_pose.yaw = yaw
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