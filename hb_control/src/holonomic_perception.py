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
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D


class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        self.aruco_dict_name = 'DICT_4X4_50'  # Choose ArUco dictionary
        
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = np.array([
                [1030.4890823364258, 0.0, 960.0],
                [0.0, 1030.489103794098, 540.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)

        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32) 
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = [[446.0, 27.0],[1474.0,26.0],[445.0,1055.0],[1475.0, 1055.0]]  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = [[0,0],[2438.4, 0],[0, 2438.4],[2438.4, 2438.4]]  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
     
 
 # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        # self.aruco_dict = cv2.aruco.getPredefinedDictionary(?)
        # self.aruco_params = cv2.aruco.DetectorParameters()
        # self.detector = cv2.aruco.ArucoDetector(?, ?)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        
        # self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # self.aruco_params.minMarkerPerimeterRate = 0.005
        # self.aruco_params.adaptiveThreshWinSizeMax = 43   # increased

        # self.aruco_params.maxMarkerPerimeterRate = 6.0

        # self.aruco_params.polygonalApproxAccuracyRate = 0.11

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.x_real = np.array([1218.84,298.2,498.2, 1219.2, 1219.2,819.2]) 
        self.x_est = np.array([1219.2,274.46,478.98, 1218.86, 1218.85, 808.45]) 
        self.y_real = np.array([205.0, 1031.2, 831.2, 1219.2, 719.2, 1019.2]) 
        self.y_est = np.array([169.4, 1027.31, 821.49, 1219.41, 707.28, 1014.27])
        self.a_x , self.b_x = np.polyfit(self.x_est,self.x_real,1)
        self.a_y , self.b_y = np.polyfit(self.y_est,self.y_real,1)

        self.get_logger().info('PoseDetector initialized')

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        - Calculate the H_matrix using: use cv2.findHomography
        - Convert the pixel coordinates into real world coordinates using: cv2.perspectiveTransform(src_pts, self.H_matrix)
        """
        # Implement pixel to world coordinate conversion
        # Step 1: Ensure H_matrix is computed
        # Step 2: Create pixel point in correct format for cv2.perspectiveTransform
        # Step 3: Apply transformation and return world coordinates
        if self.H_matrix is None:
            print("Error: Homography matrix not computed yet!")
            return None, None

       
        pixel_pt = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)

        
        world_pt = cv2.perspectiveTransform(pixel_pt, self.H_matrix)  

  
        x_world, y_world = world_pt[0][0]

        x_world = self.a_x*x_world + self.b_x
        y_world = self.a_y*y_world + self.b_y

        return x_world, y_world

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Main Steps:
        1) Convert ROS Image -> cv image using CvBridge
        2) Undistort the image using camera intrinsics
        3) Detect all the markers in the world (cv2.aruco.drawDetectedMarkers)
        4) Derive the Pixel Matrix and the World Matrix using Corner Markers
        5) Compute the Homography Matrix (cv2.findHomography)
        5) Convert center pixel of crates marker and bot markers to world coordinates
        6) Using OpenCV calculate the yaw angle of each marker (cv2.aruco.estimatePoseSingleMarkers)
        7) Convert the yaw angle as per the new coordinate system
        8) Publish the bot pose and crate poses using the given custom message type
        """
        try:
            # Step 1: Convert ROS Image -> cv image using CvBridge
            # Use self.bridge.imgmsg_to_cv2() to convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Step 2: Undistort the image using camera intrinsics
            # Use cv2.undistort() with camera_matrix and dist_coeffs
            # Convert to grayscale for marker detection

            
            undistorted = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            


            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            gray = clahe.apply(gray)
           
 




            

            
            # Step 3: Detect all the markers in the world
            # Use self.detector.detectMarkers() to find ArUco markers
            # Use cv2.aruco.drawDetectedMarkers() to visualize detected markers

            corners, ids, rejected = self.detector.detectMarkers(gray)
            
            if ids is not None:
               cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)
            

            
            # Step 4: Derive the Pixel Matrix and the World Matrix using Corner Markers
            # Identify corner markers (IDs 1, 3, 5, 7)
            # Extract their pixel coordinates and map to known world coordinates

          
            
            self.pixel_matrix = np.array(self.pixel_matrix, dtype=np.float32)
            self.world_matrix = np.array(self.world_matrix, dtype=np.float32)
            
            # Optional: Compute homography
            
            
            # Step 5: Compute the Homography Matrix
            # Use cv2.findHomography() with pixel and world points
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 70, 0.0003)
            self.pixel_matrix = cv2.cornerSubPix(gray, self.pixel_matrix, (10,10), (-1,-1), criteria)
       
            self.H_matrix, status = cv2.findHomography(self.pixel_matrix, self.world_matrix,cv2.RANSAC, 0.5)

            # Step 6: Convert center pixel of markers to world coordinates
            # For each detected marker (excluding corner markers):
            #       - Calculate center pixel coordinate
            #       - Use pixel_to_world() to convert to world coordinates
            
            marker_world_coords = {}
            if ids is not None:
                for i ,marker_id in enumerate(ids.flatten()):
                    marker_id = ids[i][0]

                
                    if marker_id in [1, 3, 5, 7]:
                        continue

                    pts = corners[i][0]  
                    center = pts.mean(axis=0)  # center pixel (x, y)

                    
                    x_w, y_w = self.pixel_to_world(center[0], center[1])

                
                    marker_world_coords[marker_id] = np.array([x_w, y_w])

                    cv2.circle(undistorted, (int(center[0]), int(center[1])), 5, (0,0,255), -1)
                    # cv2.putText(undistorted, f"{x_w:.1f}, {y_w:.1f}", (int(center[0])+10, int(center[1])),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)


            # Step 7: Calculate yaw angle of each marker
            # Use cv2.aruco.estimatePoseSingleMarkers() or any other method to get rotation vectors
            # If you are going ahead with it, convert rotation vector to rotation matrix using cv2.Rodrigues()
            # Extract yaw angle from rotation matrix
             
                        
            

                            
            
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.bots_marker_length, self.camera_matrix, self.dist_coeffs)
                    if marker_id in [1,3,5,7]:  
                        continue
                    rvec = rvecs[i][0]  
                    tvec = tvecs[i][0]
                    
                    R, _ = cv2.Rodrigues(rvec)

                    yaw = math.atan2(R[1,0],R[0,0])
                

                    # yaw_deg = (math.degrees(yaw) + 360) % 360

                    center = corners[i][0].mean(axis=0)

                    cv2.putText(
                        undistorted,
                        f"X: {x_w:.2f}, Y: {y_w:.2f}, Yaw: {yaw:.2f} deg",
                        (int(center[0]) + 10, int(center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2
                    )

                    if marker_id==0:
                        bot_pose={
                        marker_id: (x_w, y_w, yaw),
                        }
                        self.publish_bot_poses(bot_pose)
                    else:    
                        crate_pose={
                        marker_id: (x_w, y_w, yaw),
                        }
                        self.publish_crate_poses(crate_pose)
                  
                    



            

            
            # Step 8: Separate and publish poses
            # Create separate dictionaries for bot_poses and crate_poses
            # Call publish_crate_poses() and publish_bot_poses()
            
            
        
            # Display the image with detected markers
            # cv2.imshow('Detected Markers', undistorted_image)
            # cv2.waitKey(1)
            
            cv2.namedWindow("Gray Image", cv2.WINDOW_NORMAL)  
            cv2.resizeWindow("Gray Image", 1820, 1000)
            cv2.imshow("Gray Image", undistorted)
            cv2.waitKey(1)
            pass
            
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
        # Publish the messag
        msg = Poses2D()
        
        for marker_id, (x, y, w) in poses.items():
            
            pose = Pose2D()
            pose.id = int(marker_id)
           
            pose.x = float(x)
            pose.y = float(y)
            pose.w = float(w)
            msg.poses.append(pose)
        self.crate_poses_pub.publish(msg)
        pass

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

        msg = Poses2D()
        for marker_id, (x, y, w) in poses.items():
            pose = Pose2D()
            pose.id = int(marker_id)
            pose.x = float(x)
            pose.y = float(y)
            pose.w = float(w)
            msg.poses.append(pose)
        self.bot_poses_pub.publish(msg)

        pass

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
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()