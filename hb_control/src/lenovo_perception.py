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
        # self.image_sub = self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = np.array([
                [1410.750171 ,0.000000 ,804.027477],
                [0.000000 ,1405.828447, 535.979552],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)

        self.dist_coeffs = np.array([0.038664 ,-0.088372 ,-0.006480 ,-0.024568 ,0.000000]) 
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = [[446.0, 27.0],[1474.0,26.0],[445.0,1055.0],[1475.0, 1055.0]]  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = [[0,0],[2438.4, 0],[0, 2438.4],[2438.4, 2438.4]]  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
     
 
 # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 1. Allow for tiny markers (smaller than 0.5% of the frame)
        # self.aruco_params.minMarkerPerimeterRate = 0.001 

        # # 2. Make the search grid much finer
        # self.aruco_params.adaptiveThreshWinSizeMin = 3
        # self.aruco_params.adaptiveThreshWinSizeMax = 23
        # self.aruco_params.adaptiveThreshWinSizeStep = 2  # CRITICAL: Finer steps

        # # 3. Increase the threshold constant to ignore small image noise
        # self.aruco_params.adaptiveThreshConstant = 10 

        # # 4. Critical for small markers: Subpixel refinement
        # self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # self.aruco_params.cornerRefinementWinSize = 3
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        

        self.x_real_1 = np.array([1218.84,298.2,498.2, 1219.2, 1219.2,819.2,869.2]) 
        self.x_est_1 = np.array([1219.2,274.46,478.98, 1218.86, 1218.85, 808.45,864.94]) 
        self.y_real_1 = np.array([205.0, 1031.2, 831.2, 1219.2, 719.2, 1019.2,]) 
        self.y_est_1 = np.array([169.4, 1027.31, 821.49, 1219.41, 707.28, 1014.27,])
        self.a_x_1 , self.b_x_1 = np.polyfit(self.x_est_1,self.x_real_1,1)
        self.a_y_1 , self.b_y_1 = np.polyfit(self.y_est_1,self.y_real_1,1)

        self.x_real_2 = np.array([2152.2,1582.11,1951.2,1914.2,1912.0]) 
        self.x_est_2 = np.array([2174.66,1569.2,1958.97,1924.2,1921.2]) 
        self.y_real_2 = np.array([724.2,219.2,667.2,940.0,1019.0]) 
        self.y_est_2 = np.array([711.35,169.37,660.18,925,1040.0])
        self.a_x_2 , self.b_x_2 = np.polyfit(self.x_est_2,self.x_real_2,1)
        self.a_y_2 , self.b_y_2 = np.polyfit(self.y_est_2,self.y_real_2,1)

        self.cap = cv2.VideoCapture(1)

        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 90)
        # self.cap.set(cv2.CAP_PROP_CONTRAST, 60)
        # self.cap.set(cv2.CAP_PROP_SATURATION, 50)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)

        
        self.image_callback()
        # self.timer = self.create_timer(0.3, self.image_callback) 
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

        if (x_world>=0 and x_world<=1219.2) and (y_world>=0 and y_world<=1219.2):
            
            x_world = self.a_x_1*x_world + self.b_x_1
            y_world = self.a_y_1*y_world + self.b_y_1

        if (x_world>1219.2 and x_world<=2438.4) and (y_world>=0 and y_world<=1219.2):
            
            x_world = self.a_x_2*x_world + self.b_x_2
            y_world = self.a_y_2*y_world + self.b_y_2

        return x_world, y_world


    def image_callback(self):
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
        while(True):    
            try:
                
                # Step 2: Undistort the image using camera intrinsics
                # Use cv2.undistort() with camera_matrix and dist_coeffs
                # Convert to grayscale for marker detection


                if not self.cap.isOpened():
                    exit()

                bool,cv_image =self.cap.read()



                # cv_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
                # cv_image = cv_image[:,70:555]

                # # 1️⃣ Brightness
                # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                # h, s, v = cv2.split(hsv)
                # v = cv2.add(v, 70)
                # hsv = cv2.merge((h, s, v))
                # bright = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

                # # 2️⃣ Grayscale
                # gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)

                # # 3️⃣ Contrast
                # clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4,4))
                # gray = clahe.apply(gray)


# fdddddddddddrfjdfchnicwehrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                # gray = cv2.GaussianBlur(gray, (3, 3), 0)
                # clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(4,4))
                # gray = clahe.apply(gray)
                # gray = cv2.equalizeHist(gray)

                # gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
                
            #     # Step 3: Detect all the markers in the world
            #     # Use self.detector.detectMarkers() to find ArUco markers
            #     # Use cv2.aruco.drawDetectedMarkers() to visualize detected markers

                corners, ids, rejected = self.detector.detectMarkers(gray)
                
                if ids is not None:
                   cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                

                
            #     # Step 4: Derive the Pixel Matrix and the World Matrix using Corner Markers
            #     # Identify corner markers (IDs 1, 3, 5, 7)
            #     # Extract their pixel coordinates and map to known world coordinates

            
                
            #     self.pixel_matrix = np.array(self.pixel_matrix, dtype=np.float32)
            #     self.world_matrix = np.array(self.world_matrix, dtype=np.float32)
                
            #     # Optional: Compute homography
                
                
            #     # Step 5: Compute the Homography Matrix
            #     # Use cv2.findHomography() with pixel and world points
            #     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 70, 0.0003)
            #     self.pixel_matrix = cv2.cornerSubPix(gray, self.pixel_matrix, (10,10), (-1,-1), criteria)
        
            #     self.H_matrix, status = cv2.findHomography(self.pixel_matrix, self.world_matrix,cv2.RANSAC, 0.5)

            #     # Step 6: Convert center pixel of markers to world coordinates
            #     # For each detected marker (excluding corner markers):
            #     #       - Calculate center pixel coordinate
            #     #       - Use pixel_to_world() to convert to world coordinates
                
            #     marker_world_coords = {}
            #     if ids is not None:
            #         for i ,marker_id in enumerate(ids.flatten()):
            #             marker_id = ids[i][0]

                    
            #             if marker_id in [1, 3, 5, 7]:
            #                 continue

            #             pts = corners[i][0]  
            #             center = pts.mean(axis=0)  # center pixel (x, y)

                        
            #             x_w, y_w = self.pixel_to_world(center[0], center[1])

                    
            #             marker_world_coords[marker_id] = np.array([x_w, y_w])

            #             cv2.circle(undistorted, (int(center[0]), int(center[1])), 5, (0,0,255), -1)
            #             # cv2.putText(undistorted, f"{x_w:.1f}, {y_w:.1f}", (int(center[0])+10, int(center[1])),
            #             #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)


            #     # Step 7: Calculate yaw angle of each marker
            #     # Use cv2.aruco.estimatePoseSingleMarkers() or any other method to get rotation vectors
            #     # If you are going ahead with it, convert rotation vector to rotation matrix using cv2.Rodrigues()
            #     # Extract yaw angle from rotation matrix
                
                            
                

                                
                
            #             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.bots_marker_length, self.camera_matrix, self.dist_coeffs)
            #             if marker_id in [1,3,5,7]:  
            #                 continue
            #             rvec = rvecs[i][0]  
            #             tvec = tvecs[i][0]
                        
            #             R, _ = cv2.Rodrigues(rvec)

            #             yaw = math.atan2(R[1,0],R[0,0])
                    

            #             # yaw_deg = (math.degrees(yaw) + 360) % 360

            #             center = corners[i][0].mean(axis=0)
                        
            #             if marker_id == 0:
            #                 cv2.putText(
            #                     undistorted,
            #                     f"X: {x_w:.2f}, Y: {y_w:.2f}, Yaw: {yaw:.2f}",
            #                     (int(center[0]) + 10, int(center[1])),
            #                     cv2.FONT_HERSHEY_SIMPLEX,
            #                     0.6,
            #                     (0, 0, 255),
            #                     2
            #                 )
            #             elif marker_id == 2:
            #                 cv2.putText(
            #                     undistorted,
            #                     f"X: {x_w:.2f}, Y: {y_w:.2f}, Yaw: {yaw:.2f}",
            #                     (int(center[0]) + 10, int(center[1]-20)),
            #                     cv2.FONT_HERSHEY_SIMPLEX,
            #                     0.6,
            #                     (0, 0, 255),
            #                     2
            #                 )
            #             elif marker_id == 4:
            #                 cv2.putText(
            #                     undistorted,
            #                     f"X: {x_w:.2f}, Y: {y_w:.2f}, Yaw: {yaw:.2f}",
            #                     (int(center[0]) + 10, int(center[1]+20)),
            #                     cv2.FONT_HERSHEY_SIMPLEX,
            #                     0.6,
            #                     (0, 0, 255),
            #                     2
            #                 )
            #             else:
            #                 cv2.putText(
            #                         undistorted,
            #                         f"X: {x_w:.2f}, Y: {y_w:.2f}, Yaw: {yaw:.2f}",
            #                         (int(center[0]) + 10, int(center[1])),
            #                         cv2.FONT_HERSHEY_SIMPLEX,
            #                         0.6,
            #                         (0, 0, 255),
            #                         2
            #                     )
                            
            #             if marker_id==0 or marker_id == 2 or marker_id == 4 :
            #                 bot_pose={
            #                 marker_id: (x_w, y_w, yaw),
            #                 }
            #                 self.publish_bot_poses(bot_pose)
            #             else:    
            #                 crate_pose={
            #                 marker_id: (x_w, y_w, yaw),
            #                 }
            #                 self.publish_crate_poses(crate_pose)
                    
                        



                

                
            #     # Step 8: Separate and publish poses
            #     # Create separate dictionaries for bot_poses and crate_poses
            #     # Call publish_crate_poses() and publish_bot_poses()
                
                
            
            #     # Display the image with detected markers
            #     # cv2.imshow('Detected Markers', undistorted_image)
            #     # cv2.waitKey(1)
                if bool:
                    cv2.namedWindow("Gray Image", cv2.WINDOW_NORMAL)
                    # display = cv2.resize(cv_image, (1280, 1280))
                    # cv2.imshow("Gray Image", display)
                    cv2.resizeWindow("Gray Image", 1020, 1000)
                    cv2.imshow("Gray Image", cv_image)
                if cv2.waitKey(1)==ord('q'):
                    exit()

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