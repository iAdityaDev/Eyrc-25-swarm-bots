#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose
import ast 

class BattalionController(Node):
    def __init__(self):
        super().__init__('battalion_controller')
        self.get_logger().info('battalion node is created')

        # --- Service client (for targets) ---
        self.cli_coordinates = self.create_client(Trigger, '/eyantrasim/get_coordinates')
        while not self.cli_coordinates.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the coordinates')
        self.req = Trigger.Request()

        # --- Publishers (for bots) ---
        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)        
        self.pub_crystal = self.create_publisher(Twist, '/eyantrasim/crystal/cmd_vel', 10)
        self.pub_frostbite = self.create_publisher(Twist, '/eyantrasim/frostbite/cmd_vel', 10)

        # --- Subscribers (for bot poses) ---
        self.sub_glacio = self.create_subscription(Pose, '/eyantrasim/glacio/pose', self.pose_glacio_cb, 10)
        self.sub_crystal = self.create_subscription(Pose, '/eyantrasim/crystal/pose', self.pose_glacio_cb, 10)
        self.sub_frostbite = self.create_subscription(Pose, '/eyantrasim/frostbite/pose', self.pose_glacio_cb, 10)
    
        # --- Storage placeholders ---
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []

        self.current_pose_glacio = None
        self.current_pose_crystal = None
        self.current_pose_frostbite = None

        # Example: get target coordinates
        self.get_targets()

        # --- Timer for control loop ---
        self.timer = self.create_timer(0.1, self.control_loop)

    # --- Example method: request targets from service ---
    def get_targets(self):
        # call the ros2 service to get target coordinates
        future = self.cli_coordinates.call_async(self.req)
        future.add_done_callback(self.get_coordinates)
    
    def get_coordinates(self,future):

        self.all_coordinates = future.result()
        self.all_coordinates = ast.literal_eval(self.all_coordinates.message)
        # print((coordinates))

        for i,coordinate in enumerate(self.all_coordinates):
            # print(self.all_coordinates[i])
            
            if (i<50) :
                self.targets_glacio.append(coordinate)
            elif (i>49 and i<101) :
                self.targets_crystal.append(coordinate)
            else :
                self.targets_frostbite.append(coordinate)

        # print(self.all_coordinates)        
        # print(self.targets_frostbite)
        # print(self.targets_glacio)
        # print(self.targets_crystal)

    # --- Pose callbacks ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_crystal_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_frostbite_cb(self, msg):
        self.current_pose_glacio = msg

    # --- Example controller ---
    def compute_velocity(self, current_pose, target_pose):
        vel = Twist()
        # implement P/PD controller
        return vel

    # --- Control loop for all bots ---
    def control_loop(self):
        if self.current_pose_glacio and self.targets_glacio:
            vel = self.compute_velocity(self.current_pose_glacio, self.targets_glacio[0])
            self.pub_glacio.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()