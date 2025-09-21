#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose
import ast 
import math 
import time

class BattalionController(Node):
    def __init__(self):
        super().__init__('battalion_controller')
        self.get_logger().info('battalion node is created')

        
        self.cli_coordinates = self.create_client(Trigger, '/eyantrasim/get_coordinates')
        while not self.cli_coordinates.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the coordinates')
        self.req = Trigger.Request()


        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)        
        self.pub_crystal = self.create_publisher(Twist, '/eyantrasim/crystal/cmd_vel', 10)
        self.pub_frostbite = self.create_publisher(Twist, '/eyantrasim/frostbite/cmd_vel', 10)

        
        self.sub_glacio = self.create_subscription(Pose, '/eyantrasim/glacio/pose', self.pose_glacio_cb, 10)
        self.sub_crystal = self.create_subscription(Pose, '/eyantrasim/crystal/pose', self.pose_crystal_cb, 10)
        self.sub_frostbite = self.create_subscription(Pose, '/eyantrasim/frostbite/pose', self.pose_frostbite_cb, 10)
    
        
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []

        self.current_pose_glacio = None
        self.current_pose_crystal = None
        self.current_pose_frostbite = None

        # self.prev_timee = time.time()
        self.prev_dist_error = 0.0
        self.prev_yaw_error = 0.0
        self.kp_linear = 0.8
        self.kd_linear = 0.2
        self.kp_angular = 1.6
        self.kd_angular = 0.7

        self.target_reached = False
        self.glacio_reached = False
        self.crystal_reached = False
        self.frostbite_reached = False
        self.current_wp_glacio = 0
        self.current_wp_crystal = 1
        self.current_wp_frostbite = 1
        
        self.prev_time = {}
        self.prev_dist_error = {}
        self.prev_yaw_error = {}


        self.get_targets()
        
        self.timer = self.create_timer(0.1, self.control_loop)

    
    def get_targets(self):
    
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
            elif (i>49 and i<100) :
                self.targets_crystal.append(coordinate)
            else :
                self.targets_frostbite.append(coordinate)
# 
        print(len(self.targets_glacio))
        print(len(self.targets_crystal))
        # print((self.targets_crystal[49]))
        print(len(self.targets_frostbite))
        print((self.targets_frostbite[1]))

    # --- Pose callbacks ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_crystal_cb(self, msg):
        self.current_pose_crystal = msg

    def pose_frostbite_cb(self, msg):
        self.current_pose_frostbite = msg

    # --- Example controller ---
    def compute_velocity(self, bot , current_pose, target_pose):
        vel = Twist()
        current_x = current_pose.x*0.022222223
        current_y = current_pose.y*0.022222223
        current_yaw = current_pose.theta
        target_x,target_y = target_pose
        target_x = target_x*0.022222223
        target_y = target_y*0.022222223

        # print(current_yaw)

        # while current_yaw > math.pi:
        #     current_yaw -= 2 * math.pi    
        # while current_yaw < -math.pi:
        #     current_yaw += 2 * math.pi
        # print(current_yaw)
        error_x = target_x - current_x
        error_y = target_y - current_y
        target_yaw = math.atan2(error_y,error_x)
        
        dist_error = math.sqrt(error_x**2 + error_y**2)
        yaw_error = target_yaw - current_yaw

        # print('yaw_error',yaw_error)
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi    
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        # print(yaw_error)

        if self.current_wp_crystal == 0:
              self.target_reached = True


        current_time = time.time()
        prev_time = self.prev_time.get(bot, current_time - 0.1)
        dt = current_time - prev_time

        prev_dist_error = self.prev_dist_error.get(bot, 0.0)
        prev_yaw_error = self.prev_yaw_error.get(bot, 0.0)

        self.prev_time[bot] = current_time

        linear_cmd = self.kp_linear*dist_error + self.kd_linear*((dist_error-prev_dist_error)/dt)
        angular_cmd = self.kp_angular*yaw_error + self.kd_angular*((yaw_error-prev_yaw_error)/dt)
        
        # print(linear_cmd)

        self.prev_dist_error[bot] = dist_error
        self.prev_yaw_error[bot] = yaw_error

        vel.linear.x = linear_cmd
        vel.angular.z = angular_cmd
        
        self.target_reached = False
        if abs(error_x) < 0.01 and abs(error_y) < 0.01:
            # vel.linear.x = 0.0
            # vel.angular.z = 0.0
            self.target_reached = True
            # print('done'

        return vel , self.target_reached

    # --- Control loop for all bots ---
    def control_loop(self):
        if self.current_pose_glacio and self.targets_glacio and self.current_wp_glacio < len(self.targets_glacio):

            vel_glacio,self.glacio_reached = self.compute_velocity("glacio",self.current_pose_glacio,self.targets_glacio[self.current_wp_glacio])
            if self.glacio_reached:
                self.glacio_reached = False 
                self.current_wp_glacio += 1 
                # print(self.current_wp_glacio)
                
            self.pub_glacio.publish(vel_glacio)

        if self.current_pose_crystal and self.targets_crystal and self.current_wp_crystal < len(self.targets_crystal):

            vel_crystal,self.crystal_reached = self.compute_velocity("crystal",self.current_pose_crystal,self.targets_crystal[self.current_wp_crystal])
            if self.crystal_reached:
                self.crystal_reached = False 
                self.current_wp_crystal += 1 
                # print(self.current_wp_crystal)
                
            self.pub_crystal.publish(vel_crystal)

        if self.current_pose_frostbite and self.targets_frostbite and self.current_wp_frostbite < len(self.targets_frostbite):

            vel_frostbite,self.frostbite_reached = self.compute_velocity("frostbite",self.current_pose_frostbite,self.targets_frostbite[self.current_wp_frostbite])
            if self.frostbite_reached:
                self.frostbite_reached = False 
                self.current_wp_frostbite += 1 
                # print(self.current_wp_frostbite)
                
            self.pub_frostbite.publish(vel_frostbite)


def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()