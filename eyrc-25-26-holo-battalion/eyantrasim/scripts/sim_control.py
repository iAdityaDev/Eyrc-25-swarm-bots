#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose
import ast 
import math 

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

        self.kp_linear = 0.1
        self.kd_linear = 0.0
        self.kp_angular = 0.5
        self.kd_angular = 0.0
        self.dt = 0.1
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_yaw = 0.0
        self.prev_distance2target = 0.0

        self.position_tolerance = 1.0  # 10 units tolerance for reaching target
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.5

        self.current_waypoint_glacio = 0
        self.glacio_reached = False
        
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
            elif (i>49 and i<101) :
                self.targets_crystal.append(coordinate)
            else :
                self.targets_frostbite.append(coordinate)

    # --- Pose callbacks ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_crystal_cb(self, msg):
        self.current_pose_crystal = msg

    def pose_frostbite_cb(self, msg):
        self.current_pose_frostbite = msg

    # --- Example controller ---
    def compute_velocity(self, current_pose, target_pose):
        vel = Twist()
        current_x = current_pose.x
        current_y = current_pose.y
        current_yaw = current_pose.theta

        target_x , target_y = target_pose
        # print(f'current_x {current_x} current_y {current_y} current_yaw {current_yaw}')
        # print(f'traget_X {target_x} ,traget_y {target_y}')
   
        error_x = target_x - current_x  
        error_y = target_y - current_y 
        distance2target = math.sqrt(error_x**2 + error_y**2)
        # print(f'errro_x {error_x}, error_y {error_y},distance2target{distance2target}')

        desired_yaw = math.atan2(error_y, error_x)
        error_yaw = desired_yaw - current_yaw

        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi    
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
    
        # print(error_yaw)
        # print(distance2target)
        linear_cmd = self.kp_linear*error_x
        print(linear_cmd)
        # self.prev_distance2target = distance2target
        # # print(linear_cmd)

        angular_cmd = self.kp_angular*error_yaw
        self.prev_error_yaw = error_yaw
        # # print(angular_cmd)

        # # print(linear_cmd)

        vel.linear.x = linear_cmd
        vel.angular.z = angular_cmd

        if error_x == 0 and error_y == 0 :
            self.glacio_reached = True
            return vel , self.glacio_reached

        # vel.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_cmd))
        # vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_cmd))



        # if distance2target < self.position_tolerance:
        #     vel.linear.x=0.0
        #     vel.angular.z=0.0
        #     self.glacio_reached = True

        # if int(current_x) == target_x and int(current_y) == target_y:
        #     vel.linear.x = 0.0
        #     vel.angular.z = 0.0

        # vel.linear.x = -0.5
        # vel.linear.y = 0.5
        # vel.angular.z = 1.0
        # error = target_pose - current_pose
        # derivate = (error-self.previous_error)/dt

        # Kp = 1
        # Kd = 1
        # vel.linear.x = 0.5

        return vel

    # --- Control loop for all bots ---
    def control_loop(self):
        # if self.current_pose_glacio and self.targets_glacio:
            
            if not self.glacio_reached:
                print('in the if')
                target_pose = self.targets_glacio[self.current_waypoint_glacio]
                vel_glacio = self.compute_velocity(self.current_pose_glacio,target_pose)
                self.current_waypoint_glacio += 1 
                self.glacio_reached = False
                self.pub_glacio.publish(vel_glacio)

            # vel_crystal = self.compute_velocity(self.current_pose_crystal, self.targets_crystal[0])
            # vel_frostbite = self.compute_velocity(self.current_pose_frostbite, self.targets_frostbite[0])
            # self.pub_crystal.publish(vel_crystal)
            # self.pub_frostbite.publish(vel_frostbite)

def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()