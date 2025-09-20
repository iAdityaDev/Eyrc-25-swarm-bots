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


        self.kp_linear = 0.02
        self.kd_linear = 0.01
        self.kp_angular = 0.3
        self.kd_angular = 0.1
        self.dt = 0.1
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_yaw = 0.0
        self.prev_distance2target = 0.0
        self.wp_glacio = 1
        self.glacio_reached = False
        self.min_err_glacio = 100000.0

        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.5
        
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

        print(self.targets_glacio[0])

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

        if self.min_err_glacio > distance2target:
            self.min_err_glacio = distance2target
        # print(self.min_err_glacio)

        # print(f'errro_x {error_x}, error_y {error_y},distance2target{distance2target}')

        desired_yaw = math.atan2(error_y, error_x)
        error_yaw = desired_yaw - current_yaw

        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi    
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
    
        # print(error_yaw)
        # print(distance2target)
        linear_cmd = self.kp_linear*distance2target + self.kd_linear*((distance2target-self.prev_distance2target )/self.dt)
        self.prev_distance2target = distance2target
        # print(distance2target)
        # print(linear_cmd)
        # # print(linear_cmd)

        angular_cmd = self.kp_angular*error_yaw + self.kd_angular*((error_yaw-self.prev_error_yaw)/self.dt)
        self.prev_error_yaw = error_yaw
        # print(error_yaw)
        # print(angular_cmd)

        # # print(linear_cmd)
        if linear_cmd > 0.5:
            linear_cmd = 0.5
     

        # print(distance2target)
        # # print('outside the error check')
        # if  distance2target < 0.1 and error_yaw < 0.08:
        #     # print('in the error check')
        #     self.glacio_reached = True
        #     return vel , self.glacio_reached

        # if error_yaw > 0.1:
        #     vel.linear.x = 0.0
        #     vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_cmd))
        # if distance2target > 1.0 :
        #      vel.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_cmd))
        #      vel.angular.z = 0.0

        if error_yaw > 0.01 : 
            vel.angular.z = angular_cmd
            vel.linear.x = 0.0 
    
        if error_yaw < 0.1 : 
            vel.angular.z = 0.0
            vel.linear.x = linear_cmd            

        if error_x < 0.1 and error_y < 0.1 :
            vel.linear.x = 0.0 
            vel.angular.z = 0.0 
            self.glacio_reached = True 
        
        
        # vel.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_cmd))
        # vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_cmd))

        # if int(current_x) == target_x and int(current_y) == target_y:
        #     vel.linear.x = 0.0
        #     vel.angular.z = 0.0
        # error = target_pose - current_pose
        # derivate = (error-self.previous_error)/dt
        return vel , self.glacio_reached

    

    # --- Control loop for all bots ---
    def control_loop(self):
        if self.current_pose_glacio and self.targets_glacio:
            # print('in the if')
            vel , self.glacio_reached = self.compute_velocity(self.current_pose_glacio, self.targets_glacio[self.wp_glacio])
            # print(self.glacio_reached)

            if self.glacio_reached : 
                print('in the second if')
                self.wp_glacio += 1
                print(self.wp_glacio)
                self.glacio_reached = False

            self.pub_glacio.publish(vel)

        # vel , self.glacio_reached = self.compute_velocity(self.current_pose_glacio, (570,350))
        # self.pub_glacio.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()