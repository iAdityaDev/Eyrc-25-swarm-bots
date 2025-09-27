#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose
from hb_interfaces.msg import Pose2D, Poses2D
import numpy as np
import math

class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out   # velocity cap 
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        # print(error,dt)
        self.integral += error*dt
        derivative = (error-self.prev_error)/dt
        # print(self.integral)
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        output = max(min(output, self.max_out), -self.max_out)
        # print(error,derivative,self.integral,output)

        return output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node
        self.get_logger().info('HolonomicPIDController is created')

        self.bot_pose = self.create_subscription(Poses2D, 
                                                 "/bot_pose", 
                                                 self.pose_cb,
                                                 10)
        
        self.publisher = self.create_publisher(Float64MultiArray, 
                                               '/forward_velocity_controller/commands',
                                               10)

        self.current_pose_bot = None
        self.last_time = 0.0
        self.max_vel = 2.0
        self.goal_reached = False
        self.current_goal_wp = 0
        self.alpha1 = math.radians(30)
        self.alpha2 = math.radians(150)
        self.alpha3 = math.radians(270)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        self.A = np.array([
            [np.cos(self.alpha1 + np.pi/2), np.cos(self.alpha2 + np.pi/2), np.cos(self.alpha3 + np.pi/2)],
            [np.sin(self.alpha1 + np.pi/2), np.sin(self.alpha2 + np.pi/2), np.sin(self.alpha3 + np.pi/2)],
            [1, 1, 1]
        ])


        self.goals = [
            (700, 800, 0),
            (700, 1400, 0),
            (1500, 1400, 0),
            (1500, 800, 0),
            (700, 800, 0),
        ]
        # print(type(self.goals),self.goals[3])
        self.target_x,self.target_y,self.target_yaw = self.goals[self.current_goal_wp]


        self.pid_params = {
            'x': {'kp': 0.8, 'ki': 0.00, 'kd': 0.6, 'max_out': self.max_vel},
            'y': {'kp': 0.8, 'ki': 0.00, 'kd': 0.6, 'max_out': self.max_vel},
            'theta': {'kp': 1.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])


        self.timer = self.create_timer(0.03, self.control_cb) 

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    def pose_cb(self, msg):
        for self.current_pose_bot in msg.poses:
            self.current_pose_bot_id = self.current_pose_bot.id
            self.current_pose_bot_x = self.current_pose_bot.x
            self.current_pose_bot_y = self.current_pose_bot.y
            self.current_pose_bot_yaw = self.current_pose_bot.w

    def control_cb(self):

        if self.current_pose_bot is None:
            return

        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time) / 1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        if not self.goal_reached:
            error_x = self.target_x-self.current_pose_bot_x
            error_y = self.target_y-self.current_pose_bot_y
            error_yaw = self.target_yaw-self.current_pose_bot_yaw
            print(error_x,error_y,error_yaw)
            pid_x = self.pid_x.compute(error_x,dt)
            pid_y = self.pid_y.compute(error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)


            if abs(error_x) < 15 and abs(error_y)< 15 and abs(error_yaw)<0.1:
                self.goal_reached = True
            
            pose = np.array([pid_x,pid_y,pid_yaw])
            s_linalg = np.linalg.solve(self.A, pose)

            wheel_velocities = [s_linalg[0],s_linalg[1],s_linalg[2]]
            #  1 blue 
            # 2 red 
            # 3 green
            self.publish_wheel_velocities(wheel_velocities)

        print(self.goal_reached)

        if self.goal_reached:
            self.get_logger().info('changign to next goal')
            self.goal_reached = False
            self.current_goal_wp += 1
            if self.current_goal_wp == 4:
                self.get_logger().info('all th points reached')
            print(self.current_goal_wp)
            self.target_x,self.target_y,self.target_yaw = self.goals[self.current_goal_wp]
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()

        # Goal check


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = Float64MultiArray()
        msg.data = np.array(wheel_vel).tolist()
        self.publisher.publish(msg)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
