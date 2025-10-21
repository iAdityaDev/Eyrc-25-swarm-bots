#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hb_interfaces.msg import Pose2D, Poses2D
from hb_interfaces.msg import BotCmdArray , BotCmd
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
        derivative = (error-self.prev_error)/dt
        self.integral += error*dt
        self.output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        # print(error,self.prev_error)
        self.prev_error = error
        # self.output = max(min(self.output, self.max_out), -self.max_out)

        return self.output
    def print(self):
        print(self.output)
    
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
        
        self.crate_pose = self.create_subscription(Poses2D, 
                                                 "/crate_pose", 
                                                 self.pose_crate_cb,
                                                 10)
        
        self.publisher = self.create_publisher(BotCmdArray, 
                                               '/bot_cmd',
                                               10)

        self.current_pose_bot = None
        self.current_pose_crate = None
        self.goals = None
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
            [0.185,0.185,0.185]
        ])
        # self.A = np.array([
        #     [np.cos(self.alpha1), np.cos(self.alpha2), np.cos(self.alpha3)],
        #     [np.sin(self.alpha1), np.sin(self.alpha2), np.sin(self.alpha3)],
        #     [1, 1, 1]
        # ])
        # self.goals = [
        #     (700, 800, 0),
        #     (700, 1400, 0),
        #     (1500, 1400, 0),
        #     (1500, 800, 0),
        #     (700, 800, 0),
        # ]
        # self.goals = [
        #     (820, 920, 0),
        #     (820, 1520, 0),
        #     (1620, 1520, 0),
        #     (1620, 920, 0),
        #     (820, 920, 0),
        # ]
        # print(type(self.goals),self.goals[3])


        self.pid_params = {
            'x': {'kp': 0.55, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'y': {'kp': 0.55, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'theta': {'kp': 0.25, 'ki': 0.00, 'kd': 0.25, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])


        self.timer = self.create_timer(0.3, self.control_cb) 

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    def pose_cb(self, msg):
        for self.current_pose_bot in msg.poses:
            self.current_pose_bot_id = self.current_pose_bot.id
            self.current_pose_bot_x = self.current_pose_bot.x
            self.current_pose_bot_y = self.current_pose_bot.y
            self.current_pose_bot_yaw = self.current_pose_bot.w

    def pose_crate_cb(self,msg):
        for self.current_pose_crate in msg.poses:
            self.current_pose_crate_id = self.current_pose_crate.id
            self.current_pose_crate_x = self.current_pose_crate.x
            self.current_pose_crate_y = self.current_pose_crate.y
            self.current_pose_crate_yaw = self.current_pose_crate.w
            # print(self.current_pose_crate_id,self.current_pose_crate_x,self.current_pose_crate_y,self.current_pose_crate_yaw)
        
        if self.goals == None:
            self.goals = [(self.current_pose_crate_x,self.current_pose_crate_y,self.current_pose_crate_yaw)]
            self.target_x,self.target_y,self.target_yaw = self.goals[0]

    def control_cb(self):

        if (self.current_pose_bot or self.current_pose_crate) is None:
            return

        if self.goals is None:
            return 
        
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time)/1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        if not self.goal_reached:
            error_x = self.target_x-self.current_pose_bot_x
            error_y = self.target_y-self.current_pose_bot_y
            error_yaw = self.target_yaw-self.current_pose_bot_yaw
            # error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi    
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            print(error_x,error_y,error_yaw)
            # print(error_x,error_y,error_yaw)
            pid_x = self.pid_x.compute(error_x,dt)
            pid_y = self.pid_y.compute(error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)
            # self.pid_x.print()
            # self.pid_y.print()
            # self.pid_yaw.print()


            if abs(error_x) < 15 and abs(error_y)< 15 and abs(error_yaw)<0.10:
                self.goal_reached = True
            

            pose = np.array([pid_x,pid_y,pid_yaw])
            s_linalg = np.linalg.solve(self.A, pose)
            wheel_velocities = [s_linalg[0],s_linalg[1],s_linalg[2]]
            #  1 blue 
            # 2 red 
            # 3 green
        if self.goal_reached:
            self.get_logger().info('changign to next goal')
            self.goal_reached = False
            self.current_goal_wp += 1
            if self.current_goal_wp == 1:
                self.get_logger().info('all th points reached')
                wheel_velocities = [0.0, 0.0, 0.0,]
            if self.current_goal_wp < len(self.goals):
                self.target_x,self.target_y,self.target_yaw = self.goals[self.current_goal_wp]
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
        self.publish_wheel_velocities(wheel_velocities)


    def publish_wheel_velocities(self, wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = 0
        cmd.m1 = wheel_vel[0]
        cmd.m2 = wheel_vel[1]
        cmd.m3 = wheel_vel[2]
        cmd.base = 90.0
        cmd.elbow = 90.0
        msg.cmds = [cmd]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()