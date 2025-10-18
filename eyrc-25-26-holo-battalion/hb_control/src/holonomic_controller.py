#!/usr/bin/env python3

'''
This Python file runs a ROS 2 node of name holonomic_pid_controller which holds the position of a holonomic robot
and drives it through a series of predefined goals using PID controllers on [x, y, θ].

This node publishes and subscribes to the following topics:

        PUBLICATIONS                               SUBSCRIPTIONS
        /forward_velocity_controller/commands      /bot_pose

Instead of defining separate variables for each PID axis, lists/dictionaries are used.
For example: pid_params['x'], pid_params['y'], pid_params['theta'], etc.

Code modularity and clarity are maintained to make tuning and extension easier.
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# import hb_interface messages
import numpy as np
import math

from hb_interfaces.msg import Pose2D, Poses2D
from hb_interfaces.msg import BotCmd, BotCmdArray


# ---------------------- PID Controller Class --------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
#-----------------------------PID Compute Steps--------------------------------------------------------------
        # 1. Accumulate the error over time for the Integral term
        # 2. Compute the change in error for the Derivative term
        # 3. Calculate the PID output:
        # 4. Store the current error for use in the next iteration
        # 5. Limit (clip) the output between [-max_out, +max_out] to avoid unsafe velocities
#------------------------------------------------------------------------------------------------------------

        derivative = (error-self.prev_error)/dt
        self.integral += error*dt
        self.output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        # print(error,self.prev_error)
        self.prev_error = error
        return self.output
    def print(self):
        print(self.output)
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        # ---------------- Robot Parameters ----------------
        # 1. Robot ID(s)
        # 2. Current pose of the robot:
        #    - Updated from the /bot_pose topic in the callback function.
        #    - Stores [x, y, θ] information for the active robot.
        # 3. Goal tracking index
        # 4. Timing information:
        #    - Used to calculate the time difference (dt) between control loop iterations.
        # 5. Threshold for goal completion:
        #    - Defines the acceptable error tolerance for x, y, and θ.
        #    - Example: if error < 5 units → goal considered reached.
        self.bot_pose = self.create_subscription(Poses2D, 
                                                 "/bot_pose", 
                                                 self.pose_cb,
                                                 10)
        
        self.bot_pose = self.create_subscription(Poses2D, 
                                                 "/crate_pose", 
                                                 self.pose_crate,
                                                 10)
        
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        # self.cmd_publisher = self.create_publisher(BotCmdArray, '/bot_command_Array', 10)
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

        # ---------------- Goal Definitions ----------------

        # List of waypoints [(x, y, yaw_deg)]
        self.goals = []
        

        #----------------DO NOT CHNAGE----------------------

        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.07, 'ki': 0.00, 'kd': 0.00, 'max_out': self.max_vel},
            'y': {'kp': 0.07, 'ki': 0.00, 'kd': 0.00, 'max_out': self.max_vel},
            'theta': {'kp': 0.07, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel * 2}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])

      

        
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    # ---------------- Subscriber Callback ----------------
    def pose_cb(self, msg):
        """
        Callback function for /bot_pose topic.
        This function is executed each time a message is received.

        Steps:
        1. Iterate through all poses in the incoming message.
        2.  Update self.current_pose with this robot’s pose.
        """

        for self.current_pose_bot in msg.poses:
            self.current_pose_bot_id = self.current_pose_bot.id
            self.current_pose_bot_x = self.current_pose_bot.x
            self.current_pose_bot_y = self.current_pose_bot.y
            self.current_pose_bot_yaw = self.current_pose_bot.w


           
     

    def pose_crate(self, msg):
        """
        Callback function for /bot_pose topic.
        This function is executed each time a message is received.

        Steps:
        1. Iterate through all poses in the incoming message.
        2.  Update self.current_pose with this robot’s pose.
        """

        for self.crate_pose in msg.poses:
            self.crate_id = self.crate_pose.id
            self.crate_x = self.crate_pose.x
            self.crate_y = self.crate_pose.y
            self.crate_yaw = self.crate_pose.w   

            self.goals=[(self.crate_id,self.crate_x,self.crate_y,self.crate_yaw)]     

    # ---------------- Control Loop ----------------
    def control_cb(self):

        """
        Control loop callback executed periodically by the ROS 2 timer.

        Main Steps:
        1. Check if the current pose is available; if not, exit.
        2. Compute the time difference (dt) since the last control cycle.
        3. Get the current robot pose (x, y, θ).
        4. If all goals are completed → stop the robot.
        5. Select the current goal (x, y, θ) from the goals list.
        6. Compute errors in x, y, and θ between current pose and goal.
        7. Use PID controllers to calculate required body velocities [vx, vy, ω].
        8. Convert body velocities into individual wheel velocities.
        9. Limit (clip) wheel velocities within safe bounds.
        10. Publish the wheel velocities to the motor controller.
        11. Check if the goal is reached:
              - If yes → update goal index, reset PIDs, and move to the next goal.
        """

        if self.current_pose_bot is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9  # float seconds

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        if dt <= 0:
            return
        self.last_time = now


        if not self.goal_reached:
            error_x = self.target_x-self.current_pose_bot_x
            print('self.target_x',self.target_x)
            print('self.current_pose_bot_x',self.current_pose_bot_x)
            error_y = self.target_y-self.current_pose_bot_y
            error_yaw = self.target_yaw-self.current_pose_bot_yaw
            # error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi    
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            # print(error_x,error_y,error_yaw)
            pid_x = self.pid_x.compute(error_x,dt)
            pid_y = self.pid_y.compute(error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)
            self.pid_x.print()
            self.pid_y.print()
            self.pid_yaw.print()


            if abs(error_x) < 2 and abs(error_y)< 2 and abs(error_yaw)<0.40:
                self.goal_reached = True
            

            pose = np.array([pid_x,pid_y,pid_yaw])
            A = np.array(self.A, dtype=float)
            b = np.array(pose, dtype=float)
            s_linalg = np.linalg.solve(A, b)
            wheel_velocities = [s_linalg[0],s_linalg[1],s_linalg[2]]
            #  1 blue 
            # 2 red 
            # 3 green
        if self.goal_reached:
            self.get_logger().info('changign to next goal')
            self.goal_reached = False
            self.current_goal_wp += 1
            if self.current_goal_wp == 5:
                self.get_logger().info('all th points reached')
                wheel_velocities = [0.0, 0.0 ,0.0]
                print('zero')
            if self.current_goal_wp < len(self.goals):
                self.target_x,self.target_y,self.target_yaw = self.goals[self.current_goal_wp]
                print('zero',self.target_x)
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
        self.publish_wheel_velocities(wheel_velocities)
        # self.publish_bot_command(wheel_velocities[0],wheel_velocities[1],wheel_velocities[2])

        


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel):
      
        msg = BotCmdArray()
        

        cmd = BotCmd()
        cmd.id = 0
        cmd.m1 = float(wheel_vel[0])
        cmd.m2 = float(wheel_vel[1])
        cmd.m3 = float(wheel_vel[2])
        cmd.base = 0.0
        cmd.elbow = 0.0
        
     
        msg.cmds.append(cmd)
        
        self.publisher.publish(msg)

    

    # def publish_bot_command(self, m1, m2, m3, base=0.0, elbow=0.0):
    #     msg = BotCmdArray()
    #     cmd = BotCmd()
    #     cmd.id = 0
    #     cmd.m1 = float(m1)
    #     cmd.m2 = float(m2)
    #     cmd.m3 = float(m3)
    #     cmd.base = float(base)
    #     cmd.elbow = float(elbow)
    #     msg.cmds.append(cmd)

    #     self.cmd_publisher.publish(msg)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
