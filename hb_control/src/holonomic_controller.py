#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hb_interfaces.msg import Pose2D, Poses2D
from hb_interfaces.msg import BotCmdArray , BotCmd
from linkattacher_msgs.srv import AttachLink , DetachLink
import numpy as np
import math
import time
import paho.mqtt.client as mqtt
import sys
import json

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

        broker_ip = "localhost"
        self.mqtt_client = mqtt.Client()

        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to broker")
                client.publish("esp/led", "LED_ON", qos=1)
                self.mqtt_client.subscribe("esp/crystal_ir  ")
                print("Sent LED_ON command")
            else:
                print(f"Connection failed with code {rc}")
                sys.exit(1)

        def on_message(client, userdata, msg):
            print(f"[{msg.topic}] {msg.payload.decode()}")
            self.ir_state = int(msg.payload.decode())

            if self.ir_state == 0:
                print("Object Detected")
            else:
                print("No Object")

        def on_disconnect(client, userdata, rc):
            print("Disconnected from broker")

        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_message = on_message
        self.mqtt_client.on_disconnect = on_disconnect
        self.mqtt_client.connect(broker_ip,1883,60)

        self.mqtt_client.loop_start() 

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
        self.docking_zone = [1218.0,205.0,0]

        self.A = np.array([
            [np.cos(self.alpha1 + np.pi/2), np.cos(self.alpha2 + np.pi/2), np.cos(self.alpha3 + np.pi/2)],
            [np.sin(self.alpha1 + np.pi/2), np.sin(self.alpha2 + np.pi/2), np.sin(self.alpha3 + np.pi/2)],
            [0.08,0.08,0.08]
        ])
        # self.A = np.array([
        #     [np.cos(self.alpha1 + np.pi/2), np.cos(self.alpha2 + np.pi/2), np.cos(self.alpha3 + np.pi/2)],
        #     [np.sin(self.alpha1 + np.pi/2), np.sin(self.alpha2 + np.pi/2), np.sin(self.alpha3 + np.pi/2)],
        #     [0.185,0.185,0.185]
        # ])
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

#######use this for the sim bot 
        self.pid_params = {
            'x': {'kp': 2.75, 'ki': 0.00, 'kd': 0.5, 'max_out': self.max_vel},
            'y': {'kp': 2.75, 'ki': 0.00, 'kd': 0.5, 'max_out': self.max_vel},
            'theta': {'kp': 10.0, 'ki': 0.00, 'kd': 4.0, 'max_out': self.max_vel * 2}
        }

#################v
#######use this for the hard bot 
        # self.pid_params = {
        #     'x': {'kp':0.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
        #     'y': {'kp': 0.00, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
        #     'theta': {'kp': 1.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel * 2}
        # }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])


        self.timer = self.create_timer(0.9, self.control_cb) 
        self.publish_wheel_velocities([0.0, 0.0, 0.0,160.0,180.0])
        self.mqtt_client.publish("esp/crystal_elec", "FALSE", qos=1)

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
            # self.current_pose_crate_yaw = 0.0
            # print(self.current_pose_crate_id,self.current_pose_crate_x,self.current_pose_crate_y,self.current_pose_crate_yaw)
        
        if self.goals == None:
            self.goals = [(self.current_pose_crate_x,self.current_pose_crate_y,self.current_pose_crate_yaw),
                          (1253,1220.2,0.0 ),
                          (1219.2,210.0,0.0)]
            self.target_x,self.target_y,self.target_yaw = self.goals[0]
        # print(self.goals)    

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
            target_yaw = math.atan2(error_y,error_x)
            error_yaw = target_yaw - self.current_pose_bot_yaw + math.pi/2
            # error_yaw = target_yaw - self.current_pose_bot_yaw
            # if self.current_goal_wp == 1:
            #     error_yaw = 0.0 
            #     error_y = -error_y

            # error_yaw = target_yaw - self.current_pose_bot_yaw
            # error_yaw = self.target_yaw-self.current_pose_bot_yaw
            # correction = -1.03 * self.current_pose_crate_yaw + 0.8
            # error_yaw += correction
            dist_error = math.sqrt(error_x**2 + error_y**2)

            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi    
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            # print(error_x,error_y,error_yaw)
            # print(error_x,error_y,error_yaw)
            # print(dist_error)
            print(error_x,error_y,3.14-error_yaw)
            pid_x = self.pid_x.compute(error_x,dt)
            pid_y = self.pid_y.compute(error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)
            # self.pid_x.print()
            # self.pid_y.print()
            # self.pid_yaw.print()
            cos_yaw = math.cos(-self.current_pose_bot_yaw)
            sin_yaw = math.sin(-self.current_pose_bot_yaw)
            
            pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
            pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw

            # if abs(error_yaw) > 0.4:
            #     pid_x_robot = 0.0
            #     pid_y_robot = 0.0 
            # if error_x < 165:
            #     pid_x_robot = 0.0 
            # if error_y < 165:
            #     pid_y_robot = 0.0    

            pose = np.array([-pid_x_robot,pid_y_robot,pid_yaw])
            s_linalg = np.linalg.solve(self.A, pose)
            wheel_velocities = [s_linalg[0],s_linalg[1],s_linalg[2],160.0,180.0]
            if self.current_goal_wp == 0 :
                # if self.ir_state == 0:
                #     self.goal_reached = True
                if dist_error< 240 and (abs(3.14-error_yaw) < 0.2 and abs(3.14-error_yaw) > 0.15) :
                    pid_x_robot = 0.0 
                    pid_y_robot = 0.0
                    pid_yaw = 0.0
                # if dist_error< 150 and abs(error_yaw) <0.13:
                    self.goal_reached = True
                elif dist_error < 240:
                    pid_x_robot = 0.0 
                    pid_y_robot = 0.0

            elif self.current_goal_wp == 1 :
                if dist_error< 190 :
                    self.goal_reached = True
                if dist_error < 190:
                    pid_x_robot = 0.0 
                    pid_y_robot = 0.0
                    
            elif self.current_goal_wp == 2 :
                if abs(error_x) < 22.0 and abs(error_y) < 22.0:
                    self.goal_reached = True  
                    pid_x_robot = 0.0 
                    pid_y_robot = 0.0

            # pose = np.array([pid_x,pid_y,pid_yaw])


            #  1 blue 
            # 2 red 
            # 3 green

            self.publish_wheel_velocities(wheel_velocities)

        if self.goal_reached:

            if self.current_goal_wp == 0:
                self.publish_wheel_velocities([0.0, 0.0, 0.0,180.0,180.0])
                time.sleep(2.0)
                

                self.mqtt_client.publish("esp/crystal_elec", "TRUE", qos=1)
                time.sleep(4.0)
                self.publish_wheel_velocities([0.0, 0.0, 0.0,160.0,180.0])
                time.sleep(1.0)

            if self.current_goal_wp == 1:
                self.publish_wheel_velocities([0.0, 0.0, 0.0,180.0,180.0])
                time.sleep(2.0)

                self.mqtt_client.publish("esp/crystal_elec", "FALSE", qos=1)
                time.sleep(4.0)
                self.publish_wheel_velocities([0.0, 0.0, 0.0,160.0,180.0])
                time.sleep(2.0)

            self.get_logger().info('changign to next goal')
            
            if self.current_goal_wp ==2:
                self.get_logger().info('all the points reached')
                self.publish_wheel_velocities([0.0, 0.0, 0.0,180.0,180.0])
                print(self.current_goal_wp)
                self.timer.cancel()

            self.goal_reached = False
            self.current_goal_wp += 1
            if self.current_goal_wp < len(self.goals):
                self.target_x,self.target_y,self.target_yaw = self.goals[self.current_goal_wp]
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()



    def publish_wheel_velocities(self, wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = 0
        cmd.m1 = wheel_vel[0]
        cmd.m2 = wheel_vel[1]
        cmd.m3 = wheel_vel[2]
        cmd.base = wheel_vel[3]
        cmd.elbow = wheel_vel[4]
        msg.cmds = [cmd]

        self.publisher.publish(msg)
        data =  {
            "id": cmd.id,
            "m1":cmd.m1,
            "m2":cmd.m2,
            "m3":cmd.m3,
            "base":cmd.base,
            "elbow":cmd.elbow
        }
        # print(json.dumps(data))
        self.mqtt_client.publish("esp/bot_cmd", json.dumps(data))


def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()