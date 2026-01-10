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
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree
import json
import paho.mqtt.client as mqtt
import sys
# actio node 
# condition node 
# Sequence node 
# store information
# decoraotr(invertor)
#bt.cpp -> pytrees
# execution node     Behaviour 
# Sequence node      Sequence
# Fallback Node      Selector node 
# Parallel node      parallel node 
# decorator node     Decorator node 

# ros2 service call /attach_link linkattacher_msgs/srv/AttachLink "{
#   data: '{\"model1_name\": \"hb_crystal\", \"link1_name\": \"arm_link_2\", \"model2_name\": \"crate_red_18\", \"link2_name\": \"box_link_17\"}'
# }"

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

class CheckAsssignments(Behaviour):
    def __init__(self, name,main_node,botid):
        super(CheckAsssignments,self).__init__(name)
        self.main_node = main_node
        self.botid = botid

    def setup(self):
        self.logger.debug(f"setup {self.name}")

    def initialise(self):
        self.logger.debug(f"initialise {self.name}")

    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        crate_id = self.main_node.bot_to_crate[self.botid]
        if crate_id:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")


class navigate_to_assigned_crate(Behaviour):
    def __init__(self, name,main_node,botid):
        super(navigate_to_assigned_crate,self).__init__(name)
        self.main_node = main_node
        self.botid = botid
        self.last_time = 0.0
        self.max_vel = 2.0
        self.tick_count = 0 
        self.max_ticks = 30
        self.ir_topic = None
        if self.botid == 0:
            self.ir_topic = "esp/crystal_ir"
        elif self.botid == 2:
            self.ir_topic = "esp/frostbite_ir"
        elif self.botid == 4 :
            self.ir_topic = "esp/glacio_ir"
        self.rotation = False

# on sim parms 
        # self.pid_params = {
        #     'x': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
        #     'y': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
        #     'theta': {'kp': 1.5, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel * 2}
        # }

        self.pid_params = {
            'x': {'kp': 2.75, 'ki': 0.00, 'kd': 0.5, 'max_out': self.max_vel},
            'y': {'kp': 2.75, 'ki': 0.00, 'kd': 0.5, 'max_out': self.max_vel},
            'theta': {'kp': 10.0, 'ki': 0.00, 'kd': 4.0, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])

    def setup(self):
        self.logger.debug(f"navigate to crate::setup {self.name}")

    def initialise(self):
        self.tick_count = 0 
        self.logger.debug(f"navigate to crate::initialise {self.name}")

    def update(self):
        self.logger.debug(f"navigate to crate::update {self.name}")
        _,bx,by,byaw = self.main_node.all_bots_dict[self.botid]
        cid,cx,cy,cyaw = self.main_node.all_crates_dict[self.main_node.bot_to_crate[self.botid]]

        now = self.main_node.get_clock().now()
        dt = (now.nanoseconds - self.last_time)/1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        if self.rotation == False:
            error_x = cx-bx
            error_y = cy-by
            target_yaw = math.atan2(error_y,error_x)
            error_yaw = target_yaw - byaw + math.pi/2

            # error_yaw = cyaw-byaw
            # correction = -1.03 * cyaw + 0.8
            # correction = 0 
            # error_yaw += correction
            dist_error = math.sqrt(error_x**2 + error_y**2)

            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi    
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            print(error_x,error_y,3.14-error_yaw)

            pid_x = self.pid_x.compute(error_x,dt)
            pid_y = self.pid_y.compute(error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)

            cos_yaw = math.cos(-byaw)
            sin_yaw = math.sin(-byaw)
            
            pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
            pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


            # pose = np.array([pid_x,pid_y,pid_yaw])
            pose = np.array([-pid_x_robot,pid_y_robot,pid_yaw])
            s_linalg = np.linalg.solve(self.main_node.A, pose)
            wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],160.0,180.0]

            self.main_node.publish_wheel_velocities(wheel_velocities)
        if self.ir_state == 0:
            wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
            self.main_node.publish_wheel_velocities(wheel_velocities)
            self.rotation = False
            return Status.SUCCESS 

        if dist_error<140:
            self.tick_count += 1 
            self.rotation = True
            wheel_velocities = [self.botid,-850.0,-850.0,-850.0,160.0,180.0]
            self.main_node.publish_wheel_velocities(wheel_velocities)
            if self.tick_count < self.max_ticks:
                return py_trees.common.Status.RUNNING

        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")

class pickup_crate(Behaviour):
    def __init__(self, name,main_node,botid):
        super(pickup_crate,self).__init__(name)
        self.main_node = main_node
        self.botid = botid 
        self.botname = None
        if self.botid == 0:
            self.botname = "crystal"
        elif self.botid == 2:
            self.botname = "frostbite"
        elif self.botid == 4 :
            self.botname = "glacio"
        self.tick_count = 0 
        self.tick_count_2 = 0 
        self.max_ticks = 10
        self.max_ticks_2 = 10
        self.bool = True

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        self.bool = True
        self.tick_count = 0 
        self.tick_count_2 = 0 
        self.logger.debug(f"pickup::initialise {self.name}")
    
    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        self.tick_count += 1

        if self.bool:            
            self.main_node.mqtt_client.publish(f"esp/{self.botname}_elec", "TRUE", qos=1)
            self.bool = False

        if self.tick_count < self.max_ticks:
            return py_trees.common.Status.RUNNING

        self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,160.0,180.0])

        self.tick_count_2 += 1
        if self.tick_count_2 < self.max_ticks_2:
            return py_trees.common.Status.RUNNING
        return Status.SUCCESS


    def terminate(self, new_status):
        self.logger.debug(f"pickup::terminate {self.name} to {new_status}")

class navigate_to_dropzone(Behaviour):
    def __init__(self, name,main_node,botid):
        super(navigate_to_dropzone,self).__init__(name)
        self.main_node = main_node
        self.botid = botid
        self.last_time = 0.0
        self.max_vel = 2.0
        self.tick_count = 0 
        self.max_ticks = 15

        self.pid_params = {
            'x': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'y': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'theta': {'kp': 1.5, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])

    def setup(self):
        self.logger.debug(f"navigate to crate::setup {self.name}")

    def initialise(self):
        self.tick_count = 0 
        self.logger.debug(f"navigate to crate::initialise {self.name}")

    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        
        self.crateid = self.main_node.bot_to_crate[self.botid]
        self.cratecolor = self.main_node.crate_color_dict[self.crateid]
        
        if self.cratecolor == 'red':
            cx,cy = self.main_node.red_D1
        if self.cratecolor == 'blue':
            cx,cy = self.main_node.blue_D2
        if self.cratecolor == 'green':
            cx,cy = self.main_node.green_D3

        self.logger.debug(f"navigate to crate::update {self.name}")
        _,bx,by,byaw = self.main_node.all_bots_dict[self.botid]

        now = self.main_node.get_clock().now()
        dt = (now.nanoseconds - self.last_time)/1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        error_x = cx-bx
        error_y = cy-by
        target_yaw = math.atan2(error_y,error_x)
        error_yaw = target_yaw - byaw - math.pi/2

        dist_error = math.sqrt(error_x**2 + error_y**2)

        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi    
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        print(error_x,error_y,error_yaw)

        pid_x = self.pid_x.compute(error_x,dt)
        pid_y = self.pid_y.compute(error_y,dt)
        pid_yaw = self.pid_yaw.compute(error_yaw,dt)

        cos_yaw = math.cos(-byaw)
        sin_yaw = math.sin(-byaw)
        
        pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
        pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


        # pose = np.array([pid_x,pid_y,pid_yaw])
        pose = np.array([pid_x_robot,pid_y_robot,-pid_yaw])
        s_linalg = np.linalg.solve(self.main_node.A, pose)
        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],160.0,180.0]

        if dist_error<160 and abs(error_yaw) < 0.63:
            wheel_velocities = [self.botid,0,0,0,180.0,180.0]
            return Status.SUCCESS  

        self.main_node.publish_wheel_velocities(wheel_velocities)

        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")


class drop_crate(Behaviour):
    def __init__(self, name,main_node,botid):
        super(drop_crate,self).__init__(name)
        self.main_node = main_node
        self.botid = botid 
        self.botname = None
        if self.botid == 0:
            self.botname = "crystal"
        elif self.botid == 2:
            self.botname = "frostbite"
        elif self.botid == 4 :
            self.botname = "glacio"
        self.tick_count = 0 
        self.tick_count_2 = 0 
        self.max_ticks = 15
        self.max_ticks_2 = 15
        self.bool = True

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        self.bool = True
        self.tick_count = 0 
        self.tick_count_2 = 0 
        self.logger.debug(f"pickup::initialise {self.name}")
    
    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        self.tick_count += 1

        if self.bool:            
            self.main_node.mqtt_client.publish(f"esp/{self.botname}_elec", "FALSE", qos=1)
            self.bool = False

        if self.tick_count < self.max_ticks:
            return py_trees.common.Status.RUNNING

        self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,160.0,180.0])

        self.tick_count_2 += 1
        if self.tick_count_2 < self.max_ticks_2:
            return py_trees.common.Status.RUNNING
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"pickup::terminate {self.name} to {new_status}")

class dock(Behaviour):
    def __init__(self, name,main_node,botid):
        super(dock,self).__init__(name)
        self.main_node = main_node
        self.botid = botid
        self.last_time = 0.0
        self.max_vel = 2.0
        self.tick_count = 0 
        self.max_ticks = 15

        self.pid_params = {
            'x': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'y': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'theta': {'kp': 1.5, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])

    def setup(self):
        self.logger.debug(f"navigate to crate::setup {self.name}")

    def initialise(self):
        self.tick_count = 0 
        self.logger.debug(f"navigate to crate::initialise {self.name}")

    def update(self):

        if self.botid == 0:
            cx,cy = (1218.0,130.0)
        if self.botid == 2:
            cx,cy = (1593.0,130.0)
        if self.botid == 4:
            cx,cy = (864.25,130.0)

        self.logger.debug(f"navigate to crate::update {self.name}")
        _,bx,by,byaw = self.main_node.all_bots_dict[self.botid]

        now = self.main_node.get_clock().now()
        dt = (now.nanoseconds - self.last_time)/1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        error_x = cx-bx
        error_y = cy-by
        target_yaw = math.atan2(error_y,error_x)
        error_yaw = target_yaw - byaw - math.pi/2

        dist_error = math.sqrt(error_x**2 + error_y**2)

        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi    
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        print(error_x,error_y,error_yaw)

        pid_x = self.pid_x.compute(error_x,dt)
        pid_y = self.pid_y.compute(error_y,dt)
        pid_yaw = self.pid_yaw.compute(error_yaw,dt)

        cos_yaw = math.cos(-byaw)
        sin_yaw = math.sin(-byaw)
        
        pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
        pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


        # pose = np.array([pid_x,pid_y,pid_yaw])
        pose = np.array([pid_x_robot,pid_y_robot,-pid_yaw])
        s_linalg = np.linalg.solve(self.main_node.A, pose)
        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],160.0,180.0]

        if dist_error<100 and abs(error_yaw) < 0.13:
            wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
            return Status.SUCCESS  

        self.main_node.publish_wheel_velocities(wheel_velocities)

        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")

class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node
        self.get_logger().info('HolonomicPIDController is created')

        broker_ip = "localhost"
        self.mqtt_client = mqtt.Client()
        self.ir_state_crsytal = None
        self.ir_state_frostbite = None
        self.ir_state_glacio = None


        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to broker")
                client.publish("esp/led", "LED_ON", qos=1)
                self.mqtt_client.subscribe("esp/crystal_ir")
                self.mqtt_client.subscribe("esp/frostbite_ir")
                self.mqtt_client.subscribe("esp/glacio_ir")
                print("Sent LED_ON command")
            else:
                print(f"Connection failed with code {rc}")
                sys.exit(1)

        def on_message(client, userdata, msg):
            print(f"[{msg.topic}] {msg.payload.decode()}")
            if msg.topic == "esp/crystal_ir" : 
                self.ir_state_crsytal = int(msg.payload.decode())
            if msg.topic == "esp/frostbite_ir" : 
                self.ir_state_frostbite = int(msg.payload.decode())            
            if msg.topic == "esp/glacio_ir" : 
                self.ir_state_glacio = int(msg.payload.decode())


        def on_disconnect(client, userdata, rc):
            print("Disconnected from broker")

        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_message = on_message
        self.mqtt_client.on_disconnect = on_disconnect
        self.mqtt_client.connect(broker_ip,1883,60)

        self.mqtt_client.loop_start() 

        self.bot_pose = self.create_subscription(Poses2D, 
                                                 "/bot_pose", 
                                                 self.pose_bot_cb,
                                                 10)
        
        self.crate_pose = self.create_subscription(Poses2D, 
                                                 "/crate_pose", 
                                                 self.pose_crate_cb,
                                                 10)
        
        self.publisher = self.create_publisher(BotCmdArray, 
                                               '/bot_cmd',
                                               10)
        


        self.red_dict, self.green_dict, self.blue_dict ,self.all_crates_dict = {}, {}, {}, {}
        self.crystal_dict, self.frostbite_dict, self.glacio_dict,self.all_bots_dict = {}, {}, {}, {}
        self.bot_to_crate = {}
        self.crate_color_dict = {}
        self.current_pose_bot = None
        self.current_pose_crates = None
        self.all_bots = None
        self.all_crates = None
        self.tasks_assigned = False
        self.assigned_crates = None
        self.assignments = None
        self.tree = None
        self.red_crate_dropzone = ()
        self.red_D1 = (1215.0,1215.0)
        self.blue_D2 = (1616.0,2017.5)
        self.green_D3 = (820.0,2017.5)
        self.alpha1 = math.radians(30)
        self.alpha2 = math.radians(150)
        self.alpha3 = math.radians(270)
        log_tree.level = log_tree.Level.DEBUG
        self.tree = self.setup_all_trees() 

        self.A = np.array([
            [np.cos(self.alpha1 + np.pi/2), np.cos(self.alpha2 + np.pi/2), np.cos(self.alpha3 + np.pi/2)],
            [np.sin(self.alpha1 + np.pi/2), np.sin(self.alpha2 + np.pi/2), np.sin(self.alpha3 + np.pi/2)],
            [0.185,0.185,0.185]
        ])

        self.timer = self.create_timer(0.5, self.assign_task_greedy)
        self.timer_bt = self.create_timer(0.5, self.tick_trees)

        self.get_logger().info(f'Holonomic PID Controller started.')

    def reset_tree(self, botid):
        tree = self.trees[botid]
        tree.root.stop(Status.INVALID) 
        self.get_logger().info(f"Tree for bot {botid} restarted from beginning")


    def tick_trees(self):
        completed_trees = []

        for botid,tree in self.trees.items():
            tree.tick()

            result = tree.root.status
            if result == Status.SUCCESS:
                completed_trees.append(botid)
                self.get_logger().info("complete")
        
        for botid in completed_trees:
            del self.trees[botid]

        if not self.trees:
            self.get_logger().info("All trees completed. Stopping BT timer.")
            self.timer_bt.cancel()

    def setup_all_trees(self):
        bot_ids = [0,2,4]
        self.trees = {}
        for botid in bot_ids:
            self.trees[botid] = self.make_bt_for_bots(botid)

    def make_bt_for_bots(self,botid):
        # if not self.all_bots_dict or not self.all_crates_dict or not self.bot_to_crate:
        #     return 
        root = Sequence(f"MAIN_SEQUENCE_{botid}",memory=True)

        check_assign = CheckAsssignments("CheckAssignments",self,botid=botid)
        navigate = navigate_to_assigned_crate('navigate_to_assigned_crate',main_node=self,botid=botid)
        pick_crate = pickup_crate('pick',main_node=self,botid=botid)
        navigate_drop = navigate_to_dropzone('mav_drop',main_node=self,botid=botid)
        drope_crate = drop_crate('drop',main_node=self,botid=botid)
        docks = dock('dock',main_node=self,botid=botid)

        root.add_children([
            check_assign,
            navigate,
            pick_crate,
            navigate_drop,
            drope_crate,
            docks,
        ])    
        return py_trees.trees.BehaviourTree(root)
    
    def crate_color(self, i):
        if i % 3 == 0:
            return "red"
        elif i % 3 == 1:
            return "green"
        else:
            return "blue"

    def pose_bot_cb(self, msg):
        for self.current_pose_bot in msg.poses:

            bot_map = {
                0: "crystal",
                2: "frostbite",
                4: "glacio"
            }
            bot_name = bot_map.get(self.current_pose_bot.id)
            bot_tuple = (self.current_pose_bot.id,self.current_pose_bot.x,self.current_pose_bot.y,self.current_pose_bot.w)
            if bot_name == "crystal":
                self.crystal_dict[self.current_pose_bot.id] = bot_tuple
            elif bot_name == "frostbite":
                self.frostbite_dict[self.current_pose_bot.id] = bot_tuple
            elif bot_name == "glacio":
                self.glacio_dict[self.current_pose_bot.id] = bot_tuple

        self.crystal = list(self.crystal_dict.values())
        self.frostbite = list(self.frostbite_dict.values())
        self.glacio = list(self.glacio_dict.values())
        self.all_bots = self.crystal + self.frostbite + self.glacio
        self.all_bots_dict = self.crystal_dict | self.frostbite_dict | self.glacio_dict


    def assign_task_greedy(self):
        if self.tasks_assigned:
            return  

        if not self.all_bots or not self.all_crates:
            return

        self.assigned_crates = set()
        self.assignments = []

        for bot in self.all_bots:
            bid, bx, by, bw = bot
            min_dist = float("inf")
            chosen_crate = None

            for crate in self.all_crates:
                cid, cx, cy, cw = crate
                if cid in self.assigned_crates:
                    continue
                dist = math.sqrt((bx - cx)**2 + (by - cy)**2)
                if dist < min_dist:
                    min_dist = dist
                    chosen_crate = crate

            if chosen_crate:
                self.assigned_crates.add(chosen_crate[0])
                self.assignments.append((bid, chosen_crate[0]))

        for bot_id, crate_id in self.assignments:
            print(f"Bot {bot_id}  Crate {crate_id}")

        all_crate_ids = {crate[0] for crate in self.all_crates}
        self.bot_to_crate = {bot_id: crate_id for bot_id, crate_id in self.assignments}

        self.tasks_assigned = True
        if hasattr(self, 'timer'):
            self.timer.cancel()

    def pose_crate_cb(self, msg):
        for self.current_pose_crates in msg.poses:
            color = self.crate_color(self.current_pose_crates.id)
            self.crate_color_dict[self.current_pose_crates.id] = color
            crate_tuple = (self.current_pose_crates.id, self.current_pose_crates.x, self.current_pose_crates.y, self.current_pose_crates.w)

            if color == "red":
                self.red_dict[self.current_pose_crates.id] = crate_tuple
            elif color == "green":
                self.green_dict[self.current_pose_crates.id] = crate_tuple
            elif color == "blue":
                self.blue_dict[self.current_pose_crates.id] = crate_tuple

        self.red_crates = list(self.red_dict.values())
        self.green_crates = list(self.green_dict.values())
        self.blue_crates = list(self.blue_dict.values())
        self.all_crates = self.red_crates + self.green_crates + self.blue_crates
        self.all_crates_dict = self.red_dict | self.green_dict | self.blue_dict


    def publish_wheel_velocities(self, wheel_vel):
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = wheel_vel[0]
        cmd.m1 = wheel_vel[1]
        cmd.m2 = wheel_vel[2]
        cmd.m3 = wheel_vel[3]
        cmd.base = wheel_vel[4]
        cmd.elbow = wheel_vel[5]
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
        print(json.dumps(data))
        self.mqtt_client.publish("esp/bot_cmd", json.dumps(data))

def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

