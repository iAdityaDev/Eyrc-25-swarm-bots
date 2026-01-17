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


# def check_crates_assigned(self,bot_id):
#     for i,(botid,crateid) in enumerate(self.assignments):
#         if botid ==  bot_id:
#             return Status.SUCCESS
#         else : 
#             return Status.FAILURE

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
        self.logger.debug(f"navigate to crate::update {self.name}")
        _,bx,by,byaw = self.main_node.all_bots_dict[self.botid]
        cid,cx,cy,cyaw = self.main_node.all_crates_dict[self.main_node.bot_to_crate[self.botid]]

        self.main_node.bot_target[self.botid] = (cx,cy)

        now = self.main_node.get_clock().now()
        dt = (now.nanoseconds - self.last_time)/1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        error_x = cx-bx
        error_y = cy-by
        target_yaw = math.atan2(error_y,error_x)
        error_yaw = target_yaw - byaw - math.pi/2

        # error_yaw = cyaw-byaw
        # correction = -1.03 * cyaw + 0.8
        # correction = 0 
        # error_yaw += correction
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
        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],45.0,45.0]

        if dist_error<153 and abs(error_yaw) < 0.13:
            self.tick_count += 1 
            wheel_velocities = [self.botid,0.0,0.0,0.0,90.0,90.0]
            self.main_node.publish_wheel_velocities(wheel_velocities)
            if self.tick_count < self.max_ticks:
                return py_trees.common.Status.RUNNING
            return Status.SUCCESS  

        self.main_node.publish_wheel_velocities(wheel_velocities)

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
        self.max_ticks = 15
        self.max_ticks_2 = 15
        self.bool = True

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        self.bool = True
        self.tick_count = 0 
        self.logger.debug(f"pickup::initialise {self.name}")
    
    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        self.tick_count += 1
        self.tick_count_2 += 1

        if self.bool:            
            self.crateid = self.main_node.bot_to_crate[self.botid]
            self.cratecolor = self.main_node.crate_color_dict[self.crateid]

            req = AttachLink.Request()
            req.data = json.dumps({
                "model1_name": f"hb_{self.botname}",
                "link1_name": f"arm_link_2",
                "model2_name": f"crate_{self.cratecolor}_{self.crateid}",
                "link2_name": f"box_link_{self.crateid}"
                })

            # self.get_logger().info('Attach request sent, waiting for response...')
            future = self.main_node.attach_client.call_async(req)
            self.bool = False
            rclpy.spin_until_future_complete(self.main_node, future, timeout_sec=1.0)

            # if future.done() and future.result() is not None:
            #     response = future.result()
            #     if response.success:
            #         self.get_logger().info(f"Attachment successful: {response.message}")
            #     else:
            
            #         self.get_logger().error(f"Attachment failed: {response.message}")
            # else:
            #     self.get_logger().error('Attach service call timed out or did not respond.')
        if self.tick_count < self.max_ticks:
            return py_trees.common.Status.RUNNING

        self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,45.0,45.0])
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
        self.main_node.bot_target[self.botid] = (cx,cy)


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
        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],45.0,45.0]

        if dist_error<160 and abs(error_yaw) < 0.63:
            wheel_velocities = [self.botid,0,0,0,45.0,45.0]
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
        self.tick_count_2 += 1
        print('insdie the update block ')

        if self.tick_count_2 < self.max_ticks_2:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,90.0,90.0])
            print('inside the tick retrun')
            return py_trees.common.Status.RUNNING

        self.tick_count += 1
        if self.bool:
            self.crateid = self.main_node.bot_to_crate[self.botid]
            self.cratecolor = self.main_node.crate_color_dict[self.crateid]
            req = DetachLink.Request()
            req.data = json.dumps({
                "model1_name": f"hb_{self.botname}",
                "link1_name": f"arm_link_2",
                "model2_name": f"crate_{self.cratecolor}_{self.crateid}",
                "link2_name": f"box_link_{self.crateid}"
                })

            future = self.main_node.detach_client.call_async(req)
            self.bool = False
            rclpy.spin_until_future_complete(self.main_node, future, timeout_sec=1.0)

        if self.tick_count < self.max_ticks:
            return py_trees.common.Status.RUNNING

        return Status.SUCCESS


    def terminate(self, new_status):
        self.logger.debug(f"pickup::terminate {self.name} to {new_status}")

class check_other_asssign(Behaviour):
    def __init__(self, name,main_node,botid):
        super(check_other_asssign,self).__init__(name)
        self.main_node = main_node
        self.botid = botid 

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        pass

    def update(self):

##################################################
        # if self.botid == 0:
        #     return Status.SUCCESS
        # if self.botid == 2:
        #     return Status.SUCCESS        
        # if self.botid == 4:
        #     return Status.SUCCESS
#####################################################

        if self.main_node.unassigned_crates == None:
            return Status.SUCCESS
        
        cid = self.main_node.unassigned_crates[0]
        self.main_node.bot_to_crate[self.botid] = cid
        self.main_node.unassigned_crates = None
        return Status.RUNNING

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
        self.main_node.bot_target[self.botid] = (cx,cy)

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
        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],0.0,180.0]

        if dist_error<100 and abs(error_yaw) < 0.13:
            return Status.SUCCESS  

        self.main_node.publish_wheel_velocities(wheel_velocities)

        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")



class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node
        self.get_logger().info('HolonomicPIDController is created')

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
        
        self.attach_client = self.create_client(AttachLink, '/attach_link')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wits attach_link service...')

        self.detach_client = self.create_client(DetachLink, '/detach_link')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wits deattach_link service...')

        self.red_dict, self.green_dict, self.blue_dict ,self.all_crates_dict = {}, {}, {}, {}
        self.all_crates = None
        self.crystal_dict, self.frostbite_dict, self.glacio_dict,self.all_bots_dict = {}, {}, {}, {}
        self.all_bots = None
        self.bot_to_crate = {}
        self.crate_color_dict = {}
        self.current_pose_bot = None
        self.current_pose_crates = None
        self.tasks_assigned = False
        self.assigned_crates = None
        self.unassigned_crates = None
        self.assignments = None
        self.tree = None
        self.safe_dist = 350.0
        self.crystal_safe = None
        self.frostbite_safe = None
        self.glacio_safe = None
        self.red_crate_dropzone = ()
        self.red_D1 = (1215.0,1215.0)
        self.blue_D2 = (1616.0,2017.5)
        self.green_D3 = (820.0,2017.5)
        self.alpha1 = math.radians(30)
        self.alpha2 = math.radians(150)
        self.alpha3 = math.radians(270)
        log_tree.level = log_tree.Level.DEBUG

        self.bot_target = {
            0: None,
            2: None,
            4: None
        }
        
        self.tree = self.setup_all_trees() 

        self.bot_safe_check= {
            0 : True,
            2 : True,
            4 : True
        }

        self.A = np.array([
            [np.cos(self.alpha1 + np.pi/2), np.cos(self.alpha2 + np.pi/2), np.cos(self.alpha3 + np.pi/2)],
            [np.sin(self.alpha1 + np.pi/2), np.sin(self.alpha2 + np.pi/2), np.sin(self.alpha3 + np.pi/2)],
            [0.185,0.185,0.185]
        ])

        self.collision_timer = self.create_timer(0.01, self.collision_avoidance)
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

            check_node = tree.root.children[-2]
            if check_node.status == Status.RUNNING:
                self.reset_tree(botid)
                return

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
        check_other = check_other_asssign('check_other_asssign',main_node=self,botid=botid)
        docks = dock('dock',main_node=self,botid=botid)

        root.add_children([
            check_assign,
            navigate,
            pick_crate,
            navigate_drop,
            drope_crate,
            check_other,
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
        self.unassigned_crates = []
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
        self.unassigned_crates = list(all_crate_ids - self.assigned_crates)
        self.bot_to_crate = {bot_id: crate_id for bot_id, crate_id in self.assignments}

###########################################
        # self.bot_to_crate = {
        #     0 : 35,
        #     2 : 14,
        #     4 : 48,
        # }
        # self.unassigned_crates = [41]
###############################################

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


    # def collision_avoidance(self):
    #     if not self.all_bots_dict:
    #         return
    #     botids = [0,2,4]

    #     for bid in self.bot_safe_check:
    #         self.bot_safe_check[bid] = True

    #     for botid, (_, bx, by, _) in self.all_bots_dict.items():
    #         for oid, (_, ox, oy, _) in self.all_bots_dict.items():
    #             if botid == oid:
    #                 continue

    #             dist = math.hypot(bx - ox, by - oy)
    #             if dist < self.safe_dist:
    #                 # higher id stops
    #                 if botid > oid:
    #                     self.bot_safe_check[botid] = False

    def point_to_segment_dist(self,px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return math.hypot(px - x1, py - y1)

        t = ((px - x1)*dx + (py - y1)*dy) / (dx*dx + dy*dy)
        t = max(0.0, min(1.0, t))

        cx = x1 + t*dx
        cy = y1 + t*dy
        return math.hypot(px - cx, py - cy)

    def collision_avoidance(self):
        for bid in self.bot_safe_check:
            self.bot_safe_check[bid] = True

        for botid, (_, bx, by, _) in self.all_bots_dict.items():

            target = self.bot_target.get(botid)
            if target is None:
                continue

            tx, ty = target
            my_dist_to_target = math.hypot(tx - bx, ty - by)

            for oid, (_, ox, oy, _) in self.all_bots_dict.items():
                if botid == oid:
                    continue

                other_target = self.bot_target.get(oid)
                if other_target is None:
                    continue

                otx, oty = other_target
                other_dist_to_target = math.hypot(otx - ox, oty - oy)

                dist = self.point_to_segment_dist(
                    ox, oy,
                    bx, by,
                    tx, ty
                )
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                print(dist)
                # if (math.hypot(ox-bx,oy-by)>200):
                #     continue

                if dist < 180.0:
                    if my_dist_to_target > other_dist_to_target:
                        self.bot_safe_check[botid] = False
                    break



    def publish_wheel_velocities(self, wheel_vel):
            
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = wheel_vel[0]
        cmd.m1 = wheel_vel[1]
        cmd.m2 = wheel_vel[2]
        cmd.m3 = wheel_vel[3]
        cmd.base = wheel_vel[4]
        cmd.elbow = wheel_vel[5]

        if not self.bot_safe_check[cmd.id]:
            cmd.m1 = 0.0
            cmd.m2 = 0.0
            cmd.m3 = 0.0

        # if not self.crystal_safe:
        #     if cmd.id == 0:
        #         cmd.m1 = 0.0
        #         cmd.m2 = 0.0
        #         cmd.m3 = 0.0

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