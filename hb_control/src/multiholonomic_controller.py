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
        if self.main_node.assignments is None:
            return Status.RUNNING
        print(self.main_node.assignments)
        for i,(bot_id,crateid) in enumerate(self.main_node.assignments):
            if bot_id == self.botid:
                print(bot_id,crateid)
                return Status.SUCCESS
            else : 
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

        self.pid_params = {
            'x': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'y': {'kp': 0.25, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'theta': {'kp': 1.5, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])

    def setup(self):
        self.logger.debug(f"Action::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Action::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        _,bx,by,byaw = self.main_node.all_bots_dict[self.botid]
        cid,cx,cy,cyaw = self.main_node.all_crates_dict[self.main_node.bot_to_crate[self.botid]]

        now = self.main_node.get_clock().now()
        dt = (now.nanoseconds - self.last_time)/1e9
        if dt <= 0:
            return
        self.last_time = now.nanoseconds

        error_x = cx-bx
        error_y = cy-by
        error_yaw = cyaw-byaw
        correction = -1.03 * cyaw + 0.8
        error_yaw += correction
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

        self.main_node.publish_wheel_velocities(wheel_velocities)

        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")
    

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
        self.crystal_dict, self.frostbite_dict, self.glacio_dict,self.all_bots_dict = {}, {}, {}, {}
        self.bot_to_crate = {}
        self.current_pose_bot = None
        self.current_pose_crates = None
        self.all_bots = None
        self.all_crates = None
        self.tasks_assigned = False
        self.assigned_crates = None
        self.unassigned_crates = None
        self.assignments = None
        self.tree = None
        
        self.goals = None
        self.goal_reached = False
        self.current_goal_wp = 0
        self.alpha1 = math.radians(30)
        self.alpha2 = math.radians(150)
        self.alpha3 = math.radians(270)
        log_tree.level = log_tree.Level.DEBUG
        self.tree = self.make_bt() 

        self.A = np.array([
            [np.cos(self.alpha1 + np.pi/2), np.cos(self.alpha2 + np.pi/2), np.cos(self.alpha3 + np.pi/2)],
            [np.sin(self.alpha1 + np.pi/2), np.sin(self.alpha2 + np.pi/2), np.sin(self.alpha3 + np.pi/2)],
            [0.185,0.185,0.185]
        ])



        self.timer = self.create_timer(0.5, self.assign_task_greedy)

        self.timer_bt = self.create_timer(0.5, self.tick_tree)
        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')

        # self.timer = self.create_timer(0.3, self.control_crystal_cb) 

    def tick_tree(self):

        self.tree.tick()
        result = self.tree.root.status
        if result == Status.SUCCESS:
            self.get_logger().info("complete")
            self.timer_bt.cancel()

    def make_bt(self):
        # if not self.all_bots_dict or not self.all_crates_dict or not self.bot_to_crate:
        #     return 
        root = Sequence("MAIN SEQUENCE",memory=True)

        check_assign = CheckAsssignments("CheckAssignments",self,botid=0)
        navigate = navigate_to_assigned_crate('navigate_to_assigned_crate',main_node = self,botid=0)

        root.add_children([
            check_assign,
            navigate,
        ])    
        print('9875643122546789856431246579')
        return py_trees.trees.BehaviourTree(root)
    
    def crate_color(self, i):
        if i % 3 == 0:
            return "Red"
        elif i % 3 == 1:
            return "Green"
        else:
            return "Blue"

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
            return  # prevent reassigning

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


        self.tasks_assigned = True
        if hasattr(self, 'timer'):
            self.timer.cancel()


    def pose_crate_cb(self, msg):
        for self.current_pose_crates in msg.poses:
            color = self.crate_color(self.current_pose_crates.id)
            crate_tuple = (self.current_pose_crates.id, self.current_pose_crates.x, self.current_pose_crates.y, self.current_pose_crates.w)

            if color == "Red":
                self.red_dict[self.current_pose_crates.id] = crate_tuple
            elif color == "Green":
                self.green_dict[self.current_pose_crates.id] = crate_tuple
            elif color == "Blue":
                self.blue_dict[self.current_pose_crates.id] = crate_tuple

        self.red_crates = list(self.red_dict.values())
        self.green_crates = list(self.green_dict.values())
        self.blue_crates = list(self.blue_dict.values())
        self.all_crates = self.red_crates + self.green_crates + self.blue_crates
        self.all_crates_dict = self.red_dict | self.green_dict | self.blue_dict


    def control_cb(self):


        


        if not self.goal_reached:



            # if abs(error_yaw) > 0.4:
            #     pid_x_robot = 0.0
            #     pid_y_robot = 0.0 
            # if error_x < 165:
            #     pid_x_robot = 0.0 
            # if error_y < 165:    
            #     pid_y_robot = 0.0    
            
            if self.current_goal_wp == 0 :
                if dist_error< 155 and abs(error_yaw) <0.07:
                    self.goal_reached = True
                if dist_error < 145:
                    pid_x_robot = 0.0 
                    pid_y_robot = 0.0
            elif self.current_goal_wp == 1 :
                if dist_error< 125 :
                    self.goal_reached = True
            else :
                if abs(error_x) < 2.0 and abs(error_y) < 2.0 and abs(error_yaw) < 0.1:
                    self.goal_reached = True                 

            #  1 blue 
            # 2 red 
            # 3 green
        if self.goal_reached:

            if self.current_goal_wp == 0:
                self.publish_wheel_velocities([0.0, 0.0, 0.0,90.0,90.0])
                time.sleep(4.0)
                req = AttachLink.Request()
                req.data = '{"model1_name": "hb_crystal", "link1_name": "arm_link_2", "model2_name": "crate_red_27", "link2_name": "box_link_27"}'

                self.get_logger().info('Attach request sent, waiting for response...')
                future = self.attach_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

                if future.done() and future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"Attachment successful: {response.message}")
                    else:
                        self.get_logger().error(f"Attachment failed: {response.message}")
                else:
                    self.get_logger().error('Attach service call timed out or did not respond.')

                self.publish_wheel_velocities([0.0, 0.0, 0.0,45.0,45.0])
                time.sleep(4.0)

            if self.current_goal_wp == 1:
                self.publish_wheel_velocities([0.0, 0.0, 0.0,90.0,90.0])
                time.sleep(4.0)
                req = DetachLink.Request()
                req.data = '{"model1_name": "hb_crystal", "link1_name": "arm_link_2", "model2_name": "crate_red_27", "link2_name": "box_link_27"}'

                self.get_logger().info('Dettach request sent, waiting for response...')
                future = self.detach_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

                if future.done() and future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"Attachment successful: {response.message}")
                    else:
                        self.get_logger().error(f"Attachment failed: {response.message}")
                else:
                    self.get_logger().error('Attach service call timed out or did not respond.')

                self.publish_wheel_velocities([0.0, 0.0, 0.0,0.0,180.0])
                time.sleep(4.0)

            self.get_logger().info('changign to next goal')
            self.goal_reached = False
            self.current_goal_wp += 1
            
            print(self.current_goal_wp)
            if self.current_goal_wp ==3:
                self.get_logger().info('all th points reached')
                wheel_velocities = [0.0, 0.0, 0.0,45.0,45.0]
            if self.current_goal_wp < len(self.goals):
                self.target_x,self.target_y,self.target_yaw = self.goals[self.current_goal_wp]
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
        self.publish_wheel_velocities(wheel_velocities)


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


def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()