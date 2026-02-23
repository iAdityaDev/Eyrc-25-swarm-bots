#!/usr/bin/env python3

# Team Id : HB_1005
# Author List : Vansh Gupta , Aditya Dev Singh , Anurag Choudhary , Moulik Garg
# Filename: multiholonomic_controllers.py
# Theme: Holo Battalion
# Functions :
#   main(),attach_callback(),attach_done_cb(),reset_tree(),tick_trees(),setup_all_trees(),make_bt_for_bots(),crate_color()
#   pose_bot_cb(),assign_task_greedy(),pose_crate_cb(),point_to_segment_dist(),collision_avoidance(),publish_wheel_velocities()
# Classes :
#   PID,CheckAsssignments,navigate_to_assigned_crate,pickup_crate,navigate_to_dropzone,drop_crate,check_other_asssign
#   collisionAvoidance,dock,HolonomicPIDController
# Global Variables : None

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hb_interfaces.msg import Pose2D, Poses2D
from hb_interfaces.msg import BotCmdArray , BotCmd
from linkattacher_msgs.srv import AttachLink , DetachLink, Attach
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

# Class Name: PID
# FUnction Name: __init__ : intializes the main variables  
#                 compute : compute the velocity based on the error and the Kp,Kd,Ki
#                  print : print the output velocities
#                  reset : reset the prev_error and intergral values 
# * Input: __init__ : kp , kd , ki , max output   
#           compute : error,dt
#            print : None
#            reset : None
# * OutPut: __init__ : None 
#           compute : self.output velocity
#            print : velocity
#            reset : None
# * Logic: This class acts as the controller object for the pid control of the bot
# * Example Call: pid = PID(8.0, 0.0, 4.0)
#                 velocity = pid.compute(10.0, 0.02)

class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out   
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
        
# Class Name: CheckAsssignments
# FUnction Name: __init__ : initializes the behaviour with main node reference and bot id  
#                 setup : called once when behaviour tree is set up  
#                 initialise : called every time the behaviour starts running  
#                 update : checks whether the bot has been assigned a crate  
#                 terminate : called when behaviour stops running  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS / FAILURE)  
#           terminate : None  
# * Logic: This class checks if a bot has been assigned a crate.  
#          If no assignments exist keeps running.  
#          If the bot has a crate assigned , SUCCESS 
#          If not assigned , FAILURE
# * Example Call: check = CheckAsssignments("CheckAssign", main_node, 1)

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
        
        if self.main_node.bot_to_crate =={}:
            return Status.RUNNING
        
        crate_id = self.main_node.bot_to_crate[self.botid]
        if crate_id:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

# Class Name: navigate_to_assigned_crate
# FUnction Name: __init__ : initializes navigation behaviour, PID controllers and bot parameters  
#                 setup : called during tree setup  
#                 initialise : resets tick counter when behaviour starts  
#                 update : navigates the bot towards its assigned crate using PID control  
#                 terminate : called when behaviour ends  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class moves the assigned bot towards its allocated crate.  
#          It calculates position and yaw error, applies PID control,  
#          converts global velocities to robot frame, solves inverse kinematics,  
#          and publishes wheel velocities.  
#          When close enough and IR detects crate, it stops and returns SUCCESS.  
# * Example Call: assisgned_to_navigate_vrate = navigate_to_assigned_crate("NavToCrate", main_node, 0)

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
        self.cratedroppped = 0
            # if msg.topic == "esp/crystal_ir" : 
            #     self.ir_state_crsytal = int(msg.payload.decode())
        self.rotation = False
        if self.botid == 0:
            self.ircheck = self.main_node.ir_state_crystal
            self.botname = "crystal"
        elif self.botid == 2:
            self.ircheck = self.main_node.ir_state_frostbite
            self.botname = "frostbite"
        elif self.botid == 4 :
            self.ircheck = self.main_node.ir_state_glacio
            self.botname = "glacio"
        self.ir_value =None


        self.pid_params = self.main_node.pid_values

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

        if self.rotation == False:
            self.error_x = cx-bx
            self.error_y = cy-by
            target_yaw = math.atan2(self.error_y,self.error_x)
            error_yaw = target_yaw - byaw + math.pi/2

            self.dist_error = math.sqrt(self.error_x**2 + self.error_y**2)
            

            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi    
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            

            pid_x = self.pid_x.compute(self.error_x,dt)
            pid_y = self.pid_y.compute(self.error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)

            cos_yaw = math.cos(-byaw)
            sin_yaw = math.sin(-byaw)
            
            pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
            pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


            # pose = np.array([pid_x,pid_y,pid_yaw])
    
            pose = np.array([-pid_x_robot,pid_y_robot,pid_yaw])
            s_linalg = np.linalg.solve(self.main_node.A, pose)
            wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],165.0,180.0]

            print(wheel_velocities)
      
            self.main_node.publish_wheel_velocities(wheel_velocities)
        self.ir_value = self.main_node.ir_state[self.botname]
   

        thresh_dis = 195

        if self.dist_error<thresh_dis:
            if self.ir_value == 0:
                wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                
                self.main_node.publish_wheel_velocities(wheel_velocities)
                self.main_node.publish_wheel_velocities(wheel_velocities)

                self.rotation = False
                return Status.SUCCESS
            self.tick_count += 1 
            self.rotation = True
            wheel_velocities = [self.botid,-150.0,-150.0,-150.0,165.0,180.0]

            self.main_node.publish_wheel_velocities(wheel_velocities)
            if self.tick_count < self.max_ticks:
                return py_trees.common.Status.RUNNING

        return Status.RUNNING
    
    def terminate(self, new_status):
        self.cratedroppped = 1
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")

# Class Name: pickup_crate
# FUnction Name: __init__ : initializes pickup behaviour and bot parameters  
#                 setup : called during tree setup  
#                 initialise : resets counters and flags  
#                 update : attaches the crate and controls pickup timing  
#                 terminate : called when behaviour ends  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class handles crate pickup.  
#          It calls the attach service, keeps the bot stationary  
#          for fixed ticks to ensure proper gripping, and then  
#          returns SUCCESS after completion.  
# * Example Call: pickup = pickup_crate("PickupCrate", main_node, 0)

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
        self.max_ticks = 30
        self.max_ticks_2 = 30
        self.max_ticks_3 = 30
        self.max_ticks_4 = 30
        self.max_ticks_5 = 30

        self.bool = True
        self.cratedropped = 0

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        self.bool = True
        self.tick_count = 0
        self.tick_count_2 = 0 
        self.tick_count_3 = 0 
        self.tick_count_4 = 0 
        self.tick_count_5 = 0 
      
        self.logger.debug(f"pickup::initialise {self.name}")
    
    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        self.tick_count += 1

        if self.bool:            
            if self.main_node.attach_srv.service_is_ready():
                req = Attach.Request()
                req.bot_id = self.botid
                req.data = True
                self.future = self.main_node.attach_srv.call_async(req)
                self.future.add_done_callback(self.main_node.attach_done_cb)
            # self.main_node.mqtt_client.publish(f"esp/{self.botname}_elec", "TRUE", qos=1)
            self.bool = False

        if self.tick_count < self.max_ticks:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,180.0])
            return py_trees.common.Status.RUNNING



        if self.botid == 4 and self.cratedropped == 1:

            self.tick_count_3 += 1
            if self.tick_count_3 < self.max_ticks_3:
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,170.0,180.0])
                return Status.RUNNING
            self.tick_count_4 += 1
            if self.tick_count_4 < self.max_ticks_4:
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,160.0,180.0])
                return Status.RUNNING

            self.tick_count_5 += 1
            if self.tick_count_5 < self.max_ticks_5:
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,170.0,50.0])
                return Status.RUNNING

        self.tick_count_2 += 1
        if self.tick_count_2 < self.max_ticks_2:
            if self.botid == 4 and self.cratedropped == 1:
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,170.0,65.0])
            else:
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,170.0,180.0])

            return py_trees.common.Status.RUNNING
        return Status.SUCCESS


    def terminate(self, new_status):
        self.cratedropped = 1
        self.logger.debug(f"pickup::terminate {self.name} to {new_status}")


# Class Name: navigate_to_dropzone
# FUnction Name: __init__ : initializes navigation to dropzone and PID controllers  
#                 setup : called during tree setup  
#                 initialise : resets tick counter  
#                 update : navigates bot to its dropzone and aligns orientation  
#                 terminate : resets flags after completion  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class moves the bot carrying a crate to its designated dropzone.  
#          It computes position and yaw error using PID control,  
#          publishes wheel velocities, and performs final orientation alignment.  
#          Returns SUCCESS once correctly positioned and aligned.  
# * Example Call: drop_nav = navigate_to_dropzone("NavToDrop", main_node, 0)


class navigate_to_dropzone(Behaviour):
    def __init__(self, name,main_node,botid):
        super(navigate_to_dropzone,self).__init__(name)
        self.main_node = main_node
        self.botid = botid
        self.last_time = 0.0
        self.max_vel = 2.0
        self.tick_count = 0 
        self.max_ticks = 15
        self.rotation = False
        self.cratedropped = 0
        self.pid_params = self.main_node.pid_values
        self.cratedropped == 0
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
        if self.main_node.all_crates_dict is None:
            return Status.RUNNING

        self.crateid = self.main_node.bot_to_crate[self.botid]
        self.cratecolor = self.main_node.crate_color_dict[self.crateid]

        self.offset_dist_x = 150.0
        self.offset_dist_y = 150.0

        if self.cratecolor == 'red':
            cx,cy = self.main_node.red_D1
            self.cratedropped += 1
            cb_yaw = 3.0
        if self.cratecolor == 'blue':
            cx,cy = self.main_node.blue_D2
            self.cratedropped += 1
            cb_yaw = 0.0 
        if self.cratecolor == 'green':
            cx,cy = self.main_node.green_D3
            self.cratedropped += 1
            cb_yaw = 0.0

        if self.cratedropped == 1: 
            cx = cx-150
            cb_yaw = 0.0
        if self.cratecolor == 2:
            cy = cy - 150.0
            cb_yaw = 0.0 

        self.logger.debug(f"navigate to crate::update {self.name}")
        _,bx,by,byaw = self.main_node.all_bots_dict[self.botid]
        self.cid,_,_,_ = self.main_node.all_crates_dict[self.main_node.bot_to_crate[self.botid]]
        self.main_node.bot_target[self.botid] = (cx,cy)
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

            self.dist_error = math.sqrt(error_x**2 + error_y**2)

            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi    
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            # print(error_x,error_y,error_yaw)

            pid_x = self.pid_x.compute(error_x,dt)
            pid_y = self.pid_y.compute(error_y,dt)
            pid_yaw = self.pid_yaw.compute(error_yaw,dt)

            cos_yaw = math.cos(-byaw)
            sin_yaw = math.sin(-byaw)
            
            pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
            pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


            # pose = np.array([pid_x,pid_y,pid_yaw])
            pose = np.array([-pid_x_robot,pid_y_robot,-pid_yaw])
            s_linalg = np.linalg.solve(self.main_node.A, pose)

            wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],155.0,180.0]
            # if self.cratedropped == 1 and self.botid == 4:
            #     wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],180.0,65.0]
            self.main_node.publish_wheel_velocities(wheel_velocities)
            
        if self.botid == 2:
            if self.rotation == False:
                if self.dist_error<8:
                    if self.cratedropped == 0:
                        wheel_velocities = [self.botid,0.0,0.0,0.0,160.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        self.rotation = True
                    if self.cratedropped == 1:
                        wheel_velocities = [self.botid,0.0,0.0,0.0,160.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        self.rotation = True

        if self.botid == 0:
            if self.rotation == False:
                if self.dist_error<5:
                    if self.cratedropped == 0:
                        wheel_velocities = [self.botid,0.0,0.0,0.0,160.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        self.rotation = True
                    if self.cratedropped == 1:
                        wheel_velocities = [self.botid,0.0,0.0,0.0,160.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        self.rotation = True

        if self.botid == 4:
            if self.rotation == False:
                if self.dist_error<8 :
                    if self.cratedropped == 0:
                        wheel_velocities = [self.botid,0.0,0.0,0.0,160.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        self.rotation = True
                    if self.cratedropped == 1:                        
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        self.rotation = True

        if self.rotation == True:

            if self.botid == 0:
                wheel_velocities = [self.botid,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,150.0,180.0]
                self.main_node.publish_wheel_velocities(wheel_velocities)
                if self.cratedropped == 0:
                    if cb_yaw <= byaw <= cb_yaw-0.1:  
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        return Status.SUCCESS
                if self.cratedropped == 1:
                    if cb_yaw <= byaw <= cb_yaw-0.1:  
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        return Status.SUCCESS    
                
            if self.botid == 2:
                wheel_velocities = [self.botid,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,160.0,180.0]

                self.main_node.publish_wheel_velocities(wheel_velocities)

                if self.cratedropped == 0:
                    if cb_yaw <= byaw <= cb_yaw-0.1:  
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        return Status.SUCCESS
                if self.cratedropped == 1:
                    if cb_yaw <= byaw <= cb_yaw-0.1:  
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        return Status.SUCCESS  
                    
            if self.botid == 4:

                if self.cratedropped == 0:
                    wheel_velocities = [self.botid,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,160.0,180.0]
                    self.main_node.publish_wheel_velocities(wheel_velocities)
                    if  cb_yaw <= byaw <= cb_yaw-0.1:  
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        return Status.SUCCESS
                if self.cratedropped == 1:
                    wheel_velocities = [self.botid,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,(byaw-cb_yaw)*50,170.0,180.0]
                    self.main_node.publish_wheel_velocities(wheel_velocities)
                    if cb_yaw <= byaw <= cb_yaw-0.1:  
                        wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,90.0]
                        self.main_node.publish_wheel_velocities(wheel_velocities)
                        return Status.SUCCESS 
                
        return Status.RUNNING

    def terminate(self, new_status):
        self.rotation = False
        self.cratedropped = 1
        self.main_node.crates_dropped.append(self.cid)
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")

# Class Name: drop_crate
# FUnction Name: __init__ : initializes drop behaviour and bot parameters  
#                 setup : called during tree setup  
#                 initialise : resets counters and flags  
#                 update : detaches the crate and controls drop timing  
#                 terminate : final wheel stop and flag update  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class handles crate dropping.  
#          calls the detach service to release the crate,  
#          and returns SUCCESS after completion.  
# * Example Call: drop = drop_crate("DropCrate", main_node, 0)

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
        self.max_ticks = 25
        self.max_ticks_2 = 1
        self.max_ticks_3 = 50
        self.max_ticks_4 = 50
        self.max_ticks_5 = 50
        self.max_ticks_6 = 50
     
        self.bool = True
        self.cratedropped = 0

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        self.bool = True
        self.tick_count = 0
        self.tick_count_2 = 0
        self.tick_count_3 = 0
        self.tick_count_3 = 0
        self.tick_count_4 = 0
        self.tick_count_5 = 0
        self.tick_count_6 = 0

        self.logger.debug(f"pickup::initialise {self.name}")
    
    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        self.tick_count += 1

        if self.tick_count < self.max_ticks:
            
            if self.botid == 4 and self.cratedropped == 1:
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,65.0])
            else :
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,180.0])

  
            # self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,180.0])
            return py_trees.common.Status.RUNNING

        if self.bool:            
            if self.main_node.attach_srv.service_is_ready():
                req = Attach.Request()
                req.bot_id = self.botid
                req.data = False
                self.future = self.main_node.attach_srv.call_async(req)
                self.future.add_done_callback(self.main_node.attach_done_cb)
            # self.main_node.mqtt_client.publish(f"esp/{self.botname}_elec", "TRUE", qos=1)
            self.bool = False

        if self.tick_count < self.max_ticks:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,180.0])
            return py_trees.common.Status.RUNNING

        if self.botid == 4 and self.cratedropped == 1:

            self.tick_count_5 += 1
            if self.tick_count_5 < self.max_ticks_5 :
                self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,85.0])

                return py_trees.common.Status.RUNNING

        if self.cratedropped == 4 and self.botid == 1:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,85.0])
        else:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,180.0])

        self.tick_count_2 += 1
        if self.tick_count_2 < self.max_ticks_2:
            return py_trees.common.Status.RUNNING
        return Status.SUCCESS

    def terminate(self, new_status):

        if self.cratedropped == 4 and self.botid == 1:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,85.0])
        else:
            self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,160.0,180.0])
        self.cratedropped = 1
        self.logger.debug(f"pickup::terminate {self.name} to {new_status}")

# Class Name: check_other_asssign
# FUnction Name: __init__ : initializes reassignment flags for each bot  
#                 setup : called during tree setup  
#                 initialise : no specific initialization  
#                 update : reassigns unassigned crates to bots if available  
#                 terminate : called when behaviour ends  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class checks for remaining unassigned crates and reassigns  
#          them to the respective bot. Once reassigned, it returns SUCCESS.  
#          Otherwise, it keeps running until assignment is done.  
# * Example Call: reass = check_other_asssign("CheckReassign", main_node, 0)


class check_other_asssign(Behaviour):
    def __init__(self, name,main_node,botid):
        super(check_other_asssign,self).__init__(name)
        self.main_node = main_node
        self.botid = botid 

        self.crystal_reassigned = False
        self.frostbite_reassigned = False
        self.glacio_reassigned = False

    def setup(self):
        self.logger.debug(f"pickup::setup {self.name}")

    def initialise(self):
        pass

    def update(self):

#################################################
        # if self.botid == 0:
        #     return Status.SUCCESS
        # if self.botid == 2:
        #     return Status.SUCCESS        
        # if self.botid == 4:
        #     return Status.SUCCESS
        if self.crystal_reassigned:
            return Status.SUCCESS
        if self.frostbite_reassigned:
            return Status.SUCCESS        
        if self.glacio_reassigned :
            return Status.SUCCESS
#####################################################

        # if self.main_node.unassigned_crates == None:
        #     return Status.SUCCESS
        
        if self.botid == 0:
            cid = self.main_node.unassigned_crates[0]
            self.crystal_reassigned = True
        if self.botid == 2:
            cid = self.main_node.unassigned_crates[1]
            self.frostbite_reassigned = True
        if self.botid == 4:
            cid = self.main_node.unassigned_crates[2]
            self.glacio_reassigned = True

        self.main_node.bot_to_crate[self.botid] = cid
        # self.main_node.unassigned_crates = None
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"pickup::terminate {self.name} to {new_status}")

# Class Name: collisionAvoidance
# FUnction Name: __init__ : initializes automatic local collision avoidance manager  
#                 setup : called during tree setup  
#                 initialise : resets variables  
#                 update : detects nearby bots and redirects to avoid collision  
#                 terminate : called when behaviour ends  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class automatically checks inter-bot distance.  
#          If bots are too close, it generates a temporary avoidance motion  
#          using PID control. Returns SUCCESS once safe distance is achieved.  
# * Example Call: avoid = collisionAvoidance("CollisionAvoid", main_node, 4)

class collisionAvoidance(Behaviour):
    def __init__(self, name,main_node,botid):
        super(collisionAvoidance,self).__init__(name)
        self.main_node = main_node
        self.botid = botid
        self.last_time = 0.0
        self.max_vel = 2.0
        self.tick_count = 0 
        self.max_ticks = 15

        self.pid_params = self.main_node.pid_values

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
        if self.botid == 0:
            cx,cy = (1770.25,800.0)
            return Status.SUCCESS
        if self.botid == 2:
            cx,cy = (2300.25,1560.0)
            return Status.SUCCESS
        if self.botid == 4:
            cx,cy = (760.25,1588.0)
            return Status.SUCCESS

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
        error_yaw = target_yaw - byaw + math.pi/2

        self.dist_error = math.sqrt(error_x**2 + error_y**2)

        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi    
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        # print(error_x,error_y,error_yaw)

        pid_x = self.pid_x.compute(error_x,dt)
        pid_y = self.pid_y.compute(error_y,dt)
        pid_yaw = self.pid_yaw.compute(error_yaw,dt)

        cos_yaw = math.cos(-byaw)
        sin_yaw = math.sin(-byaw)
        
        pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
        pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


        # pose = np.array([pid_x,pid_y,pid_yaw])
        pose = np.array([-pid_x_robot,pid_y_robot,-pid_yaw])
        s_linalg = np.linalg.solve(self.main_node.A, pose)
        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],140.0,180.0]
        if self.botid == 4 :
            wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],180.0,90.0]

        if self.botid == 2:
            if self.dist_error<30:
                # wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                # self.main_node.publish_wheel_velocities(wheel_velocities)

                return Status.SUCCESS 
        if self.botid == 0:
            if self.dist_error<250:
                # wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
                # self.main_node.publish_wheel_velocities(wheel_velocities)

                return Status.SUCCESS 
        if self.botid == 4:
            if self.dist_error<50:
                # wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,90.0]
                # self.main_node.publish_wheel_velocities(wheel_velocities)

                return Status.SUCCESS 

        self.main_node.publish_wheel_velocities(wheel_velocities)
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")

# Class Name: dock
# FUnction Name: __init__ : initializes docking manager  
#                 setup : called during tree setup  
#                 initialise : resets variables  
#                 update : moves bot to its dock position  
#                 terminate : called when behaviour ends  
# * Input: __init__ : name , main_node , botid  
#           setup : None  
#           initialise : None  
#           update : None  
#           terminate : new_status  
# * OutPut: __init__ : None  
#           setup : None  
#           initialise : None  
#           update : Status (RUNNING / SUCCESS)  
#           terminate : None  
# * Logic: This class drives the bot to its final docking location  
#          using PID control and returns SUCCESS when aligned.  
# * Example Call: dock_bot = dock("Dock", main_node, 0)

class dock(Behaviour):
    def __init__(self, name,main_node,botid):
        super(dock,self).__init__(name)
        self.main_node = main_node
        self.botid = botid
        self.last_time = 0.0
        self.max_vel = 2.0
        self.tick_count = 0 
        self.max_ticks = 15

        self.pid_params = self.main_node.pid_values

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_yaw = PID(**self.pid_params['theta'])
        self.cratedropped = 1

    def setup(self):
        self.logger.debug(f"navigate to crate::setup {self.name}")

    def initialise(self):
        self.tick_count = 0 
        self.logger.debug(f"navigate to crate::initialise {self.name}")

    def update(self):
        if self.main_node.bot_to_crate is None:
            return Status.RUNNING
        
        if self.botid == 0:
            cx,cy = (1220.0,203.0)
        if self.botid == 2:
            cx,cy = (1592.0,204.0)
        if self.botid == 4:
            cx,cy = (860.25,200.0)

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
        error_yaw = target_yaw - byaw + math.pi/2

        self.dist_error = math.sqrt(error_x**2 + error_y**2)

        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi    
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        # print(error_x,error_y,error_yaw)

        pid_x = self.pid_x.compute(error_x,dt)
        pid_y = self.pid_y.compute(error_y,dt)
        pid_yaw = self.pid_yaw.compute(error_yaw,dt)

        cos_yaw = math.cos(-byaw)
        sin_yaw = math.sin(-byaw)
        
        pid_x_robot = pid_x * cos_yaw - pid_y * sin_yaw
        pid_y_robot = pid_x * sin_yaw + pid_y * cos_yaw


        # pose = np.array([pid_x,pid_y,pid_yaw])
        pose = np.array([-pid_x_robot,pid_y_robot,-pid_yaw])
        s_linalg = np.linalg.solve(self.main_node.A, pose)


        wheel_velocities = [self.botid,s_linalg[0],s_linalg[1],s_linalg[2],165.0,180.0]


        if abs(error_x)<2.0 and abs(error_y)<2.0:
            wheel_velocities = [self.botid,0.0,0.0,0.0,180.0,180.0]
            self.main_node.publish_wheel_velocities(wheel_velocities)

            return Status.SUCCESS  

        self.main_node.publish_wheel_velocities(wheel_velocities)
        return Status.RUNNING

    def terminate(self, new_status):
        self.cratedropped = 1
        self.logger.debug(f"navigate::terminate {self.name} to {new_status}")



# Class Name: HolonomicPIDController
# FUnction Name:__init__(), attach_callback(), attach_done_cb(), reset_tree(), tick_trees(), setup_all_trees(), 
#               make_bt_for_bots(), assign_task_greedy(), pose_bot_cb(), pose_crate_cb(), collision_avoidance(), 
#               publish_wheel_velocities()
# * Input: Multiple ROS topics, MQTT messages, service requests  
# * OutPut: Bot wheel commands, gripper commands, BT status updates  
# * Logic: This is the main system controller node.  
#          It manages multi-bot coordination using PID control,  
#          Behaviour Trees, greedy task allocation, MQTT communication,  
#          and automatic collision avoidance.  
# * Example Call: node = HolonomicPIDController()

class HolonomicPIDController(Node):

    # Function Name: __init__
    # * Input: self
    # * Output: None
    # * Logic: Initializes ROS2 node, MQTT, PID parameters, services, publishers,
    #          subscribers, behaviour trees, task allocation variables,
    #          kinematics matrix, and timers for collision avoidance and BT ticking.
    # * Example Call: node = HolonomicPIDController()
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node
        self.get_logger().info('HolonomicPIDController is created')

        broker_ip = "localhost"
        self.mqtt_client = mqtt.Client()
        self.ir_state_crystal = None
        self.ir_state_frostbite = None
        self.ir_state_glacio = None
        self.ir_state = {
            "crystal": None,
            "frostbite": None,
            "glacio": None
        }

        # Function Name: on_connect, on_message, on_disconnect
        # * Input: MQTT client callbacks (client, userdata, flags/msg, rc)
        # * Output: None
        # * Logic: on_connect subscribes to IR topics and sends LED_ON;
        #          on_message updates IR sensor states;
        #          on_disconnect logs broker disconnection.
        # * Example Call: Auto-triggered by MQTT client

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
            # print(f"[{msg.topic}] {msg.payload.decode()}")
            if msg.topic == "esp/crystal_ir":
                self.ir_state["crystal"] = int(msg.payload.decode())
            elif msg.topic == "esp/frostbite_ir":
                self.ir_state["frostbite"] = int(msg.payload.decode())
            elif msg.topic == "esp/glacio_ir":
                self.ir_state["glacio"] = int(msg.payload.decode())


        def on_disconnect(client, userdata, rc):
            print("Disconnected from broker")

        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_message = on_message
        self.mqtt_client.on_disconnect = on_disconnect
        self.mqtt_client.connect(broker_ip,1883,60)

        self.mqtt_client.loop_start() 

        self.attach_server = self.create_service(
            Attach,
            'attach',
            self.attach_callback
        )
        self.max_vel = 0.0 

        # self.pid_values = {
        #     'x': {'kp': 8.0, 'ki': 0.0, 'kd': 4.0, 'max_out': self.max_vel},
        #     'y': {'kp': 8.0, 'ki': 0.0, 'kd': 4.0, 'max_out': self.max_vel},
        #     'theta': {'kp': 10.0, 'ki': 0.00, 'kd': 3.0, 'max_out': self.max_vel * 2}
        # }

        # good params
        self.pid_values = {
            'x': {'kp': 6.0, 'ki': 0.00, 'kd': 1.0, 'max_out': self.max_vel},
            'y': {'kp': 6.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
            'theta': {'kp': 22.50, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel * 2}
        }
        # self.pid_values = {
        #     'x': {'kp': 7.0, 'ki': 0.00, 'kd': 2.5, 'max_out': self.max_vel},
        #     'y': {'kp': 7.0, 'ki': 0.00, 'kd': 2.5, 'max_out': self.max_vel},
        #     'theta': {'kp': 22.50, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel * 2}
        # }


        self.attach_srv = self.create_client(Attach, 'attach')
        while not self.attach_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')


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
        self.blue_D3 = (1616.0,2017.5)
        self.green_D2 = (780.0,2060.5)
        self.alpha1 = math.radians(30)
        self.alpha2 = math.radians(150)
        self.alpha3 = math.radians(270)

        self.crates_dropped = []
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

        self.collision_timer = self.create_timer(0.05, self.collision_avoidance)
        self.timer = self.create_timer(0.5, self.assign_task_greedy)
        self.timer_bt = self.create_timer(0.05, self.tick_trees)    

        self.get_logger().info(f'Holonomic PID Controller started.')

        self.BOT_ELEC_TOPIC = {
            0: "esp/crystal_elec",
            2: "esp/frostbite_elec",
            4: "esp/glacio_elec",
        }

    # Function Name: attach_callback, attach_done_cb
    # * Input: request, response (service); future (async result)
    # * Output: Service response (success flag)
    # * Logic: attach_callback publishes gripper ON/OFF via MQTT and returns success;
    #          attach_done_cb logs service result or error.
    # * Example Call: Called automatically by ROS2 service
    def attach_callback(self, request, response):
        topic = self.BOT_ELEC_TOPIC[request.bot_id]
        payload = "TRUE" if request.data else "FALSE"

        self.mqtt_client.publish(topic, payload, qos=1)

        response.success = True
        response.message = "MQTT published"
        return response

    

    def attach_done_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info("Attach service success")
        except Exception as e:
        # if self.botid == 0 and self.cratedropped ==0:
        #     self.tick_count_3 += 1
        #     if self.tick_count_3 < self.max_ticks_3:
        #         self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,150.0,180.0])
        #         return Status.RUNNING
        #     self.tick_count_4 += 1
        #     if self.tick_count_4 < self.max_ticks_4:
        #         self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,150.0,90.0])
        #         return Status.RUNNING
            
        #     self.tick_count_5 += 1
        #     if self.tick_count_5 < self.max_ticks_5:
        #         self.main_node.publish_wheel_velocities([self.botid,0.0, 0.0, 0.0,180.0,90.0])
        #         return Status.RUNNING
            self.get_logger().error(f"Service failed: {e}")

    # Function Name: reset_tree, tick_trees
    # * Input: botid (for reset), None (for ticking)    
    # * Output: None
    # * Logic: reset_tree restarts a bot’s Behaviour Tree;
    #          tick_trees ticks all trees, resets if needed,
    #          removes completed trees, and stops timer when all finish.
    # * Example Call: self.tick_trees()

    def reset_tree(self, botid):
        tree = self.trees[botid]
        tree.root.stop(Status.INVALID) 
        self.get_logger().info(f"Tree for bot {botid} restarted from beginning")


    def tick_trees(self):
        completed_trees = []

        for botid,tree in self.trees.items():
            tree.tick()

            check_node = tree.root.children[-3]
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

    # Function Name: setup_all_trees, make_bt_for_bots
    # * Input: botid (for tree creation)
    # * Output: BehaviourTree object (per bot)
    # * Logic: Creates a Behaviour Tree for each bot with sequence:
    #          check → navigate → pick → drop → recheck → collision avoid → dock.
    # * Example Call: self.setup_all_trees()

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
        avoid_collision =  collisionAvoidance('avoid_collision',main_node=self,botid=botid)
        docks = dock('dock',main_node=self,botid=botid)

        root.add_children([
            check_assign,
            navigate,
            pick_crate,
            navigate_drop,
            drope_crate,
            check_other,
            avoid_collision,
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

    # Function Name: pose_bot_cb
    # * Input: msg (bot poses)
    # * Output: None
    # * Logic: Updates bot pose dictionaries by ID and refreshes combined bot lists.
    # * Example Call: Auto-called by subscriber
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


    # Function Name: assign_task_greedy
    # * Input: None
    # * Output: None
    # * Logic: Greedily assigns nearest crate to each bot and updates mappings.
    # * Example Call: self.assign_task_greedy()

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

        self.tasks_assigned = True
        if hasattr(self, 'timer'):
            self.timer.cancel()

    # Function Name: pose_crate_cb
    # * Input: msg (crate poses)
    # * Output: None
    # * Logic: Sorts crates by color and updates crate dictionaries.
    # * Example Call: Auto-called by subscriber
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

    # Function Name: point_to_segment_dist, collision_avoidance
    # * Input: (px, py, x1, y1, x2, y2) for distance calc; uses bot positions & targets for collision check
    # * Output: Distance (float) from point to segment; updates bot_safe_check (True/False)
    # * Logic: Computes shortest distance from a bot to another bot’s path; if within safety limit,
    #          stops the bot that is farther from its target to avoid collision.
    # * Example Call: self.collision_avoidance()
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
                if dist < 150.0:
                    if my_dist_to_target > other_dist_to_target:
                        self.bot_safe_check[botid] = False
                    break

# Function Name: publish_wheel_velocities
# * Input: wheel_vel (list → [id, m1, m2, m3, base, elbow])
# * Output: None
# * Logic: Creates BotCmd message, applies safety check (stops wheels if unsafe),
#          publishes command to ROS2 topic and sends same data via MQTT.
# * Example Call: self.publish_wheel_velocities([0, v1, v2, v3, 180.0, 180.0])
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
        data =  {
            "id": cmd.id,
            "m1":cmd.m1,
            "m2":cmd.m2,
            "m3":cmd.m3,
            "base":cmd.base,
            "elbow":cmd.elbow
        }
        # print(json.dumps(data))
        # print('publishing')
        self.mqtt_client.publish("esp/bot_cmd", json.dumps(data))


# Function Name: main
# * Input: args (optional ROS2 arguments)
# * Output: None
# * Logic: Initializes ROS2, creates HolonomicPIDController node,
#          spins the node to keep it running, then properly shuts down.
# * Example Call: main()
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()