#!/usr/bin/env python3
import pybullet as p
from robots.ur5 import UR5
import gin
import numpy as np
import time
import pybullet_data
from ros_wrapper_pkg.ros_msg.ros_dtype import ROSDtype
import os
from scipy.spatial.transform import Rotation
from moving_object.moving_object import MovingObject
import yaml
import csv
from datetime import datetime

ROS_CLOCK_TOPIC = "clock"
ROS_TARGET_POSITION_TOPIC = "target_position"

@gin.configurable
class Environment():
    def __init__(self, setRealTimeSimulation, dt, realtime_factor, dynamic_obs_T):
        self.client = p.connect(p.GUI) # p.connect(p.DIRECT)
        p.setRealTimeSimulation(setRealTimeSimulation)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(dt)
        config_file = os.path.dirname(os.path.abspath(__file__)) + "/config/config.yaml"
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        self.robot = UR5()
        self.dt = dt
        self.time = 0.0
        self.realtime_factor = realtime_factor
        self.ros_wrapper = self.robot.ros_wrapper
        self.ros_wrapper.add_publisher(ROS_CLOCK_TOPIC, ROSDtype.CLOCK, False)
        self.ros_wrapper.add_publisher(ROS_TARGET_POSITION_TOPIC, ROSDtype.FLOAT_ARRAY, True)
        self.udrf_path = os.path.dirname(os.path.abspath(__file__)) + "/urdf/objects/"
        self.grasp_target = []
        self.dynamic_obs_T = dynamic_obs_T
        
        path = os.path.dirname(os.path.abspath(__file__)) + "/log/env_log/" + datetime.now().strftime("%Y%m%d%H%M%S") + ".csv"
        self.csv_file = open(path, 'w')
        self.csv_writter = csv.writer(self.csv_file)
        self.start_sec = time.time()
        self.csv_writter.writerow(['time','ee_x', 'ee_y', 'ee_z', 'chassis_x', 'chassis_y', 'chassis_yaw'])
        
        self.load_scenes()

    def csv_wirte_log(self):
        pos, ori = self.robot.get_end_state()
        base_orientation = Rotation.from_euler('xyz', [0, 0, self.robot.chassis.pos[2]], degrees=False).as_quat() 
        ee_pos_r = np.dot(np.linalg.inv(Rotation.from_quat(base_orientation).as_matrix()), (pos - np.array([self.robot.chassis.pos[0], self.robot.chassis.pos[1], 0.0])))
        self.csv_writter.writerow([time.time() - self.start_sec, ee_pos_r[0], ee_pos_r[1], ee_pos_r[2], self.robot.chassis.pos[0], self.robot.chassis.pos[1], self.robot.chassis.pos[2]])
    
    def step(self):
        start_time = time.time() 
        self.ros_wrapper.publish_msg(ROS_CLOCK_TOPIC, ROSDtype.CLOCK.value(self.time))  
        self.time += self.dt
        self.ros_wrapper.ros_time = self.time
        self.robot.apply_control(self.robot.set_angle[0], "joint")
        self.moving_object1.update_position(self.time)
        self.fake_grasp()
        # self.robot.set_base_twist([0.0,0.0, 0.3])
        p.stepSimulation()
        end_time = time.time()
        elapsed_time = end_time - start_time
        self.csv_wirte_log()
        if self.dt / self.realtime_factor - elapsed_time > 0:
            time.sleep(self.dt / self.realtime_factor - elapsed_time)

    def load_scenes(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # print(pybullet_data.getDataPath()) #/usr/local/lib/python3.8/dist-packages/pybullet_data
        p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
        table1 = p.loadURDF(self.udrf_path + 'table/table.urdf', [0.8, -1.5, -0.25], [0, 0, 0, 1], useFixedBase=True, globalScaling = 1.0)
        # cube_small = p.loadURDF(self.udrf_path + 'cube/cube.urdf', [0.8, -1.6, 0.6], [0, 0, 0, 1], globalScaling = 0.2)
        table2 = p.loadURDF(self.udrf_path + 'table/table.urdf', [-6.5, 1.5, -0.25], [0, 0, 0, 1], useFixedBase=True, globalScaling = 1.0)
        p.loadURDF('tray/tray.urdf', [-6.7, 1.35, 0.38], [0, 0, 0, 1],  globalScaling = 0.6)
        p.loadURDF('tray/tray.urdf', [1.0, -1.35, 0.38], [0, 0, 0, 1],  globalScaling = 0.6)
        self.load_grasp_target()
        self.load_room()
        self.load_moving_obstacle()
        if self.config["use_config"]:
            self.load_config_obstacles()

    def load_room(self):
        p.loadURDF(self.udrf_path + 'block10.urdf', [-3, 2.5, 0.5], [0, 0, 0, 1], useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block10.urdf', [-3, -2.5, 0.5], [0, 0, 0, 1], useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block5.urdf', [-8, 0.0, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block5.urdf', [2, 0.0, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block2.urdf', [-1, 1.5, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block3.urdf', [-3, -1.0, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        static_cube1 = p.loadURDF(self.udrf_path + 'cube/cube.urdf', [-3, -2, 0.3], [0, 0, 0, 1], globalScaling = 0.7, useFixedBase=True)
    
    def load_grasp_target(self):
        self.grasp_target.append(p.loadURDF(self.udrf_path + 'ball/red_ball.urdf', [0.65, -1.4, 0.5], [0, 0, 0, 1]))
        self.grasp_target.append(p.loadURDF(self.udrf_path + 'ball/green_ball.urdf', [-6.25, 1.4, 0.5], [0, 0, 0, 1]))
    
    def load_moving_obstacle(self):
        moving_object_id_1 = p.loadURDF(self.udrf_path + 'cylinder.urdf', [-1.0, 1.4, 0.5], [0, 0, 0, 1])
        self.moving_object1 = MovingObject(moving_object_id_1, [-1.5, 0, 0], [-1.5, -2, 0], self.dynamic_obs_T)
    
    def load_config_obstacles(self):
        path = os.path.dirname(os.path.abspath(__file__)) + "/urdf/obstacles/"
        if self.config["obstacles"]:
            for obstacle in self.config["obstacles"]:
                p.loadURDF(path + obstacle["name"], obstacle["pos"], [0, 0, 0, 1], useFixedBase=True)
    
    def fake_grasp(self):
        pos, ori = self.robot.get_end_state()
        offset = np.array([0.13, 0, 0.0])
        grip_pos = np.array(pos) + np.dot(Rotation.from_quat(ori).as_matrix(),offset)
        target_position = []
        for target in self.grasp_target:
            target_pos = p.getBasePositionAndOrientation(target)[0]
            target_position.extend(target_pos)
            dist = np.linalg.norm((np.array(target_pos) - np.array(grip_pos)))
            if dist < 0.08 and self.robot.gripper_open_ratio[0] < 0.8:
                p.resetBasePositionAndOrientation(target, grip_pos, ori)
        self.ros_wrapper.publish_msg(ROS_TARGET_POSITION_TOPIC, target_position)  
                
        
        
        
        
CONFIG_FILE = os.path.dirname(os.path.abspath(__file__)) + "/config/env_default.gin"
gin.parse_config_file(CONFIG_FILE)

if __name__ == "__main__":
    env = Environment()
    while True:
        env.step()