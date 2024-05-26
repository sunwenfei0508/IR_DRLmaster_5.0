#!/usr/bin/env python3
from controller.ur5_moveit_interface import UR5MoveitInterface
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import time
from enum import Enum
from std_msgs.msg import Float64, Float64MultiArray
import threading

WORK1_POS = [np.array([0.65, -0.82, 0.0]), -0.5 * np.pi]
PRE_WORK1_POS = [np.array([0.65, -0.0, 0.0]), -0.5 * np.pi]
WORK2_POS = [np.array([-6.6, 0.82, 0.0]), 0.5 * np.pi]
PRE_WORK2_POS = [np.array([-6.6, 0.0, 0.0]), 0.5 * np.pi]
WORK3_POS = [np.array([-6.35, 0.82, 0.0]), 0.5 * np.pi]
WORK4_POS = [np.array([1.0, -0.82, 0.0]), -0.5 * np.pi]
PRE_WORK4_POS = [np.array([1.0, -0.0, 0.0]), -0.5 * np.pi]

class JointConfiguration(Enum):
    UPRIGHT = [0.0,-0.5 * np.pi,0,0,0,0]
    DOWN_VIEW = [0.0, -0.5 *np.pi, 0.4 * np.pi, -0.5 * np.pi, -0.5 * np.pi, 0]

class TaskController:
    def __init__(self):
        self.moveit_interface = UR5MoveitInterface()
        self.movebase_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.grip_ratio_pub = rospy.Publisher("/ur5_pybullet/gripper_open_ratio", Float64, queue_size=1)
        
        self.navi_target = [np.array([0, 0, 0]), 0.0]
        self.robot_pos = [np.array([0, 0, 0]), 0.0]
        self.target_pos = np.array([[0, 0, 0], [0, 0, 0]]).astype(Float64)
        self.gripper_ratio = 1.0
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/ur5_pybullet/target_position", Float64MultiArray, self.target_position_callback)
        self.send_command_thread = threading.Thread(target=self.send_command_thread_fun)
        self.send_command_thread.setDaemon(True)
        self.send_command_thread.start()
    
    def target_position_callback(self, msg:Float64MultiArray):
        self.target_pos[0] = msg.data[0:3]
        self.target_pos[1] = msg.data[3:6]
    
    def odom_callback(self, msg:Odometry):
        self.robot_pos[0] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y ,msg.pose.pose.position.z])
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.robot_pos[1] = Rotation.from_quat(orientation).as_euler('xyz')[2]
        
    def send_navigation_target(self, pos):
        position = pos[0]
        yaw = pos[1]
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        orientation = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False).as_quat()
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        self.movebase_goal_pub.publish(msg)
    
    def send_command_thread_fun(self):
        while not rospy.is_shutdown():
            if self.navi_target:
                self.send_navigation_target(self.navi_target)
            self.moveit_interface.publish_status()
            self.set_gripper_ratio(self.gripper_ratio)
            time.sleep(0.2)
    
    def set_arm_joint_configuration(self, config:JointConfiguration):
        self.moveit_interface.go_to_joint_state(config.value)
    
    def grab_target(self, target_pose):
        target_pose = np.array(target_pose)
        
        target_pose[2] += 0.25
        base_orientation =  Rotation.from_euler('xyz', [0, 0, self.robot_pos[1]], degrees=False).as_quat()
        base_position = self.robot_pos[0] + np.array([0, 0, 0.06])
        target_pose_r = np.dot(np.linalg.inv(Rotation.from_quat(base_orientation).as_matrix()), (target_pose - base_position))
        self.moveit_interface.go_to_pose_goal(target_pose_r, Rotation.from_euler('xyz', [0, 0.5 * np.pi, 0], degrees=False).as_quat(), "base_link")
        time.sleep(0.1)
        target_pose[2] -= 0.10
        base_orientation =  Rotation.from_euler('xyz', [0, 0, self.robot_pos[1]], degrees=False).as_quat()
        base_position = self.robot_pos[0] + np.array([0, 0, 0.06])
        target_pose_r = np.dot(np.linalg.inv(Rotation.from_quat(base_orientation).as_matrix()), (target_pose - base_position))
        self.moveit_interface.go_to_pose_goal(target_pose_r, Rotation.from_euler('xyz', [0, 0.5 * np.pi, 0], degrees=False).as_quat(), "base_link")
        time.sleep(0.1)
        
        target_pose[2] -= 0.07
        base_orientation =  Rotation.from_euler('xyz', [0, 0, self.robot_pos[1]], degrees=False).as_quat()
        base_position = self.robot_pos[0] + np.array([0, 0, 0.06])
        target_pose_r = np.dot(np.linalg.inv(Rotation.from_quat(base_orientation).as_matrix()), (target_pose - base_position))
        self.moveit_interface.go_to_pose_goal(target_pose_r, Rotation.from_euler('xyz', [0, 0.5 * np.pi, 0], degrees=False).as_quat(), "base_link")
        time.sleep(0.5)
        self.gripper_ratio = 0.5
    
    def set_gripper_ratio(self, ratio):
        msg = Float64()
        msg.data = ratio
        self.grip_ratio_pub.publish(msg)
    
    def has_reach_target(self, dist_thres=0.30, ang_thres=10.0):
        dist_err = np.linalg.norm(self.robot_pos[0] - self.navi_target[0])
        ang_err = np.linalg.norm(self.robot_pos[1] - self.navi_target[1])
        return dist_err < dist_thres and ang_err < ang_thres
    
    def navigate_and_wait(self, target, timeout=50.0):
        self.navi_target = target
        wait = 0.0
        while not self.has_reach_target():
            time.sleep(0.1)
            wait += 0.1
            if wait > timeout:
                print("navigation timeout!")
                self.navi_target = None
                return
        self.navi_target = None
        time.sleep(4.0)
        
    
    def work(self):
        self.set_arm_joint_configuration(JointConfiguration.UPRIGHT)
        self.navigate_and_wait(PRE_WORK1_POS)
        self.set_arm_joint_configuration(JointConfiguration.DOWN_VIEW)
        self.navigate_and_wait(WORK1_POS)
        time.sleep(1)
        self.grab_target(self.target_pos[0])
        time.sleep(0.5)
        self.set_arm_joint_configuration(JointConfiguration.UPRIGHT)
        #self.navigate_and_wait(PRE_WORK1_POS)
        self.navigate_and_wait(PRE_WORK2_POS)
        self.set_arm_joint_configuration(JointConfiguration.DOWN_VIEW)
        self.navigate_and_wait(WORK2_POS)
        self.gripper_ratio = 1.0
        time.sleep(1)
        self.navigate_and_wait(WORK3_POS)
        self.grab_target(self.target_pos[1])
        time.sleep(0.5)
        self.set_arm_joint_configuration(JointConfiguration.UPRIGHT)
        #self.navigate_and_wait(PRE_WORK2_POS)
        self.navigate_and_wait(PRE_WORK4_POS)
        self.set_arm_joint_configuration(JointConfiguration.DOWN_VIEW)
        self.navigate_and_wait(WORK4_POS)
        self.gripper_ratio = 1.0
        time.sleep(0.5)
        self.navigate_and_wait(PRE_WORK4_POS)
        self.set_arm_joint_configuration(JointConfiguration.UPRIGHT)
        
        
        
        
        
    
    
        


if __name__ == "__main__":
    rospy.init_node("task_controller")
    task_controller = TaskController()
    start_sec = time.time()
    task_controller.work()
    end = time.time()
    print("time_usage:" , end - start_sec) # 92 150 155
    # while not rospy.is_shutdown():
    #     task_controller.step()
    #     time.sleep(0.1)
    
    