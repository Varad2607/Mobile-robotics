#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/car/scan' 
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
POSE_TOPIC = '/car/car_pose' 
VIZ_TOPIC = '/laser_wanderer/rollouts' 
MAP_TOPIC = '/static_map' 
MAX_PENALTY = 10000 
                    

class LaserWanderer:

  def __init__(self, speed, min_delta, max_delta, traj_nums, dt, T, compute_time, laser_offset, car_length, car_width):

    self.speed = speed
    self.min_delta = min_delta    
    self.max_delta = max_delta
    self.traj_nums = traj_nums
    self.dt = dt 
    self.T = T
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    self.car_length = car_length
    self.car_width = car_width
    self.rollouts, self.deltas = self.generate_mpc_rollouts(speed, min_delta, max_delta, 
                                                            traj_nums, dt, T, car_length)
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=10)
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_callback)
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size=10)
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub_callback)

  def viz_sub_callback(self, msg):

    pos_arr = PoseArray()
    pos_arr.header.frame_id = '/map'
    pos_arr.header.stamp = rospy.Time.now()
    traj_pose = Pose()
    traj_poses = []
    for trj_num, trj_point in enumerate(self.rollouts):
      vect = np.array([[trj_point[-1, 0]], [trj_point[-1, 1]]]) 
      theta_ = trj_point[-1, 2]
      theta_car = utils.quaternion_to_angle(msg.pose.orientation)
      traj_pose.position.x = utils.rotation_matrix(theta_car).dot(vect)[0] + msg.pose.position.x
      traj_pose.position.y = utils.rotation_matrix(theta_car).dot(vect)[1] + msg.pose.position.y
      traj_pose.orientation = utils.angle_to_quaternion(theta_ + theta_car)
      traj_poses.append(traj_pose)
    pos_arr.poses = traj_poses
    self.viz_pub.publish(pos_arr)
  
  def compute_cost(self, delta, rollout_pose, laser_msg):

    computed_cost = np.abs(delta)
    traj_len = np.sqrt(np.square(rollout_pose[0]) + np.square(rollout_pose[1]))
    d_theta = np.arctan2(rollout_pose[1], rollout_pose[0])
    if not np.isfinite(d_theta):
      d_theta = np.pi / 2
      if rollout_pose[1] < 0:
        d_theta *= -1
    idx = int((d_theta - laser_msg.angle_min) / laser_msg.angle_increment)
    laser_measurement = laser_msg.ranges[idx]
    computed_cost -= 100 * traj_len
    if ((laser_measurement - np.abs(self.laser_offset)) <= traj_len) and np.isfinite(laser_measurement):
      computed_cost += MAX_PENALTY
    return computed_cost

  def wander_callback(self, msg):
    start = rospy.Time.now().to_sec() 
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    traj_depth = 0
    while (rospy.Time.now().to_sec() - start <= self.compute_time) and (traj_depth < self.T):
      for traj_num, delta in enumerate(self.deltas):
          delta_costs[traj_num] += self.compute_cost(delta, self.rollouts[traj_num][traj_depth], msg)
      traj_depth += 1
    min_cost_id = np.argmin(delta_costs, axis=0)
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/laser_link'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = self.deltas[min_cost_id]
    ads.drive.speed = self.speed
    self.cmd_pub.publish(ads)

  def kinematic_model_step(self, current_pose, control, car_length):
    nxt_pos = current_pose.copy()
    v = control[0]
    delta = control[1]
    dt = control[2]
    theta = current_pose[2]
    nxt_pos[0] += v * np.cos(theta) * dt
    nxt_pos[1] += v * np.sin(theta) * dt
    nxt_pos[2] += v * np.tan(delta) * dt / car_length

    return nxt_pos

  def generate_rollout(self, init_pose, controls, car_length):
    rollout_ = np.zeros((controls.shape[0], 3), dtype=np.float)
    for pose_num, traj_detail in enumerate(controls):
      cur_pos = init_pose.copy()
      if pose_num > 0:
        cur_pos += rollout_[pose_num-1]
      rollout_[pose_num] = self.kinematic_model_step(cur_pos, traj_detail, car_length)
    return rollout_

  def generate_mpc_rollouts(self, speed, min_delta, max_delta, traj_nums, dt, T, car_length):
    delta_step = (max_delta - min_delta - 0.001) / traj_nums
    delta_arr = np.arange(min_delta, max_delta, delta_step)
    N = delta_arr.shape[0]
    init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
    rollouts = np.zeros((N,T,3), dtype=np.float)
    for i in range(N):
      ctrl = np.zeros((T,3), dtype=np.float)
      ctrl[:,0] = speed
      ctrl[:,1] = delta_arr[i]
      ctrl[:,2] = dt
      rollouts[i,:,:] = self.generate_rollout(init_pose, ctrl, car_length)
      
    return rollouts, delta_arr

def main():

  rospy.init_node('laser_wanderer', anonymous=True)
  speed = rospy.get_param("speed")
  min_delta = rospy.get_param("min_delta")
  max_delta = rospy.get_param("max_delta")
  traj_nums = rospy.get_param("~traj_nums")
  dt = rospy.get_param("dt")
  T = rospy.get_param("~T")
  compute_time = rospy.get_param("compute_time")
  laser_offset = rospy.get_param("~laser_offset")
  car_length = rospy.get_param("/car/vesc/chassis_length", 0.33)
  car_width = rospy.get_param("/car/vesc/wheelbase", 0.25)
  rospy.loginfo("1.1.Create Laser Wanderer Instance")                                         
  laser_wanderer = LaserWanderer(speed, min_delta, max_delta, traj_nums, dt, T, compute_time, laser_offset, car_length, car_width)
  rospy.loginfo("1.2.Laser Wanderer Instance Created!")
  rospy.spin()

  
if __name__ == '__main__':
  main()