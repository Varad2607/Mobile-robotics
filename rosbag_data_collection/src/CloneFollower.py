#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from Utils import get_map
from Utils import world_to_map
from Utils import map_to_world
from Utils import quaternion_to_angle
from Utils import angle_to_quaternion
import matplotlib.pyplot as plt

SUB_TOPIC = '/car/car_pose' 
PUB_TOPIC = '/clone_follower_pose/pose' 
MAP_TOPIC = '/static_map'

# Follows the simulated robot around
class CloneFollower:

  '''
  Initializes a CloneFollower object
  In:
    follow_offset: The required x offset between the robot and its clone follower
    force_in_bounds: Whether the clone should toggle between following in front
                     and behind when it goes out of bounds of the map
  '''
  def __init__(self, follow_offset, force_in_bounds):

    self.follow_offset = follow_offset
    self.force_in_bounds = force_in_bounds
    self.map_img, self.map_info = get_map('/static_map')
    self.pub = rospy.Publisher('/clone_follower_pose/pose', PoseStamped, queue_size=10)
    self.sub = 	rospy.Subscriber('/car/car_pose', PoseStamped, callback=self.update_pose)
    
  '''
  Given the translation and rotation between the robot and map, computes the pose
  of the clone
  (This function is optional, but you can use it as a helper function in update_pose())
  In:
    trans: The translation between the robot and map
    rot: The rotation between the robot and map
  Out:
    The pose of the clone
  '''
  def compute_follow_pose(self, trans, rot):
    pass
    
  '''
  Callback that runs each time a sim pose is received. Should publish an updated
  pose of the clone.
  In:
    msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
  '''  
  def update_pose(self, msg):
    
    yaw_angle = quaternion_to_angle(msg.pose.orientation)
    computed_pose = [msg.pose.position.x, msg.pose.position.y, yaw_angle]
    computed_pose[0] += self.follow_offset*np.cos(yaw_angle)
    computed_pose[1] += self.follow_offset*np.sin(yaw_angle)
    
    if self.force_in_bounds:
        map_pixel = world_to_map(computed_pose, self.map_info)
        if(self.map_img[map_pixel[1]][map_pixel[0]] == False):
            self.follow_offset *= -1
        
    new_pose = PoseStamped()
    new_pose.header = Header(stamp=rospy.Time.now(), frame_id=msg.header.frame_id)
    new_pose.pose.position.x = computed_pose[0]
    new_pose.pose.position.y = computed_pose[1]
    new_pose.pose.position.z = 0.0
    new_pose.pose.orientation = angle_to_quaternion(computed_pose[2])
    self.pub.publish(new_pose)

if __name__ == '__main__':

  rospy.init_node('clone_follower', anonymous=True)
  follow_offset = rospy.get_param('~follow_offset', 0.0)
  force_in_bounds = rospy.get_param('~force_in_bounds', False)
  cf = CloneFollower(follow_offset, force_in_bounds)
  rospy.spin() 