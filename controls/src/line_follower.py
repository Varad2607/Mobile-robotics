#!/usr/bin/env python
import os
import collections
import sys
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import utils
import matplotlib.pyplot as plt

PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
POSE_TOPIC = '/car/car_pose'


'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed, count_pose_cb):

    self.plan = plan
    self.plan_lookahead = plan_lookahead
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
    self.pose_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, callback=self.pose_cb)
    self.count_pose_cb = count_pose_cb
  
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    print("cur_pose: ", cur_pose)
    while len(self.plan) > 0:
      print("len(self.plan): ", len(self.plan))
      print("plan[0] ", self.plan[0])
      angle = utils.coordinate_transformation(self.plan[0], cur_pose)
      print("angle: ", angle)
      if angle < 0:
        self.plan.pop(0)
      else:
        break

    if len(self.plan) == 0:
      print("plan is empty")
      return (False, 0.0)
    
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
    translation_error = -np.sin(cur_pose[2])*(self.plan[goal_idx][0] - cur_pose[0]) + np.cos(cur_pose[2]) * (self.plan[goal_idx][1] - cur_pose[1]) 
    print("translation error:",translation_error)
    rotation_error = 0 # rotation error is accounted for by the translation component, hence disregarding rotation_error
    error = self.translation_weight * translation_error + self.rotation_weight * rotation_error
    print("error: ", error)


    return True, error
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):

    now = rospy.Time.now().to_sec()

    if len(self.error_buff) > 0:
      most_recent_error, most_recent_time = self.error_buff[-1]
      derivative_error = (error - most_recent_error) / (now - most_recent_time)
    else:
      derivative_error = 0.0
    self.error_buff.append((error, now))
    
    integral_error = 0.0
    for i in range(1, len(self.error_buff)):
      previous_error, previous_time = self.error_buff[i - 1]
      current_error, current_time = self.error_buff[i]
      interval = current_time - previous_time
      integral_error += (previous_error + current_error) / 2.0 * interval

    return self.kp*error + self.ki*integral_error + self.kd * derivative_error

  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''  
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    
    success, error = self.compute_error(cur_pose)
    
    if not success:
      self.pose_sub = None
      self.speed = 0.0
      
    # Error plots
    self.count_pose_cb += 1
    pose_cb_iteration.append(self.count_pose_cb)
    error_plot.append(error)
    delta = self.compute_steering_angle(error)
    print("delta: ", delta)
    file_path_1 = os.path.abspath('/home/robot/mushr_ws/src/assignment-3-team-6/npy_files/first_parameter_set.npy')
    file_path_2 = os.path.abspath('/home/robot/mushr_ws/src/assignment-3-team-6/npy_files/second_parameter_set.npy')
    file_path_3 = os.path.abspath('/home/robot/mushr_ws/src/assignment-3-team-6/npy_files/third_parameter_set.npy')
    # np.save(file_path,np.array(error_plot))
    first_parameter_set = np.load(file_path_1)
    second_parameter_set = np.load(file_path_2)
    third_parameter_set = np.load(file_path_3)
    
    if len(self.plan) == 0:
      plt.figure()
      plt.xlabel('pose_callabck index')
      plt.ylabel('error_plot')
      plt.plot(pose_cb_iteration,first_parameter_set[:len(pose_cb_iteration)],label='first set of parameters')
      plt.plot(pose_cb_iteration,second_parameter_set[:len(pose_cb_iteration)],label='second set of parameters')
      plt.plot(pose_cb_iteration,third_parameter_set[:len(pose_cb_iteration)],label='third set of parameters')
      plt.legend(loc='best')
      plt.show()

    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed

    self.cmd_pub.publish(ads)

def main():

  rospy.init_node('line_follower', anonymous=True)
  plan_topic = '/planner_node/car_plan'
  raw_input("Press Enter to when plan available...")

  plan = []
  plan_ = rospy.wait_for_message(plan_topic, PoseArray, timeout=None).poses
  for pose in plan_:
    plan_msg_structured = np.array([pose.position.x, 
                                    pose.position.y, 
                                    utils.quaternion_to_angle(pose.orientation)])
    plan.append(plan_msg_structured)

  global count_pose_cb
  global pose_cb_iteration 
  global error_plot
  pose_cb_iteration = []
  error_plot = []
  count_pose_cb = 0
  raw_input("Press Enter to when plan available...")
  line_follower_ = LineFollower(
      plan=plan,
      pose_topic='/car/car_pose',
      plan_lookahead=rospy.get_param('~plan_lookahead'),
      translation_weight=rospy.get_param('~translation_weight'),
      rotation_weight=rospy.get_param('~rotation_weight'),
      kp=rospy.get_param('~kp'),
      ki=rospy.get_param('~ki'),
      kd=rospy.get_param('~kd'),
      error_buff_length=rospy.get_param('~error_buff_length'),
      speed = 1,
      count_pose_cb = count_pose_cb
      )
  rospy.spin()

if __name__ == '__main__':
  main()