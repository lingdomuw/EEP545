#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import Utils as utils

# The topic to publish control commands to
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
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
               rotation_weight, kp, ki, kd, error_buff_length, speed):
    # Store the passed parameters
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    self.integ_error = 0
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher (PUB_TOPIC, AckermannDriveStamped, queue_size=10) # Create a publisher to PUB_TOPIC
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)  # Create a subscriber to pose_topic, with callback 'self.pose_cb'
    
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):    
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
    
    while len(self.plan) > 0:
      
      # YOUR CODE HERE
      #print(cur_pose[2])
      #rotation coord to make x of car direction as x of map axis
      car_rotation_matrix = utils.rotation_matrix(-cur_pose[2])
      car_coord = car_rotation_matrix*np.matrix([[cur_pose[0]], [cur_pose[1]]])
      plan_coord = car_rotation_matrix*np.matrix([[self.plan[0][0]], [self.plan[0][1]]])
      #print("ori",np.matrix([[cur_pose[0]],[ cur_pose[1]]]),np.matrix([[self.plan[0][1]], [self.plan[0][0]]]))
      
      #print(cur_pose[2])
      #print(cur_pose[0], cur_pose[1])
      #print("rot:",car_coord)
      #print("ori:",plan_coord)
      
      #car toward direction is x-axis. checking the configuration is in behind of car   
      if (plan_coord[0,0]-car_coord[0,0]) < 0 and (plan_coord[0,0]-car_coord[0,0]) >-0.2 :#\
          #and (plan_coord[1,0]-car_coord[1,0]) < 0.5 and (plan_coord[1,0]-car_coord[1,0]) >-0.5 :
        
        del self.plan[0]
      else: 
        break     
    # Check if the plan is empty. If so, return (False, 0.0)
    # YOUR CODE HERE
    if len(self.plan) == 0:
      return False,0.0
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
   
    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    # YOUR CODE HERE    
    #translation_error =  np.arctan((self.plan[goal_idx][1]-cur_pose[1])/(self.plan[goal_idx][0]-cur_pose[0]))   

    car_rotation_matrix = utils.rotation_matrix(-cur_pose[2])
    car_coord = car_rotation_matrix*np.matrix([[cur_pose[0]], [cur_pose[1]]])
    plan_coord = car_rotation_matrix*np.matrix([[self.plan[goal_idx][0]], [self.plan[goal_idx][1]]])
    translation_error = np.arctan((plan_coord[1,0]-car_coord[1,0])/(plan_coord[0,0]-car_coord[0,0]))
    
    '''
    m = np.matrix([[car_coord[0,0]],[car_coord[1,0]],[0],[1]])
    n = np.matrix([[plan_coord[0,0]-car_coord[0,0]],[plan_coord[1,0]-car_coord[1,0]],[0],[1]])
    trans_error = np.linalg.norm(n - m)
    print(trans_error,translation_error)
    translation_error =trans_error
    '''       
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error
    # YOUR CODE HERE
    
    rotation_error = self.plan[goal_idx][2]-cur_pose[2]    
    
    error =  self.translation_weight * translation_error + self.rotation_weight * rotation_error # self.translation_weight * translation_error + self.rotation_weight * rotation_error
    #print(self.plan[0][2])
    return True, error
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    # YOUR CODE HERE
    
    if len(self.error_buff) == 0:
      time_period = 0
      deriv_error = 0
    else:
      time_period = now-self.error_buff[-1][1]
      deriv_error = (error-self.error_buff[-1][0])/time_period
    # Add the current error to the buffer
    self.error_buff.append((error, now))
    
    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
    # YOUR CODE HERE
    #global integ_error
    integ_error = self.integ_error
    integ_error = integ_error+error*time_period
    ''''
    if integ_error>0.7854: #~45 degree
      integ_error = 0.7854
    elif integ_error<-0.7854:
      integ_error = -0.7854
    '''
    # Compute the steering angle as the sum of the pid errors
    # YOUR CODE HERE
    return self.kp*error + self.ki*integ_error + self.kd * deriv_error #self.kp*error + self.ki*integ_error + self.kd * deriv_error
    
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
    #print(error)
    if not success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle(error)
    #print("delta",delta)
    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)

def main():

  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  
  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # YOUR CODE HERE
  plan_topic = '/PlannerNode/car_plan' # Default val: '/planner_node/car_plan'
  pose_topic = '/pf/viz/inferred_pose'  # Default val: '/car/pose'
  plan_lookahead = 2 # Starting val: 5
  translation_weight = 2 # Starting val: 1.0
  rotation_weight = 0.0 # Starting val: 0.0
  kp = 2.5 # Startinig val: 1.0
  ki = 0.4 # Starting val: 0.0
  kd = 0.5 # Starting val: 0.0
  error_buff_length = 10 # Starting val: 10
  speed = 2 # Default val: 1.0
  
  # Populate params with values passed by launch file
  try:
    # Values from Launch file
    plan_topic = rospy.get_param("~plan_topic", None)
    pose_topic = rospy.get_param("~pose_topic", None)
    plan_lookahead = rospy.get_param("~plan_lookahead", None)
    translation_weight = rospy.get_param("~translation_weight", None)
    rotation_weight = rospy.get_param("~rotation_weight", None)
    kp = rospy.get_param("~kp", None)
    ki = rospy.get_param("~ki", None)
    kd = rospy.get_param("~kd", None)
    error_buff_length = rospy.get_param("~error_buff_length", None)
    speed = rospy.get_param("~speed", None)
  except:
    pass

  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press
  
  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  # YOUR CODE HERE
  datas = rospy.wait_for_message(plan_topic,PoseArray)
  plan = [np.array([data.position.x,
                         data.position.y,
                         utils.quaternion_to_angle(data.orientation)]) for data in datas.poses]

  
  lf = LineFollower(plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed)
  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':  
  main()

