#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from collections import deque

import utils

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
    self.iteration_counter=0

    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = deque(maxlen=error_buff_length)
    self.speed = speed
    
    # Publisher 
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size =10)

    # Create a subscriber to pose_topic, with callback 'self.pose_cb'
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, queue_size = 1)

  
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

    #Current pose of  the car     
    cur_pose_x = cur_pose[0]
    cur_pose_y = cur_pose[1]
    cur_pose_th = cur_pose[2]
    
    while True:
      try:
        [target_x, target_y, target_th] = self.plan.popleft()

        # Vector between ith pose and the current car pse
        Goalvector = [target_x - cur_pose_x, target_y - cur_pose_y]

        # Unit vector in the direction of the car's current pose 
        curPoseVector = [np.cos(cur_pose_th), np.sin(cur_pose_th)]

        dotProduct = np.dot(Goalvector, curPoseVector)

        # If dot product is positive value, then the target pose is in front 
        if dotProduct > 0:   
          self.plan.appendleft(np.array([target_x, target_y, target_th])) #if the target pose is in front, append it 
          #print("self.plan length")
          #print(len(self.plan))
          break
      except IndexError:
        # self.plan is empty, so return (False, 0.0)
        print("THIS SHOULD BE THE END OF MSGs")
        return (False, 0.0)

    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index

    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
    [goal_idx_x, goal_idx_y, goal_idx_th] = self.plan[goal_idx]

    ### Compute the translation error between the robot and the configuration at goal_idx in the plan

    # Car left vector
    car_left_vector = np.array([np.cos(cur_pose_th+np.pi/2), np.sin(cur_pose_th+np.pi/2)])

    
    E = np.array([cur_pose_x - goal_idx_x, cur_pose_y - goal_idx_y]) # Error vector
    E_hat = E/np.linalg.norm(E) # Unit error vector


    # Use the cross product to calculate the distance error to capture positive or negative wrt to goal_unit_vector
    translation_error = -np.dot(E_hat, car_left_vector)
    rotation_error= goal_idx_th - cur_pose_th  # Rotation error is the difference in yaw between the robot and goal configuration
  
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error

    error = self.translation_weight * translation_error + self.rotation_weight * rotation_error

    return (True, error)
    
    
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
    deriv_error = 0.0
    if len(self.error_buff) > 0:

      [last_time, last_error] = self.error_buff[-1]
      deriv_error = (error - last_error) / (now - last_time)




    # Compute the integral error by applying rectangular integration to the elements
    integ_error = 0.0
    next_time = now 
    next_error= error
    for elem in reversed(self.error_buff): 

      error_element = (elem[1]+next_error)/2 * (next_time - elem[0])
      integ_error += error_element

      # update next time and error 
      next_time = elem[0]
      next_error= elem[1]


    # Add the current error to the buffer
    self.error_buff.append((now,error))

    # Compute the steering angle as the sum of the pid errors
    steering_angle = self.kp*error + self.ki*integ_error + self.kd * deriv_error
    print("kp = %f"%self.kp) 
    print("ki = %f"%self.ki) 
    print("kd = %f"%self.kd) 
  
    return steering_angle
    
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
      # We have reached our goal 
      self.pose_sub.unregister() # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops

    self.iteration_counter+=1    
    print("error = %f"%error)
    print("iteration index = %d "%self.iteration_counter)

    # write error and pose iteration index to data file for plotting
    f=open("/home/robot/catkin_ws/src/lab3/src/data.txt","a")
    f.write(str(self.iteration_counter)+", "+str(error)+"\n")
    f.close()


    delta = self.compute_steering_angle(error)

    print("Steer Angle = %f"%delta)


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

  # Default values 
  plan_topic = '/planner_node/car_plan'
  pose_topic = '/car/car_pose'
  plan_lookahead = 5
  translation_weight = 1.0
  rotation_weight = 0.0
  kp = 1.0
  ki = 0.0
  kd = 0.0
  error_buff_length = 10
  speed = 1.0

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

  # Waits for ENTER key press
  raw_input("Press Enter to when plan available...")  
  
  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  plan_msg = rospy.wait_for_message(plan_topic, PoseArray)
  plan = deque()
  for msg in plan_msg.poses:
    x=msg.position.x
    y=msg.position.y
    theta= utils.quaternion_to_angle(msg.orientation)
    pose = np.array([x,y,theta])
    plan.append(pose)

  # Create a clone follower
  lf = LineFollower(plan, pose_topic, plan_lookahead, translation_weight,
                  rotation_weight, kp, ki, kd, error_buff_length, speed) 

  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()