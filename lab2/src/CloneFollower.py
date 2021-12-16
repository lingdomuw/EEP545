#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import Utils # <----------- LOOK AT THESE FUNCTIONS ***************************

SUB_TOPIC = '/car/car_pose' # The topic that provides the simulated car pose
PUB_TOPIC = '/clone_follower_pose/pose' # The topic that you should publish to
MAP_TOPIC = '/static_map' # The service topic that will provide the map

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
    # YOUR CODE HERE 
    self.follow_offset = follow_offset# Store the input params in self
    self.force_in_bounds = force_in_bounds # Store the input params in self
    self.map_img, self.map_info = Utils.get_map(MAP_TOPIC) # Get and store the map
                                  # for bounds checking
    
    # Setup publisher that publishes to PUB_TOPIC
    self.pub = rospy.Publisher (PUB_TOPIC, PoseStamped, queue_size=10)
    
    # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
    # callback
    self.sub = rospy.Subscriber (SUB_TOPIC, PoseStamped, self.update_pose)
    
  '''
  Given the translation and rotation between the robot and map, computes the pose
  of the clone
  (This function is optional)
  In:
    trans: The translation between the robot and map
    rot: The rotation between the robot and map
  Out:
    The pose of the clone
  '''
  def compute_follow_pose(self, trans, rot):
    angle = Utils.quaternion_to_angle(rot)
    TransX = trans.x +self.follow_offset*np.cos(angle)
    TransY = trans.y +self.follow_offset*np.sin(angle)

    return [TransX, TransY, angle]
    
  '''
  Callback that runs each time a sim pose is received. Should publish an updated
  pose of the clone.
  In:
    msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
  '''  
  def update_pose(self, msg):
    followPose = self.compute_follow_pose(msg.pose.position, msg.pose.orientation)

    # Check bounds if required
    if self.force_in_bounds:
      location = Utils.world_to_map (followPose, self.map_info)
      if self.map_img[location[1]][location[0]] == 0:
        self.follow_offset *= -1
        followPose = self.compute_follow_pose(msg.pose.position, msg.pose.orientation)


    poseStp = PoseStamped()
    poseStp.header.frame_id = "/map"
    poseStp.header.stamp = rospy.Time.now()

    poseStp.pose.position.x = followPose[0]
    poseStp.pose.position.y = followPose[1]
    poseStp.pose.orientation = Utils.angle_to_quaternion(followPose[2])

    self.pub.publish(poseStp)
      
    
if __name__ == '__main__':
  follow_offset = 1.5 # The offset between the robot and clone
  force_in_bounds = False # Whether or not map bounds should be enforced
  
  rospy.init_node('clone_follower', anonymous=True) # Initialize the node
  
  # Populate params with values passed by launch file
  follow_offset =  rospy.get_param ('follow_offset')
  force_in_bounds = rospy.get_param ('force_in_bounds')


  cf = CloneFollower(follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin() # Spin
  
