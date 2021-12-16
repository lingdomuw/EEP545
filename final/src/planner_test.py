#!/usr/bin/env python

import rospy
import numpy as np
from final.srv import *
import Utils
import csv
import os
import math
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose,PoseArray,PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray

PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'

# Testing waypoint poses:
START_WAYPOINT_CSV = '/waypoints/real_car/start.csv'
GOOD_WAYPOINT_CSV = '/waypoints/real_car/good_waypoints.csv'
#BAD_WAYPOINT_CSV = '/waypoints/real_car/bad_waypoints.csv'


def get_waypoint(path):
  dir_path = str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  print(dir_path)
  csv_path = dir_path + path
  pose = []
  with open(csv_path) as csv_file: 
    csv_reader = csv.reader(csv_file, delimiter=',')
    count = 0
    for row in csv_reader:
      if count == 0:
        count += 1
        continue
      else:
        new_pose = [float(row[0]), float(row[1]), 0.0]
        pose.append(new_pose)
  return pose

def waypoint_map2world(waypoints,mapinfo):
  world_waypoints = []
  for i in waypoints:
    world_waypoints.append(Utils.map_to_world(i,mapinfo))
  return world_waypoints

def get_path_poses(start_point, good_points): 
  path_poses = []
  path_poses.append(start_point[0])
  pool = good_points[:]
  curr = path_poses[0]
  while pool:
    mindist = 99999.99 
    minidx = 0
    for i in range(len(pool)):
      curr_dist = min(np.linalg.norm(curr-pool[i]), mindist)
      if curr_dist < mindist:
        minidx = i
        mindist = curr_dist
    curr = pool[minidx]
    path_poses.append(curr)
    del pool[minidx]
  return path_poses

def get_plan_posearray(plan):
    pa = PoseArray()
    pa.header.frame_id = "/map"
    for i in xrange(len(plan)):
      config = plan[i]
      #print(type(config[2]))
      pose = Pose()
      pose.position.x = config[0]
      pose.position.y = config[1]
      pose.position.z = 0.0
      pose.orientation = Utils.angle_to_quaternion(config[2])
      pa.poses.append(pose)
    return pa

def add_orientation(plan):
  source_yaw=0.0
  target_yaw=0.0
  orientation_window_size = 21

  print("Path Plan Shape")
  print(plan.shape)
  oriented_plan = np.zeros((plan.shape[0],3),dtype=np.float)
  oriented_plan[:,0:2] = plan[:,:]

  if plan.shape[0] >= 2:  
    oriented_plan[0,2] = source_yaw
    oriented_plan[oriented_plan.shape[0]-1, 2] = target_yaw   
        
    plan_diffs = np.zeros(plan.shape, np.float)
    plan_diffs[0:plan_diffs.shape[0]-1] = plan[1:plan.shape[0]]-plan[0:plan.shape[0]-1]
    plan_diffs[plan_diffs.shape[0]-1] = np.array([np.cos(target_yaw), np.sin(target_yaw)], dtype=np.float) 
  
    avg_diffs = np.empty(plan_diffs.shape, dtype=np.float)
    for i in xrange(plan_diffs.shape[0]):
      avg_diffs[i] = np.mean(plan_diffs[np.max((0,i-orientation_window_size/2)):
                                        np.min((plan_diffs.shape[0]-1, i+orientation_window_size/2+1))],
                              axis=0)

    oriented_plan[1:oriented_plan.shape[0]-1,2] = np.arctan2(avg_diffs[1:oriented_plan.shape[0]-1,1],
                                                              avg_diffs[1:oriented_plan.shape[0]-1,0])
  
  elif plan.shape[0] == 2:
    oriented_plan[:,2] = np.arctan2(plan[1,1]-plan[0,1],plan[1,0]-plan[0,0])

  return oriented_plan

def add_intermediate_waypoints(path_poses,index,n):

  for i in range(n,0,-1):
    im_waypoint=i*(path_poses[index+1]-path_poses[index])/(n+1)+path_poses[index]
    path_poses.insert(index+1,im_waypoint)


if __name__ == '__main__':

    rospy.init_node('planner_test', anonymous=True)
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

    start_waypoint_m=get_waypoint(START_WAYPOINT_CSV)
    start_waypoint_w=waypoint_map2world(start_waypoint_m,map_info)
    good_waypoints_m=get_waypoint(GOOD_WAYPOINT_CSV)
    good_waypoints_w=waypoint_map2world(good_waypoints_m,map_info)
    print(start_waypoint_m,good_waypoints_m)

    path_poses=get_path_poses(start_waypoint_w,good_waypoints_w)
    print(path_poses)
    path_poses.insert(1,np.array([51.525,10.526,0.0]))

    path_poses.insert(1,np.array([50.111,10.480,0.0]))
    #path_poses.insert(4,np.array([40.177,17.300,0.0]))

    print(path_poses)

    #add_intermediate_waypoints(path_poses,0,3)    # add intermediate waypoints to avoid sharp turns
    pub_topic='/PlannerNode/car_plan'
    #path_poses = [float(x) for x in path_poses]

    #print(type(path_poses[0]))

    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=3)
    #full_poses_pub = rospy.Publisher('/fullposes', Float32MultiArray, queue_size=10)
    plan_pub = rospy.Publisher(pub_topic, PoseArray, queue_size=1)            

    rospy.sleep(3)
    ip=PoseWithCovarianceStamped()
    ip.header.frame_id = "/map"
    ip.header.stamp = rospy.Time()
    ip.pose.pose.position.x = 49.999
    ip.pose.pose.position.y = 11.919
    ip.pose.pose.position.z = 0.0
    ip.pose.pose.orientation.x = 0.0
    ip.pose.pose.orientation.y = 0.0
    ip.pose.pose.orientation.z = -0.7062
    ip.pose.pose.orientation.w = 0.7079
    
    
    pa=PoseArray()
    
    #fp=Float32MultiArray()
    #print(np.array(path_poses).reshape(-1,3))
    #fp.data=np.array(path_poses).reshape(-1,3).astype(float).tolist()
    #full_poses_pub.publish(fp)
    #initial_pose=Utils.map_to_world(start_waypoint[0],map_info)
    #print(initial_pose)
    
    full_plan=[]
    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)
    #resp = get_plan(path_poses[0],path_poses[1])
    #plan_list=np.array(resp.plan).reshape(-1, 3).tolist()
    #full_plan.extend(plan_list)
    for i in range(len(path_poses)-1):
      try:
          resp = get_plan(path_poses[i],path_poses[i+1])
          #full_plan.extend(resp.plan) 
          #print(resp.plan)
          full_plan.extend(resp.plan)
          #print(full_plan)
          print resp.success
      except rospy.ServiceException, e:
          print 'Service call failed: %s' % e

    if (plan_pub is not None) and (full_plan is not None):
      full_plan=add_orientation(np.array(full_plan).reshape(-1,2)).reshape(-1,3)
      print(full_plan.shape)
      full_plan=full_plan.tolist()
      
      while not rospy.is_shutdown():
        raw_input("Press Enter to set initial pose...")  # Waits for ENTER key press
        initial_pose_pub.publish(ip)

        raw_input("Press Enter to publish the full car plan...")

        pa=get_plan_posearray(full_plan)
        print("Publishing Full Plan")
        plan_pub.publish(pa)
        rospy.sleep(1.0)

     