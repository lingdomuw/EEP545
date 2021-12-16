import heapq
import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random
from heapq import *
import copy

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
   
    #priority_q = [] # priority queue
    #heappush(priority_q,(self.open[self.sid],self.sid))
    print("Starting Node:")
    print(self.sid,self.tid)
    while self.open:

      #(current_f,current_id) = heappop(priority_q) # current=[cost,id]
      current_id = min(self.open, key=self.open.get) # get the corresponding key of the minium value in the dict

      # Early Exit
      if current_id == self.tid:
        break

      current_f = self.open.pop(current_id)
      self.closed[current_id] = current_f
      adj = self.planningEnv.get_successors(current_id) #adjacent nodes
      #print(adj)
      for child_id in adj:
        if child_id in self.closed:
          continue
        #edge collision check
        parentConfig = self.planningEnv.get_config(current_id)
        childConfig = self.planningEnv.get_config(child_id) #current adjacent node
        if not self.planningEnv.manager.get_edge_validity(parentConfig, childConfig):
          continue
        child_g = self.gValues[current_id] + self.planningEnv.get_distance(current_id, child_id)
        child_h = self.planningEnv.get_heuristic(child_id,self.tid)
        child_f = child_g+child_h
        if child_id in self.open:
          if child_g > self.gValues[child_id]:
            continue
        #print("Child Node:")
        #print(child_id)
        self.gValues[child_id] =  child_g
        self.open[child_id] = child_f 
        self.parent[child_id] = current_id  
        #heappush(priority_q,(self.open[child_id],child_id))
    #return  self.post_process(self.get_solution(self.tid)) #here we return get_solution(selected_indices)
    return self.get_solution(self.tid)

  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  def post_process(self, plan, timeout):
    #print("Original Plan")
    #print(type(plan))
    t1 = time.time()
    elapsed = 0
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
     
      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly
      i, j = 0, 0
      while i == j:
        i, j = random.randint(0, len(plan)-1), random.randint(0, len(plan)-1)
      if i > j:
        i, j = j, i
      config_i = plan[i]
      config_j= plan[j]

      #print(config_i,config_j,i,j)
      if self.planningEnv.manager.get_edge_validity(config_i, config_j):
        new_plan=plan[:]
        del new_plan[i+1:j]
        px, py, clen = self.planningEnv.manager.discretize_edge(config_i, config_j)
        path_insert = [list(a) for a in zip(px, py)]
        
        path_insert.reverse() #reverse the list for inserting element by element
        for insert in path_insert[1:-1]:
            new_plan.insert(i+1,insert)
        plan=new_plan
        #print(plan)
        path_length=0
        for i in range(len(plan)-1): 
          config1=plan[i]
          config2=plan[i+1]
          edge_length=numpy.linalg.norm(numpy.array(config2)-numpy.array(config1))
          path_length+=edge_length
        #print("Path length updated")
        self.cost=path_length
        #print("Post Processed Plan")
        #print(type(plan))
      elapsed = time.time() - t1

    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    #print 'Plan length:'
    #print(self.cost)
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)
    cv2.imwrite('/home/robot/catkin_ws/src/final/path_graphs/simulation.png',envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
