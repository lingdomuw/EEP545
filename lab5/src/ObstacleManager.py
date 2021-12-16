import math
import numpy
import Utils
import rospy
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap


class ObstacleManager(object):

	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))

		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height
		self.mapWidth = width
		self.mapChannels = channels

		# Binarize the Image
		self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
		self.mapImageBW[self.mapImageGS == 0] = 0

		# Obtain the car length and width in pixels
		self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
		self.robotLength = int(car_length / self.map_info.resolution + 0.5)
		self.collision_delta = collision_delta

	# Check if the passed config is in collision
	# config: The configuration to check (in meters and radians)
	# Returns False if in collision, True if not in collision
	def get_state_validity(self, config):

		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		'''
		if len(config)==3: # configuration in meters and radians
			mapConfig = Utils.world_to_map(config, self.map_info)
			[config_x,config_y]=[mapConfig[0],mapConfig[1]]
			print([config_x,config_y])

		elif len(config)==2:
			[config_x,config_y]=[config[0],config[1]]
			print([config_x,config_y])
		else:
			raise TypeError(config +"is not a valid configuration")
		'''
		mapConfig = Utils.world_to_map(config, self.map_info)
		[config_x,config_y]=[mapConfig[0],mapConfig[1]]

		# ---------------------------------------------------------
		# YOUR CODE HERE
		#
		# Return true or false based on whether the robot's configuration is in collision
		# Use a square to represent the robot, return true only when all points within
		# the square are collision free
		#
		# Also return false if the robot is out of bounds of the map
		#
		# Although our configuration includes rotation, assume that the
		# square representing the robot is always aligned with the coordinate axes of the
		# map for simplicity
		# ----------------------------------------------------------

		
		# Represent the robot using a squaure with length=sqrt(robot_width**2+robot_length**2)
		square_length=math.sqrt(pow(self.robotLength,2)+pow(self.robotWidth,2))
		half_length=int(math.ceil(0.5*square_length))
		#print(half_length)		
		boundries=numpy.array([config_x-half_length,config_x+half_length,config_y-half_length,config_y+half_length])
		#print(boundries)
		if (boundries.any()<0) or boundries[1]>self.mapWidth or boundries[3]>self.mapHeight:
			#print("Out of boundry!")
			return False
		elif self.mapImageBW[boundries[2]:boundries[3],boundries[0]:boundries[1]].sum():
			#print ("Collision!")
			return False

		return True

	# Discretize the path into N configurations, where N = path_length / self.collision_delta
	#
	# input: an edge represented by the start and end configurations
	#
	# return three variables:
	# list_x - a list of x values of all intermediate points in the path
	# list_y - a list of y values of all intermediate points in the path
	# edgeLength - The euclidean distance between config1 and config2
	def discretize_edge(self, config1, config2):
		list_x, list_y = [], []
		edgeLength = 0
		# -----------------------------------------------------------
		# YOUR CODE HERE
		# -----------------------------------------------------------
		
		# Convert the configuration to map-coordinates in pixel-space
		#config1_p=Utils.world_to_map(config1,self.map_info)
		#config2_p=Utils.world_to_map(config2,self.map_info)

		
		edgeLength = numpy.linalg.norm(numpy.array(config2)-numpy.array(config1))
		assert edgeLength>0, "Configuration 1 and Configuration 2 are the same! No edge can be discretized!"

		N = int(edgeLength / self.collision_delta)
		list_x = numpy.linspace(config1[0], config2[0], N)
		list_y=numpy.linspace(config1[1], config2[1], N)		
		return list_x, list_y, edgeLength


	# Check if there is an unobstructed edge between the passed configs
	# config1, config2: The configurations to check (in meters and radians)
	# Returns false if obstructed edge, True otherwise
	def get_edge_validity(self, config1, config2):
		# -----------------------------------------------------------
		# YOUR CODE HERE
		#
		# Check if endpoints are obstructed, if either is, return false
		# Find path between two configs by connecting them with a straight line
		# Discretize the path with the discretized_edge function above
		# Check if all configurations along path are obstructed
		# -----------------------------------------------------------

		if not self.get_state_validity(config1):
			#print("Configuration 1 is not valid!")
			return False
		elif not self.get_state_validity(config2):
			#print("Configuration 2 is not valid!")
			return False
		elif numpy.linalg.norm(numpy.array(config2)-numpy.array(config1))==0:
			return False
		else:	
			list_x,list_y,edgeLength =self.discretize_edge(config1,config2)
			for x,y in zip(list_x,list_y):
				if not self.get_state_validity([x,y]):
					return False
		return True


# Write Your Test Code For Debugging

if __name__ == '__main__':
	# Write test code here!
	rospy.init_node('obstacle_manager_test',anonymous=True)
	map_service_name = rospy.get_param("~static_map", "static_map")
	print("Getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)
	
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
	car_width= 0.33
	car_length= 0.33
	collision_delta=0.05

	



	om=ObstacleManager(map_msg, car_width, car_length, collision_delta)
	print(om.mapImageGS.shape)
	print(om.robotLength)
	config1 = [156, 1080] # invalid
	config2 = [30, 20]  # valid
	print(map_msg.info.resolution)
	#print(Utils.world_to_map(config1,map_msg.info))
	#print(Utils.world_to_map(config2,map_msg.info))
	print(type(om.mapImageBW))
	plt.imshow(om.mapImageBW[:,:,0],cmap='gray')
	plt.show()
	print(om.get_state_validity(config1))
	print(om.get_state_validity(config2))
	print(om.get_edge_validity(config1,config2))


