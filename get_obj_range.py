###################################################################
#TeamVegeta - Abirath Raju & Sreeranj Jayadevan
###################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class ObjectRange(Node):
	def __init__(self):
		super().__init__('get_object_range')
		image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
		self._pos_subscriber = self.create_subscription(Point, '/ball_pos', self._angle_callback, image_qos_profile)
		self._lidar_subscriber = self.create_subscription(LaserScan, '/scan', self._distance_callback, image_qos_profile)
		self._lidar_subscriber 
		self._pos_subscriber 
		self._angle_publisher = self.create_publisher(Float64, '/desired_angle', 10)
		self._distance_publisher = self.create_publisher(Float64, '/desired_distance', 10)
		self.obj_angle = np.NaN
		self.obj_distance = np.NaN

	def _angle_callback(self, point):

		self.x = point.x
		self.y = point.y
		
		self.obj_angle = (self.x - 160) * 62.2/ 320 * (np.pi/180)
		angle = Float64()
		angle.data = self.obj_angle
		self._angle_publisher.publish(angle)


	def _distance_callback(self, scan):

		min_angle = scan.angle_min * (180/np.pi)
		if not (np.isnan(self.obj_angle)): 
			obj_angle = int(self.obj_angle) * -1
			# Angle of desired object along which we need the range
			idx = int(obj_angle - min_angle)
			temp = np.copy(scan.ranges)
			#Max value to check for objects
			max_val = 10.0
			temp[np.isnan(temp)] = max_val
			# iF there is some value of range along idx and object is within 5m
			if not (np.isnan(scan.ranges[idx])) and scan.ranges[idx] <= 5.0:
				# Get desired distance
				self.obj_distance = float(temp[idx])
				distance = Float64()
				distance.data = self.obj_distance
				self._distance_publisher.publish(distance)


		else: 
			#No object found
			distance = Float64()
			distance.data = np.NaN
			# PUblish NaN if no object found
			self._distance_publisher.publish(distance)

def main():
	rclpy.init()  
	pos_subscriber = ObjectRange()  
	while rclpy.ok():
		rclpy.spin(pos_subscriber)  
	pos_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()