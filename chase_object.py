###################################################################
#TeamVegeta - Abirath Raju & Sreeranj Jayadevan
###################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge


class ObjectPosSubscriber(Node):
    def __init__(self):
        super().__init__('chase_object')

        image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.angle_subscription = self.create_subscription(Float64, 'desired_angle', self.angle_callback, image_qos_profile)
        self.angle_subscription
        self.distance_subscription = self.create_subscription(Float64, '/desired_distance', self.distance_callback, image_qos_profile)
        self.distance_subscription
        
        self.angle = 0.0
        self.distance = 0.0
        self.angle_des = 0.0
        self.positive_desired_threshold = 0.65
        self.negative_desired_threshold = 0.45
        self.buffer = 0.09
        self.angle_buffer = 0.2
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def angle_callback(self, angle):
        self.angle = angle.data 

    def distance_callback(self, distance):
        self.distance = distance.data

    def timer_callback(self):
        robot_vel = Twist()
        if np.isnan(self.angle) or np.isnan(self.distance): 
            robot_vel.angular.z = 0.0
            robot_vel.linear.x = 0.0
        else:
            # Discard all noise values
            if abs(self.angle) > self.angle_des + self.angle_buffer:
                error_z = self.angle - self.angle_des
                # P controller
                z_val = 0.5 * error_z
                if z_val < 0: 
                    z_val = max(z_val, -0.12)
                else:
                    z_val = min(z_val, 0.15)
            else:
                z_val = 0.0
            if self.distance > self.positive_desired_threshold :
                error_x = self.distance - 0.56
                x_val = max((0.9 * error_x), 0.03)
                x_val = min(x_val, 0.22) 
            elif self.distance < self.negative_desired_threshold :
                error_x = self.distance - 0.56
                x_val = max(error_x, -0.22) 
            else:
                x_val = 0.0
            robot_vel.linear.x = x_val
            robot_vel.angular.z = z_val
        self.publisher.publish(robot_vel)

def main():
    rclpy.init() 
    robot_node = ObjectPosSubscriber() 
    while rclpy.ok():
        rclpy.spin(robot_node) 	
    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
		
