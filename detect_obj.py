###################################################################
#TeamVegeta - Abirath Raju & Sreeranj Jayadevan
###################################################################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge

def read_rgb_image(image_name, show):
    color_image = cv2.imread(image_name)
    return color_image

def filter_color(color_image, low_color, high_color):
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, low_color, high_color)
    return mask

def getContours(binary_image):
    (contours, hierarchy) = cv2.findContours(binary_image.copy(),
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_ball_contour(binary_image, color_image, contours):
    (x_coord, y_coord) = (np.NaN, np.NaN)
    black_image = np.zeros([binary_image.shape[0],binary_image.shape[1], 3], 'uint8')
    for c in contours:
        area = cv2.contourArea(c)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        (x_coord, y_coord) = (x, y)
        if area > 3000:
            cv2.drawContours(color_image, [c], -1, (150, 250, 150), 1)
            cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
            (cx, cy) = get_contour_center(c)
            cv2.circle(color_image, (cx, cy), int(radius), (0, 0, 255),1)
            cv2.circle(black_image, (cx, cy), int(radius), (0, 0, 255),1)
            cv2.circle(black_image, (cx, cy), 5, (150, 150, 255), -1)

    return (x_coord, y_coord)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    return (cx, cy)

def detect_ball_in_a_frame(image_frame):
    yellowLower = (20, 100, 100)
    yellowUpper = (30, 255, 255)
    color_image = image_frame
    binary_image_mask = filter_color(color_image, yellowLower,yellowUpper)
    contours = getContours(binary_image_mask)
    (x, y) = draw_ball_contour(binary_image_mask, color_image, contours)
    return (x, y)

class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', 'Raw Image')
        image_qos_profile = \
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self._video_subscriber = self.create_subscription(CompressedImage,'/image_raw/compressed', self._image_callback,image_qos_profile)
        self._video_subscriber 
        self._point_publisher = self.create_publisher(Point, '/ball_pos', 10)
        self.x = np.NaN
        self.y = np.NaN

    def _image_callback(self, CompressedImage):
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, 'bgr8')
        (self.x, self.y) = detect_ball_in_a_frame(self._imgBGR)
        point = Point()
        point.x = float(self.x)
        point.y = float(self.y)
        point.z = 0.0        
        self._point_publisher.publish(point)
        self._user_input = cv2.waitKey(50)  
        if self._user_input == ord('q'):
            cv2	.destroyAllWindows()
            raise SystemExit

def main():
    rclpy.init() 
    video_subscriber = ObjectDetector() 
    while rclpy.ok():
        rclpy.spin(video_subscriber)  
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

