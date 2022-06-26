"""
Project grid

Publishes a grid pattern image to /desired_projection topic
Used for testing alignment
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import json
from typing import Tuple

def create_grid_image(shape:Tuple[int, int], band_thickness_pixels:int) -> np.array:
    img = np.full(shape, False)
    u = 0 
    v = 0 

    while u < shape[1]:
        img[:,u:u+band_thickness_pixels] = np.bitwise_not(img[:,u:u+band_thickness_pixels])
        u += 2*band_thickness_pixels

    while v < shape[0]:
        img[v:v+band_thickness_pixels, :] = np.bitwise_not(img[v:v+band_thickness_pixels, :])
        v += 2*band_thickness_pixels

    img = img.astype(np.uint8)
    img = img * np.iinfo(img.dtype).max
    return img

class GridProjectorNode(Node):

    def __init__(self):
        super().__init__('grid_projector')
        self.projector_image_pub = self.create_publisher(Image, 'desired_projection', 10)

        cal_filename = "/tmp/projector_calibration.json"
        self.read_calibration_from_file(cal_filename)
        cv_bridge = CvBridge()
        grid_image = create_grid_image((self.camera_img_size[1], self.camera_img_size[0]), 50)
        grid_image = cv2.cvtColor(grid_image, cv2.COLOR_GRAY2BGR)
        self.grid_img_msg = cv_bridge.cv2_to_imgmsg(grid_image, encoding="bgr8")
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def read_calibration_from_file(self, filename:str):
        with open(filename) as cal_file:
            calibration = json.load(cal_file)
        self.camera_img_size = tuple(calibration["camera_img_size"])

    def timer_callback(self):
        self.projector_image_pub.publish(self.grid_img_msg)


def main(args=None):
    rclpy.init(args=args)    

    grid_projector_node = GridProjectorNode()
    grid_projector_node.get_logger().info('Begin publishing')
    rclpy.spin(grid_projector_node)

    grid_projector_node.get_logger().info('Shutdown')
    grid_projector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
