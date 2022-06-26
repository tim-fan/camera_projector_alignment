"""
Projector align

Node to subscribe to image specifying the desired projection in the image frame,
and publish the required projector image to achieve this.
Accounts for the perspective difference between the camera and projector
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import json


# from https://stackoverflow.com/a/27871067
def cv_size(img):
    return (img.shape[1], img.shape[0])

class ProjectorAlignmentNode(Node):

    def __init__(self):
        super().__init__('projector_alignment_node')
        self.projector_image_pub = self.create_publisher(Image, 'projector_image', 10)
        self.cv_bridge = CvBridge()

        cal_filename = "/tmp/projector_calibration.json"
        self.read_calibration_from_file(cal_filename)

        self.camera_sub = self.create_subscription(
            Image,
            'desired_projection',
            self.camera_callback,
            10)
        self.camera_sub 

    def camera_callback(self, img_msg:Image):
        camera_img = self.cv_bridge.imgmsg_to_cv2(img_msg)
        assert cv_size(camera_img) == self.camera_img_size, f"Error: received camera image size {cv_size(camera_img)} does not match calibrated size {self.camera_img_size}"
        camera_image_warped = cv2.warpPerspective(
            camera_img, self.M, self.projector_img_size, flags=cv2.WARP_INVERSE_MAP)
        projector_image_msg = self.cv_bridge.cv2_to_imgmsg(camera_image_warped, encoding="bgr8")
        self.projector_image_pub.publish(projector_image_msg)
    
    def read_calibration_from_file(self, filename:str):
        with open(filename) as cal_file:
            calibration = json.load(cal_file)
        self.M = np.array(calibration["M"])
        self.projector_img_size = tuple(calibration["projector_img_size"])
        self.camera_img_size = tuple(calibration["camera_img_size"])

def main(args=None):
    rclpy.init(args=args)
    
    projector_alignment_node = ProjectorAlignmentNode()
    rclpy.spin(projector_alignment_node)
    projector_alignment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
