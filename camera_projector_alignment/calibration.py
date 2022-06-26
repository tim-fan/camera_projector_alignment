import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import itertools
import json

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')
        self.projector_image_pub = self.create_publisher(Image, 'projector_image', 10)

        self.last_camera_image = None

        self.cv_bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            Image,
            'camera_image',
            self.camera_callback,
            10)
        self.camera_sub 

    
    def camera_callback(self, img_msg:Image):
        self.last_camera_image = img_msg

    def publish_projector_img(self, projector_image_msg: Image):
        self.projector_image_pub.publish(projector_image_msg)

    def last_camera_image_as_np(self) -> np.array:
        camera_image = self.cv_bridge.imgmsg_to_cv2(self.last_camera_image)
        if len(camera_image.shape) == 3:
            camera_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)
        return camera_image

    def project_and_take_photo(self, projector_image:np.array) -> np.array:
        """
        Given an image to project (np array)
        project it
        wait a bit, 
        return the associated camera frame (as np array)
        """
        projector_image_msg = self.cv_bridge.cv2_to_imgmsg(projector_image)
        self.publish_projector_img(projector_image_msg)
        self.sleep_for_seconds(0.5)
        if self.last_camera_image is not None:
            return self.last_camera_image_as_np()
        else:
            self.get_logger().error("Failed to receive camera image for projection")
            return None

    def sleep_for_seconds(self, seconds:float):
        clock = self.get_clock()
        start_time = clock.now()
        while clock.now() < start_time + Duration(seconds=seconds):
            rclpy.spin_once(self, timeout_sec=0.001)


def main(args=None):
    rclpy.init(args=args)

    projector_image_shape = (720, 1280)
    n_samples = 3
    dilatation_size = 10
    # dilatation_size = 0

    pixel_dilating_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
                                       
    u_coords = np.linspace(dilatation_size, projector_image_shape[1]-dilatation_size-1, n_samples, dtype=np.integer)
    v_coords = np.linspace(dilatation_size, projector_image_shape[0]-dilatation_size-1, n_samples, dtype=np.integer)
    projector_pixel_samples = np.array(list(itertools.product(u_coords, v_coords)))
    projector_image_blank = np.zeros(projector_image_shape, dtype=np.uint8)
    calibrationNode = CalibrationNode()

    calibrationNode.get_logger().info("Begin projecting calibration pattern")

    # take camera image with projector off
    camera_image_no_projection = calibrationNode.project_and_take_photo(projector_image_blank)

    camera_detection_pixels = np.zeros_like(projector_pixel_samples)
    for i, projector_pixel in enumerate(projector_pixel_samples):

        projector_image = projector_image_blank.copy()
        projector_image[projector_pixel[1], projector_pixel[0]] = np.iinfo(projector_image.dtype).max
        projector_image = cv2.dilate(projector_image, pixel_dilating_element)

        camera_image = calibrationNode.project_and_take_photo(projector_image)

        # # save debug images:
        # cv2.imwrite("/tmp/camera_image.png", camera_image)
        # cv2.imwrite("/tmp/camera_image_no_projection.png", camera_image_no_projection)

        difference_image = np.abs(camera_image.astype(np.int32) - camera_image_no_projection).astype(np.uint8)

        # to find centroid of projected spot, blur then find max
        # Note blur size should be independent param from dilation size
        # - relates to size of the spot in the camera image

        # gaussian blur kernel size needs to be odd int
        gaussian_kernel_size = int(np.ceil(dilatation_size/2)*4+1) 
        difference_image = cv2.GaussianBlur(difference_image, (gaussian_kernel_size, gaussian_kernel_size), 0)


        # gaussian blur then find peak?
        # publish diff image for comparison?
        detected_pixel = np.flip(np.unravel_index(difference_image.argmax(), difference_image.shape))
        # print(f"{projector_pixel=},\t{detected_pixel=}")

        # annotated_diff_image = cv2.cvtColor(difference_image, cv2.COLOR_GRAY2BGR)
        # cv2.circle(annotated_diff_image,tuple(detected_pixel), 3, (0,0,255), -1)
        # cv2.imshow("annotated_diff_image", annotated_diff_image)
        # cv2.waitKey(0) 

        camera_detection_pixels[i,:] = detected_pixel

    # result: we now have a mapping of projector coords to image coords
    calibrationNode.get_logger().info("Finished projecting calibration pattern")

    print("Projection to detected (camera frame) pixel mappings:")
    for p,d in zip(projector_pixel_samples, camera_detection_pixels):
        print(f"{p=},\t{d=}")

    # Compute homography
    M, _ = cv2.findHomography(projector_pixel_samples, camera_detection_pixels)
    print()
    print(f"homography:")
    print(M)
    print()


    # draw projector image bounds on camera image
    # find where the projector extents lie in the camera image
    height, width = projector_image_shape
    corners = np.array([
    [0, 0],
    [0, height - 1],
    [width - 1, height - 1],
    [width - 1, 0]
    ])
    corners = cv2.perspectiveTransform(np.float32([corners]), M)[0].astype(np.int32)
    annotated_bounds_image = calibrationNode.last_camera_image_as_np()
    if len(camera_image.shape) == 2:
        annotated_bounds_image = cv2.cvtColor(annotated_bounds_image, cv2.COLOR_GRAY2BGR)
    for detected_point in camera_detection_pixels:
        cv2.circle(annotated_bounds_image,tuple(detected_point), 10, (255,0,0), -1)
    cv2.polylines(annotated_bounds_image,[corners.reshape((-1,1,2))], True, (0,0,255), 3)
    cv2.imshow("annotated_bounds_image (press any key)", annotated_bounds_image)
    cv2.waitKey(0) 

    # write cal file
    calibration = dict(
        projector_img_size = (projector_image_shape[1], projector_image_shape[0]),
        camera_img_size = (calibrationNode.last_camera_image.width, calibrationNode.last_camera_image.height),
        M=M.tolist(),
    )
    calibration_str = json.dumps(calibration, indent=4)
    cal_filename = "/tmp/projector_calibration.json"
    with open(cal_filename, "w") as cal_file:
        cal_file.write(calibration_str)

    print(f"calibration saved to {cal_filename}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
