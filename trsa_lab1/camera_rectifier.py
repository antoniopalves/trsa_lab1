import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os
import ament_index_python
from image_geometry import PinholeCameraModel
import cv2

class CameraRectifier(Node):
    def __init__(self):
        super().__init__('camera_rectifier')

        self.bridge = CvBridge()
        self.image_pub_rect = self.create_publisher(Image, '/camera/image_rect', 10)

        # carregar calibração
        calibration_path = os.path.join(
            ament_index_python.get_package_share_directory('trsa_lab1'),
            'calibration',
            'ost.yaml'
        )
        self.load_camera_calibration(calibration_path)

        # subscrever ao tópico image_raw
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def load_camera_calibration(self, calibration_path):
        with open(calibration_path, 'r') as file:
            self.camera_info = CameraInfo()
            calibration_data = yaml.safe_load(file)
            self.camera_info.width = calibration_data['image_width']
            self.camera_info.height = calibration_data['image_height']
            self.camera_info.distortion_model = calibration_data['distortion_model']
            self.camera_info.d = calibration_data['distortion_coefficients']['data']
            self.camera_info.k = calibration_data['camera_matrix']['data']
            self.camera_info.r = calibration_data['rectification_matrix']['data']
            self.camera_info.p = calibration_data['projection_matrix']['data']

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        rectified = frame.copy()
        self.camera_model.rectifyImage(frame, rectified)

        msg_rect = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        msg_rect.header.stamp = self.get_clock().now().to_msg()
        msg_rect.header.frame_id = "camera"
        self.image_pub_rect.publish(msg_rect)

def main(args=None):
    rclpy.init(args=args)
    node = CameraRectifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
