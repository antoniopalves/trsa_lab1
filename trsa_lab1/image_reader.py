import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')
        self.bridge = CvBridge()

        # subscrição à imagem raw
        self.subscription_raw = self.create_subscription(
            Image,
            '/camera/image_raw',    
            self.raw_callback,
            10)

        # subscrição à imagem processada
        self.subscription_proc = self.create_subscription(
            Image,
            '/camera/image_processed',
            self.proc_callback,
            10)

        self.frame_raw = None
        self.frame_proc = None

    def raw_callback(self, msg):
        self.frame_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.show_images()

    def proc_callback(self, msg):
        # como a imagem processada foi publicada em mono8, forçamos isso
        self.frame_proc = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.show_images()

    def show_images(self):
        if self.frame_raw is not None:
            cv2.imshow("Image Raw", self.frame_raw)
        if self.frame_proc is not None:
            cv2.imshow("Image Processed", self.frame_proc)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageReader()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

