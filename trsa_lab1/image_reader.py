import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')
        self.bridge = CvBridge()

        # Subscriptions
        self.sub_raw = self.create_subscription(Image, '/camera/image_raw', self.cb_raw, 10)
        self.sub_proc = self.create_subscription(Image, '/camera/image_processed', self.cb_proc, 10)
        self.sub_detect = self.create_subscription(Image, '/camera2/object_detected', self.cb_detect, 10)

        self.frame_raw = None
        self.frame_proc = None
        self.frame_detect = None

    def cb_raw(self, msg):
        self.frame_raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.display()

    def cb_proc(self, msg):
        self.frame_proc = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.display()

    def cb_detect(self, msg):
        self.frame_detect = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.display()

    def display(self):
        if self.frame_raw is not None:
            cv2.imshow('Raw Image', self.frame_raw)
        if self.frame_proc is not None:
            cv2.imshow('Processed (Edges)', self.frame_proc)
        if self.frame_detect is not None:
            cv2.imshow('Object Detection', self.frame_detect)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageReader()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
