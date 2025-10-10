import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageReader2(Node):
    def __init__(self):
        super().__init__('image_reader2')
        self.bridge = CvBridge()

        # subscrições: vídeo original + processado
        self.sub_raw = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.cb_raw,
            10)

        self.sub_proc = self.create_subscription(
            Image,
            '/camera2/object_detected',
            self.cb_proc,
            10)

        self.frame_raw = None
        self.frame_proc = None

    def cb_raw(self, msg):
        self.frame_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.show()

    def cb_proc(self, msg):
        self.frame_proc = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.show()

    def show(self):
        if self.frame_raw is not None:
            cv2.imshow("Camera2 Raw", self.frame_raw)
        if self.frame_proc is not None:
            cv2.imshow("Camera2 Object Detected", self.frame_proc)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageReader2()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
