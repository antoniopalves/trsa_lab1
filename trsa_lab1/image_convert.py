import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageConvert(Node):
    def __init__(self):
        super().__init__('image_convert')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',        # usa o tópico que existe
            self.listener_callback,
            10
        )

        self.publisher_ = self.create_publisher(Image, '/camera/image_processed', 10)
        self.get_logger().info('ImageConvert node initialized.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert input: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.0)
        edges = cv2.Canny(blurred, 20, 60, apertureSize=3, L2gradient=True)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.header.frame_id = "camera_frame"

        self.publisher_.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageConvert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
