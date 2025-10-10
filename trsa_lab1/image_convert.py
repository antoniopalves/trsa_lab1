import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageConvert(Node):
    def __init__(self):
        super().__init__('image_convert')

        self.bridge = CvBridge()

        # Subscrição à imagem retificada
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.listener_callback,
            10)

        # Publicador da imagem processada
        self.publisher_ = self.create_publisher(Image, '/camera/image_processed', 10)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        # Desfoque um pouco maior para reduzir ruído de fundo
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.0)

        # Canny ligeiramente menos sensível
        edges = cv2.Canny(
            blurred,
            threshold1=20,
            threshold2=60,
            apertureSize=3,
            L2gradient=True
        )

        # Dilatar + erodir (abre e fecha pequenas falhas)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        processed_msg.header = msg.header
        self.publisher_.publish(processed_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ImageConvert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
