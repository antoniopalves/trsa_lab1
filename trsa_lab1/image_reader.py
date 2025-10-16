import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')
        self.bridge = CvBridge()
        self.subscribers = {}
        self.create_timer(2.0, self.check_topics)

    def check_topics(self):
        topics = [name for name, _ in self.get_topic_names_and_types()]
        wanted = [
            '/camera/image_raw',
            '/camera/image_processed',
            '/camera/image_rect',
            '/camera2/image_raw',
            '/camera2/object_detected'
        ]

        for t in wanted:
            if t in topics and t not in self.subscribers:
                self.get_logger().info(f'Subscribing to {t}')
                if 'processed' in t:
                    sub = self.create_subscription(Image, t, self.make_cb('mono8', t), 10)
                else:
                    sub = self.create_subscription(Image, t, self.make_cb('bgr8', t), 10)
                self.subscribers[t] = sub

    def make_cb(self, encoding, name):
        def callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
                window = f'{name}'
                cv2.imshow(window, frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'Error processing {name}: {e}')
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = ImageReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
