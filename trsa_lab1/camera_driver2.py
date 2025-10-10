import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraDriver2(Node):
    def __init__(self):
        super().__init__('camera_driver2')
        self.publisher_ = self.create_publisher(Image, '/camera2/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS

        # vídeo alternativo (muda caminho conforme necessário)
        self.cap = cv2.VideoCapture('/home/antoniopalves/ros2_ws/src/trsa_lab1/video/IMG_7504.mp4')
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # recomeça o vídeo
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver2()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
