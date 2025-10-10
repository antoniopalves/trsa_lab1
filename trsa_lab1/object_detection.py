import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.callback,
            10)

        self.publisher_ = self.create_publisher(
            Image,
            '/camera2/object_detected',
            10)

        # HSV ajustado para capa verde escura (teal)
        self.lower_green = np.array([70, 30, 40])   # limite inferior
        self.upper_green = np.array([95, 255, 255]) # limite superior

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Criação da máscara verde
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        # Limpeza de ruído
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Mostrar máscara para debugging
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        # Contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area > 800:  # ignora ruído pequeno
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Green Case", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Publicar imagem anotada
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.publisher_.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
