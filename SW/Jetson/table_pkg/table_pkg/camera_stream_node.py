import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        self.publisher_ = self.create_publisher(Image, '/camera_image', 10)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
            exit(1)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('camera_stream_node 시작됨. /camera_image 퍼블리시 중')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('camera_stream_node 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()
