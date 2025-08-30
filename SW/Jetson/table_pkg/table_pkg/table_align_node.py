import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading

class TableAlignNode(Node):
    def __init__(self):
        super().__init__('table_align_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.align_done_pub = self.create_publisher(Bool, '/table_alignment_done', 10)
        self.align_service = self.create_service(Trigger, 'table_align_service', self.handle_align_request)
        self.subscription = self.create_subscription(Image, '/camera_image', self.image_callback, 10)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.output_frame = None  # 시각화용

        self.tolerance = 40
        self.move_speed = 0.05
        self.required_duration = 0.5

        self.alignment_active = False
        self.lock = threading.Lock()

        self.create_timer(0.05, self.display_frame)

        self.get_logger().info('TableAlignNode started.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')

    def handle_align_request(self, request, response):
        if self.alignment_active:
            response.success = False
            response.message = '이미 정렬 중입니다.'
            return response

        self.get_logger().info('정렬 요청 수신. 스레드 시작...')
        self.alignment_active = True
        thread = threading.Thread(target=self.run_alignment)
        thread.start()

        response.success = True
        response.message = '정렬 중입니다.'
        return response

    def run_alignment(self):
        self.get_logger().info('▶정렬 시작')

        aligned = False
        satisfy_start_time = None

        while rclpy.ok():
            with self.lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()

            output = frame.copy()
            h, w = frame.shape[:2]
            image_cx = w // 2
            twist = Twist()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (15, 15), 0)
            edges = cv2.Canny(blurred, 15, 80)

            lines = cv2.HoughLinesP(edges, 0.2, np.pi / 520, threshold=30, minLineLength=100, maxLineGap=40)

            best_line = None
            max_length = 0

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    dx = x2 - x1
                    dy = y2 - y1
                    length = np.hypot(dx, dy)
                    angle = abs(np.arctan2(dy, dx) * 180 / np.pi)

                    if length >= (w * 0.3) and angle <= 20:
                        if length > max_length:
                            best_line = (x1, y1, x2, y2)
                            max_length = length

            found = False
            delta_x = 0
            table_cx = None

            if best_line is not None:
                x1, y1, x2, y2 = best_line
                table_cx = (x1 + x2) // 2
                delta_x = table_cx - image_cx
                found = True

                cv2.line(output, (x1, y1), (x2, y2), (0, 0, 255), 3)
                cv2.line(output, (image_cx, 0), (image_cx, h), (255, 0, 0), 2)
                cv2.line(output, (table_cx, 0), (table_cx, h), (0, 255, 0), 2)
                cv2.putText(output, f'delta_x: {delta_x}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)

            with self.lock:
                self.output_frame = output.copy()

            if found:
                self.get_logger().info(f'delta_x = {delta_x}')
                if abs(delta_x) < self.tolerance:
                    if satisfy_start_time is None:
                        satisfy_start_time = time.time()
                    elif time.time() - satisfy_start_time >= self.required_duration:
                        aligned = True
                        break
                    twist.linear.x = 0.0
                else:
                    satisfy_start_time = None
                    twist.linear.x = self.move_speed if delta_x < 0 else -self.move_speed
            else:
                satisfy_start_time = None
                twist.linear.x = 0.0

            self.cmd_vel_pub.publish(twist)

        self.cmd_vel_pub.publish(Twist())
        self.alignment_active = False
        self.align_done_pub.publish(Bool(data=aligned))
        self.get_logger().info('정렬 완료 퍼블리시됨' if aligned else '정렬 실패 퍼블리시됨')

    def display_frame(self):
        with self.lock:
            if self.output_frame is not None:
                cv2.imshow("Hough Line Detection", self.output_frame)
                cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TableAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()
