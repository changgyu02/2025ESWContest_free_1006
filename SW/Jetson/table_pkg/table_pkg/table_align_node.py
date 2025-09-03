import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import time
import threading
import os

from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class TableAlignNode(Node):
    def __init__(self):
        super().__init__('table_align_node')

        # --------------------------
        # ROS Interfaces
        # --------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_align', 10)
        self.align_done_pub = self.create_publisher(Bool, '/table_alignment_done', 10)
        self.align_service = self.create_service(Trigger, 'table_align_service', self.handle_align_request)
        self.subscription = self.create_subscription(Image, '/camera_image', self.image_callback, 10)

        # --------------------------
        # Image / GUI
        # --------------------------
        self.bridge = CvBridge()
        self.latest_frame = None
        self.output_frame = None
        self.lock = threading.Lock()

        self.enable_gui = self.declare_parameter('enable_gui', True).get_parameter_value().bool_value
        if self.enable_gui:
            self.create_timer(0.03, self.display_frame)  # ~30Hz

        # --------------------------
        # Alignment Parameters
        # --------------------------
        self.move_speed = self.declare_parameter('move_speed', 0.05).get_parameter_value().double_value
        self.required_duration = self.declare_parameter('required_duration', 0.5).get_parameter_value().double_value

        # 카메라 오프셋: 정렬 시 dx가 +127 → 보정값 dx_corr = dx - 127
        self.camera_center_offset_px = self.declare_parameter('camera_center_offset_px', 127).get_parameter_value().integer_value

        # 정렬 완료 판정은 dx_corr 기준 ±10(px)
        self.completion_tolerance_px = self.declare_parameter('completion_tolerance_px', 10).get_parameter_value().integer_value

        # 간단 로그 스로틀
        self.log_period = self.declare_parameter('log_period', 0.2).get_parameter_value().double_value

        # --------------------------
        # YOLO Model
        # --------------------------
        pkg_share = get_package_share_directory('table_pkg')
        default_model_path = os.path.join(pkg_share, 'model', 'table_number_best.pt')

        self.model_path = self.declare_parameter('yolo_model_path', default_model_path).get_parameter_value().string_value
        self.target_class_name = self.declare_parameter('target_class_name', 'table_number').get_parameter_value().string_value
        self.conf_threshold = self.declare_parameter('conf_threshold', 0.35).get_parameter_value().double_value
        self.device = self.declare_parameter('device', '').get_parameter_value().string_value  # ''=auto

        try:
            self.model = YOLO(self.model_path)
            self.names = getattr(self.model, 'names', None)
            self.get_logger().info(f'YOLOv8 model loaded: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
            self.names = None

        # 상태
        self.alignment_active = False
        self.get_logger().info('TableAlignNode ready.')

    # --------------------------
    # Callbacks
    # --------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def handle_align_request(self, request, response):
        if self.alignment_active:
            response.success = False
            response.message = 'Alignment is already in progress.'
            return response

        if self.model is None:
            response.success = False
            response.message = 'YOLO model not loaded.'
            return response

        self.get_logger().info('Alignment request received. Starting thread...')
        self.alignment_active = True
        thread = threading.Thread(target=self.run_alignment, daemon=True)
        thread.start()

        response.success = True
        response.message = 'Alignment started.'
        return response

    # --------------------------
    # Main Alignment Loop
    # --------------------------
    def run_alignment(self):
        self.get_logger().info('Alignment started.')
        aligned = False
        satisfy_start_time = None
        last_log_time = 0.0

        while rclpy.ok():
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            image_cx = w // 2

            # YOLO inference
            try:
                results = self.model(frame, verbose=False, device=self.device)
                r0 = results[0]
            except Exception as e:
                self.get_logger().error(f'YOLO inference failed: {e}')
                time.sleep(0.02)
                continue

            # Pick best target box
            best_box = None
            try:
                if hasattr(r0, 'boxes') and r0.boxes is not None and len(r0.boxes) > 0:
                    xyxy = r0.boxes.xyxy.cpu().numpy()
                    confs = r0.boxes.conf.cpu().numpy()
                    clss = r0.boxes.cls.cpu().numpy().astype(int)

                    for i in range(len(xyxy)):
                        if confs[i] < self.conf_threshold:
                            continue

                        cls_id = clss[i]
                        cls_name = None
                        if self.names is not None:
                            if isinstance(self.names, dict):
                                cls_name = self.names.get(cls_id, str(cls_id))
                            elif isinstance(self.names, list) and 0 <= cls_id < len(self.names):
                                cls_name = str(self.names[cls_id])
                            else:
                                cls_name = str(cls_id)

                        if cls_name is not None and cls_name != self.target_class_name:
                            continue

                        x1, y1, x2, y2 = xyxy[i]
                        area = (x2 - x1) * (y2 - y1)
                        score = confs[i] * (1.0 + 1e-7 * area)
                        if best_box is None or score > best_box[-1]:
                            best_box = (int(x1), int(y1), int(x2), int(y2),
                                        float(confs[i]), cls_id, score)
            except Exception as e:
                self.get_logger().warn(f'Postprocess error: {e}')

            # Visualization + Metrics
            output = frame.copy()
            found = False
            dx = 0                 # raw: sticker_cx - image_cx
            dx_corr = 0            # 보정 후: dx - camera_center_offset_px

            if best_box is not None:
                x1, y1, x2, y2, conf, cls_id, _ = best_box
                sticker_cx = (x1 + x2) // 2
                dx = sticker_cx - image_cx
                dx_corr = dx - self.camera_center_offset_px
                found = True

                # 박스/중심선 표시
                cv2.rectangle(output, (x1, y1), (x2, y2), (0, 170, 255), 2)
                cv2.line(output, (image_cx, 0), (image_cx, h), (255, 0, 0), 2)
                cv2.line(output, (sticker_cx, 0), (sticker_cx, h), (0, 255, 0), 2)

            # 우상단 고정 오버레이: dx, dx_corr만
            if self.enable_gui:
                overlay_lines = [f"dx: {dx} px", f"dx_corr: {dx_corr} px"]
                output = self.draw_overlay_topright(output, overlay_lines)
                with self.lock:
                    self.output_frame = output

            # --------------------------
            # Control: linear.x only
            # --------------------------
            twist = Twist()
            twist.angular.z = 0.0  # 회전 금지

            if found:
                # ✅ 정렬 완료 판정: dx_corr 기준 ±completion_tolerance_px
                if abs(dx_corr) <= self.completion_tolerance_px:
                    if satisfy_start_time is None:
                        satisfy_start_time = time.time()
                    elif time.time() - satisfy_start_time >= self.required_duration:
                        aligned = True
                        break
                    twist.linear.x = 0.0
                else:
                    satisfy_start_time = None
                    # 부호 기반 전/후진
                    twist.linear.x = self.move_speed if dx_corr < 0 else -self.move_speed
            else:
                satisfy_start_time = None
                twist.linear.x = 0.0

            now = time.time()
            if now - last_log_time > self.log_period:
                if found:
                    self.get_logger().info(
                        f'dx_corr={dx_corr} px (tol=±{self.completion_tolerance_px}), lin.x={twist.linear.x:.3f}'
                    )
                else:
                    self.get_logger().info('target not found')
                last_log_time = now

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.01)

        # 종료 처리
        self.cmd_vel_pub.publish(Twist())  # linear/ang 모두 0
        self.alignment_active = False
        self.align_done_pub.publish(Bool(data=aligned))
        self.get_logger().info('Alignment finished: ' + ('success' if aligned else 'failed'))

    # --------------------------
    # GUI helpers
    # --------------------------
    def draw_overlay_topright(self, img, lines, margin=10, pad=8, line_h=22):
        """오른쪽 위 고정 위치에 반투명 박스 + 텍스트(고정)"""
        if img is None:
            return img
        h, w = img.shape[:2]

        # 텍스트 영역 크기
        max_text_w = 0
        for t in lines:
            (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            max_text_w = max(max_text_w, tw)

        box_w = max_text_w + pad * 2
        box_h = line_h * len(lines) + pad * 2

        x2 = w - margin
        y1 = margin
        x1 = x2 - box_w
        y2 = y1 + box_h

        # 반투명 박스
        overlay = img.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
        alpha = 0.35
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

        # 텍스트
        y = y1 + pad + 18
        for t in lines:
            cv2.putText(img, t, (x1 + pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            y += line_h
        return img

    def display_frame(self):
        if not self.enable_gui:
            return
        with self.lock:
            if self.output_frame is not None:
                cv2.imshow('YOLO table_number alignment', self.output_frame)
                cv2.waitKey(1)

    def destroy_node(self):
        if self.enable_gui:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TableAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()
