import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import torch
from torchvision import transforms, models
import cv2
from PIL import Image as PILImage
import os
from ament_index_python.packages import get_package_share_directory


class DirtCheckNode(Node):
    def __init__(self):
        super().__init__('dirt_check_node')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if self.device.type == 'cuda':
            self.get_logger().info('GPU 사용 중 (CUDA)')
        else:
            self.get_logger().warn('GPU 사용 불가, CPU로 실행 중')

        package_share_dir = get_package_share_directory('table_pkg')
        model_path = os.path.join(package_share_dir, 'model', 'dirt_classifier.pt')

        self.class_names = ['clean_table', 'dirty_table']

        self.model = models.mobilenet_v2(weights=None)
        self.model.classifier[1] = torch.nn.Linear(self.model.last_channel, 2)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.Resize((640, 640)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])

        self.bridge = CvBridge()
        self.latest_frame = None
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',
            self.image_callback,
            10
        )

        self.dirt_done_pub = self.create_publisher(Bool, '/dirt_check_done', 10)

        self.srv = self.create_service(SetBool, 'dirt_check_service', self.handle_dirt_check)

        self.get_logger().info('dirt_check_node 시작 (동기 응답 구조)')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')

    def handle_dirt_check(self, request, response):
        try:
            self.get_logger().info('오염도 판단 요청 수신')

            if self.latest_frame is None:
                self.get_logger().error('이미지 없음. 즉시 실패 처리.')
                response.success = False
                response.message = 'no image'
                self.dirt_done_pub.publish(Bool(data=False))
                return response

            frame = self.latest_frame.copy()
            label, confidence = self.predict_image(frame)
            is_dirty = (label == 'dirty_table')

            self.get_logger().info(f'분류 결과: {label} ({confidence * 100:.2f}%)')
            self.dirt_done_pub.publish(Bool(data=is_dirty))

            response.success = is_dirty
            response.message = 'dirty' if is_dirty else 'clean'
            self.get_logger().info(f'dirt_check 응답 반환: {response.message}')
            return response

        except Exception as e:
            self.get_logger().error(f'dirt_check 처리 중 예외 발생: {e}')
            response.success = False
            response.message = 'error'
            return response

    def predict_image(self, frame):
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(image)
        input_tensor = self.transform(image).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(input_tensor)
            predicted_class = torch.argmax(output, dim=1).item()
            confidence = torch.nn.functional.softmax(output, dim=1)[0][predicted_class].item()

        class_name = self.class_names[predicted_class]
        return class_name, confidence

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DirtCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청 수신')
    node.destroy_node()
    rclpy.shutdown()
