import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool

class TableManagerNode(Node):
    def __init__(self):
        super().__init__('table_manager_node')

        # 상태 변수
        self.waiting_for_alignment = False
        self.waiting_for_cleaning = False
        self.waiting_for_dirt_check = False
        self.waiting_for_home_return = False

        # 구독자 등록
        self.create_subscription(Bool, '/drive_done', self.drive_done_callback, 10)
        self.create_subscription(Bool, '/table_alignment_done', self.align_done_callback, 10)
        self.create_subscription(Bool, '/cleaning_done', self.cleaning_done_callback, 10)
        self.create_subscription(Bool, '/dirt_check_done', self.dirt_check_done_callback, 10)
        self.create_subscription(Bool, '/home_return_done', self.home_return_callback, 10)

        # 서비스 클라이언트 생성
        self.align_client = self.create_client(Trigger, 'table_align_service')
        self.cleaning_client = self.create_client(Trigger, 'table_cleaning_service')
        self.dirt_check_client = self.create_client(SetBool, 'dirt_check_service')
        self.go_home_client = self.create_client(Trigger, 'go_home')

        self.get_logger().info('Table Manager Node initialized. drive_done 수신 대기 중...')

    def drive_done_callback(self, msg):
        if not msg.data:
            return

        self.get_logger().info('주행 완료 감지됨. 테이블 정렬부터 시작합니다.')
        self.get_logger().info('[1/4] 테이블 정렬 시작...')
        self.get_logger().info('정렬 서비스 요청 중...')
        self.waiting_for_alignment = True

        if self.align_client.wait_for_service(timeout_sec=3.0):
            req = Trigger.Request()
            self.align_client.call_async(req)
            self.get_logger().info('정렬 서비스 요청 완료.')
        else:
            self.get_logger().error('정렬 서비스 요청 실패: 서비스 존재하지 않음')

    def align_done_callback(self, msg):
        if not self.waiting_for_alignment or not msg.data:
            return

        self.get_logger().info('정렬 완료 토픽 수신: True')
        self.get_logger().info('정렬 완료 확인됨. 다음 단계로 진행.')
        self.waiting_for_alignment = False

        self.get_logger().info('[2/4] 클리닝 시작...')
        self.waiting_for_cleaning = True
        if self.cleaning_client.wait_for_service(timeout_sec=3.0):
            req = Trigger.Request()
            self.cleaning_client.call_async(req)
            self.get_logger().info('클리닝 서비스 요청 완료.')
        else:
            self.get_logger().error('클리닝 서비스 요청 실패: 서비스 없음')

    def cleaning_done_callback(self, msg):
        if not self.waiting_for_cleaning or not msg.data:
            return

        self.get_logger().info('클리닝 완료 토픽 수신됨. 오염도 판단으로 진행.')
        self.waiting_for_cleaning = False

        self.get_logger().info('[3/4] 오염도 판단 시작...')
        self.waiting_for_dirt_check = True
        if self.dirt_check_client.wait_for_service(timeout_sec=3.0):
            req = SetBool.Request()
            req.data = True
            self.dirt_check_client.call_async(req)
            self.get_logger().info('오염도 판단 서비스 요청 완료. 응답 대기 중...')
        else:
            self.get_logger().error('오염도 판단 서비스 요청 실패: 서비스 없음')

    def dirt_check_done_callback(self, msg):
        if not self.waiting_for_dirt_check:
            return
        self.waiting_for_dirt_check = False

        result = msg.data
        self.get_logger().info(f'오염도 판단 토픽 수신: {"더러움" if result else "깨끗함"}')

        if result:
            self.get_logger().info('[2/4] 클리닝 재시작...')
            self.waiting_for_cleaning = True
            if self.cleaning_client.wait_for_service(timeout_sec=3.0):
                req = Trigger.Request()
                self.cleaning_client.call_async(req)
                self.get_logger().info('클리닝 서비스 재요청 완료.')
            else:
                self.get_logger().error('클리닝 재요청 실패: 서비스 없음')
        else:
            self.get_logger().info('테이블 깨끗함. 복귀 시작.')
            self.waiting_for_home_return = True
            if self.go_home_client.wait_for_service(timeout_sec=3.0):
                req = Trigger.Request()
                self.go_home_client.call_async(req)
                self.get_logger().info('원점 복귀 요청 완료.')
            else:
                self.get_logger().error('복귀 서비스(go_home) 없음')

    def home_return_callback(self, msg):
        if not self.waiting_for_home_return or not msg.data:
            return

        self.get_logger().info('원점 복귀 완료 확인. 다시 drive_done 대기 상태로 진입합니다.')
        self.waiting_for_home_return = False


def main(args=None):
    rclpy.init(args=args)
    node = TableManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청 수신')
    finally:
        node.destroy_node()
        rclpy.shutdown()
