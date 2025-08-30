#!/usr/bin/env python3
# table_cleaning_bridge.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import serial
import threading

class TableCleaningBridge(Node):
  def __init__(self):
    super().__init__('table_cleaning_bridge')

    # 파라미터 선언
    self.declare_parameter('serial_port', '/dev/ttyCH341USB3')  
    self.declare_parameter('baud_rate', 115200)
    self.declare_parameter('write_timeout_ms', 1000)
    self.declare_parameter('read_timeout_ms', 2000)
    self.declare_parameter('wait_ok', False)

    # 파라미터 로드
    self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
    self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
    self.write_timeout_ms = self.get_parameter('write_timeout_ms').get_parameter_value().integer_value
    self.read_timeout_ms = self.get_parameter('read_timeout_ms').get_parameter_value().integer_value
    self.wait_ok = self.get_parameter('wait_ok').get_parameter_value().bool_value

    # 직렬 포트 오픈
    self.ser = None
    self._ser_lock = threading.Lock()
    self._open_serial()

    # 서비스 생성 (절대 경로 유지)
    self.srv = self.create_service(Trigger, '/table_cleaning_service', self.on_trigger)

    self.get_logger().info(f'table_cleaning_service ready on {self.serial_port} @ {self.baud_rate}')

  # 직렬 포트 열기
  def _open_serial(self):
    try:
      self.ser = serial.Serial(
        port=self.serial_port,
        baudrate=self.baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,  
        write_timeout=self.write_timeout_ms / 1000.0
      )
      # 입력 버퍼 비우기
      self.ser.reset_input_buffer()
      self.ser.reset_output_buffer()
    except Exception as e:
      self.get_logger().error(f'Failed to open serial: {e}')
      self.ser = None

  # 한 줄 쓰기 (개행 자동 추가)
  def _write_line(self, s: str) -> bool:
    if self.ser is None or not self.ser.is_open:
      return False
    try:
      data = s if s.endswith('\n') else (s + '\n')
      with self._ser_lock:
        self.ser.write(data.encode('ascii', errors='ignore'))
        self.ser.flush()  # OS로 밀어넣기
      return True
    except Exception as e:
      self.get_logger().error(f'Write failed: {e}')
      return False

  # 타임아웃 포함 라인 읽기 ("\n"까지 수신 또는 타임아웃)
  def _read_line_with_timeout(self, timeout_ms: int) -> str:
    if self.ser is None or not self.ser.is_open:
      return ''
    # 기존 timeout 저장 후 임시로 변경
    original_timeout = self.ser.timeout
    try:
      self.ser.timeout = timeout_ms / 1000.0
      line_bytes = self.ser.readline()  # '\n'까지 or timeout
      return line_bytes.decode('ascii', errors='ignore').strip('\r\n')
    except Exception as e:
      self.get_logger().warn(f'Read failed: {e}')
      return ''
    finally:
      self.ser.timeout = original_timeout

  # 서비스 콜백
  def on_trigger(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
    # 직렬 포트 확인 및 필요 시 재오픈
    if self.ser is None or not self.ser.is_open:
      self._open_serial()
      if self.ser is None or not self.ser.is_open:
        response.success = False
        response.message = 'serial not open'
        return response

    try:
      with self._ser_lock:
        self.ser.reset_input_buffer()

      # ESP32에 TRIG 전송
      if not self._write_line('TRIG'):
        response.success = False
        response.message = 'write failed'
        return response

      if self.wait_ok:
        # 1차 대기
        reply = self._read_line_with_timeout(self.read_timeout_ms)
        if reply == 'OK':
          response.success = True
          response.message = 'OK'
          return response

        # 부트 메시지 등 대비해 한 줄 더
        reply2 = self._read_line_with_timeout(500)
        if reply2 == 'OK':
          response.success = True
          response.message = 'OK'
          return response

        response.success = False
        response.message = f"unexpected reply: '{reply}', '{reply2}'"
      else:
        # 확인 없이 즉시 성공 처리
        response.success = True
        response.message = 'sent TRIG'

      return response

    except Exception as e:
      self.get_logger().error(f'Exception in service: {e}')
      response.success = False
      response.message = f'error: {e}'
      return response

  def destroy_node(self):
    try:
      if self.ser is not None and self.ser.is_open:
        self.ser.close()
    except Exception:
      pass
    return super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = TableCleaningBridge()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

