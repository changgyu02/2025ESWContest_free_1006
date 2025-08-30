from fastapi import FastAPI
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger  
from drive_pkg.action import NavigateUwbPose
import threading
import uvicorn
import requests
import time

# 중앙서버 배터리/쓰레기 데이터 POST 경로
CENTRAL_SERVER_URL = "https://9d061e0bf84b.ngrok-free.app/insert_battery_data"
CENTRAL_TRASH_URL = "https://9d061e0bf84b.ngrok-free.app/insert_trash_data"
MAIN_SERVER_CLEAN_URL = "https://9d061e0bf84b.ngrok-free.app/request_table_cleaning"

# 쓰레기 적재량 변환 기준 (cm → %)
TRASH_EMPTY_CM = 11.0  # 0%
TRASH_FULL_CM  = 6.0   # 100%

app = FastAPI()

class TablePose(BaseModel):
    table_id: int
    x: float
    y: float
    yaw: float

class NavigateClient(Node):
    def __init__(self):
        super().__init__('jetson_server_node')

        self._client = ActionClient(self, NavigateUwbPose, '/navigate_uwb_pose')

        self.battery_status = [0.0, 0.0]

        # 쓰레기 적재량(cm) 및 변환된 % 보관
        self.trash_cm = None
        self.trash_percent = None          # 마지막 유효 변환값(%)
        self.last_sent_trash_percent = None
        self._lock = threading.Lock()

        self.trash_sub_ = self.create_subscription(
            Float32, '/trash_fill_level', self.trash_callback, 10
        )
        self.battery_sub_ = self.create_subscription(
            Float32MultiArray, '/battery_status_array', self.battery_callback, 10
        )

        self.clean_srv_ = self.create_service(
            Trigger,
            '/table_cleaning_service',
            self.handle_table_cleaning_service
        )

        threading.Thread(target=self.send_battery_to_server_loop, daemon=True).start()
        threading.Thread(target=self.send_trash_to_server_loop, daemon=True).start()

    def handle_table_cleaning_service(self, request: Trigger.Request, response: Trigger.Response):
        try:
            payload = {"source": "jetson", "event": "table_cleaning_service_triggered"}
            r = requests.post(MAIN_SERVER_CLEAN_URL, json=payload, timeout=5)
            ok = 200 <= r.status_code < 300
            response.success = ok
            response.message = f"Forwarded to main_server: status={r.status_code}"
            if ok:
                self.get_logger().info(f"[CLEAN] forwarded to main_server (status={r.status_code})")
            else:
                self.get_logger().error(f"[CLEAN] main_server HTTP {r.status_code} - {r.reason}")
        except Exception as e:
            response.success = False
            response.message = f"Forward failed: {e}"
            self.get_logger().error(f"[CLEAN] forward failed: {e}")
        return response

    # cm → % 변환: 11cm=0%, 6cm=100% (클램프)
    def cm_to_percent(self, cm: float) -> float:
        rng = TRASH_EMPTY_CM - TRASH_FULL_CM  # 5.0
        if rng <= 0:
            return 0.0
        pct = (TRASH_EMPTY_CM - cm) / rng * 100.0
        if pct < 0.0:
            pct = 0.0
        if pct > 100.0:
            pct = 100.0
        return pct

    # 쓰레기 콜백: 6cm 미만이면 무시(이전 % 유지), 그 외에는 변환 후 저장
    def trash_callback(self, msg: Float32):
        cm = float(msg.data)
        with self._lock:
            self.trash_cm = cm
            if cm >= TRASH_FULL_CM:
                self.trash_percent = self.cm_to_percent(cm)
            else:
                # 6cm 미만: 이전 self.trash_percent를 그대로 유지
                pass

    # 배터리 콜백
    def battery_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.battery_status[0] = float(msg.data[0])
            self.battery_status[1] = float(msg.data[1])
            self.get_logger().info(f"BAT1={self.battery_status[0]:.1f}%, BAT2={self.battery_status[1]:.1f}%")

    # 배터리: 1분마다 중앙서버 전송
    def send_battery_to_server_loop(self):
        while rclpy.ok():
            try:
                payload = [
                    {"number": 1, "percent": round(self.battery_status[0], 1)},
                    {"number": 2, "percent": round(self.battery_status[1], 1)},
                ]
                res = requests.post(CENTRAL_SERVER_URL, json=payload, timeout=5)
                if res.ok:
                    self.get_logger().info(f"battery sent: {res.status_code}")
                else:
                    self.get_logger().warning(f"battery send HTTP {res.status_code} - {res.reason}")
            except Exception as e:
                self.get_logger().error(f"battery send failed: {e}")
            time.sleep(60)

    # 쓰레기: 1분마다 중앙서버 전송 (6cm 미만 값이 들어와도 이전 %를 보냄)
    def send_trash_to_server_loop(self):
        while rclpy.ok():
            try:
                with self._lock:
                    current_percent = (
                        self.trash_percent if self.trash_percent is not None else self.last_sent_trash_percent
                    )

                if current_percent is None:
                    self.get_logger().info("trash: no valid data yet; skip sending")
                else:
                    payload = {"percent": round(current_percent, 1)}
                    res = requests.post(CENTRAL_TRASH_URL, json=payload, timeout=5)
                    if res.ok:
                        self.get_logger().info(f"trash sent: {res.status_code}")
                        with self._lock:
                            self.last_sent_trash_percent = current_percent
                    else:
                        self.get_logger().warning(f"trash send HTTP {res.status_code} - {res.reason}")
            except Exception as e:
                self.get_logger().error(f"trash send failed: {e}")
            time.sleep(60)

    # Navigate 액션 전송
    def send_goal(self, x, y, yaw_deg):
        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')
        goal_msg = NavigateUwbPose.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.yaw_deg = float(yaw_deg)
        self.get_logger().info(f'Sending goal: x={goal_msg.x}, y={goal_msg.y}, yaw={goal_msg.yaw_deg} deg')
        return self._client.send_goal_async(goal_msg)

@app.post("/receive_coords")
async def receive_coords(data: TablePose):
    print(f"received: table {data.table_id} -> x={data.x}, y={data.y}, yaw={data.yaw}")
    future = navigator.send_goal(data.x, data.y, data.yaw)

    def result_cb(fut):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            navigator.get_logger().error("Goal rejected")
            return
        navigator.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()

        def final_cb(result_fut):
            result = result_fut.result().result
            code = result_fut.result().status
            if code == 4:
                navigator.get_logger().info("Goal reached successfully")
            else:
                navigator.get_logger().error(f"Goal failed with status code: {code}")

        result_future.add_done_callback(final_cb)

    future.add_done_callback(result_cb)
    return {"message": "ok", "table_id": data.table_id, "x": data.x, "y": data.y, "yaw": data.yaw}

def main():
    global navigator
    rclpy.init()
    navigator = NavigateClient()

    def spin_ros():
        rclpy.spin(navigator)

    threading.Thread(target=spin_ros, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()

