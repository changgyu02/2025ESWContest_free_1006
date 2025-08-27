import os
import cv2
import time
import threading
import requests
from collections import deque
from datetime import datetime
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from ultralytics import YOLO

# -------------------- 설정 --------------------
app = FastAPI()
model = YOLO("best.pt")
cap = cv2.VideoCapture(0)

CENTRAL_SERVER_URL = "https://1e72960a9e9e.ngrok-free.app/receive_table_id"

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------- 데이터 모델 --------------------
class TableMapping(BaseModel):
    box_id: str
    table_id: int

# -------------------- 전역 상태 --------------------
latest_boxes = []          # YOLO 추론 결과
confirmed_boxes = []       # /get_detections 응답용
table_coords = {}          # {table_id: [x1,y1,x2,y2]}
table_states = {}          # 테이블 상태 머신
image_path = "latest.jpg"
TIME_THRESHOLD = 10        # 초: 연속 감지 안정화 시간
box_id_counter = 1

# -------------------- 로그 버퍼 --------------------
recent_logs = deque(maxlen=200)

def add_log(msg: str):
    """YYYY-MM-DD HH:MM:SS prefix로 로그 저장"""
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    recent_logs.append(f"{ts} {msg}")
    # print(f"{ts} {msg}")  # 콘솔에도 찍고 싶으면 주석 해제

# -------------------- 보조 함수 --------------------
def is_overlapping(box1, box2):
    """두 박스가 겹치는지 확인 (입력: [x1, y1, x2, y2])"""
    x1_max = max(box1[0], box2[0])
    y1_max = max(box1[1], box2[1])
    x2_min = min(box1[2], box2[2])
    y2_min = min(box1[3], box2[3])
    return x1_max < x2_min and y1_max < y2_min

def is_similar_box(box1, box2, tol=20):
    """좌표가 유사한 박스인지 판단 (dict 비교)"""
    return (
        abs(box1['x1'] - box2['x1']) < tol and
        abs(box1['y1'] - box2['y1']) < tol and
        abs(box1['x2'] - box2['x2']) < tol and
        abs(box1['y2'] - box2['y2']) < tol
    )

def update_boxes_from_yolo(yolo_table_boxes):
    """YOLO 테이블 박스들을 id 매칭해서 안정화"""
    global confirmed_boxes, latest_boxes, box_id_counter
    new_boxes = []
    for box in yolo_table_boxes:
        matched = False
        for old_box in confirmed_boxes:
            if is_similar_box(box, old_box):
                new_boxes.append({**box, "id": old_box["id"]})
                matched = True
                break
        if not matched:
            new_id = f"box_id_{box_id_counter}"
            box_id_counter += 1
            new_boxes.append({**box, "id": new_id})
    confirmed_boxes[:] = new_boxes
    latest_boxes[:] = new_boxes

def init_table_state_if_needed(table_id):
    if table_id not in table_states:
        table_states[table_id] = {
            "has_fox": False,
            "has_tray": False,
            "has_fox_time": False,   # 안정 감지(임계 시간 충족) 여부
            "has_tray_time": False,  # 안정 감지(임계 시간 충족) 여부
            "fox_start_time": None,
            "tray_start_time": None,
            "last_print_time": datetime.now(),  # (디버그 콘솔 출력 주기용)
            "last_log_time": None,              # ⬅️ 5초 주기 로그용 타임스탬프
        }

def handle_all_tables(fox_boxes, tray_boxes):
    now = datetime.now()
    for table_id, table_box in table_coords.items():
        init_table_state_if_needed(table_id)
        state = table_states[table_id]

        foxes = [f for f in fox_boxes if is_overlapping(table_box, f)]
        trays = [t for t in tray_boxes if is_overlapping(table_box, t)]

        # fox
        if foxes:
            if not state["has_fox"]:
                state["fox_start_time"] = now
            elif (now - state["fox_start_time"]).total_seconds() >= TIME_THRESHOLD:
                state["has_fox_time"] = True
        else:
            state["fox_start_time"] = None
        state["has_fox"] = bool(foxes)

        # tray
        if trays:
            if not state["has_tray"]:
                state["tray_start_time"] = now
            elif (now - state["tray_start_time"]).total_seconds() >= TIME_THRESHOLD:
                state["has_tray_time"] = True
        else:
            state["tray_start_time"] = None
        state["has_tray"] = bool(trays)

        # (옵션) 5초마다 상태 디버그 출력
        if (now - state["last_print_time"]).total_seconds() >= 5:
            print(f"[Table {table_id}] "
                  f"F: {state['has_fox']} (F_time: {state['has_fox_time']}), "
                  f"T: {state['has_tray']} (T_time: {state['has_tray_time']})")
            state["last_print_time"] = now

        # ▶ 무조건 5초마다 현재 감지 상태를 로그로 남김 (조건 없음)
        if state["last_log_time"] is None or (now - state["last_log_time"]).total_seconds() >= 5:
            fox_msg = "여우 감지" if state["has_fox"] else "여우 미감지"
            tray_msg = "쟁반 감지" if state["has_tray"] else "쟁반 미감지"
            add_log(f"[Table {table_id}] {fox_msg}, {tray_msg}")
            state["last_log_time"] = now

        # ▶ 조건 충족 시 청소 요청
        if (not state["has_fox"] and state["has_fox_time"] and
            not state["has_tray"] and state["has_tray_time"]):
            try:
                requests.post(CENTRAL_SERVER_URL, json={"table_id": table_id})
                print(f"[✓] 테이블 {table_id} 청소 요청 전송됨")
                add_log(f"[Table {table_id}] 테이블 청소 요청 전송됨")

                # 상태 초기화 (다음 사이클 대비)
                state.update({
                    "has_fox": False,
                    "has_tray": False,
                    "has_fox_time": False,
                    "has_tray_time": False,
                    "fox_start_time": None,
                    "tray_start_time": None,
                    # "last_log_time": None,  # ← 즉시 로그를 다시 찍게 하려면 주석 해제
                })
            except Exception as e:
                print(f"[X] 중앙 서버 전송 실패: {e}")

# -------------------- 백그라운드 YOLO 추론 --------------------
def background_loop():
    global cap
    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(1)
            continue

        results = model.predict(frame, conf=0.5, verbose=False)
        boxes = results[0].boxes.data
        names = model.names
        table_boxes = []
        fox_boxes, tray_boxes = [], []

        for r in boxes:
            x1, y1, x2, y2, conf, cls = map(float, r)
            box = {"x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2)}
            cls_name = names[int(cls)]
            if cls_name == "table":
                table_boxes.append(box)
            elif cls_name == "fox":
                fox_boxes.append([int(x1), int(y1), int(x2), int(y2)])
            elif cls_name == "tray":
                tray_boxes.append([int(x1), int(y1), int(x2), int(y2)])

        update_boxes_from_yolo(table_boxes)
        cv2.imwrite(image_path, frame)
        handle_all_tables(fox_boxes, tray_boxes)
        time.sleep(1)

@app.on_event("startup")
def start_background_thread():
    thread = threading.Thread(target=background_loop, daemon=True)
    thread.start()

# -------------------- API --------------------
@app.get("/video_feed")
def video_feed():
    def generate():
        while True:
            success, frame = cap.read()
            if not success:
                continue
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    return StreamingResponse(generate(), media_type='multipart/x-mixed-replace; boundary=frame')

@app.get("/get_detections")
def get_detections():
    global latest_boxes, confirmed_boxes
    if not confirmed_boxes:
        confirmed_boxes = latest_boxes.copy()
        print("[DEBUG] confirmed_boxes 초기화됨")
    return {"image_url": "/get_image", "detections": confirmed_boxes}

@app.post("/reset_confirmed")
def reset_confirmed():
    global confirmed_boxes
    confirmed_boxes = []
    return {"status": "confirmed_boxes reset"}

@app.get("/get_image")
def get_image():
    if os.path.exists(image_path):
        return FileResponse(image_path, media_type="image/jpeg")
    return JSONResponse(content={"error": "이미지 없음"}, status_code=404)

@app.post("/set_table_id")
def set_table_id(mapping: TableMapping):
    for b in confirmed_boxes:
        if b["id"] == mapping.box_id:
            table_coords[mapping.table_id] = [b["x1"], b["y1"], b["x2"], b["y2"]]
            return {"status": "success", "saved": {mapping.table_id: table_coords[mapping.table_id]}}
    return JSONResponse(content={"error": "box_id가 일치하는 테이블 없음"}, status_code=404)

# 로그 조회 API
@app.get("/logs")
def get_logs():
    return {"logs": list(recent_logs)}

# -------------------- 메인 --------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("yolo_detect_server:app", host="127.0.0.1", port=3000, reload=True)