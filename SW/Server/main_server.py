from fastapi import FastAPI, Request, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import asyncpg
import psycopg2
import requests
import json
import asyncio

# FastAPI 앱 생성
app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# PostgreSQL 연결 정보 (psycopg2용)
DB_CONFIG = {
    "host": "localhost",
    "database": "postgres",
    "user": "postgres",
    "password": "0617"
}

# asyncpg용 DB URL
DATABASE_URL = "postgresql://postgres:0617@localhost:5432/postgres"

# 클래스 모델
class UpdateInfoRequest(BaseModel):
    id: str
    email: str
    name: str
    gender: str

class PasswordUpdateRequest(BaseModel):
    id: str
    current_password: str
    new_password: str

class BatteryData(BaseModel):
    number: int
    percent: float

class TableID(BaseModel):
    table_id: int

class TrashData(BaseModel):
    percent: float

class MopFillPayload(BaseModel):
    mop: int

class AlcoholFillPayload(BaseModel):
    alcohol: int

# Jetson 서버 주소
JETSON_URL = "https://2cbaaeb9ecfd.ngrok-free.app/receive_coords"

# 배터리 상태 INSERT (asyncpg 사용)
@app.post("/insert_battery_data")
async def insert_battery_data(data: List[BatteryData]):
    query = """
        INSERT INTO robot_status_battery (battery_number, battery)
        VALUES ($1, $2)
    """
    async with app.state.db.acquire() as conn:
        async with conn.transaction():
            for entry in data:
                await conn.execute(query, entry.number, entry.percent)
    return {"status": "success", "rows_inserted": len(data)}

@app.post("/insert_trash_data")
async def insert_trash_data(data: TrashData):
    value = int(round(float(data.percent)))
    value = max(0, min(100, value))

    query = """
        INSERT INTO robot_status_trash (trash)
        VALUES ($1)
    """
    async with app.state.db.acquire() as conn:
        await conn.execute(query, value)

    return {"status": "success", "inserted": value}

@app.post("/request_table_cleaning")
async def request_table_cleaning(request: Request):
    try:
        payload = await request.json()
    except Exception:
        payload = {}

    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()

        # 1) 최신 알코올 값 조회 → 5 감소(0 미만으로는 내려가지 않도록)
        cur.execute("SELECT alcohol FROM robot_status_alcohol ORDER BY update_at DESC LIMIT 1")
        row_alc = cur.fetchone()
        last_alc = int(row_alc[0]) if row_alc and row_alc[0] is not None else 0
        new_alc = max(0, last_alc - 5)
        cur.execute("INSERT INTO robot_status_alcohol (alcohol) VALUES (%s)", (new_alc,))

        # 2) 최신 걸레 수 조회 → 1 감소(0 미만 방지)
        cur.execute("SELECT mop FROM robot_status_mop ORDER BY update_at DESC LIMIT 1")
        row_mop = cur.fetchone()
        last_mop = int(row_mop[0]) if row_mop and row_mop[0] is not None else 0
        new_mop = max(0, last_mop - 1)
        cur.execute("INSERT INTO robot_status_mop (mop) VALUES (%s)", (new_mop,))

        conn.commit()
        cur.close()
        conn.close()

        return {
            "ok": True,
            "from": payload,               # Jetson에서 보낸 내용(참고용)
            "alcohol_before": last_alc,
            "alcohol_after": new_alc,
            "mop_before": last_mop,
            "mop_after": new_mop
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# 테이블 ID 수신
@app.post("/receive_table_id")
async def receive_table_id(data: TableID):
    table_id = data.table_id
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        cur.execute('SELECT x, y, yam FROM "table" WHERE table_id = %s', (table_id,))
        result = cur.fetchone()
        cur.close()
        conn.close()

        if not result:
            return {"error": f"테이블 ID {table_id}의 좌표 정보가 없습니다."}

        x, y, yaw = map(float, result)
        payload = {"table_id": table_id, "x": x, "y": y, "yaw": yaw}
        headers = {"Content-Type": "application/json"}

        r = requests.post(JETSON_URL, json=payload, headers=headers)

        return {
            "message": "Jetson으로 좌표 전송 완료",
            "table_id": table_id,
            "x": x,
            "y": y,
            "yaw": yaw,
            "status_code": r.status_code
        }

    except Exception as e:
        return {"error": str(e)}

# 로그인
@app.post("/login")
async def login(request: Request):
    try:
        data = await request.json()
        user_id = data.get("_id")
        password = data.get("password")
        if not user_id or not password:
            raise HTTPException(status_code=400, detail="ID 또는 비밀번호 누락")

        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        cur.execute('SELECT id FROM "users" WHERE id = %s AND password = %s', (user_id, password))
        result = cur.fetchone()
        cur.close()
        conn.close()

        if result:
            return {"_id": result[0]}
        else:
            raise HTTPException(status_code=401, detail="ID 또는 비밀번호가 일치하지 않습니다.")

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# 회원가입
@app.post("/register")
async def register(request: Request):
    try:
        data = await request.json()
        name = data.get("name")
        email = data.get("email")
        user_id = data.get("id")
        password = data.get("password")
        gender = data.get("gender")

        if not all([name, email, user_id, password, gender]):
            raise HTTPException(status_code=400, detail="모든 필드를 입력해주세요.")

        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        cur.execute('SELECT id FROM "users" WHERE id = %s', (user_id,))
        if cur.fetchone():
            cur.close()
            conn.close()
            raise HTTPException(status_code=409, detail="이미 존재하는 ID입니다.")

        cur.execute('INSERT INTO "users" (name, email, id, password, gender) VALUES (%s, %s, %s, %s, %s)',
                    (name, email, user_id, password, gender))
        conn.commit()
        cur.close()
        conn.close()

        return {"success": True, "message": "회원가입 성공"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

# 회원정보 조회
@app.get("/userinfo")
async def get_user_info(id: str):
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        cur.execute('SELECT id, name, email, gender FROM users WHERE id = %s', (id,))
        row = cur.fetchone()
        cur.close()
        conn.close()

        if row:
            return {
                "id": row[0],
                "name": row[1],
                "email": row[2],
                "gender": row[3],
            }
        else:
            raise HTTPException(status_code=404, detail="사용자 정보를 찾을 수 없습니다.")

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

# 회원정보 수정
@app.patch("/update_info")
async def update_user_info(data: UpdateInfoRequest):
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        cur.execute('UPDATE users SET email = %s, name = %s, gender = %s WHERE id = %s',
                    (data.email, data.name, data.gender, data.id))
        conn.commit()
        cur.close()
        conn.close()
        return {"success": True, "message": "회원 정보 수정 완료"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

# 비밀번호 수정
@app.patch("/update_password")
async def update_password(request: Request):
    try:
        data = await request.json()
        user_id = data['id']
        current_pw = data['current_password']
        new_pw = data['new_password']

        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        cur.execute("SELECT password FROM users WHERE id = %s", (user_id,))
        result = cur.fetchone()

        if not result:
            return {"detail": "유저를 찾을 수 없습니다."}, 404

        if result[0] != current_pw:
            return {"detail": "현재 비밀번호가 일치하지 않습니다."}, 400

        cur.execute("UPDATE users SET password = %s WHERE id = %s", (new_pw, user_id))
        conn.commit()
        conn.close()

        return {"message": "비밀번호가 변경되었습니다."}, 200

    except Exception as e:
        return {"detail": f"서버 오류: {str(e)}"}, 500

# 상태 WebSocket
@app.websocket("/ws/status")
async def status_websocket(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            try:
                conn = psycopg2.connect(**DB_CONFIG)
                cur = conn.cursor()

                cur.execute("SELECT battery FROM robot_status_battery WHERE battery_number = 1 ORDER BY update_at DESC LIMIT 1")
                battery1 = cur.fetchone()
                cur.execute("SELECT battery FROM robot_status_battery WHERE battery_number = 2 ORDER BY update_at DESC LIMIT 1")
                battery2 = cur.fetchone()
                cur.execute("SELECT alcohol FROM robot_status_alcohol ORDER BY update_at DESC LIMIT 1")
                alcohol = cur.fetchone()
                cur.execute("SELECT trash FROM robot_status_trash ORDER BY update_at DESC LIMIT 1")
                trash = cur.fetchone()
                cur.execute("SELECT mop FROM robot_status_mop ORDER BY update_at DESC LIMIT 1")
                mop = cur.fetchone()

                cur.close()
                conn.close()

                # 값 할당
                b1 = battery1[0] if battery1 else 0
                b2 = battery2[0] if battery2 else 0
                alc = alcohol[0] if alcohol else 0
                tr = trash[0] if trash else 0
                mp = mop[0] if mop else 0

                data = {
                    "battery_1": b1,
                    "battery_2": b2,
                    "alcohol": alc,
                    "trash": tr,
                    "mop": mp,
                    "status_battery_1": "정상" if b1 > 20 else "배터리1 부족",
                    "status_battery_2": "정상" if b2 > 20 else "배터리2 부족",
                    "status_alcohol": "정상" if alc > 20 else "알코올 부족",
                    "status_trash": "정상" if tr < 80 else "쓰레기 적재량 많음",
                    "status_mop": "정상" if mp > 2 else "걸레 부족"
                }

                await websocket.send_text(json.dumps(data))

            except Exception as e:
                await websocket.send_text(json.dumps({
                    "battery_1": 0, "battery_2": 0, "alcohol": 0,
                    "trash": 0, "mop": 0,
                    "status_battery_1": "DB 오류",
                    "status_battery_2": "DB 오류",
                    "status_alcohol": str(e),
                    "status_trash": "DB 오류",
                    "status_mop": "DB 오류"
                }))
            await asyncio.sleep(2)

    except WebSocketDisconnect:
        print("WebSocket 클라이언트 연결 종료")

@app.post("/mop/fill")
async def mop_fill(payload: MopFillPayload):
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        # 안전하게 파라미터 바인딩 사용
        cur.execute('INSERT INTO robot_status_mop (mop) VALUES (%s)', (payload.mop,))
        conn.commit()
        cur.close()
        conn.close()
        return {"ok": True, "mop": payload.mop}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    
@app.post("/alcohol/fill")
async def alcohol_fill(payload: AlcoholFillPayload):
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cur = conn.cursor()
        # 안전하게 파라미터 바인딩
        cur.execute('INSERT INTO robot_status_alcohol (alcohol) VALUES (%s)', (payload.alcohol,))
        conn.commit()
        cur.close()
        conn.close()
        return {"ok": True, "alcohol": payload.alcohol}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))