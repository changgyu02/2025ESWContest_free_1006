# 2025ESWContest_free_1006
# 🔖Intro
5INT 팀이 개발한 시스템은 BnW(자율주행 테이블 클리너 봇)으로, 패스트푸드점, 푸드코트와 같은 셀프 리턴 식당 환경에서 CCTV를 통한 식사 종료 탐지 → BnW 자율주행 → 테이블 클리닝 → 원점 복귀 과정을 자동화하는 것을 목표로 합니다.

# 💡 Inspiration
최근 패스트푸드점, 푸드코트와 같은 셀프 리턴 식당이 증가함에 따라 테이블 청결 관리의 중요성이 커지고 있습니다. 하지만 기존에는 손님이 떠난 후 직원이 직접 테이블을 확인하고 청소해야 하며, 이 과정에서 인력 소모와 서비스 품질 저하 문제가 발생합니다.

저희는 이러한 문제의 원인을 테이블 청소 과정의 전면적 수동 관리에서 찾았습니다. 만약 CCTV가 손님의 식사 종료를 자동으로 감지하고, 로봇이 스스로 테이블로 이동해 청소와 정리까지 수행한다면 직원의 부담은 줄고 매장 운영 효율은 올라갈 것이라 기대했습니다. 나아가 이러한 기술은 인건비 절감뿐 아니라 위생 관리의 일관성과 신뢰도를 높여, 미래형 매장 관리의 새로운 기준이 될 수 있을 것이라 생각했습니다.

# 📸 Overview
(플로우 차트 사진 & SW 구조도 사진)
1. CCTV가 손님의 식사 종료 여부를 탐지
2. Main Server가 로봇에게 해당 테이블 청소 명령 전송
3. 로봇이 UWB + IMU + 엔코더 기반 위치 추정으로 자율주행 시작
4. Table Align Node를 통해 카메라 기반 테이블 정렬 수행
5. 정렬 완료 신호 수신 후, 테이블 클리닝 수행
6. 테이블 오염도 판단(CNN 분류 모델 -> 깨끗/더러움 이진 분류)
7. 오염도 결과가 더러움이면 추가 클리닝 1회 수행
8. 최종적으로 원점 복귀(Go Home)

# 👀 Main feature

# ⚙️ Environment

<table>
  <tr>
    <td><b>🖥️ Main Processor & Sub-Controller</b></td>
    <td>
      <img src="https://img.shields.io/badge/Jetson%20Orin%20Nano-76B900?style=for-the-badge&logo=nvidia&logoColor=white"/>
      <img src="https://img.shields.io/badge/ESP32-E7352C?style=for-the-badge&logo=espressif&logoColor=white"/>
      <img src="https://img.shields.io/badge/Arduino%20Nano-00979D?style=for-the-badge&logo=arduino&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>🐧 OS & Middleware</b></td>
    <td>
      <img src="https://img.shields.io/badge/Ubuntu%2022.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white"/>
      <img src="https://img.shields.io/badge/ROS2%20Humble-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>💻 Programming Languages</b><br/>(Autonomous Driving, Cleaning)</td>
    <td>
      <img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"/>
      <img src="https://img.shields.io/badge/Python%203.10-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>🎨 Frontend</b></td>
    <td>
      <img src="https://img.shields.io/badge/Dart-0175C2?style=for-the-badge&logo=dart&logoColor=white"/>
      <img src="https://img.shields.io/badge/Flutter-02569B?style=for-the-badge&logo=flutter&logoColor=white"/>
      <img src="https://img.shields.io/badge/TypeScript-3178C6?style=for-the-badge&logo=typescript&logoColor=white"/>
      <img src="https://img.shields.io/badge/React-61DAFB?style=for-the-badge&logo=react&logoColor=black"/>
    </td>
  </tr>
  <tr>
    <td><b>🔧 Backend</b></td>
    <td>
      <img src="https://img.shields.io/badge/FastAPI-009688?style=for-the-badge&logo=fastapi&logoColor=white"/>
      <img src="https://img.shields.io/badge/Uvicorn-499848?style=for-the-badge&logo=python&logoColor=white"/>
      <img src="https://img.shields.io/badge/PostgreSQL-4169E1?style=for-the-badge&logo=postgresql&logoColor=white"/>
      <img src="https://img.shields.io/badge/TimescaleDB-FDB515?style=for-the-badge&logo=timescale&logoColor=black"/>
    </td>
  </tr>
  <tr>
    <td><b>🤖 AI Model Training</b></td>
    <td>
      <img src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white"/>
      <img src="https://img.shields.io/badge/YOLOv8-FF0000?style=for-the-badge&logo=yolo&logoColor=white"/>
      <img src="https://img.shields.io/badge/MobileNet%20V2-4285F4?style=for-the-badge&logo=tensorflow&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>📡 Network</b></td>
    <td>
      <img src="https://img.shields.io/badge/Jetson%20%E2%86%94%20Server-4CAF50?style=for-the-badge"/>
      <img src="https://img.shields.io/badge/App%20%2F%20Web%20%E2%86%94%20Server-2196F3?style=for-the-badge"/>
      <img src="https://img.shields.io/badge/REST%20API-FF6F00?style=for-the-badge&logo=swagger&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>📚 Libraries & Tools</b></td>
    <td>
      <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"/>
      <img src="https://img.shields.io/badge/rclpy-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/rclcpp-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/Rviz2-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/Gazebo-FF6600?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white"/>
    </td>
  </tr>
</table>

