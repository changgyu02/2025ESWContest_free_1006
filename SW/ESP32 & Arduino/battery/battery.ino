#include <Wire.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260_1; //Battery1: Motor
Adafruit_INA260 ina260_2; //Battery2: Jetson

unsigned long lastPrintTime = 0;
const unsigned long interval = 1000;  // 10초

// ====== 인산철 전압→SOC 표 (이미지 표 그대로) ======
struct VoltageSOC { float voltage; float soc; };
// 전압은 높은→낮은 순서로 정렬
static const VoltageSOC kLFPTable[] = {
  {13.6f, 100.0f},
  {13.4f,  99.0f},
  {13.3f,  90.0f},
  {13.2f,  70.0f},
  {13.1f,  40.0f},
  {13.0f,  30.0f},
  {12.9f,  20.0f},
  {12.8f,  17.0f},
  {12.5f,  14.0f},
  {12.0f,   9.0f},
  {10.0f,   0.0f}
};
static constexpr size_t kN = sizeof(kLFPTable)/sizeof(kLFPTable[0]);

// ====== 구간 선형보간 함수 ======
float voltageToSOC(float v) {
  // 상/하한 클램프
  if (v >= kLFPTable[0].voltage)      return kLFPTable[0].soc;
  if (v <= kLFPTable[kN-1].voltage)   return kLFPTable[kN-1].soc;

  // 인접 두 점 사이에서 선형보간
  for (size_t i = 0; i < kN - 1; ++i) {
    float v_hi = kLFPTable[i].voltage;
    float s_hi = kLFPTable[i].soc;
    float v_lo = kLFPTable[i+1].voltage;
    float s_lo = kLFPTable[i+1].soc;

    if (v <= v_hi && v >= v_lo) {
      float t = (v - v_lo) / (v_hi - v_lo);   // 0~1
      return s_lo + (s_hi - s_lo) * t;
    }
  }
  return 0.0f; // 도달하지 않으면 안전하게 0
}

// ====== 전압 EMA 필터(노이즈 완화: 원하면 alpha=1.0f로 꺼도 됨) ======
float ema(float raw, float &state, float alpha = 0.3f) {
  // alpha: 0~1 (클수록 반응 빠름)
  state = alpha * raw + (1.0f - alpha) * state;
  return state;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ina260_1.begin(0x40)) { Serial.println("INA260 #1 연결 실패"); while (1) {} }
  if (!ina260_2.begin(0x44)) { Serial.println("INA260 #2 연결 실패"); while (1) {} }

  Serial.println("INA260 초기화 완료");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintTime >= interval) {
    lastPrintTime = now;

    // mV → V
    float v1_raw = ina260_1.readBusVoltage() / 1000.0f;
    float v2_raw = ina260_2.readBusVoltage() / 1000.0f;

    // 전압 필터링(원하면 alpha=1.0f로 즉시값 사용)
    static float v1_state = 0.0f, v2_state = 0.0f;
    static bool  init = false;
    if (!init) { v1_state = v1_raw; v2_state = v2_raw; init = true; }
    float v1 = ema(v1_raw, v1_state, 0.3f);
    float v2 = ema(v2_raw, v2_state, 0.3f);

    // 표 기반 선형보간 → SOC
    float soc1 = voltageToSOC(v1);
    float soc2 = voltageToSOC(v2);

    Serial.print("BAT1:");
    Serial.print(soc1, 1);
    Serial.print(",BAT2:");
    Serial.println(soc2, 1);
  }
}
