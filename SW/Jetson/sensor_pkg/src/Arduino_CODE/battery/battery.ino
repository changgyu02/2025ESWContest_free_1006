#include <Wire.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260_1;
Adafruit_INA260 ina260_2;

unsigned long lastPrintTime = 0;
const unsigned long interval = 60000;  // 1분

float voltageToSOC(float voltage) {
  struct VoltageSOC {
    float voltage;
    float soc;
  };

  VoltageSOC table[] = {
    {13.6, 100},
    {13.4, 99},
    {13.3, 90},
    {13.2, 70},
    {13.1, 40},
    {13.0, 30},
    {12.9, 20},
    {12.8, 17},
    {12.5, 14},
    {12.0, 9},
    {10.0, 0}
  };

  const int n = sizeof(table) / sizeof(table[0]);

  if (voltage >= table[0].voltage) return 100.0;
  if (voltage <= table[n - 1].voltage) return 0.0;

  for (int i = 0; i < n - 1; i++) {
    if (voltage <= table[i].voltage && voltage >= table[i + 1].voltage) {
      float v1 = table[i].voltage;
      float soc1 = table[i].soc;
      float v2 = table[i + 1].voltage;
      float soc2 = table[i + 1].soc;

      float ratio = (voltage - v2) / (v1 - v2);
      return soc2 + (soc1 - soc2) * ratio;
    }
  }

  return 0.0; // fallback
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ina260_1.begin(0x40)) {
    Serial.println("INA260 #1 연결 실패");
    while (1);
  }

  if (!ina260_2.begin(0x44)) {
    Serial.println("INA260 #2 연결 실패");
    while (1);
  }

  Serial.println("INA260 초기화 완료");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintTime >= interval) {
    lastPrintTime = now;

    float voltage1 = ina260_1.readBusVoltage() / 1000.0;  // mV → V
    float voltage2 = ina260_2.readBusVoltage() / 1000.0;

    float soc1 = voltageToSOC(voltage1);
    float soc2 = voltageToSOC(voltage2);

    Serial.print("BAT1:");
    Serial.print(soc1, 2);  // 소수점 2자리
    Serial.print(",BAT2:");
    Serial.println(soc2, 2);
  }
}
