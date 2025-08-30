#include <HardwareSerial.h>

// UART1 사용 설정 (TX: GPIO 17, RX: GPIO 16)
#define MDDS10_TX_PIN 17
#define MDDS10_RX_PIN 16
HardwareSerial mdSerial(1); 

// MDDS10 주소 설정
const uint8_t MDDS10_ADDRESS = 0b000;

void setup() {
    Serial.begin(115200); 
    mdSerial.begin(9600, SERIAL_8N1, MDDS10_RX_PIN, MDDS10_TX_PIN);  

    delay(700); 
    mdSerial.write(0x55);  // MDDS10 초기 헤더 전송

    Serial.println("ESP32 MDDS10 중계 시작");
}

void loop() {
    while (Serial.available() >= 4) {
        if (Serial.read() != 0x55) continue;

        uint8_t packet[4];
        packet[0] = 0x55;
        Serial.readBytes(packet + 1, 3);

        // 체크섬 검증
        uint8_t checksum = packet[0] + packet[1] + packet[2];
        if (packet[3] != checksum) {
            Serial.println("체크섬 불일치! 무시");
            continue;
        }

        // 주소 확인 (하위 3비트)
        uint8_t received_addr = packet[1] & 0x07;
        if (received_addr != MDDS10_ADDRESS) {
            Serial.print("내 주소 아님 (");
            Serial.print(received_addr);
            Serial.println("). 무시");
            continue;
        }

        // 오른쪽 모터 여부
        bool is_right_motor = packet[1] & 0x08;

        // 디버깅 출력
        Serial.print("MDDS10 전송 (모터: ");
        Serial.print(is_right_motor ? "오른쪽" : "왼쪽");
        Serial.print(", 속도: ");
        Serial.print(packet[2]);
        Serial.println(")");

        // MDDS10로 패킷 전송
        mdSerial.write(packet, 4);
    }
}
