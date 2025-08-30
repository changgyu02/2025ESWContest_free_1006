// 초음파1: 장애믈
// 초음파2: 쓰레기통
#define TRIG1 8
#define ECHO1 7
#define TRIG2 10
#define ECHO2 9

void setup() {
  Serial.begin(9600);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  float distance = readUltrasonic(TRIG1, ECHO1);   // 장애물
  float trash = readUltrasonic(TRIG2, ECHO2);      // 쓰레기 적재량

  Serial.print("O:");
  Serial.print(distance);
  Serial.print(" T:");
  Serial.println(trash);

  delay(100);
}
