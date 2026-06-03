#include <Servo.h>

#define IN1 2
#define IN2 A0
#define IN3 A1
#define IN4 A2
#define ENA 3
#define ENB 11

#define TRIG      6
#define ECHO      7
#define SERVO_PIN 9

#define BT_CTRL   Serial1   // HC-05 A — nhận lệnh điều khiển
#define BT_SENSOR Serial3   // HC-05 B — gửi data sensor về web

#define BASE_SPEED  80    // 90 → 55
#define TURN_SPEED  120   // 150 → 100
#define TURN_TIME   900   // 1000 → 800
#define STOP_DIST   30

char mode = 'M';
Servo scanner;

void setSpeed(int l, int r) { analogWrite(ENA, l); analogWrite(ENB, r); }
void stopMotors()    { setSpeed(0,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); }
void moveForward(int s)  { setSpeed(s,s); digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  }
void moveBackward(int s) { setSpeed(s,s); digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); }
void turnLeft(int ms)    { setSpeed(TURN_SPEED,TURN_SPEED); digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  delay(ms); stopMotors(); }
void turnRight(int ms)   { setSpeed(TURN_SPEED,TURN_SPEED); digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); delay(ms); stopMotors(); }
void turnLeftBT()        { setSpeed(TURN_SPEED,TURN_SPEED); digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  }
void turnRightBT()       { setSpeed(TURN_SPEED,TURN_SPEED); digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); }

float readDistance() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 25000);
  if (dur == 0) return 0;
  float d = dur * 0.034 / 2.0;
  return (d > 400) ? 0 : d;
}

float readDistanceAvg() {
  float a = readDistance(); delay(10);
  float b = readDistance();
  if (a == 0 && b == 0) return 0;
  if (a == 0) return b;
  if (b == 0) return a;
  return (a + b) / 2.0;
}

void handleBT(char cmd) {
  if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;
  cmd = toupper(cmd);
  Serial.print("CMD:["); Serial.print(cmd); Serial.println("]");
  if (cmd == 'A') { mode = 'A'; scanner.write(90); Serial.println("AUTO"); return; }
  if (cmd == 'M') { mode = 'M'; stopMotors(); Serial.println("MANUAL"); return; }
  if (mode == 'M') {
    switch (cmd) {
      case 'F': moveForward(BASE_SPEED);  break;
      case 'B': moveBackward(BASE_SPEED); break;
      case 'L': turnLeftBT();             break;
      case 'R': turnRightBT();            break;
      case 'S': stopMotors();             break;
    }
  }
}

void avoidObstacle() {
  Serial.println("OBSTACLE - stopping");
  stopMotors();
  delay(200);

  // Back up
  moveBackward(BASE_SPEED);
  delay(400);
  stopMotors();
  delay(200);

  // Scan left
  scanner.write(160);
  delay(400);
  float dL = readDistance();
  if (dL == 0) dL = 999;

  // Scan right
  scanner.write(20);
  delay(400);
  float dR = readDistance();
  if (dR == 0) dR = 999;

  // Return to center
  scanner.write(90);
  delay(200);

  Serial.print("L="); Serial.print(dL);
  Serial.print(" R="); Serial.println(dR);

  // Turn toward open side
  if (dL >= dR) {
    Serial.println("TURN LEFT");
    turnLeft(TURN_TIME);
  } else {
    Serial.println("TURN RIGHT");
    turnRight(TURN_TIME);
  }

  // Small pause then resume forward
  delay(100);
  Serial.println("RESUME FORWARD");
  moveForward(BASE_SPEED);
}

void runAuto() {
  float dist = readDistanceAvg();

  if (dist > 0 && dist <= STOP_DIST) {
    // Obstacle — handle it
    avoidObstacle();
  } else {
    // Clear — keep moving forward
    moveForward(BASE_SPEED);
  }
}

void setup() {
  Serial.begin(9600);
  BT_CTRL.begin(9600);
  BT_SENSOR.begin(9600);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

  scanner.attach(SERVO_PIN);
  scanner.write(90);
  delay(500);
  stopMotors();

  Serial.println("=== Ready ===");
  Serial.println("F/B/L/R/S  A=auto  M=manual");
}

void loop() {
  char lastCmd = 0;
  while (BT_CTRL.available()) {          // BT → BT_CTRL
    char c = (char)BT_CTRL.read();       // BT → BT_CTRL
    if (c != '\n' && c != '\r' && c != ' ') lastCmd = c;
  }
  if (lastCmd != 0) handleBT(lastCmd);

  if (mode == 'A') runAuto();

  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 200) {
    lastSend = millis();
    float dist = readDistanceAvg();
    if (dist == 0) dist = 999;
    BT_SENSOR.print("D:");               // BT → BT_SENSOR
    BT_SENSOR.print(dist, 1);
    BT_SENSOR.print(",M:");
    BT_SENSOR.print(mode);
    BT_SENSOR.print('\n');
  }
}
