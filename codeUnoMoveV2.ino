#include <Servo.h>

#define IN1 2
#define IN2 A0
#define IN3 A1
#define IN4 A2
#define ENA 3
#define ENB 11
#define TRIG 6
#define ECHO 7
#define SERVO_PIN 9

#define BASE_SPEED  150
#define STOP_DIST    25
#define SLOW_DIST    40
#define VERY_SLOW    15

float memLeft=999, memRight=999, memFront=999;
float memFrontLeft=999, memFrontRight=999;
int failLeft=0, failRight=0, failFront=0;
char lastDir='F';
int stuckCount=0;
char dirHistory[5]={'F','F','F','F','F'};
int histIdx=0;

Servo scanner;

float readDistance() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 20000);
  if (dur == 0) return 999;
  return dur * 0.034 / 2.0;
}

float scanAt(int angle, int waitMs) {
  scanner.write(angle);
  delay(waitMs);
  return readDistance();
}

// ─────────────────────────────────────────────────
// FAST scan 3 hướng — ~450ms
// Dùng khi vật cản đơn giản ở vùng 1
// ─────────────────────────────────────────────────
void scanThreeDirections() {
  memRight = scanAt(0,   150);
  memFront = scanAt(90,  150);
  memLeft  = scanAt(180, 150);
  memFrontRight = (memRight + memFront) / 2.0; // ước tính góc chéo
  memFrontLeft  = (memLeft  + memFront) / 2.0;
  scanner.write(90); delay(100);
  Serial.print("FAST P="); Serial.print(memRight);
  Serial.print(" T=");     Serial.print(memFront);
  Serial.print(" Tr=");    Serial.println(memLeft);
}

// ─────────────────────────────────────────────────
// FULL scan 5 hướng — ~800ms
// Dùng khi kẹt hoặc vùng khẩn cấp
// ─────────────────────────────────────────────────
void scanFiveDirections() {
  memRight      = scanAt(0,   150);
  memFrontRight = scanAt(45,  150);
  memFront      = scanAt(90,  150);
  memFrontLeft  = scanAt(135, 150);
  memLeft       = scanAt(180, 150);
  scanner.write(90); delay(100);
  Serial.print("FULL P=");  Serial.print(memRight);
  Serial.print(" CP=");     Serial.print(memFrontRight);
  Serial.print(" T=");      Serial.print(memFront);
  Serial.print(" CT=");     Serial.print(memFrontLeft);
  Serial.print(" Tr=");     Serial.println(memLeft);
}

bool isLooping() {
  int lCount=0, rCount=0;
  for (int i=0; i<5; i++) {
    if (dirHistory[i]=='L') lCount++;
    if (dirHistory[i]=='R') rCount++;
  }
  return (lCount>=2 && rCount>=2);
}

void addHistory(char dir) {
  dirHistory[histIdx]=dir;
  histIdx=(histIdx+1)%5;
}

char chooseBestDirection() {
  float sR  = memRight      - (failRight*8.0);
  float sFR = memFrontRight - (failRight*4.0);
  float sF  = memFront      - (failFront*8.0);
  float sFL = memFrontLeft  - (failLeft *4.0);
  float sL  = memLeft       - (failLeft *8.0);

  if (lastDir=='R') { sR-=20; sFR-=10; }
  if (lastDir=='L') { sL-=20; sFL-=10; }
  if (lastDir=='F') { sF-=15; }

  Serial.print(">> Score P="); Serial.print(sR);
  Serial.print(" T=");         Serial.print(sF);
  Serial.print(" Tr=");        Serial.println(sL);

  if (memRight<STOP_DIST && memFront<STOP_DIST && memLeft<STOP_DIST
   && memFrontRight<STOP_DIST && memFrontLeft<STOP_DIST) {
    stuckCount++; return 'B';
  }

  float best=sF; char dir='F';
  if (sFR>best){best=sFR;dir='D';}
  if (sFL>best){best=sFL;dir='E';}
  if (sR >best){best=sR; dir='R';}
  if (sL >best){best=sL; dir='L';}
  return dir;
}

void setSpeed(int l, int r) { analogWrite(ENA,l); analogWrite(ENB,r); }

void moveForward(int spd) {
  setSpeed(spd,spd);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

void moveBackward(int spd) {
  setSpeed(spd,spd);
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
}

void moveDiagRight(int spd) {
  setSpeed(spd, spd/2);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

void moveDiagLeft(int spd) {
  setSpeed(spd/2, spd);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

void turnLeft(int ms) {
  setSpeed(BASE_SPEED,BASE_SPEED);
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  delay(ms); stopMotors();
}

void turnRight(int ms) {
  setSpeed(BASE_SPEED,BASE_SPEED);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  delay(ms); stopMotors();
}

void stopMotors() {
  setSpeed(0,0);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}

void escapeLoop() {
  Serial.println(">> VONG LAP — thoat!");
  moveBackward(BASE_SPEED); delay(600);
  stopMotors(); delay(150);
  if (millis()%2==0) turnLeft(1200); else turnRight(1200);
  for (int i=0;i<5;i++) dirHistory[i]='F';
  stuckCount=0;
}

void escapeStuck() {
  Serial.println(">> KET — escape!");
  stopMotors(); delay(150);
  moveBackward(BASE_SPEED); delay(1000);
  stopMotors(); delay(150);
  int t=random(500,1500);
  if (millis()%2==0) turnLeft(t); else turnRight(t);
  failLeft=0; failRight=0; failFront=0; stuckCount=0;
  memLeft=999; memRight=999; memFront=999;
  memFrontLeft=999; memFrontRight=999;
  for (int i=0;i<5;i++) dirHistory[i]='F';
}

bool checkEscape() {
  delay(200);
  return readDistance() > STOP_DIST;
}

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A3));
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  scanner.attach(SERVO_PIN);
  scanner.write(90); delay(500);
  stopMotors();
  Serial.println("=== San sang ===");
  delay(2000);
}

void loop() {
  float dist = readDistance();
  Serial.print("D="); Serial.println(dist);

  // ── VÙNG 0: Khẩn cấp < 15cm ───────────────────
  if (dist < VERY_SLOW) {
    Serial.println(">> KHAN CAP!");
    stopMotors();
    moveBackward(BASE_SPEED); delay(400);
    stopMotors(); delay(150);

    // Full 5 hướng cho tình huống nguy hiểm
    scanFiveDirections();
    char dir=chooseBestDirection();
    addHistory(dir); lastDir=dir;

    if (dir=='B'||stuckCount>=3){ escapeStuck(); return; }
    if (isLooping())            { escapeLoop();  return; }

    if      (dir=='L'){ turnLeft(700);  if(!checkEscape())failLeft++;  else failLeft=0; }
    else if (dir=='R'){ turnRight(700); if(!checkEscape())failRight++; else failRight=0; }
    else if (dir=='D'){ moveDiagRight(BASE_SPEED); delay(400); stopMotors(); }
    else if (dir=='E'){ moveDiagLeft(BASE_SPEED);  delay(400); stopMotors(); }
    stopMotors(); delay(150);
    return;
  }

  // ── VÙNG 1: Vật cản 15–25cm ───────────────────
  if (dist < STOP_DIST) {
    Serial.println(">> VAT CAN!");
    stopMotors(); delay(100);
    moveBackward(BASE_SPEED); delay(250); // ✅ Giảm từ 400→250ms
    stopMotors(); delay(100);

    // ✅ Fast scan 3 hướng (~450ms) thay vì 5 hướng (~1000ms)
    scanThreeDirections();
    char dir=chooseBestDirection();
    addHistory(dir); lastDir=dir;

    // Nếu kẹt hoặc lặp → dùng full scan rồi escape
    if (dir=='B'||stuckCount>=3){
      scanFiveDirections(); // scan lại chi tiết hơn
      escapeStuck(); return;
    }
    if (isLooping()){ escapeLoop(); return; }

    switch(dir) {
      case 'L':
        Serial.println(">> TRAI");
        turnLeft(500); // ✅ Giảm từ 600→500ms
        if(!checkEscape())failLeft++; else{failLeft=0;Serial.println("OK!");}
        break;
      case 'R':
        Serial.println(">> PHAI");
        turnRight(500);
        if(!checkEscape())failRight++; else{failRight=0;Serial.println("OK!");}
        break;
      case 'D':
        moveDiagRight(BASE_SPEED); delay(400); stopMotors(); break;
      case 'E':
        moveDiagLeft(BASE_SPEED);  delay(400); stopMotors(); break;
      case 'F':
        Serial.println(">> THANG tiep"); break;
    }
    stopMotors(); delay(100);

  // ── VÙNG 2: Giảm tốc ──────────────────────────
  } else if (dist < SLOW_DIST) {
    int s=map((int)dist,STOP_DIST,SLOW_DIST,60,BASE_SPEED);
    moveForward(s);
    if(failLeft >0)failLeft--;
    if(failRight>0)failRight--;
    if(stuckCount>0)stuckCount--;

  // ── VÙNG 3: Thông thoáng ──────────────────────
  } else {
    moveForward(BASE_SPEED);
    if(failLeft >0)failLeft--;
    if(failRight>0)failRight--;
    if(stuckCount>0)stuckCount--;
  }

  delay(60); // ✅ Giảm từ 80→60ms
}