#include <Servo.h>

// PINS
const int pinBI1 = 7, pinBI2 = 8;   // MOTOR B blanco grande
const int pinAI1 = 11,  pinAI2 = 12;    // MOTOR A  blanco grande
const int pinPWMB = 9;               // PWM MOTOR B
const int pinPWMA = 10;                // PWM MOTOR A
const int pinT1 = 24;                 // Trigger sensor 1
const int pinE1 = 25;                 // Echo sensor 1

#define ENC_K 3840
#define PINAa 21 // blue / yellow
#define PINBa 20 // green / gray
#define PINAb 18 // green / gray 
#define PINBb 19 // blue

volatile long encCountA = 0;
volatile long encCountB = 0;
volatile float wheelAngle_A = 0;
volatile float wheelAngle_B = 0;

bool directionCWA = true;
bool directionCWB = true;

unsigned long lastDisplay = 0;
#define delayDisplay 250

// PID giro
float kp = 1.0;
float ki = 0.0;
float kd = 0.2;

float error_prev = 0;
float integral = 0;

// ANGULAR PULSES
long PULSOS_360 = 11150;
long PULSOS_180 = (PULSOS_360/2) - 250;
long PULSOS_90 = (PULSOS_180/2) - 150;

// Servos
Servo base;
Servo holder;

// ===============================
// ULTRASONIC READING
// ===============================
long readUltrasonic() {
  digitalWrite(pinT1, LOW);
  delayMicroseconds(2);
  digitalWrite(pinT1, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinT1, LOW);

  long duration = pulseIn(pinE1, HIGH, 30000); 
  long cm = duration / 58;
  return cm;
}

// ===============================
// MOVE FUNCTION (igual que tú)
// ===============================
void move(unsigned char pwm, bool rotDirect){
  int I1, I2;

  if(rotDirect){
    I1 = 1; I2 = 0;   // CW
  } else {
    I1 = 0; I2 = 1;   // CCW
  }
  // MOTOR A
  digitalWrite(pinAI1, I1);
  digitalWrite(pinAI2, I2);
  analogWrite(pinPWMA, pwm);
  // MOTOR B
  digitalWrite(pinBI1, I1);
  digitalWrite(pinBI2, I2);
  analogWrite(pinPWMB, pwm); 
}

// ===============================
// *** NUEVA moveDistance() ***
// AHORA USA SOLO ULTRASÓNICO
// ===============================
void moveDistance(float targetCM) {

  int pwmA = 120;
  int pwmB = 115;

  bool forward = (targetCM > 0);
  long target = abs(targetCM);

  while(true) {
    
    long d = readUltrasonic();
    if (d == 0 || d > 400) d = 400;  // evitar lecturas basura

    // ----- AVANZAR -----
    if(forward){
      if(d <= target) break; // ya llegó
      digitalWrite(pinAI1,1); digitalWrite(pinAI2,0);
      digitalWrite(pinBI1,1); digitalWrite(pinBI2,0);
    }
    // ----- RETROCEDER -----
    else {
      if(d >= target) break; // ya se alejó suficiente
      digitalWrite(pinAI1,0); digitalWrite(pinAI2,1);
      digitalWrite(pinBI1,0); digitalWrite(pinBI2,1);
    }

    analogWrite(pinPWMA, pwmA);
    analogWrite(pinPWMB, pwmB);

    delay(30);
  }

  move(0,0);
  delay(150);
}

// ===============================
// TURN FUNCTION (igual que tú)
// ===============================
void turnAngle(long target) {
  int I1, I2;

  if(target > 0){ I1 = 1; I2 = 0; }
  else { I1 = 0; I2 = 1; }

  target = abs(target);

  encCountA = 0;
  encCountB = 0;

  unsigned long lastTime = millis();

  while ( true ) {

    long error = (abs(encCountA) - abs(encCountB));

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if(dt == 0) dt = 0.001;

    integral += error * dt;
    float derivative = (error - error_prev) / dt;

    float output = kp * error + ki * integral + kd * derivative;

    error_prev = error;
    lastTime = now;

    int basePWM = 100;

    int pwmA = constrain(basePWM - output, 100, 255);
    int pwmB = constrain(basePWM + output, 100, 255);

    // A
    digitalWrite(pinAI1, I1);
    digitalWrite(pinAI2, I2);
    analogWrite(pinPWMA, pwmA);

    // B (al revés)
    digitalWrite(pinBI1, I2);
    digitalWrite(pinBI2, I1);
    analogWrite(pinPWMB, pwmB);

    if (abs(encCountA) >= target && abs(encCountB) >= target) break;
  }

  move(0,0);
  delay(200);
}

// ===============================
// INTERRUPTS (igual que tú)
// ===============================
void channelA_A() {
  int stateA = digitalRead(PINAa);
  int stateB = digitalRead(PINBa);
  if (stateA != stateB) { encCountA++; directionCWA = true; }
  else { encCountA--; directionCWA = false; }
}

void channelB_A() {
  int stateA = digitalRead(PINAa);
  int stateB = digitalRead(PINBa);
  if (stateA == stateB) { encCountA++; directionCWA = true; }
  else { encCountA--; directionCWA = false; }
}

void channelA_B() {
  int stateA = digitalRead(PINAb);
  int stateB = digitalRead(PINBb);
  if (stateA != stateB) { encCountB--; directionCWB = true; }
  else { encCountB++; directionCWB = false; }
}

void channelB_B() {
  int stateA = digitalRead(PINAb);
  int stateB = digitalRead(PINBb);
  if (stateA == stateB) { encCountB--; directionCWB = true; }
  else { encCountB++; directionCWB = false; }
}

// ===============================
// SETUP
// ===============================
void setup() {
  Serial.begin(9600);

  pinMode(pinT1, OUTPUT);
  pinMode(pinE1, INPUT);

  pinMode(PINAa, INPUT); pinMode(PINBa, INPUT);
  pinMode(PINAb, INPUT); pinMode(PINBb, INPUT);

  attachInterrupt(digitalPinToInterrupt(PINAa), channelA_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINBa), channelB_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINAb), channelA_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINBb), channelB_B, CHANGE);

  base.attach(6);
  base.write(50);
  holder.attach(5);
  holder.write(50);
  delay(500);
}

// ===============================
// LOOP DE PRUEBA (igual que tú)
// ===============================
void loop() {

  if (millis() >= lastDisplay + delayDisplay) {

    lastDisplay = millis();
    delay(5000);
    
    // CASE 1 VUELTA 360
    turnAngle(PULSOS_360); // 360
    delay(2000);

    // CASE 2 AVANZAR HASTA LA ESQUINA Y SALIR DE LA ZONA AMARILLA
    moveDistance(12.5);   // 10.5
    delay(2000);

    turnAngle(PULSOS_90);
    delay(2000);

    moveDistance(5);  // retrocede hasta que el sensor lea 25cm
    delay(2000);

    turnAngle(-PULSOS_180);
    delay(2000);

    // CASE 3 TODOS LOS CASES +1 A PARTIR DE AQUI prepara marcador
    base.write(90);
    delay(500);
    base.write(110);
    delay(500);
    base.write(130);
    delay(1500);

    holder.write(90);
    delay(500);
    holder.write(130);
    delay(1500);


    // CASE 3 AVANZAR HASTA EL PIZARRON E HACER CONTACTO 
    moveDistance(11);
    delay(2000);

    // CASE 4 REGRESAR BRAZO A HOME Y RETROCEDER DEL PIZARRON
    base.write(110);
    delay(500);
    base.write(90);
    delay(1500);

    holder.write(100);
    delay(500);
    holder.write(60);
    delay(500);
    holder.write(30);
    delay(500);
    holder.write(0);
    delay(1500);

    // CASE 5 AVANZAR A LA ESQUINA FINAL
    turnAngle(-PULSOS_90);
    delay(2000);

    moveDistance(12.5);
    delay(2000);

    // CASE 6 AVANZAR AL PUNTO DE PARTIDA
    turnAngle(-PULSOS_90);
    delay(2000);

    moveDistance(15);
    delay(2000);

    // CASE 7 SOLTAR MARCADOR EN EL SUELO
    turnAngle(-PULSOS_90);
    delay(2000);

    //marcador
    base.write(110);
    delay(500);
    base.write(140);
    delay(1500);

    delay(5000);
    while(1);
  }
}
