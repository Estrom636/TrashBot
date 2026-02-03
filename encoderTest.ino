#include <ESP32Servo.h>
#include <math.h>

// Define pins
const int encoderAL = 39;
const int encoderBL = 36;
const int encoderAR = 34;
const int encoderBR = 35;

const float pulsesPerRevolution = 385.4;
const double DIAMETERmm = 72;
const double DIAMETERin = DIAMETERmm/25.4;
const double CIRCUMFERENCE = DIAMETERin * PI;
const double TRACKWIDTHmm = 586;
const double TRACKWIDTHin = TRACKWIDTHmm/25.4;

Servo motorL;
Servo motorR;
double power = 0.25;

float rotationsL;
float rotationsR;

volatile long encoderTicksL = 0;
volatile long encoderTicksR = 0;

void IRAM_ATTR handleEncoderL() {
  int sA = digitalRead(encoderAL);
  int sB = digitalRead(encoderBL);

  static int lastA = 0;
  static int lastB = 0;

  if (sA != lastA || sB != lastB) {
    if ((lastA == LOW && sA == HIGH && sB == LOW) || 
        (lastA == HIGH && sA == LOW && sB == HIGH) ||
        (lastB == LOW && sB == HIGH && sA == HIGH) ||
        (lastB == HIGH && sB == LOW && sA == LOW)) {
      encoderTicksL++;
    } else {
      encoderTicksL--;
    }
  }
  lastA = sA;
  lastB = sB;
}

void IRAM_ATTR handleEncoderR() {
  int sA = digitalRead(encoderAR);
  int sB = digitalRead(encoderBR);

  static int lastA = 0;
  static int lastB = 0;

  if (sA != lastA || sB != lastB) {
    if ((lastA == LOW && sA == HIGH && sB == LOW) || 
        (lastA == HIGH && sA == LOW && sB == HIGH) ||
        (lastB == LOW && sB == HIGH && sA == HIGH) ||
        (lastB == HIGH && sB == LOW && sA == LOW)) {
      encoderTicksR++;
    } else {
      encoderTicksR--;
    }
  }
  lastA = sA;
  lastB = sB;
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motorL.setPeriodHertz(50);
  motorL.attach(26, 1000, 2000);

  motorR.setPeriodHertz(50);
  motorR.attach(27, 1000, 2000);
  
  pinMode(encoderAL, INPUT_PULLUP);
  pinMode(encoderBL, INPUT_PULLUP);

  pinMode(encoderAR, INPUT_PULLUP);
  pinMode(encoderBR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderAL), handleEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBL), handleEncoderL, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderAR), handleEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBR), handleEncoderR, CHANGE);
}

void loop() {
  noInterrupts();
  long currentTicksL = encoderTicksL;
  long currentTicksR = encoderTicksR;
  interrupts();

  rotationsL = currentTicksL / pulsesPerRevolution;
  rotationsR = currentTicksR / pulsesPerRevolution;

  rotateDegrees(180.0);
  //moveDistance(5, 5);

  Serial.print("Left Ticks: ");
  Serial.print(currentTicksL);
  Serial.print(" | Left Rotations: ");
  Serial.print(rotationsL, 3);

  Serial.print(" ! Right Ticks: ");
  Serial.print(currentTicksR);
  Serial.print(" | Right Rotations: ");
  Serial.println(rotationsR, 3);
  
  delay(100);
}

double getRotationsL() {
  noInterrupts();
  long ticks = encoderTicksL;
  interrupts();
  return (double)ticks / pulsesPerRevolution;
}

double getRotationsR() {
  noInterrupts();
  long ticks = encoderTicksR;
  interrupts();
  return (double)ticks / pulsesPerRevolution;
}

void moveDistance(double leftDis, double rightDis){
  double leftRot  = leftDis  / CIRCUMFERENCE;
  double rightRot = rightDis / CIRCUMFERENCE;

  while(getRotationsL() != leftRot || getRotationsR() != rightRot){
    double leftError  = leftRot  - getRotationsL();
    double rightError = rightRot - getRotationsR();

    //Left Wheel
    if(abs(leftError) > 0.01){
      if(leftError > 0){
        MotorPowL(0.2);
      }else{
        MotorPowL(-0.2);
      }
    }else{
      MotorPowL(0);
    }

    //Right Wheel
    if(abs(rightError) > 0.01){
      if(rightError > 0){
        MotorPowR(0.2);
      }else{
        MotorPowR(-0.2);
      }
    }else{
      MotorPowR(0);
    }
  }

  MotorPowL(0);
  MotorPowR(0);
}

void rotateDegrees(double degree){
  if(abs(degree) > 360) degree = 360 * (degree / abs(degree));
  double inches = (degree / 360) * PI * TRACKWIDTHin;
  
  if(degree > 0){
    //+ Rotation -> Clockwise
    moveDistance(inches, -1 * inches);
  }else{
    //- Rotation -> Counterclockwise
    moveDistance(-1 * inches, inches);
  }
}

void MotorPowL(double power) {
  if(power >  1) power =  1;
  if(power < -1) power = -1;

  motorL.write(90 + (power * 90));
}
void MotorPowR(double power) {
  if(power >  1) power =  1;
  if(power < -1) power = -1;

  motorR.write(90 + (power * -90));
}