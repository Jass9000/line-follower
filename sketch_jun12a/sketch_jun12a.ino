#include <Servo.h>

// Motor driver pins
//Left motor pins
const int motorL2 = 7;   
const int motorL1 = 2; 
const int enableL = 3;   // PWM for speed
//Right motor pins
const int motorR2 = 4;   
const int motorR1 = 5;   
const int enableR = 6;   // PWM for speed

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// Servo pin
const int servoPin = 11;

Servo scanServo;
const int motorSpeed = 70;  //0-255
const int Loffset=0;  //
const int Roffset=15;  //


long readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void setup() {
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(enableL, OUTPUT);
  pinMode(enableR, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  scanServo.attach(servoPin);
  Serial.begin(9600);
  scanServo.write(90);
  delay(300); 
}

void loop() {

  int centerDist = readDistance();
  Serial.print("Center Distance: ");
  Serial.println(centerDist);

  if (centerDist > 30) 
  {
    Serial.println("Moving Forward");
    moveForward();
  } 
  else 
  {
    Serial.println("Obstacle Ahead — Stopping");
    stopMotors();

    // Scan left
    scanServo.write(150);
    delay(500);
    int leftDist = readDistance();
    Serial.print("Left Distance: ");
    Serial.println(leftDist);

    // Scan right
    scanServo.write(30);
    delay(500);
    int rightDist = readDistance();
    Serial.print("Right Distance: ");
    Serial.println(rightDist);

    // Reset to center
    scanServo.write(90);
    delay(300);

    // Decision
    if (leftDist > rightDist) 
    {
      Serial.println("Turning Left");
      turnLeft();
    } 
    else 
    {
      Serial.println("Turning Right");
      turnRight();
    }
    delay(90);
  }
}

// Movement functions
void moveForward() {
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);
  analogWrite(enableL, motorSpeed+Loffset);
  analogWrite(enableR, motorSpeed+Roffset);
  printMotorStates("Forward");
}

void stopMotors() {
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);
  analogWrite(enableL, 0);
  analogWrite(enableR, 0);
  printMotorStates("Stop");
}

void turnRight() {
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);
  analogWrite(enableL, motorSpeed+Loffset);
  analogWrite(enableR, motorSpeed+Roffset);
  printMotorStates("Turn Right");
}

void turnLeft() {
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);
  analogWrite(enableL, motorSpeed+Loffset);
  analogWrite(enableR, motorSpeed+Roffset);
  printMotorStates("Turn Left");
}

void printMotorStates(String label) {
  Serial.println("----- " + label + " -----");
  Serial.print("motorL1: "); Serial.println(digitalRead(motorL1));
  Serial.print("motorL2: "); Serial.println(digitalRead(motorL2));
  Serial.print("motorR1: "); Serial.println(digitalRead(motorR1));
  Serial.print("motorR2: "); Serial.println(digitalRead(motorR2));
  Serial.print("enableL (PWM): "); Serial.println(motorSpeed+Loffset);
  Serial.print("enableR (PWM): "); Serial.println(motorSpeed+Roffset);
  Serial.println("------------------------");
}

