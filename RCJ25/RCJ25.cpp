#include "RCJ25.h"
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver();
MPU6050 mpu(Wire);

const int motor1_pins[] = {0, 1, 2}; 
const int motor2_pins[] = {3, 4, 5};
const int motor3_pins[] = {6, 7, 8};  
const int motor4_pins[] = {9, 10, 11}; 

RCJ25::RCJ25()
    : motor1(motor1_pins[0], motor1_pins[1], true, motor1_pins[2]), 
      motor2(motor2_pins[0], motor2_pins[1], true, motor2_pins[2]),
      motor3(motor3_pins[0], motor3_pins[1], true, motor3_pins[2]),
      motor4(motor4_pins[0], motor4_pins[1], true, motor4_pins[2]),
      mpu(Wire),
      previousError(0),
      integral(0),
      previousTime(0),
{}

void RCJ25::begin() {
    Serial.begin(115200);
    pwmDriver.begin();
    pwmDriver.setPWMFreq(1000);  // Set frequency for PCA9685
    calibrateMPU();              
    Serial.println("Started!");
}

void RCJ25::write(String txt){
  Serial.print("Received character: ");
  Serial.println(txt);
}


void RCJ25::movein(int degree, int speed) {
    float out = pidcalc(degree, getYaw(), 1.0, 0.01, 0.2);
    int frontRightSpeed = speed + out;
    int frontLeftSpeed = speed - out;
    int backRightSpeed = speed + out;
    int backLeftSpeed = speed - out;

    motor1.setSpeed(frontRightSpeed);
    motor2.setSpeed(frontLeftSpeed);
    motor3.setSpeed(backRightSpeed);
    motor4.setSpeed(backLeftSpeed);
}

float RCJ25::pidcalc(float sp, float pv, float kp, float ki, float kd) {
    float error = sp - pv;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;

    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;

    float output = (kp * error) + (ki * integral) + (kd * derivative);

    previousError = error;
    previousTime = currentTime;

    return output;
}
void RCJ25::turn(int sp, int speed) {
    int pv = getYaw();
    
    while (pv != sp) { 
        int turnOutput = pidcalc(sp, pv, 1.0, 0.01, 0.2);

        int RightSpeed = speed + turnOutput;
        int leftSpeed = speed - turnOutput;

        motor1.setSpeed(RightSpeed);
        motor2.setSpeed(leftSpeed);
        motor3.setSpeed(RightSpeed);
        motor4.setSpeed(leftSpeed);
        
        WriteYaw();
        
        Serial.print("Speed Right: ");
        Serial.print(RightSpeed);
        Serial.print(", Speed Left: ");
        Serial.println(leftSpeed);

        pv = getYaw();  
    }

    stopMotors();
    Serial.print("Robot in:");
    Serial.println(sp);
}

int RCJ25::GetLineData(unsigned int addr) {
    Wire.requestFrom(addr, 4);  
    int i = 0;
    while (Wire.available() && i < 4) {
        linedata[i] = Wire.read();
        i++;
    }

    if (i < 4) {
        Serial.println("Error: Data was not fully received!");
    }

    Serial.print("Line data: ");
    for (int j = 0; j < 4; j++) {
        Serial.print(linedata[j]);
        Serial.println(" ");
    }

    return linedata[0];  
}


void RCJ25::followline(int speed) {
    float out = receiveAngleFromOpenMV();
    movein(out, speed);
}

void RCJ25::stopMotors() {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
}

void RCJ25::setMotor1Speed(int speed) {
    motor1.setSpeed(speed);
    Serial.println(speed);
}

void RCJ25::setMotor2Speed(int speed) {
    motor2.setSpeed(speed);
    Serial.println(speed);
}

void RCJ25::setMotor3Speed(int speed) {
    motor3.setSpeed(speed);
    Serial.println(speed);
}

void RCJ25::setMotor4Speed(int speed) {
    motor4.setSpeed(speed);
    Serial.println(speed);
}



RCJ25::Motor::Motor(int _pinA, int _pinB, bool usePCA, int _pinE)
    : pinA(_pinA), pinB(_pinB), pinE(_pinE), usingPCA(usePCA) 
{}

void RCJ25::Motor::setSpeed(int speed) {
      if (speed > 0) { // forward:
          pwmDriver.setPWM(pinA, 0, 4095);  
          pwmDriver.setPWM(pinB, 0, 0);
      } else { // backward:
          pwmDriver.setPWM(pinA, 0, 0);     
          pwmDriver.setPWM(pinB, 0, 4095);
      }
      pwmDriver.setPWM(pinE, 0, map(abs(speed), 0, 254, 0, 4096));
 
}

void RCJ25::Motor::stop() {
    pwmDriver.setPWM(pinA, 0, 0);
    pwmDriver.setPWM(pinB, 0, 0);
    pwmDriver.setPWM(pinE, 0, 0);
}

// GYRO FUNCTIONS:

void RCJ25::calibrateMPU() {
    mpu.begin();
    mpu.calcOffsets();
    Serial.println("MPU calibration complete.");
}

int RCJ25::getYaw() {
    mpu.update();
    return mpu.getAngleZ();
}


int RCJ25::getPitch() {
    mpu.update();
    return mpu.getAngleX();
}

int RCJ25::getRoll() {
    mpu.update();
    return mpu.getAngleY();
}

void RCJ25::WriteYaw() {
  int yaw = getYaw();
  Serial.print("| YAW =");
  Serial.print(yaw);
  Serial.println(" |");
}

void RCJ25::WritePitch() {
  int Pitch = getPitch();
  Serial.print("| Pitch =");
  Serial.print(Pitch);
  Serial.println(" |");
}

void RCJ25::WriteRoll() {
  int Roll = getRoll();
  Serial.print("| Roll =");
  Serial.print(Roll);
  Serial.println(" |");
}
