#ifndef RCJ25_H
#define RCJ25_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_PWMServoDriver.h>

class RCJ25 {
public:
    RCJ25();
    void begin();
    void write(String txt);
    void movein(int degree, int speed);
    void turn(int sp, int speed);
    int GetLineData(unsigned int addr);
    void followline(int speed);
    void stopMotors();
    void setMotor1Speed(int speed);
    void setMotor2Speed(int speed);
    void setMotor3Speed(int speed);
    void setMotor4Speed(int speed);
    void WriteYaw();
    void WritePitch();
    void WriteRoll();
    
private:
    void calibrateMPU();
    int getYaw();
    int getPitch();
    int getRoll();
    float pidcalc(float sp, float pv, float kp, float ki, float kd);

    class Motor {
    public:
        Motor(int _pinA, int _pinB, bool usePCA, int _pinE);
        void setSpeed(int speed);
        void stop();

    private:
        int pinA;
        int pinB;
        int pinE;
        bool usingPCA;
    };

    Motor motor1;
    Motor motor2;
    Motor motor3;
    Motor motor4;

    Adafruit_PWMServoDriver pwmDriver;
    MPU6050 mpu;

    int linedata[4];  
    float previousError;
    float integral;
    unsigned long previousTime;
};

#endif
