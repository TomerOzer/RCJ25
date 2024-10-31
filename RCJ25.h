#ifndef RCJ25_H
#define RCJ25_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

class RCJ25 {
public:
    RCJ25();  
    void begin(); 
    
    void movein(int degree, int speed);
    void turn(int degree, int speed);
    int getline();
    void followline(int speed);
    float pidcalc(float sp, float pv, float kp, float ki, float kd);
    void write(String txt);
    void stopMotors();
    void calibrateMPU();
    int getYaw();
    int getRoll();
    int getPitch();
    void WriteYaw();
    void WriteRoll();
    void WritePitch();
    void setMotor1Speed(int speed);
    void setMotor2Speed(int speed);
    void setMotor3Speed(int speed);
    void setMotor4Speed(int speed);
    


private:
    class Motor {
    public:
        Motor(int _pinA, int _pinB, bool usePCA, int _pinE); 
        void setSpeed(int speed);
        void stop();

    private:
        int pinA;
        int pinB;
        int pinE;  
        bool usingPCA;  // Flag to determine if PCA9865 is used
    };

    Motor motor1;
    Motor motor2;
    Motor motor3;
    Motor motor4;

    MPU6050 mpu;  

    float previousError;
    float integral;
    unsigned long previousTime;
    int TargetAngle;

    int receiveAngleFromOpenMV();
    void turnR(int degree, int speed);
    void turnL(int degree, int speed);
};

#endif
