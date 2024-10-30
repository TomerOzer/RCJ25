#include "RCJ25.h"

RCJ25 Robot;
String txt = "Hello word"; 
int speed = 150;
void setup() {
  Robot.begin();
  pinMode(13, OUTPUT);
}

void loop() {
  // Robot.write(txt);
  Robot.setMotor1Speed(speed);
  Robot.setMotor2Speed(speed);
  Robot.setMotor3Speed(speed);
  Robot.setMotor4Speed(speed);


}
