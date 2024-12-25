from machine import Pin, SoftI2C, I2C
from time import sleep_ms
from math import atan2, sqrt
import time
from L298N_MINI import motor
from MPU6050 import MPU6050





class RCJ25: 
    def __init__(self):
        self.gyroDefined = 0
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time() 
        self.i2c_scl = 22
        self.i2c_sda = 21
        self.i2c = I2C(0, scl=Pin(self.i2c_scl), sda=Pin(self.i2c_sda))
        self.motor1 = motor(32, 33)
        self.motor2 = motor(25, 26)
        self.motor3 = motor(27, 14)
        self.motor4 = motor(12, 13)
        self.gyro = MPU6050()



    def pid_calc(self, current_value, Kp, Ki, Kd):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time  

        error = self.setpoint - current_value

        self.integral += error * dt

        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        output = Kp * error + Ki * integral + Kd * derivative

        self.previous_error = error

        return output

#    def DefineGyro(self):
#        if self.gyroDefined == 0:
#            self.gyro = MPU6050()
#            self.gyroDefined = 1
        

        
    def read_gyro(self):
        data = self.gyro.read_gyro_data()
        return data


    def get_yaw(self):
        return int(self.gyro.getYaw())


    def get_pitch(self):
        return self.gyro.getPitch()

    def get_roll(self):
        return self.gyro.getRoll()

    def turnRight(self, speed, setpoint):
        print("Right")
        
    def turnLeft(self, speed, setpoint):
        print("Left")
        
    def turn(self, speed, setpoint):
        self.turnRight(speed, setpoint) if setpoint > 0 else self.turnLeft(speed, setpoint)


    def run(self):
        print(self.get_yaw())
        






print("Hello World!")