from machine import Pin, PWM, I2C
from time import sleep, ticks_ms
from mpu6050 import MPU6050
import struct

class RCJ25:
    def __init__(self):
        # Initialize I2C for MPU6050 and OpenMV camera
        self.i2c = I2C(scl=Pin(22), sda=Pin(21))
        self.mpu = MPU6050(self.i2c)
        self.camera_address = 0x42  
        
        # Initialize motors
        self.motor1 = self.Motor(15, 2, 4)
        self.motor2 = self.Motor(5, 18, 19)
        self.motor3 = self.Motor(21, 22, 23)
        self.motor4 = self.Motor(25, 26, 27)
        
        # PID-related variables
        self.previous_error = 0
        self.integral = 0
        self.previous_time = ticks_ms()

    class Motor:
        def __init__(self, pinA, pinB, pinE):
            self.pinA = PWM(Pin(pinA), freq=1000)
            self.pinB = PWM(Pin(pinB), freq=1000)
            self.pinE = PWM(Pin(pinE), freq=1000)

        def set_speed(self, speed):
            if speed > 0:  # Forward
                self.pinA.duty_u16(65535)
                self.pinB.duty_u16(0)
            elif speed < 0:  # Backward
                self.pinA.duty_u16(0)
                self.pinB.duty_u16(65535)
            else:  # Stop
                self.pinA.duty_u16(0)
                self.pinB.duty_u16(0)
            self.pinE.duty_u16(abs(speed) * 256)

        def stop(self):
            self.pinA.duty_u16(0)
            self.pinB.duty_u16(0)
            self.pinE.duty_u16(0)

    def pidcalc(self, sp, pv, kp, ki, kd):
        error = sp - pv
        current_time = ticks_ms()
        delta_time = (current_time - self.previous_time) / 1000.0
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0

        output = kp * error + ki * self.integral + kd * derivative
        self.previous_error = error
        self.previous_time = current_time

        return output

    def movein(self, degree, speed):
        yaw = self.get_yaw()
        output = self.pidcalc(degree, yaw, kp=1.0, ki=0.01, kd=0.2)
        front_right_speed = speed + output
        front_left_speed = speed - output
        back_right_speed = speed + output
        back_left_speed = speed - output

        self.motor1.set_speed(front_right_speed)
        self.motor2.set_speed(front_left_speed)
        self.motor3.set_speed(back_right_speed)
        self.motor4.set_speed(back_left_speed)

    def stop_motors(self):
        self.motor1.stop()
        self.motor2.stop()
        self.motor3.stop()
        self.motor4.stop()

    def get_yaw(self):
        self.mpu.update()
        return self.mpu.yaw

    def get_pitch(self):
        self.mpu.update()
        return self.mpu.pitch

    def get_roll(self):
        self.mpu.update()
        return self.mpu.roll

    def receive_camera_angle(self):
        try:
            self.i2c.writeto(self.camera_address, b'\x01')  # Example command
            data = self.i2c.readfrom(self.camera_address, 4)
            angle = struct.unpack('f', data)[0]
            return angle
        except OSError:
            print("Error communicating with camera.")
            return 0.0

    def follow_line(self, speed):
        line_angle = self.receive_camera_angle()
        self.movein(line_angle, speed)
