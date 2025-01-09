from machine import Pin, SoftI2C, I2C, UART
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
        #self.gyro = MPU6050()
        self.uart = UART(2, tx=17, rx=16, baudrate=115200)

        self.uart.init(115200, bits=8, parity=None, stop=1)
        print("ESP32 UART initialized.")
        self.i = 0

    def pid_calc(self, current_value, Kp, Ki, Kd):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        error = self.setpoint - current_value

        self.integral += error * dt

        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        output = Kp * error + Ki * self.integral + Kd * derivative

        self.previous_error = error

        return output

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

    def receive_line_data(self):
        if self.uart.any():  # Check if data is available
            try:
                line = self.uart.readline()  # Read the line from UART
                if line:
                    data = line.decode('utf-8').strip()
                    print(f"Received data: {data}")
                else:
                    print("No data received!")
            except Exception as e:
                print(f"Error receiving data: {e}")
        else:
            self.i += 1
            print(f"Waiting for data...{self.i}")


    def run(self):
        self.receive_line_data()


robot = RCJ25()
while True:
    robot.run()

