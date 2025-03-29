from machine import Pin, I2C
from time import sleep_ms
import time
from L298N_MINI import motor
from MPU6050 import MPU6050
from LineProcessor import LineProcessor

m1_pins = [32, 33]
m2_pins = [26, 25]

class RCJ25:
    def __init__(self):
        self.gyroDefined = 0
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.i2c_scl = 22
        self.i2c_sda = 21
        self.i2c = I2C(0, scl=Pin(self.i2c_scl), sda=Pin(self.i2c_sda), freq=400000)

        # Initialize motors
        self.motor1 = motor(*m1_pins)
        self.motor2 = motor(*m2_pins)

        # Line Processor
        self.line_processor = LineProcessor(tx_pin=17, rx_pin=16, baudrate=115200)
        print("ESP32 UART initialized.")

    def run(self, speed):
        self.motor1.set_speed(speed)
        self.motor2.set_speed(speed)

    def update_from_camera(self):
        self.line_processor.read_and_parse()
        deltaX = self.line_processor.get_deltaX()
        if deltaX is not None:
            print("Processed deltaX:", deltaX)  # Add control logic here

