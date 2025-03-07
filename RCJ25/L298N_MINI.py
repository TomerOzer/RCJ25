from machine import Pin, SoftI2C, I2C
from time import sleep_ms
from math import atan2, sqrt
import time

class motor:
    def __init__(self, Ain, Bin):
        self.Apin = Pin(Ain, Pin.OUT)
        self.Bpin = Pin(Bin, Pin.OUT)

    def set_speed(self, speed):
        if speed < 0:
            self.Apin.value(0) 
            self.Bpin.duty_u16(abs(int(speed * 65535 / 100)))
        else:
            self.Bin.value(0)  
            self.Apin.duty_u16(abs(int(speed * 65535 / 100)))

    def stop(self):a
        self.Apin.value(0)
        self.Bpin.value(0)
