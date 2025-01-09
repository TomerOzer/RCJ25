from machine import Pin, SoftI2C, I2C
from time import sleep_ms, ticks_ms, ticks_diff
from math import atan2, sqrt
import time

def signedIntFromBytes(x, endian="big"):
    y = int.from_bytes(x, endian)
    if y >= 0x8000:
        return -((65535 - y) + 1)
    return y

class MPU6050:
    def __init__(self, addr=0x68):
        # Initialize VARs:

        self._GRAVITIY_MS2 = 9.80665
        self._ACC_SCLR_2G = 16384.0
        self._ACC_SCLR_4G = 8192.0
        self._ACC_SCLR_8G = 4096.0
        self._ACC_SCLR_16G = 2048.0
        self._GYR_SCLR_250DEG = 131.0
        self._GYR_SCLR_500DEG = 65.5
        self._GYR_SCLR_1000DEG = 32.8
        self._GYR_SCLR_2000DEG = 16.4
        self._ACC_RNG_2G = 0x00
        self._ACC_RNG_4G = 0x08
        self._ACC_RNG_8G = 0x10
        self._ACC_RNG_16G = 0x18
        self._GYR_RNG_250DEG = 0x00
        self._GYR_RNG_500DEG = 0x08
        self._GYR_RNG_1000DEG = 0x10
        self._GYR_RNG_2000DEG = 0x18
        self._PWR_MGMT_1 = 0x6B
        self._ACCEL_XOUT0 = 0x3B
        self._TEMP_OUT0 = 0x41
        self._GYRO_XOUT0 = 0x43
        self._ACCEL_CONFIG = 0x1C
        self._GYRO_CONFIG = 0x1B
        self._maxFails = 3
        self._MPU6050_ADDRESS = addr

        # Initialize IIC:

        self.i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)
        self.addr = self._MPU6050_ADDRESS

        self.i2c.writeto_mem(self.addr, self._PWR_MGMT_1, bytes([0x00]))  # Wake up MPU6050
        sleep_ms(5)

        self._accel_range = self.get_accel_range(True)
        self._gyro_range = self.get_gyro_range(True)

        self.accel_scale = {
            self._ACC_RNG_2G: self._ACC_SCLR_2G,
            self._ACC_RNG_4G: self._ACC_SCLR_4G,
            self._ACC_RNG_8G: self._ACC_SCLR_8G,
            self._ACC_RNG_16G: self._ACC_SCLR_16G
        }.get(self._accel_range, self._ACC_SCLR_2G)

        self.gyro_scale = {
            self._GYR_RNG_250DEG: self._GYR_SCLR_250DEG,
            self._GYR_RNG_500DEG: self._GYR_SCLR_500DEG,
            self._GYR_RNG_1000DEG: self._GYR_SCLR_1000DEG,
            self._GYR_RNG_2000DEG: self._GYR_SCLR_2000DEG
        }.get(self._gyro_range, self._GYR_SCLR_250DEG)
        
        self.gyro_scale = self._GYR_SCLR_250DEG
        self.total_yaw = 0.0  # Accumulated yaw angle
        self.last_time = ticks_ms()  # Timestamp for integration

    def readData(self, register):
        failCount = 0
        while failCount < self._maxFails:
            try:
                sleep_ms(10)
                data = self.i2c.readfrom_mem(self.addr, register, 6)
                break
            except:
                failCount += 1
                if failCount >= self._maxFails:
                    return {"x": float("NaN"), "y": float("NaN"), "z": float("NaN")}
        x = signedIntFromBytes(data[0:2])
        y = signedIntFromBytes(data[2:4])
        z = signedIntFromBytes(data[4:6])
        return {"x": x, "y": y, "z": z}

    def read_accel_data(self, g=False):
        accel_data = self.readData(self._ACCEL_XOUT0)
        x = accel_data["x"] / self.accel_scale
        y = accel_data["y"] / self.accel_scale
        z = accel_data["z"] / self.accel_scale
        if not g:
            x *= self._GRAVITIY_MS2
            y *= self._GRAVITIY_MS2
            z *= self._GRAVITIY_MS2
        return {"x": x, "y": y, "z": z}

    def read_gyro_data(self):
        fail_count = 0
        while fail_count < 3:  
            try:
                sleep_ms(10)
                data = self.i2c.readfrom_mem(self.addr, self._GYRO_XOUT0, 6)
                break
            except:
                fail_count += 1
                if fail_count >= 3:
                    return {"x": float("NaN"), "y": float("NaN"), "z": float("NaN")}
        x = signedIntFromBytes(data[0:2]) / self.gyro_scale
        y = signedIntFromBytes(data[2:4]) / self.gyro_scale
        z = signedIntFromBytes(data[4:6]) / self.gyro_scale
        return {"x": x, "y": y, "z": z}

    def getYaw(self):
        current_time = ticks_ms()
        dt = ticks_diff(current_time, self.last_time) / 1000.0
        self.last_time = current_time

        gyro = self.read_gyro_data()
        delta_yaw = (gyro["z"] * dt) 
        self.total_yaw += delta_yaw
        if self.total_yaw > 360:
            self.total_yaw -= 360
        elif self.total_yaw < -360:
            self.total_yaw += 360
        

            
        return int(self.total_yaw)

    def resetYaw(self):
        self.total_yaw = 0.0


    def getRoll(self):
        accel = self.read_accel_data()
        return atan2(accel["y"], accel["z"])

    def getPitch(self):
        accel = self.read_accel_data()
        return atan2(-accel["x"], sqrt(accel["y"]**2 + accel["z"]**2))

    def get_accel_range(self, raw=False):
        raw_data = self.i2c.readfrom_mem(self.addr, self._ACCEL_CONFIG, 2)
        if raw:
            return raw_data[0]
        return {
            self._ACC_RNG_2G: 2,
            self._ACC_RNG_4G: 4,
            self._ACC_RNG_8G: 8,
            self._ACC_RNG_16G: 16
        }.get(raw_data[0], -1)

    def get_gyro_range(self, raw=False):
        raw_data = self.i2c.readfrom_mem(self.addr, self._GYRO_CONFIG, 2)
        if raw:
            return raw_data[0]
        return {
            self._GYR_RNG_250DEG: 250,
            self._GYR_RNG_500DEG: 500,
            self._GYR_RNG_1000DEG: 1000,
            self._GYR_RNG_2000DEG: 2000
        }.get(raw_data[0], -1)

if __name__ == "__main__":
    mpu = MPU6050()  


