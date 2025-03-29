from machine import Pin, PWM

class motor:
    def __init__(self, Ain, Bin):
        self.Apin = PWM(Pin(Ain))
        self.Bpin = PWM(Pin(Bin))
        self.Apin.init(freq=1000, duty_u16=0)
        self.Bpin.init(freq=1000, duty_u16=0)

    def set_speed(self, speed):
        speed = max(-100, min(100, speed))  
        duty = int(abs(speed) * 65535 / 100)  

        if speed < 0:
            self.Apin.duty_u16(0)
            self.Bpin.duty_u16(duty)
        else:
            self.Bpin.duty_u16(0)
            self.Apin.duty_u16(duty)

        print(f"Motor running: Pin {self.Apin} Duty {self.Apin.duty_u16()}")
        print(f"Motor running: Pin {self.Bpin} Duty {self.Bpin.duty_u16()}")

    def stop(self):
        self.Apin.duty_u16(0)
        self.Bpin.duty_u16(0)

    def deinit(self):
        self.Apin.deinit()
        self.Bpin.deinit()

