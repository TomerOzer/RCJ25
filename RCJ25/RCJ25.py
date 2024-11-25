import sensor, time
from pyb import I2C

class RCJ25:
    def __init__(self):
        self.i2c = I2C(2, I2C.SLAVE, addr=0x42) # needs to be checked.
        self.line_data = [0, 0, 0, 0]
        print("Started!\n")

    def write(self, txt):
        print(str(txt))

    def DisplayScreen(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=200)
        print("Display initialized.")

    def DetectBlackLine(self, max_intensity):
        img = sensor.snapshot()
        lines = img.find_lines(threshold=2000, theta_margin=25, rho_margin=25)

        if lines:
            for line in lines:
                x1, y1, x2, y2 = line.x1(), line.y1(), line.x2(), line.y2()
                r1, g1, b1 = img.get_pixel(x1, y1)
                r2, g2, b2 = img.get_pixel(x2, y2)
                avg_intensity = (r1 + g1 + b1 + r2 + g2 + b2) // 6
                if avg_intensity <= max_intensity:
                    img.draw_line(line.line(), color=(255, 255, 255))
                    print(f"Black line detected: x1={x1}, y1={y1}, x2={x2}, y2={y2}, Avg Intensity: {avg_intensity}")
                    self.line_data = [x1, y1, x2, y2]
                    return self.line_data

        print("No lines detected.")
        self.line_data = [0, 0, 0, 0]

    def SendLine(self, max_intensity=90): # NEEDS TO BE CHECKED
        print("Sending line data over I2C...")
        self.DetectBlackLine(max_intensity)
        try:
            data_to_send = bytearray(self.line_data)
            self.i2c.send(data_to_send)
            print(f"Line data sent: {list(data_to_send)}")
        except Exception:
            print(f"Error while sending line data: {Exception}")
        time.sleep(0.1)  #To prevent overloading IIC.

cam = RCJ25()

cam.DisplayScreen()
while True:
     cam.DetectBlackLine(80)


