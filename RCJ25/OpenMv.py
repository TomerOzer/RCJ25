import sensor
from pyb import UART, LED
from time import sleep_ms
class RCJ25:
    def __init__(self):
        self.uart = UART(3, 115200)
        self.line_data = [0, 0, 0, 0]
        print("OpenMV UART initialized.")
        led = LED(2)
        led.on()
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=200)
        self.prev_data = None


    def detect_black_line(self, max_intensity):

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
                    self.line_data = [x1, y1, x2, y2]
                    self.send_line_data()

                    return
        else:
            self.line_data = [0, 0, 0, 0]
            self.send_line_data()

    def send_line_data(self):
        data = ','.join(map(str, self.line_data))
        if self.prev_data == data:
            return
        else:
            self.uart.write(f"{data}\n")  # Send only the data without "Line Data:"
            print(f"Sent line data: {data}")
            self.prev_data = data




rcj25 = RCJ25()
while True:
    rcj25.detect_black_line(max_intensity=50)
