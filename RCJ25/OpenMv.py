import sensor, image, machine, pyb
from pyb import LED

class RCJ25:
    def __init__(self):
        self.uart = pyb.UART(1, 115200)
        print("OpenMV UART initialized.")
        led1 = LED(2)
        led2 = LED(3)
        led1.on()
        led2.on()

        # Camera initialization
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)  # Keep RGB565 as requested
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

    def detect_black_line(self, max_intensity=50):  # Adjust max_intensity as needed
        img = sensor.snapshot()
        lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)

        if lines:
            for line in lines:
                x1, y1, x2, y2 = line.x1(), line.y1(), line.x2(), line.y2()

                r1, g1, b1 = img.get_pixel(x1, y1)
                r2, g2, b2 = img.get_pixel(x2, y2)

                avg_intensity = (r1 + g1 + b1 + r2 + g2 + b2) // 6

                if avg_intensity <= max_intensity:
                    img.draw_line(line.line(), color=(0, 0, 0))
                    deltaX = x2 - x1
                    line_data = f"{x1},{y1},{x2},{y2},{line.theta()},{deltaX}\n"
                    return line_data

        # No valid lines detected
        return None

    def send_line_data(self):
        line_data = self.detect_black_line()
        if line_data:
            self.uart.write(line_data.encode('utf-8'))
            print("Sent:", line_data.strip())


