import sensor, pyb
from pyb import LED

class OpenMv:
    def __init__(self):
        self.uart = pyb.UART(1, 115200)
        print("OpenMV UART initialized.")
        self.ledR = LED(1)
        self.ledG = LED(2)
        self.ledB = LED(3)
        self.ledG.on()
        self.deltaX = 0

        # Camera initialization
        try:
            print("Resetting sensor...")
            sensor.reset()
            print("Setting pixel format...")
            sensor.set_pixformat(sensor.RGB565)
            print("Setting frame size...")
            sensor.set_framesize(sensor.QVGA)
            print("Skipping frames...")
            sensor.skip_frames(time=2000)
            print("Sensor initialization complete.")
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
        except Exception as e:
            print("Error during sensor initialization:", e)
            self.ledR.on()  # Turn red LED on to signal error

    def detect_black_line(self, max_intensity=50):
        try:
            img = sensor.snapshot()
            lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)

            if lines:
                self.ledR.on()
                self.ledB.on()
                self.ledG.off()

                for line in lines:
                    x1, y1, x2, y2 = line.x1(), line.y1(), line.x2(), line.y2()

                    r1, g1, b1 = img.get_pixel(x1, y1)
                    r2, g2, b2 = img.get_pixel(x2, y2)

                    avg_intensity = (r1 + g1 + b1 + r2 + g2 + b2) // 6

                    if avg_intensity <= max_intensity:
                        img.draw_line(line.line(), color=(0, 10, 240))
                        self.deltaX = x2 - x1
                        line_data = f"{x1},{y1},{x2},{y2},{line.theta()},{self.deltaX}\n"
                        return line_data



            return None
        except Exception as e:
            print("Error capturing frame or processing image:", e)
            return None

    def send_line_data(self):
        line_data = self.detect_black_line()
        if line_data:
            self.uart.write(line_data.encode('utf-8'))
            print("Sent:", self.deltaX)
