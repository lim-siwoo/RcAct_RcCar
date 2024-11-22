# hardware/sense_hat.py
from sense_hat import SenseHat
from faces import normal, happy, sad

class SenseHatManager:
    def __init__(self):
        self.sense = SenseHat()
        self.sense.clear()

    def set_led_color(self, color):
        self.sense.clear()
        color_map = {
            "GREEN": (0, 100, 0),
            "RED": (100, 0, 0),
            "YELLOW": (100, 100, 0)
        }
        
        if color in color_map:
            for y in range(8):
                for x in range(8):
                    self.sense.set_pixel(x, y, *color_map[color])
        elif color == "HAPPY":
            self.sense.set_pixels(happy)
        elif color == "SAD":
            self.sense.set_pixels(sad)
        elif color == "NORMAL":
            self.sense.set_pixels(normal)

    def get_sensors_data(self):
        return {
            'humidity': round(self.sense.get_humidity(), 2),
            'temperature': round(self.sense.get_temperature(), 2)
        }

    def clear(self):
        self.sense.clear()