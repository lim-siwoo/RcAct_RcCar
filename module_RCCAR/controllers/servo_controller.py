# controllers/servo_controller.py
from Raspi_PWM_Servo_Driver import PWM

class ServoController:
    def __init__(self, address, mid_pos, left_pos, right_pos):
        self.servo = PWM(address)
        self.servo.setPWMFreq(60)
        self.mid_pos = mid_pos
        self.left_pos = left_pos
        self.right_pos = right_pos

    def set_position(self, position, is_reversing=False):
        if is_reversing:
            position = self.mid_pos + (self.mid_pos - position)
        position = max(self.right_pos, min(position, self.left_pos))
        self.servo.setPWM(0, 0, position)