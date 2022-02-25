from typing import Optional, List, Tuple
from sensor_msgs.msg import Joy
import pygame


class JoystickInput(object):

    NUM_BUTTONS: int = 12

    def __init__(self,
                 left_stick_vert: Optional[float] = None,
                 left_stick_horz: Optional[float] = None,
                 right_stick_vert: Optional[float] = None,
                 right_stick_horz: Optional[float] = None,
                 buttons: Optional[List[bool]] = None,
                 dpad: Tuple[float, float] = (0, 0)
                 ):
        self.left_stick_vert: float = left_stick_vert
        self.left_stick_horz: float = left_stick_horz
        self.right_stick_vert: float = right_stick_vert
        self.right_stick_horz: float = right_stick_horz
        self.buttons: List[bool] = buttons if buttons else [0]*12
        # First element is horizontal, second is vertical
        self.dpad: Tuple[float, float] = dpad

    def to_msg(self) -> Joy:
        jmsg: Joy = Joy()
        jmsg.axes = [
            self.left_stick_horz, self.left_stick_vert, self.right_stick_horz, self.right_stick_vert,
            self.dpad[0], self.dpad[1]
        ]
        jmsg.buttons = self.buttons
        return jmsg

    def __str__(self):
        return ("Left Stick: vertical: %f, horizontal: %f\n"
                "Right Stick: vertical: %f, horizontal: %f\n"
                "Buttons 1-4: %d, %d, %d, %d\n"
                "Left triggers 5, 7: %d, %d\n"
                "Right triggers 6, 8: %d, %d\n"
                "Center buttons 9, 10: %d, %d\n"
                "Dpad (horz axis, vert axis): %s"
                "" % (self.left_stick_vert,
                      self.left_stick_horz,
                      self.right_stick_vert,
                      self.right_stick_horz,
                      self.buttons[0], self.buttons[1], self.buttons[2], self.buttons[3],
                      self.buttons[4], self.buttons[6],
                      self.buttons[5], self.buttons[7],
                      self.buttons[8], self.buttons[9],
                      str(self.dpad)
                      ))


class Joystick(object):

    def __init__(self, fid=0):
        pygame.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(fid)
        self.js.init()

    def get_input(self) -> JoystickInput:
        pygame.event.get()
        jinput: JoystickInput = JoystickInput(
            left_stick_horz=self.js.get_axis(0),
            left_stick_vert=-self.js.get_axis(1),
            right_stick_horz=self.js.get_axis(2),
            right_stick_vert=-self.js.get_axis(3),
            buttons=[self.js.get_button(b) for b in range(JoystickInput.NUM_BUTTONS)],
            dpad=self.js.get_hat(0)
        )
        return jinput


def joystick_input_from_msg(msg) -> JoystickInput:
    return JoystickInput(
        left_stick_vert=msg.axes[1], left_stick_horz=msg.axes[0],
        right_stick_vert=msg.axes[3], right_stick_horz=msg.axes[2],
        buttons=msg.buttons, dpad=tuple((msg.axes[4], msg.axes[5]))
    )

