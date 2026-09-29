# behaviors/manual.py
from robot_controller.behaviors.base import BaseBehavior
from robot_controller.config import SPEED_FORWARD_MANUAL, SPEED_TURN_MANUAL


class ManualBehavior(BaseBehavior):
    def __init__(self, node):
        super().__init__(node)

    def forward(self):
        self.publish_vel(linear=SPEED_FORWARD_MANUAL)

    def backward(self):
        self.publish_vel(linear=-SPEED_FORWARD_MANUAL)

    def turn_left(self):
        self.publish_vel(angular=SPEED_TURN_MANUAL)

    def turn_right(self):
        self.publish_vel(angular=-SPEED_TURN_MANUAL)