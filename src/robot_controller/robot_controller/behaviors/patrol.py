from robot_controller.behaviors.base import BaseBehavior
from robot_controller.config import (
    SPEED_FORWARD_PATROL, SPEED_TURN_PATROL
)

import random

class PatrolBehavior(BaseBehavior):
    def __init__(self, node):
        super().__init__(node)
        self._avoid_direction = 1  # 1 = gauche, -1 = droite

    def output_forward(self):
        # Se balade tout droit
        self.publish_vel(linear=SPEED_FORWARD_PATROL)

    def output_follow(self):
        # Identique à FollowBehavior
        ir = self.ir

        if ir.is_too_close():
            self.stop()

        elif ir.is_object_detected():
            error = ir.front_left() - ir.front_right()
            correction = error * 0.001
            self.publish_vel(linear=SPEED_FORWARD_PATROL, angular=correction)

        elif ir.is_object_left() or ir.is_object_far_left():
            self.publish_vel(linear=SPEED_FORWARD_PATROL * 0.5, angular=SPEED_TURN_PATROL)

        elif ir.is_object_right() or ir.is_object_far_right():
            self.publish_vel(linear=SPEED_FORWARD_PATROL * 0.5, angular=-SPEED_TURN_PATROL)

        else:
            self.stop()

    def output_inspect(self):
        # Attend sur place
        self.stop()

    def output_avoid(self):
        # Tire la direction aléatoire au début de l'avoid
        if self._node._time_in_state < 0.05:
            self._avoid_direction = random.choice([1, -1])

        if self._node._time_in_state < 1.0:
            # Phase 1 → recule
            self.publish_vel(linear=-SPEED_FORWARD_PATROL)
        else:
            # Phase 2 → tourne dans la direction tirée
            self.publish_vel(angular=SPEED_TURN_PATROL * self._avoid_direction)

    def output_cooldown(self):
        # Avance sans chercher
        self.publish_vel(linear=SPEED_FORWARD_PATROL)