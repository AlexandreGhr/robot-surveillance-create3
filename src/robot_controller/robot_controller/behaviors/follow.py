from robot_controller.behaviors.base import BaseBehavior
from robot_controller.config import SPEED_FORWARD_FOLLOW, SPEED_TURN_FOLLOW, CLOSE_THRESHOLD, SPEED_SLOW_FORWARD


class FollowBehavior(BaseBehavior):
    def __init__(self, node):
        super().__init__(node)

    def output_follow_object(self):
        ir = self.ir

        if ir.is_too_close():
            # Trop proche → s'arrêter
            self.stop()

        elif ir.is_object_detected():
            # Objet devant → avancer (avec correction agulaire)
            error = ir.front_left() - ir.front_right()
            correction = error * 0.001
            self.publish_vel(linear=SPEED_FORWARD_FOLLOW, angular=correction)

        elif ir.is_object_left() or ir.is_object_far_left():
            # Objet à gauche → tourner gauche
            self.publish_vel(linear=SPEED_SLOW_FORWARD, angular=SPEED_TURN_FOLLOW)

        elif ir.is_object_right() or ir.is_object_far_right():
            # Objet à droite → tourner droite
            self.publish_vel(linear=SPEED_SLOW_FORWARD, angular=-SPEED_TURN_FOLLOW)

        else:
            # Rien détecté → attendre
            self.stop()

