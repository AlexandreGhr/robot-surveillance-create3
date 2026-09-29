from robot_controller.behaviors.base import BaseBehavior
from robot_controller.config import SPEED_FORWARD_MAPPING, WALL_LEFT, SPEED_TURN_MAPPING, LEFT_TRY
import math

class MappingBehavior(BaseBehavior):
    def __init__(self, node):
        super().__init__(node)

    def output_forward(self):
        ir = self.ir

        if ir.is_obstacle_ahead_mapping():
            # Mur trouvé devant → s'arrête
            self.stop()
        else:
            # Avance tout droit
            self.publish_vel(linear=SPEED_FORWARD_MAPPING)

    def output_follow_wall(self):
        ir = self.ir

        if ir.is_obstacle_ahead_mapping():
            # Mur devant → tourne à droite sur place
            self.publish_vel(angular=-SPEED_TURN_MAPPING)

        elif ir.is_wall_strictly_left():
            if self._node.time_since_last_transition < (math.radians(30) / SPEED_TURN_MAPPING):
                self.publish_vel(angular=SPEED_TURN_MAPPING)
                    
            # Mur à gauche → avance en corrigeant
            wall_signal = ir.get('side_left')
            error = wall_signal - WALL_LEFT
            correction = error * 0.001
            self.publish_vel(linear=SPEED_FORWARD_MAPPING, angular=correction)

        else:
            # Mur perdu → tourne doucement à gauche pour le retrouver
            self.publish_vel(linear=SPEED_FORWARD_MAPPING * 0.5, angular=SPEED_TURN_MAPPING * 0.5)

    def output_turn_left(self):
        # Tourne à gauche pour suivre une ouverture
        self.publish_vel(angular=SPEED_TURN_MAPPING)

    def output_turn_right(self):
        # Tourne à droite pour éviter un obstacle
        self.publish_vel(angular=-SPEED_TURN_MAPPING)