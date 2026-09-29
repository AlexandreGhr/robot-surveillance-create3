import math
from robot_controller.states import RobotState
from robot_controller.behaviors.base import BaseBehavior
from robot_controller.config import SPEED_RETURN_HOME, SPEED_TURN_RETURN_HOME


class BackToHomeBehavior(BaseBehavior):
    def __init__(self, node):
        super().__init__(node)

    def _get_pos(self):
        odom = self.odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        theta = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        return x, y, theta

    def output_back_to_home(self):
        """Étape 1 : navigue vers la position de pré-dock (position après le recul)."""
        if self._node._predock_x is None:
            self.stop()
            self.log('Position pré-dock inconnue — fais un undock dabord')
            return

        x, y, theta = self._get_pos()
        dx = self._node._predock_x - x
        dy = self._node._predock_y - y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.15:
            # Arrivé en position de pré-dock → passe à l'alignement
            self.stop()
            self._node._state = RobotState.BACK_TO_PREDOCK
        else:
            # Navigue vers la position de pré-dock
            target_angle = math.atan2(dy, dx)
            diff = self.normalize_angle(target_angle - theta)
            self.publish_vel(linear=SPEED_RETURN_HOME, angular=diff * 2.0)

    def output_back_to_predock(self):
        """Étape 2 : tourne pour faire face au dock, puis lance le docking."""
        if self._node._predock_angle is None:
            self._node._state = RobotState.DOCK
            return

        x, y, theta = self._get_pos()
        diff = self.normalize_angle(self._node._predock_angle - theta)

        if abs(diff) < 0.1:
            # Bien aligné face au dock → lance le docking natif
            self.stop()
            self._node._state = RobotState.DOCK
        else:
            # Tourne sur place pour s'aligner
            self.publish_vel(angular=math.copysign(SPEED_TURN_RETURN_HOME * 0.5, diff))