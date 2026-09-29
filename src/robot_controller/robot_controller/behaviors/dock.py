from irobot_create_msgs.action import Dock, Undock
from rclpy.action import ActionClient
from robot_controller.behaviors.base import BaseBehavior
from robot_controller.states import RobotState

class DockBehavior(BaseBehavior):
    def __init__(self, node):
        super().__init__(node)
        self._undock_client = ActionClient(node, Undock, '/Robot3/undock')
        self._dock_client = ActionClient(node, Dock, '/Robot3/dock')

    def output_undock(self):
        # Verif que le serv undock du robot est dispo
        if self._undock_client.wait_for_server(timeout_sec=1.0):
            goal = Undock.Goal()
            future = self._undock_client.send_goal_async(goal)
            future.add_done_callback(self._undock_done)
            self._node._state = RobotState.REST # attend la fin

        else:
            self.log('Serveur undock non disponible')

    def _undock_done(self, future):
        # Callback appelé automatique quand l'undock est fini
        self.log('Undock terminé')
        # Sauvegarde la position du dock
        odom = self._node.in_odom

        self._node._dock_x = odom.pose.pose.position.x
        self._node._dock_y = odom.pose.pose.position.y

        
        # Position après recul = position actuelle
        self._node._predock_x = odom.pose.pose.position.x
        self._node._predock_y = odom.pose.pose.position.y

        q = odom.pose.pose.orientation
        import math
        heading = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        # Le dock est derrière le robot (il a reculé) → angle = heading + 180°
        self._node._predock_angle = self.normalize_angle(heading + math.pi)

        self.log(
            f'Dock : ({self._node._dock_x:.2f}, {self._node._dock_y:.2f}) | '
            f'Pré-dock : ({self._node._predock_x:.2f}, {self._node._predock_y:.2f}) | '
            f'Angle vers dock : {math.degrees(self._node._predock_angle):.1f}°'
        )

    def output_dock(self):
        # Verif que le serv dock du robot est dispo
        if self._dock_client.wait_for_server(timeout_sec=1.0):
            goal = Dock.Goal()
            future = self._dock_client.send_goal_async(goal)
            future.add_done_callback(self._dock_done)
            self._node._state = RobotState.REST
        else:
            self.log('Serveur dock non disponible')

    def _dock_done(self, future):
        # Callback appelé automatique quand le dock est fini
        self.log('Dock terminé')