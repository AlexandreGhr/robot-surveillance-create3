import sys
import tty
import termios
import select
import rclpy
from robot_controller.states import RobotState

class KeyboardController :
    def __init__(self, node):
        self._node = node

    def loop(self):
        n = self._node
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    # Lecture séquence touches fléchées (3 caractères)
                    if key == '\x1b': # Fleches directionnels
                        extra = sys.stdin.read(2)
                        key = key + extra
                    if key == '\x03':  # Ctrl+C
                        rclpy.shutdown()
                        break
                    # --- Mode manuel : touches fléchées ---
                    elif key == '\x1b[A':       # flèche haut → avance
                        if n._state == RobotState.MANUAL:
                            n._manual.forward()
                    elif key == '\x1b[B':       # flèche bas → recule
                        if n._state == RobotState.MANUAL:
                            n._manual.backward()
                    elif key == '\x1b[D':       # flèche gauche → tourne gauche
                        if n._state == RobotState.MANUAL:
                            n._manual.turn_left()
                    elif key == '\x1b[C':       # flèche droite → tourne droite
                        if n._state == RobotState.MANUAL:
                            n._manual.turn_right()

                    # COMMANDES GLOBALES
                    elif key == 'i': # interruption → mode manuel
                        n._light.set_manual()

                        n._state = RobotState.MANUAL
                        n.get_logger().info('Mode manuel — flèches pour bouger, r pour reprendre')

                    elif key == 'r':  # rest
                        n._light.set_rest()
                        n._sound.play_rest()
                        n._state = RobotState.REST

                    elif key =='u': #undock
                        n._light.set_dock()
                        n._sound.play_dock()
                        n._state = RobotState.UNDOCK

                    elif key == 'd': # dock
                        n._light.set_dock()
                        n._sound.play_dock()
                        n._state = RobotState.DOCK

                    elif key == 'm': # mapping (wall-following)
                        # Reset
                        n._start_x = None
                        n._start_y = None

                        n._light.set_mapping()
                        n._sound.play_mapping_start()

                        n._state = RobotState.MAPPING_FORWARD
                        n.get_logger().info('Mapping démarré - carte réinitialisé')

                    elif key == 'f': # follow object
                        n._state = RobotState.FOLLOW_OBJECT
                        n._sound.play_follow()
                        n._light.start_siren()
                    elif key == 'p':  # patrol
                        n._light.set_patrol()
                        n._sound.play_patrol_start()
                        n._state = RobotState.PATROL_FORWARD
                        n.get_logger().info('Patrol démarré')

                    elif key == 'h': # homing
                        if n._dock_x is not None:

                            n._light.set_back_to_home()
                            n._sound.play_back_to_home()

                            n._state = RobotState.BACK_TO_HOME
                            n.get_logger().info('Retour au dock')
                        else:
                            n.get_logger().warn('Position dock inconnue — fais un undock d\'abord')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

# A FAIRE : Avoir le log qui se remet a la ligne et au debut
# Verif que les sons/visuels sont ajouter