import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry  # position du robot
from irobot_create_msgs.msg import IrIntensityVector  # capteur IR
from geometry_msgs.msg import Twist

import threading
import math

# Modules internes
from robot_controller.states import RobotState
from robot_controller.config import *
from robot_controller.ir_sensors import IRSensor
from robot_controller.sound import SoundController
from robot_controller.light import LightController
from robot_controller.transitions import Transitions, STATE_DURATIONS
from robot_controller.keyboard import KeyboardController

# Behaviors
from robot_controller.behaviors.dock import DockBehavior
from robot_controller.behaviors.follow import FollowBehavior
from robot_controller.behaviors.mapping import MappingBehavior
from robot_controller.behaviors.manual import ManualBehavior
from robot_controller.behaviors.back_to_home import BackToHomeBehavior
from robot_controller.behaviors.patrol import PatrolBehavior


class RobotFSMNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Initial state
        self._state = RobotState.REST

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber : recoit les valeurs IR
        self.in_sub_ir = self.create_subscription(
            IrIntensityVector,
            '/Robot3/ir_intensity',
            self.read_ir,
            qos_profile
        )
        self.in_ir = IrIntensityVector() # Stocke le dernier message IR recu

        # Subscriber : recoit la position du robot via odométrie
        self.in_sub_odom = self.create_subscription(
            Odometry,
            '/Robot3/odom',
            self.read_odom,
            qos_profile
        )

        self.in_odom = Odometry() # Stocke le dernier message odom recu

        # Publisher : envoie les commandes de mouvement au robot
        self.out_pub_vel = self.create_publisher(Twist, '/Robot3/cmd_vel', qos_profile)

        self._patrol_inspect_signal = 0

        # Modules
        self._sound = SoundController(self)
        self._light = LightController(self)

        # Behaviors
        self._dock = DockBehavior(self)
        self._follow = FollowBehavior(self)
        self._mapping = MappingBehavior(self)
        self._manual = ManualBehavior(self)
        self._back_to_home = BackToHomeBehavior(self)
        self._patrol = PatrolBehavior(self)

        # FSM & Clavier
        self._transitions = Transitions(self)
        self._keyboard = KeyboardController(self)

        # Temps passé dans l'état actuel
        self._time_in_state = 0.0

        # Keeping track of time based on number of cycles
        self.cycle_dt = 0.05   # 20Hz 
        self.cycle_current = 0
        self.cycle_last_transition = 0
        self.time_since_last_transition = 0

        # Timer to control state transitions and publishing
        self.timer = self.create_timer(self.cycle_dt, self.control_cycle)

        # Thread clavier
        self._kb_thread = threading.Thread(target=self._keyboard.loop, daemon=True)
        self._kb_thread.start()

        # Position Memoriser
        self._start_x = None
        self._start_y = None
        self._dock_x = None
        self._dock_y = None
        self._predock_x = None
        self._predock_y = None
        self._predock_angle = None

    
    # Callback : Stock message IR recu
    def read_ir(self, msg):
        self.in_ir = msg

    # Callback : Stock position recu
    def read_odom(self, msg):
        self.in_odom = msg


    # Publie la commande selon l'etat actuel & calcul prochain etat
    def control_cycle(self):
        # --- RESTING ---
        if self._state == RobotState.REST:
            self.output_stop()

        # --- DOCK & UNDOCK ---
        elif self._state == RobotState.UNDOCK:
            self._dock.output_undock()
        elif self._state == RobotState.DOCK:
            self._dock.output_dock()

        # ---- MAPING ----
        elif self._state == RobotState.MAPPING_FORWARD:
            self._mapping.output_forward()
        elif self._state == RobotState.MAPPING_FOLLOW_WALL:
            self._mapping.output_follow_wall()
        elif self._state == RobotState.MAPPING_TURN_LEFT:
            self._mapping.output_turn_left()
        elif self._state == RobotState.MAPPING_TURN_RIGHT:
            self._mapping.output_turn_right()
        elif self._state == RobotState.MAPPING_DONE:
            self.output_stop()
        
        #--- PATROL ---
        elif self._state == RobotState.PATROL_FORWARD:
            self._patrol.output_forward()
        elif self._state == RobotState.PATROL_FOLLOW:
            self._patrol.output_follow()
        elif self._state == RobotState.PATROL_INSPECT:
            self._patrol.output_inspect()
        elif self._state == RobotState.PATROL_AVOID:
            self._patrol.output_avoid()
        elif self._state == RobotState.PATROL_COOLDOWN:
            self._patrol.output_cooldown()

        # --- MANUEL ---
        elif self._state == RobotState.MANUAL:
            pass

        # --- HOMING ---
        elif self._state == RobotState.BACK_TO_HOME:
            self._back_to_home.output_back_to_home()

        # ---- FOLLOW ----
        elif self._state == RobotState.FOLLOW_OBJECT:
            self._follow.output_follow_object()
        elif self._state == RobotState.BACK_TO_PREDOCK:
            self._back_to_home.output_back_to_predock()

     
        self._light.update()
        self.next_state()

###########
########### UTILITAIRE
###########

    def output_stop(self):
        self.out_pub_vel.publish(Twist())



    def next_state(self):
       self._transitions.compute()

    def time_state_exceeded(self):
        limit = STATE_DURATIONS.get(self._state)
        if limit is None:
            return False
        return self._time_in_state > limit

    def _is_back_at_start(self):
        if self._start_x is None:
            return False
        x = self.in_odom.pose.pose.position.x
        y = self.in_odom.pose.pose.position.y
        dist = math.sqrt((x - self._start_x)**2 + (y - self._start_y)**2)
        # Doit avoir parcouru au moins 2m avant de vérifier (évite faux positif au démarrage)
        return dist < 0.3


def main(args=None):
    rclpy.init(args=args)
    node = RobotFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()