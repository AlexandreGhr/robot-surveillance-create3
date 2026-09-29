from irobot_create_msgs.msg import LightringLeds, LedColor
from robot_controller.config import LIGHT_ENABLE
from robot_controller.behaviors.base import BaseBehavior

class LightController:
    def __init__(self, node):
        self._node = node
        # Publiser vers le topic des LEDs du robot
        self._led_pub = node.create_publisher(LightringLeds, '/Robot3/cmd_lightring', 10)
        self._active = False # active ou non
        self._cycle = 0
        self._max_cycles = 48  # 48 ticks x 0.05s = 2.4s (même durée que la sirène)
        self._mode = 'siren'  # mode par défaut

    def update(self):
        # Appelé à chaque tick du timer dans control_cycle
        if not self._active:
            return

    def update(self):
        if not self._active:
            return

        msg = LightringLeds()
        msg.override_system = True
        first_half = self._cycle % 10 < 5

        if self._mode == 'inspect':
            # Alterne jaune / rose
            msg.leds = [LedColor(red=255, green=255, blue=0)] * 6 if first_half else [LedColor(red=255, green=100, blue=100)] * 6
        else:
            # Alterne rouge / bleu (sirène follow)
            msg.leds = [LedColor(red=255, green=0, blue=0)] * 6 if first_half else [LedColor(red=0, green=0, blue=255)] * 6

        self._led_pub.publish(msg)
        self._cycle += 1

        # if self._cycle >= self._max_cycles:
        #     self._active = False
        #     self._reset()

    def start_siren(self):
        if not LIGHT_ENABLE :
            return
        # Lance l'effet sirène
        self._active = True
        self._mode = 'siren' 
        self._cycle = 0 # Remet compteur a 0

    def set_color(self, r, g, b):
        self._active = False # stop la couleur précedente
        # Couleur fixe
        msg = LightringLeds()
        msg.override_system = True
        msg.leds = [LedColor(red=r, green=g, blue=b)] * 6
        self._led_pub.publish(msg)


    # Effet visuel a faire 

    def set_mapping(self):
        self.set_color(255, 100, 100)  # rose

    def set_manual(self):
        self.set_color(0, 255, 0)  # vert

    def set_back_to_home(self):
        self.set_color(0, 0, 255)  # bleu

    def set_dock(self):
        self.set_color(255, 0, 0)  # rouge

    def set_rest(self):
        self.set_color(255, 255, 255)  # blanc
    def set_patrol(self):
        self.set_color(255, 165, 0)  # orange

    def _reset(self):
        # Éteint les LEDs à la fin
        msg = LightringLeds()
        msg.override_system = False
        msg.leds = [LedColor(red=0, green=0, blue=0)] * 6
        self._led_pub.publish(msg)

    def set_patrol_follow(self):
        self.set_color(255, 100, 100)  # rose

    def start_patrol_inspect(self):
        if not LIGHT_ENABLE:
            return
        self._active = True
        self._cycle = 0
        self._mode = 'inspect'

# Déplacer dans behaviors ??