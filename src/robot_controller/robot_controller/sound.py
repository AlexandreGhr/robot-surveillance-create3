import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


from robot_controller.behaviors.base import BaseBehavior
from robot_controller.config import SOUND_ENABLE

def _note(freq, ms):
    return AudioNote(
        frequency=freq,
        max_runtime=Duration(sec=0, nanosec=ms * 1_000_000)
    )


class SoundController(BaseBehavior):
    def __init__(self,node):
        super().__init__(node)
        # Client qui envoie des goals au serveur audio du Robot
        self.client = ActionClient(node, AudioNoteSequence, '/Robot3/audio_note_sequence')

    def play(self, notes, iterations=1):
        if not SOUND_ENABLE:
            return
        # Attend que le serv soit pret
        if not self.client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().warn('Serveur audio non disponible')
            return
        goal = AudioNoteSequence.Goal()
        goal.iterations  = iterations
        goal.note_sequence = AudioNoteVector(append=False, notes=notes)
        # Envoie le goal
        self.client.send_goal_async(goal)

    def play_follow(self):
        # Sière de 'police'
        self.play([
            AudioNote(frequency=700,  max_runtime=Duration(sec=0, nanosec=500_000_000)),
            AudioNote(frequency=1000, max_runtime=Duration(sec=0, nanosec=500_000_000)),
        ], iterations=2)

    def play_mapping_start(self):
        # Montée → mapping lancé
        self.play([_note(440, 150), _note(660, 150), _note(880, 300)])
 
    def play_mapping_done(self):
        # Descente → mapping terminé
        self.play([_note(880, 150), _note(660, 150), _note(440, 300)])
 
    def play_patrol_start(self):
        # Montée rapide → patrol lancé
        self.play([_note(440, 80), _note(554, 80), _note(659, 80), _note(880, 300)])
 
    def play_patrol_false_alert(self):
        # Bip descendant → fausse alerte
        self.play([_note(600, 100), _note(400, 200)])
 
    def play_dock(self):
        # Montée → dock lancé
        self.play([_note(523, 120), _note(659, 120), _note(784, 120), _note(1047, 400)])
 
    def play_dock_end(self):
        # Deux bips courts → dock terminé
        self.play([_note(880, 100), _note(880, 100)], iterations=2)
 
    def play_back_to_home(self):
        # Descente douce → retour au dock
        self.play([_note(880, 200), _note(660, 200), _note(523, 400)])
 
    def play_rest(self):
        # Bip court neutre → repos
        self.play([_note(440, 200)])
