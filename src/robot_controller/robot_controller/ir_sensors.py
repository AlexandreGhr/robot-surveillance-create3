# ir_sensor.py

from robot_controller.config import (
    OBSTACLE_FRONT, WALL_LEFT, WALL_LOST,
    DETECTION_THRESHOLD, CLOSE_THRESHOLD, DETECTION_THRESHOLD_SIDE, LEFT_TRY, OBSTACLE_FRONT_LEFT
)

class IRSensor:
    def __init__(self, ir_msg):
        self._values = self._parse(ir_msg)

    def _parse(self, ir_msg):
        # Lit tous les capteurs et retourne un dict
        values = {
            'side_left': 0, 'left': 0,
            'front_left': 0, 'front_center_left': 0,
            'front_center_right': 0, 'front_right': 0,
            'right': 0
        }
        for reading in ir_msg.readings:
            fid = reading.header.frame_id
            if fid == 'ir_intensity_side_left':
                values['side_left'] = reading.value
            elif fid == 'ir_intensity_left':
                values['left'] = reading.value
            elif fid == 'ir_intensity_front_left':
                values['front_left'] = reading.value
            elif fid == 'ir_intensity_front_center_left':
                values['front_center_left'] = reading.value
            elif fid == 'ir_intensity_front_center_right':
                values['front_center_right'] = reading.value
            elif fid == 'ir_intensity_front_right':
                values['front_right'] = reading.value
            elif fid == 'ir_intensity_right':
                values['right'] = reading.value
        return values

    # Valeur brute d'un capteur
    def get(self, key):
        return self._values.get(key, 0)

    # --- Compositions des signaux

    # Zone avant (centre)
    def front(self):
        return self._values['front_center_left'] + self._values['front_center_right']

    # Zone avant gauche
    def front_left(self):
        return self._values['front_left'] + self._values['front_center_left']

    # Zone avant droite
    def front_right(self):
        return self._values['front_right'] + self._values['front_center_right']

    # Zone gauche (suivi de mur)
    def left_wall(self):
        return self._values['left'] + self._values['side_left']

    # --- Detections - Mapping

    # Obstacle détecté devant
    def is_obstacle_ahead(self):
        return self.front() > OBSTACLE_FRONT

    def is_wall_strictly_left(self):
        return self._values['side_left'] > WALL_LEFT or self._values['left'] > WALL_LEFT
    
    def is_only_left(self):
        return self._values['front_center_left'] < LEFT_TRY or self._values['front_left'] < LEFT_TRY

    # Mur détecté à gauche
    def is_wall_left(self):
        return self.left_wall() > WALL_LEFT

    # --- Detections - Follow / Patrol

    # Objet détecté (pour follow)
    def is_object_detected(self):
        return self.front() > DETECTION_THRESHOLD

    # Objet trop proche
    def is_too_close(self):
        return self.front() > CLOSE_THRESHOLD

    # Objet plus à gauche qu'à droite
    def is_object_left(self):
        return self.front_left() > self.front_right() and self.front_left() > DETECTION_THRESHOLD

    # Objet plus à droite qu'à gauche
    def is_object_right(self):
        return self.front_right() > self.front_left() and self.front_right() > DETECTION_THRESHOLD
    
    # Objet détecté sur le côté gauche total
    def is_object_far_left(self):
        return self._values['side_left'] > DETECTION_THRESHOLD_SIDE

    # Objet détecté sur le côté droit total
    def is_object_far_right(self):
        return self._values['right'] > DETECTION_THRESHOLD_SIDE

# A VOIR

    # Mur perdu à gauche
    def is_wall_lost(self):
        return self.left_wall() < WALL_LOST
    
    def is_wall_on_left_flank(self):
        # Mur bien positionné sur le flanc gauche
        return self._values['side_left'] > WALL_LEFT

    def is_front_clear(self):
        # Plus rien devant
        return self.front() < OBSTACLE_FRONT
    def is_front_left_clear(self):
        return self._values['front_left'] < OBSTACLE_FRONT
    
    def is_obstacle_ahead_mapping(self):
        return self.front() > OBSTACLE_FRONT or self._values['front_left'] > OBSTACLE_FRONT_LEFT
