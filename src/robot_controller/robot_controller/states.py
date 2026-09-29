from enum import Enum

# Enumeration des states du FSM
class RobotState(Enum):
    # Dock & Undock
    DOCK = 0
    UNDOCK = 1

    # Etat de repos
    REST = 2

    # Retour au dock
    BACK_TO_HOME = 3

    # Suit un objet
    FOLLOW_OBJECT = 4

    # Mapping wall-following
    MAPPING_FORWARD = 5    # avance tout droit pendant le mapping
    MAPPING_FOLLOW_WALL = 6  # longe le mur à gauche
    MAPPING_TURN_LEFT = 7    # tourne à gauche pour entrer dans l'ouverture
    MAPPING_TURN_RIGHT = 8   # tourne à droite pour éviter obstacle devant
    MAPPING_DONE = 9

    # Manuel
    MANUAL = 10

    # Patrol
    PATROL_FORWARD = 11      # se balade
    PATROL_FOLLOW = 12       # suit l'objet détecté
    PATROL_INSPECT = 13      # attend de voir si l'objet bouge
    PATROL_AVOID = 14        # considère obstacle, repart
    PATROL_COOLDOWN = 15     # ignore la détection temporairement

    BACK_TO_PREDOCK = 16
