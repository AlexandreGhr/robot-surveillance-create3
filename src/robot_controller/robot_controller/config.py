# MAPPING
OBSTACLE_FRONT = 400   # obstacle devant (mapping)
WALL_LEFT = 250      # mur détecté à gauche (mapping)
WALL_LOST = 30         # mur perdu à gauche (mapping)
LEFT_TRY = 30
OBSTACLE_FRONT_LEFT = 200

SPEED_FORWARD_MAPPING = 0.1
SPEED_TURN_MAPPING = 0.1


# SPEED_TURN = 2.0
# SPEED_SEARCH = 0.1



# FOLOW
DETECTION_THRESHOLD = 35  # objet détecté (follow)
CLOSE_THRESHOLD = 1800    # objet trop proche (follow)
DETECTION_THRESHOLD_SIDE = 10  # plus sensible sur les côtés

SPEED_FORWARD_FOLLOW = 0.4
SPEED_SLOW_FORWARD = 0.2
SPEED_TURN_FOLLOW = 2.0

# MANUAL
SPEED_FORWARD_MANUAL = 2.0
SPEED_TURN_MANUAL = 2.0

# PATROL
SPEED_FORWARD_PATROL = 0.2
SPEED_TURN_PATROL = 2.0

PATROL_INSPECT_DURATION = 3.0      # secondes avant de considérer obstacle
PATROL_AVOID_DURATION = 2.5       # secondes pour s'éloigner de l'obstacle
PATROL_COOLDOWN_DURATION = 1.0     # secondes sans détection avant de reprendre

PATROL_MOVE_THRESHOLD = 50         # variation IR pour considérer que l'objet bouge

# Back to home
SPEED_RETURN_HOME = 0.2
SPEED_TURN_RETURN_HOME = 0.1

# Audio
SOUND_ENABLE = True

# Visuel (Leds)
LIGHT_ENABLE = True


