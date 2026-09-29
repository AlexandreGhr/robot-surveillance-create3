from robot_controller.states import RobotState
from robot_controller.ir_sensors import IRSensor

from robot_controller.config import PATROL_INSPECT_DURATION, PATROL_AVOID_DURATION, PATROL_COOLDOWN_DURATION, PATROL_MOVE_THRESHOLD

# A VOIR SI ON GARDE
STATE_DURATIONS = {
    RobotState.DOCK: None,
    RobotState.UNDOCK: 120.0,
    RobotState.REST: None,
    RobotState.BACK_TO_HOME: None,
    RobotState.FOLLOW_OBJECT: 6000.0,
    RobotState.MAPPING_TURN_LEFT: 1000.0,
    RobotState.MAPPING_TURN_RIGHT: 1000.0,
}

class Transitions:
    def __init__(self,node):
        self._node = node

    def compute(self):
        n = self._node
        transition = False
        ir = IRSensor(n.in_ir)
        
        # --- Follow object ---
        if n._state == RobotState.FOLLOW_OBJECT:
            if n.time_state_exceeded():
                transition = True
                n._state = RobotState.REST


        # --- Mapping (Wall following)
        elif n._state == RobotState.MAPPING_FORWARD:
            if ir.is_obstacle_ahead_mapping():
                transition = True
                n._state = RobotState.MAPPING_TURN_RIGHT

        elif n._state == RobotState.MAPPING_FOLLOW_WALL:
            if ir.is_obstacle_ahead_mapping() or not ir.is_front_clear():
                transition = True
                n._state = RobotState.MAPPING_TURN_RIGHT
            # elif not ir.is_wall_strictly_left(): #
            #     transition = True
            #     n._state = RobotState.MAPPING_TURN_LEFT

            # A PACTH
            
            # elif n._is_back_at_start() and n.time_since_last_transition > 5.0: # A voir ca peut bloquer dans certain cas
            #     transition = True
            #     n._state = RobotState.MAPPING_DONE

        elif n._state == RobotState.MAPPING_TURN_LEFT:
            if ir.is_wall_strictly_left():
                transition = True
                n._state = RobotState.MAPPING_FOLLOW_WALL


        elif n._state == RobotState.MAPPING_TURN_RIGHT:
            if ir.is_front_clear() and ir.is_front_left_clear() and ir.is_wall_on_left_flank():
                transition = True
                n._state = RobotState.MAPPING_FOLLOW_WALL
                # Mémorise le point de départ la première fois seulement
                if n._start_x is None:
                    n._start_x = n.in_odom.pose.pose.position.x
                    n._start_y = n.in_odom.pose.pose.position.y
                    #n.get_logger().info(f'Point de départ mur : ({n._start_x:.2f}, {n._start_y:.2f})')
            elif not ir.is_obstacle_ahead_mapping() and n.time_since_last_transition > 2.0:
                transition = True
                n._state = RobotState.MAPPING_FORWARD
        elif n._state == RobotState.MAPPING_DONE:
            pass

        # --- Patrol ---
        elif n._state == RobotState.PATROL_FORWARD:
            if ir.is_object_detected():
                transition = True
                n._light.set_patrol_follow()
                n._state = RobotState.PATROL_FOLLOW
        elif n._state == RobotState.PATROL_FOLLOW:
            if ir.is_too_close():
                transition = True
                n._light.start_patrol_inspect()
                n._patrol_inspect_signal = ir.front()
                n._state = RobotState.PATROL_INSPECT
            elif not ir.is_object_detected() and not ir.is_object_far_left() and not ir.is_object_far_right():
                # Objet perdu → reprend la ronde
                transition = True
                n._light.set_patrol()  # retour orange
                n._state = RobotState.PATROL_COOLDOWN

        elif n._state == RobotState.PATROL_INSPECT:
            signal_diff = abs(ir.front() - n._patrol_inspect_signal)
            if signal_diff > PATROL_MOVE_THRESHOLD:
                # L'objet a bougé → reprend le suivi
                transition = True
                n._light.set_patrol_follow()
                n._state = RobotState.PATROL_FOLLOW
            elif n._time_in_state > PATROL_INSPECT_DURATION:
                # L'objet n'a pas bougé → c'est un obstacle
                transition = True
                n._light.set_patrol()
                n._state = RobotState.PATROL_AVOID

        elif n._state == RobotState.PATROL_AVOID:
            if n._time_in_state > PATROL_AVOID_DURATION:
                transition = True
                n._state = RobotState.PATROL_COOLDOWN

        elif n._state == RobotState.PATROL_COOLDOWN:
            if n._time_in_state > PATROL_COOLDOWN_DURATION:
                transition = True
                n._state = RobotState.PATROL_FORWARD

        elif n._state == RobotState.MANUAL:
            pass
        elif n._state == RobotState.BACK_TO_PREDOCK:
            pass


        # Update cycle count
        n.cycle_current +=1
        if transition:
            n.get_logger().info(f'State: {n._state}')
            n.cycle_last_transition = n.cycle_current
            n.time_since_last_transition = 0
            n._time_in_state = 0.0
        else:
            n.time_since_last_transition += n.cycle_dt
        
            # Update du temps dans chaque etat
            n._time_in_state += n.cycle_dt # a verif