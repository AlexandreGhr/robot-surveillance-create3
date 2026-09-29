import math
class BaseBehavior:
    """
    Classe parente de tous les behaviors.
    Donne accès au node ROS2 et ses ressources.
    """
    def __init__(self, node):
        self._node = node

    # Raccourcis pratiques utilisables dans tous les behaviors
    @property
    def ir(self):
        from robot_controller.ir_sensors import IRSensor
        return IRSensor(self._node.in_ir)

    @property
    def odom(self):
        return self._node.in_odom

    def publish_vel(self, linear=0.0, angular=0.0):
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._node.out_pub_vel.publish(msg)

    def stop(self):
        self.publish_vel(0.0, 0.0)

    def log(self, text):
        self._node.get_logger().info(text)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle