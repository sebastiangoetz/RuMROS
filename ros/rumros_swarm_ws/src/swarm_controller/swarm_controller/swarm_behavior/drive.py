from geometry_msgs.msg import Twist
from .swarm_behavior import SwarmBehavior

class DriveBehavior(SwarmBehavior):
    """
    Implements simple drive behavior, which makes robots move with the given velocities.
    """
    def __init__(self, v_lin: float = 1.0, v_ang: float = 0.0, **kwargs):
        """
        Initializes the drive behavior.

        Args:
            v_lin (float, optional): The linear velocity of the robot in m/s. Defaults to 1.0.
            v_ang (float, optional): The angular velocity (turn velocity) of the robot in m/s. Defaults to 0.0.
        """
        super().__init__('driving')

        self.v_lin = v_lin
        self.v_ang = v_ang

    def _tick(self) -> Twist:
        twist = Twist()
        twist.linear.x = self.v_lin
        twist.angular.z = self.v_ang
        return twist
