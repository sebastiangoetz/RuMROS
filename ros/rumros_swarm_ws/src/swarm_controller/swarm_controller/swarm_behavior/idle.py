from .swarm_behavior import SwarmBehavior
from geometry_msgs.msg import Twist
from rclpy.node import Node

class IdleBehavior(SwarmBehavior):
    """A simple behavior that does nothing, used when the robot is idle."""
    def __init__(self, **kwargs):
        super().__init__('idle')
    
    def _tick(self) -> Twist:
        return Twist() # Empty -> zero velocity
