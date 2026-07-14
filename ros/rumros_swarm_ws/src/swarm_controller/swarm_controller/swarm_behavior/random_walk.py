import random as rand
from enum import Enum
import time
from geometry_msgs.msg import Twist
from .swarm_behavior import SwarmBehavior
from rclpy.node import Node
from rclpy.time import Duration

class State(Enum):
    DRIVE = 0
    TURN = 1

class RandomWalkBehavior(SwarmBehavior):
    """
    Implements random walk behavior, which makes the robot move linearly for a random time
    in a given interval. After the time elapses, the robot turns in a random direction
    and continues.
    """
    def __init__(self, node: Node, v_lin: float = 1.0, v_ang: float = 1.0,
                 t_lin_min: float = 1.0, t_lin_max: float = 5.0,
                 t_ang: float = 1.5, **kwargs):
        """
        Initializes the behavior.

        Args:
            node (Node): The currently running node, used to create timers.
            v_lin (float, optional): The linear velocity (m/s). Defaults to 1.0.
            v_ang (float, optional): The angular velocity (m/s). Defaults to 1.0.
            t_lin_min (float, optional): The minimum time to drive linearly before turning (s). Defaults to 1.0.
            t_lin_max (float, optional): The maximum time to drive linearly before turning(s). Defaults to 5.0.
            t_ang (float, optional): How long to turn for (s). Defaults to 1.5.
        """
        super().__init__('random_walking', [], node)

        self.v_lin = v_lin
        self.v_ang = v_ang
        self.t_lin_min = t_lin_min
        self.t_lin_max = t_lin_max
        self.turn_duration = Duration(seconds=t_ang)

        self.state_start_time = -1.0

        self.forward_duration = self.sample_forward_duration()
        self.turn_direction = 1.0

    def sample_turn_direction(self) -> float:
        """Samples a random turn direction between left and right.

        Returns:
            float: The sampled turn direction.
        """
        return 1 if rand.random() < 0.5 else -1

    def sample_forward_duration(self) -> Duration:
        """Samples a random duration between the specified min and max
        intervals.

        Returns:
            float: The sampled duration in seconds.
        """
        return Duration(seconds=rand.uniform(self.t_lin_min, self.t_lin_max))

    def start(self):
        # Additional state machine setup
        self.state = State.DRIVE
        self.state_start_time = self.node.get_clock().now()

        super().start()

    def _tick(self) -> Twist:
        now = self.node.get_clock().now()
        elapsed = now - self.state_start_time

        twist = Twist()
        if self.state == State.DRIVE:
            if elapsed < self.forward_duration:
                # Drive linearly
                twist.linear.x = self.v_lin
            else:
                # Drive finished, start turning
                self.state = State.TURN
                self.turn_direction = self.sample_turn_direction()
                self.state_start_time = self.node.get_clock().now()
        elif self.state == State.TURN:
            if elapsed < self.turn_duration:
                # Turn
                twist.angular.z = self.turn_direction * self.v_ang
            else:
                # Turn finished, return to driving linearly
                self.state = State.DRIVE
                self.forward_duration = self.sample_forward_duration()
                self.state_start_time = self.node.get_clock().now() 
        
        return twist
