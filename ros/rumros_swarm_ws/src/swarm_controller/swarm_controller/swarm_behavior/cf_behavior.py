from crazyflie_interfaces.srv import GoTo, Takeoff, Land
from geometry_msgs.msg import Point
from rclpy.client import Client
from .swarm_behavior import SwarmBehavior
from rclpy import spin_until_future_complete
from rclpy.node import Node

def call_service(node: Node, client: Client, req):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Waiting for service for {req.__name__}...')
    future = client.call_async(req)
    #spin_until_future_complete(node, future)
    return future.result()

class CFIdleBehavior(SwarmBehavior):
    """
    Does nothing.
    """
    def __init__(self, **kwargs):
        super().__init__('idle') # Semantically equivalent to standard idle behavior, so no distinction

    def _tick(self):
        pass

class CFGoToBehavior(SwarmBehavior):
    """
    Implements moving to a given position and yaw setpoint.
    """
    def __init__(self, node: Node, client: Client, x: float, y: float, z: float, relative=False, **kwargs):
        super().__init__('moving')

        self.called = False
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.duration = 3.0
        self.relative = relative
        self.yaw = kwargs.get('theta', 0.0)
        self.client = client
        self.node = node

    def _tick(self):
        if not self.called:
            # Ensure call is only performed once
            self.called = True
            req = GoTo.Request()
            req.duration.sec = int(self.duration)
            req.duration.nanosec = int((self.duration - int(self.duration)) * 1e9)
            req.goal = Point(x = self.target_x, y = self.target_y, z = self.target_z)
            req.relative = self.relative
            req.yaw = self.yaw
            call_service(self.node, self.client, req)
            return None

class CFTakeoffBehavior(SwarmBehavior):
    """
    Implements taking off from the ground.
    """
    def __init__(self, node: Node, client: Client, height: float, duration: float, **kwargs):
        super().__init__('takeoff')

        self.called = False
        self.height = height
        self.duration = duration
        self.client = client
        self.node = node

    def _tick(self):
        if not self.called:
            # Ensure call is only performed once
            self.called = True
            req = Takeoff.Request()
            req.height = self.height
            req.duration.sec = int(self.duration)
            req.duration.nanosec = int((self.duration - int(self.duration)) * 1e9)
            call_service(self.node, self.client, req)
            return None

class CFLandBehavior(SwarmBehavior):
    """
    Implements landing the Crazyflie.
    """
    def __init__(self, node: Node, client: Client, height: float, duration: float, **kwargs):
        super().__init__('landing')

        self.called = False
        self.height = height
        self.duration = duration
        self.client = client
        self.node = node

    def _tick(self):
        if not self.called:
            # Ensure call is only performed once
            self.called = True
            req = Land.Request()
            req.height = self.height
            req.duration.sec = int(self.duration)
            req.duration.nanosec = int((self.duration - int(self.duration)) * 1e9)
            call_service(self.node, self.client, req)
            return None

