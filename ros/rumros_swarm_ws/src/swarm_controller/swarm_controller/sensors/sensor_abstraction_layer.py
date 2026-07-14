"""
This module defines adapters that convert data from various sensor types to a common
middleware interface, thus forming a hardware abstraction layer. Concrete adapters
are then defined in modules within this package. Each concrete adapter implements
the interface of one of the middleware adapters, e.g. ConcreteLidarAdapter implements
RadSensorAdapter. The middleware adapters all inherit from SensorAdapter, providing
common functionality.
"""

from abc import ABC
from typing import Tuple, List
import time
from rclpy.node import Node
from swarm_controller.pattern_implementations import PushObserver
from geometry_msgs.msg import PoseStamped

# Sensor data type declarations
TRadData = List[Tuple[float, float]]

class Sensor(PushObserver, ABC):
    """An abstract superclass that provides a common interface for concrete
    subclassing sensors and sensor types.
    """
    def __init__(self, node: Node, topic: str, message_class):
        """Initializes the sensor on the node creating a subscription to the
        given topic.

        Args:
            node (Node): The node to create the subscription on.
            topic (str): The topic to subscribe to.
            message_class (_type_): The type of messages that are sent on the topic.
        """
        super().__init__() # PushObserver init

        self.subscription = node.create_subscription(
            message_class,
            topic,
            self.sensor_callback,
            10
        )
        self._last_update_time = time.time()

    def sensor_callback(self, msg):
        """Intended to update the internal sensor state from ROS message in subclasses.
        Implementing classes should also call self.notify() here to call observers.

        Args:
            msg (Any): A sensor message of type `message_class`.
        """
        self._last_update_time = time.time()
    
    def get_last_update_time(self):
        """Gets the timestamp of when the sensor was last updated.

        Returns:
            float: The time in seconds since the Epoch.
        """
        return self._last_update_time

# ===== Middleware interface definitions below =====

class RadSensor(Sensor, ABC):
    """
    Adapts Ranging-And-Detection type sensors like Lidar and distance sensor arrays.
    The provided interface for Lidar-type sensors is TRadData - List[Tuple[float, float]],
    an array of tuples of (angle, distance). Angles should be in radians, with 0 at
    the top and increasing counterclockwise until wrapping around at the top from 2*pi.
    Distances and ranges are in meters. When no obstacles are detected, the distance
    should be 0.0.
    """
    def __init__(self, node, topic, message_class, min_range: float, max_range: float):
        """Initializes the Ranging-And-Detection sensor.

        Args:
            node (Node): The node to create the subscription on.
            topic (str): The topic to subscribe to.
            message_class (_type_): The type of messages that are sent on the topic.
            min_range (float): The minimum range at which obstacles can be detected.
            max_range (float): The maximum range at which obstacles can be detected.
        """
        super().__init__(node, topic, message_class)
        self._proximities : TRadData = []
        self.min_range = min_range
        self.max_range = max_range

    def sensor_callback(self, msg):
        """Updates proximities. Implementation is up to a concrete sensor adapter.
        
        Args:
            msg (Any): A sensor message of type `message_class`.
        """
        super().sensor_callback(msg)

    def get_proximities(self):
        """Gets the list of proximities of the sensor.

        Returns:
            TRadData: The list of proximities.
        """
        return self._proximities

# ===== Basic ready-to-use ROS sensor definitions below =====    
class PoseStampedSensor(Sensor):
    """
    A basic sensor for processing ROS2's PoseStamped messages. This is not a middleware
    sensor, it just uses the middleware ROS2 provides. Subscribers can directly process
    the PoseStamped messages.
    """
    def __init__(self, node, topic):
        super().__init__(node, topic, PoseStamped)
        self._last_pose: PoseStamped = None
    
    def sensor_callback(self, msg: PoseStamped):
        """Processes sensor message when they are received.
        Updates the internal pose and notifies subscribers.

        Args:
            msg (PoseStamped): The received `PoseStamped` message.
        """
        super().sensor_callback(msg)
        self._last_pose = msg
        self.notify(msg)