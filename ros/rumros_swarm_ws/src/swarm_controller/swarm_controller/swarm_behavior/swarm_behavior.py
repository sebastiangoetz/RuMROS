from typing import List, Any
from swarm_controller.pattern_implementations import PushObserver
from swarm_controller.sensors.sensor_abstraction_layer import *
from geometry_msgs.msg import Twist
from abc import ABC, abstractmethod
from rclpy.node import Node


class SwarmBehavior(PushObserver, ABC):
    """
    Defines a common interfaces for classes that implement swarm behavior.
    It assumes that robots are steered via ROS Twist messages.
    """
    def __init__(self, name: str, sensors: List[Sensor] = [], node: Node = None):
        """Creates the behavior in an inactive state. A heterogeneous list of
        sensors can be provided, allowing their data to be used to compute
        movement commands.

        Args:
            name (str): The name of the behavior.
            sensors (List[Sensor], optional): A list of sensor objects. Defaults to [].
            node (Node, optional): The ROS node for additional functionality such as logging. Defaults to None.
        """
        super().__init__()
        self.node = node
        self.name = name
        self.sensors = sensors
        self.started_sensors : List[int] = [-1 for _ in sensors] # Holds observer registration IDs
        self.sensor_data: list = [None for _ in sensors] # Holds sensor data
        self.sensor_timestamps : List[float] = [-1 for _ in sensors] # Holds sensor last update timestamps
        self.is_stopped = True

        # Assign fields for indices of different sensor types
        # For now assume only one of each sensor type
        # Extend with new sensor types as needed..
        self.rad_sensor_index = -1
        self.pose_stamped_sensor_index = -1
        for i in range(len(self.sensors)):
            if isinstance(self.sensors[i], RadSensor):
                self.rad_sensor_index = i
            if isinstance(self.sensors[i], PoseStampedSensor):
                self.pose_stamped_sensor_index = i

    def _tick_sensor(self, sensor_index: int, sensor_data):
        """
        Ticks a sensor at the given index.

        Args:
            sensor_index (int): The internal index of the sensor.
            sensor_data (Any): The sensor data.
        """
        # Only tick if sensor exists
        if sensor_index >= 0:
            self.sensor_data[sensor_index] = sensor_data
            self.sensor_timestamps[sensor_index] = self.sensors[sensor_index].get_last_update_time()

    # === Sensor specific methods for better type safety and finer control in subclasses ===
    def tick_rad_sensor(self, sensor_data: TRadData):
        """
        Stores data from the RadSensor in the the internal data array.

        Args:
            sensor_data (TRadData): The sensor data to store.
        """
        self._tick_sensor(self.rad_sensor_index, sensor_data)

    def tick_pose_stamped_sensor(self, sensor_data: PoseStamped):
        """
        Stores data from the RadSensor in the the internal data array.

        Args:
            sensor_data (PoseStamped): The PoseStamped message to store.
        """
        self._tick_sensor(self.pose_stamped_sensor_index, sensor_data)


    def tick(self) -> Any:
        """Template method. Checks if the behavior should run and then
        conditionally executes it.

        Returns:
            Any: The ROS message to steer the robot.
        """
        if self.is_stopped:
            return Twist() # Zero velocities -> no movement
        return self._tick()

    @abstractmethod
    def _tick(self) -> Twist:
        """Hook method of the template method pattern. Used in subclasses
        to implement the behavior of the swarm node.

        Returns:
            Twist: The `Twist` message to steer the robot.
        """
        pass

    def start(self):
        """Starts the behavior by registering the sensor callback"""
        for i in range(len(self.sensors)):
            # Add more sensor types here once implemented..
            if i == self.rad_sensor_index:
                self.started_sensors[i] = self.sensors[i].register(self.tick_rad_sensor)
            elif i == self.pose_stamped_sensor_index:
                self.started_sensors[i] = self.sensors[i].register(self.tick_pose_stamped_sensor)

        self.is_stopped = False
    
    def stop(self):
        """Stops the behavior by unregistering the sensor callbacks"""
        for i in range(len(self.sensors)):
            obs_id = self.started_sensors[i]
            if obs_id >= 0:
                self.sensors[i].unregister(obs_id)
                self.started_sensors[i] = -1
                self.sensor_data[i] = None
        self.is_stopped = True
