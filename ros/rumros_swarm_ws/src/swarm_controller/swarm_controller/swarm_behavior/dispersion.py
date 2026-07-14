import math
from geometry_msgs.msg import Twist
from .swarm_behavior import SwarmBehavior
from swarm_controller.sensors.sensor_abstraction_layer import RadSensor, TRadData

class DispersionBehavior(SwarmBehavior):
    """
    Implements dispersion/diffusion behavior, which makes robots move away from obstacles
    and other robots. Moves around the arena linearly, avoiding obstacles and other robots
    by turning on obstacle detection. Requires a Ranging-And-Detection type sensor.
    """
    def __init__(self, rad_sensor: RadSensor, angle_tolerance_deg: float = 60,
                 d_min: float = 0.05, d_max: float = 0.1, velocity: float = 0.5, 
                 turn_velocity: float = 1.0, **kwargs):
        """
        Initializes the diffusion behavior.

        Args:
            rad_sensor (RadSensor): The Ranging-And-Detection type sensor.
            angle_tolerance_deg (float, optional): The opening angle [degrees] in front of the robot inside which collisions are detected. Defaults to 60 degrees.
            d_min (float, optional): The minimum distance threshold [meters] above which a collision is detected. Defaults to 0.05 meters.
            d_max (float, optional): The maximum distance threshold [meters] below which a collision is detected. Defaults to 0.1 meters.
            velocity (float, optional): The speed of the robot while moving linearly. Defaults to 0.5.
            turn_velocity (float, optional): The turning speed of the robot. Defaults to 1.0.
        """
        super().__init__('diffusing', [rad_sensor])

        self.angle_tolerance = math.radians(angle_tolerance_deg * 0.5)
        self.d_min = d_min
        self.d_max = d_max
        self.velocity = velocity
        self.turn_velocity = turn_velocity

    def in_threshold(self, dist: float):
        """Checks whether a distance value is within the interval
        specified by `self.d_min` and `self.d_max` (inclusive).

        Args:
            dist (float): The distance value to check.

        Returns:
            _type_: True if within the interval, false otherwise.
        """
        return self.d_min <= dist <= self.d_max

    def _tick(self) -> Twist:
        twist = Twist()

        # Assume sensor index exists because super() was initialized with a RadSensor
        data: TRadData = self.sensor_data[self.rad_sensor_index]
        if data:
            # Categorize sensor readings by angle (data tuples index 0) and assign values (index 1)
            # For simplicity, only consider obstacles in front

            # Detect obstacles that are in front and within range
            d_front = sum(
                1 for p in data if (p[0] >= (2 * math.pi - self.angle_tolerance) or p[0] <= self.angle_tolerance)
                and self.in_threshold(p[1])
            )

            d_left = sum(
                p[1] for p in data
                if self.angle_tolerance < p[0] <= math.pi
                and self.in_threshold(p[1])
            )

            d_right = sum(
                p[1] for p in data
                if math.pi < p[0] < (2 * math.pi - self.angle_tolerance)
                and self.in_threshold(p[1])
            )

            if d_front > 0:
                # Obstacle ahead, turn
                twist.linear.x = 0.0
                twist.angular.z = self.turn_velocity if d_left < d_right else -self.turn_velocity
            else:
                # Drive straight if no obstacle ahead
                twist.linear.x = self.velocity
                twist.angular.z = 0.0
        return twist

    def tick_rad_sensor(self, sensor_data: TRadData):
        self.sensor_data[0] = sensor_data