import numpy as np
import math
from typing import Callable, Optional
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from .swarm_behavior import SwarmBehavior
from swarm_controller.sensors.sensor_abstraction_layer import RadSensor, TRadData, PoseStampedSensor
from swarm_controller.utils.movement import force_to_twist

def is_inside_bounding_volume(bb_min: np.ndarray,
                              bb_max: np.ndarray,
                              pos: np.ndarray) -> bool:
    if len(pos) > len(bb_min):
        raise ValueError("pos has more dimensions than the bounding volume")

    return np.all((bb_min[:len(pos)] <= pos) & (pos <= bb_max[:len(pos)]))
class MovingBehavior(SwarmBehavior):
    """
    Implements simple moving behavior towards a target position.
    """
    def __init__(self, node: Node, rad_sensor: RadSensor, pose_sensor: PoseStampedSensor,
                 x: float, y: float, z: float, theta: float, velocity: float, **kwargs):
        """
        Initializes the moving behavior.

        Args:
            node (Node): The currently running node, used to print logs.
            rad_sensor (RadSensor): The Ranging-And-Detection type sensor of the robot.
            pose_sensor (PoseStampedSensor): The ROS2 PoseStamped message sensor of the robot.
            x (float): The x-coordinate of the target location.
            y (float): The y-coordinate of the target location.
            z (float): The z-coordinate of the target location. Currently unused
            theta (float): The target orientation.
            velocity (float): Maximum speed of the robot.
        """
        super().__init__('moving', [rad_sensor, pose_sensor], node)

        self.target = np.array([x, y])
        self.theta = theta
        self.velocity = velocity
        self.goal_tolerance = 0.1 # Within this radius around the target, the location is assumed to be reached
        self.bb_min = None
        self.bb_max = None

        if 'on_finish' in kwargs:
            self.register(kwargs['on_finish']) # Callback to call when arrived
        if 'goal_tolerance' in kwargs:
            self.goal_tolerance = kwargs['goal_tolerance']
        if 'bb_min' in kwargs and 'bb_max' in kwargs:
            self.bb_min = kwargs['bb_min']
            self.bb_max = kwargs['bb_max']

    def compute_heading_force(self,
                              target_vector_local: np.ndarray,
                              target_dist: float,
                              lidar_data: Optional[TRadData]) -> np.ndarray:
        """Compute a heading force in the robot local frame.
        Subclasses can override this to extend functionality.

        Args:
            target_vector_local (np.ndarray): The direction to the target position.
            target_dist (float): The distance to the target position.
            lidar_data (Optional[TRadData]): Optional RAD sensor data to use for processing in subclasses.

        Returns:
            np.ndarray: The local (x,y) heading vector.
        """
        if target_dist > 1e-6:
            return target_vector_local / target_dist
        return np.zeros(2)

    def _tick(self) -> Twist:
        twist = Twist()

        # Move towards target location
        pose: PoseStamped = self.sensor_data[self.pose_stamped_sensor_index]
        if pose:
            current_pos = np.array([pose.pose.position.x, pose.pose.position.y])
            target_vector_world = self.target - current_pos
            target_dist = np.linalg.norm(target_vector_world)

            # Convert target direction into robot local frame
            q = pose.pose.orientation
            theta = 2 * np.arctan2(q.z, q.w)

            # Rotate world vector into local frame
            rot_matrix = np.array([
                [np.cos(-theta), -np.sin(-theta)],
                [np.sin(-theta),  np.cos(-theta)]
            ])

            # 90 deg counterclockwise rotation matrix
            # to counteract turtlebot rotation bug
            # TODO: apply this fix only if the current robot is a turtlebot
            S = [[0, -1],
                 [1,  0]]
            rot_matrix = rot_matrix @ S
            target_vector_local = rot_matrix @ target_vector_world

            heading_force = np.zeros(2)

            lidar_data: TRadData = self.sensor_data[self.rad_sensor_index]
            nearest_obstacle_dist = 10000
            if (lidar_data):
                for angle,dist in lidar_data:
                    # Check if front obstructed
                    if (angle >= (2 * math.pi - math.radians(10.0)) or angle <= math.radians(10.0)):
                        nearest_obstacle_dist = min(nearest_obstacle_dist, dist)
                
            # When arrived (or slightly off but obstructed), moving is done
            if (target_dist <= self.goal_tolerance) or (self.bb_min is not None and is_inside_bounding_volume(self.bb_min, self.bb_max, current_pos)):
                # Arrived at target location, rotate towards desired heading
                current_yaw = theta
                desired_yaw = self.theta
                angle_diff = (desired_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi # Normalize

                if abs(angle_diff) > 0.05: # Arbitrary threshold
                    twist.angular.z = np.clip(angle_diff, -0.5, 0.5) # Low rotation speed
                    twist.linear.x = 0.0
                else:
                    # Once rotating, invoke callback
                    self.notify()
                    twist = Twist()      
            else:
                # Drive towards target location
                heading_force = self.compute_heading_force(target_vector_local, target_dist, lidar_data)

                twist = force_to_twist(heading_force,
                                       epsilon=0.2,
                                       max_linear=self.velocity,
                                       max_angular=self.velocity) 
        return twist
