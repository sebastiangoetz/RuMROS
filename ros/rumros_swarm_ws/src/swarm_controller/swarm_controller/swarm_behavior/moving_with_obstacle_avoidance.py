import numpy as np
from typing import Callable
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from .moving import MovingBehavior
from swarm_controller.sensors.sensor_abstraction_layer import RadSensor, TRadData, PoseStampedSensor
from swarm_controller.utils.movement import force_to_twist

class MovingWithOABehavior(MovingBehavior):
    """
    Extends MovingBehavior with obstacle avoidance using RAD sensor data.
    Uses a "follow-the-gap" scheme: obstacles within range are binned by
    direction and then inflated by the robot's own body width, so that any
    direction still considered free is guaranteed wide enough to drive through.
    The robot then steers towards the free direction closest to the goal. This
    lets it line up with the centre of tight corridors and gates instead of
    clipping their edges and stalling.
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
            z (float): The z-coordinate of the target location. Currently unused.
            theta (float): The target orientation.
            velocity (float): Maximum speed of the robot.
        """
        super().__init__(node, rad_sensor, pose_sensor, x, y, z, theta, velocity, **kwargs)

        # Tunable parameters (static in constructor for now)
        self.num_bins = 36           # Number of directional bins (finer = tighter gaps resolvable)
        self.repulsion_radius = 0.4  # Distance within which obstacles are considered [m]
        self.goal_tolerance = 0.1    # Stop distance to goal
        self.safety_radius = 0.22    # Robot half-width plus margin, used to inflate obstacles [m]

    def get_bin_index(self, bins, angle_rad):
        """Given bins spanning 360 degrees (or [0-2pi] in radians),
        determines which bin the given angle falls into.

        Args:
            bins (numpy.ndarray): The array of bins (content irrelevant, size determines which bin the angle falls into).
            angle_rad (_type_): The angle in radians.

        Returns:
            _type_: The index of the bin the angle falls into.
        """
        n_bins = len(bins)
        bin_width = 2 * np.pi / n_bins
        angle_rad = float(angle_rad % (2 * np.pi))
        id = int(np.floor(angle_rad / bin_width))
        return id % n_bins # Wrap to 0 at 2pi

    def direction_obstructed(self, bins, dir_rad):
        """Checks whether a given direction is obstructed.

        Args:
            bins (numpy.ndarray): The array of directional bins.
            dir_rad (float): The direction in radians.

        Returns:
            _type_: A boolan corresponding to whether the direction is obstructed or not.
        """
        return bins[self.get_bin_index(bins, dir_rad)] > 0.0

    def get_closest_non_obstructed_bin(self, bins, target_id):
        """Given bins spanning 360 degrees (or [0-2pi] in radians),
        finds the closest non-obstructed bin to a given bin index.
        This function only scans counterclockwise instead of both
        ways in order to avoid oscillation.

        Args:
            bins (numpy.ndarray): The array of directional bins.
            target_id (int): The bin index closest to which non-obstructed bins should be searched for.

        Returns:
            _type_: The index of the closest non-obstructed bin or `-1` if all are obstructed
        """
        n_bins = len(bins)

        if bins[target_id] == 0.0:
            return target_id

        for offset in range(1, n_bins):
            # Introduces oscillation when scanning in both directions
            # causing the robot to wiggle in place. Therefore, only one
            # direction is checked by convention
            # forward_idx = (target_id + offset) % n_bins
            # if bins[forward_idx] == 0.0:
            #     return forward_idx

            backward_idx = (target_id - offset) % n_bins
            if bins[backward_idx] == 0.0:
                return backward_idx

        # All bins obstructed
        return -1

    def angle_to_vector(self, theta: float) -> np.ndarray:
        """Converts heading angle (0 rad = forward, CCW positive) to heading vector (x, y),
        with y pointing forward, negative x left and positive x right.

        Args:
            theta (float): The angle to convert.

        Returns:
            np.ndarray: The resulting (x,y) vector.
        """
        x = -np.sin(theta)
        y =  np.cos(theta)
        return np.array([x, y])

    def vector_to_angle(self, v: np.ndarray) -> float:
        """Convert heading vector (x, y) to heading angle in [0, 2pi].

        Args:
            v (np.ndarray): The vector to convert.

        Returns:
            float: The converted angle.
        """
        x, y = v
        theta = np.arctan2(-x, y)
        if theta < 0:
            theta += 2 * np.pi
        return theta

    def inflate_obstacles(self, bins):
        """Grows every obstructed bin sideways by the angular half-width the
        robot's own body subtends at the measured distance. Without this the
        robot aims at gaps that are too narrow for it and clips the edges of
        tight corridors and gates. Any direction left free afterwards is
        therefore guaranteed wide enough for the robot to actually pass.

        Args:
            bins (numpy.ndarray): Directional bins holding the nearest obstacle distance (0 if free).

        Returns:
            numpy.ndarray: A boolean array, True where the direction is blocked for the robot's width.
        """
        n_bins = len(bins)
        bin_width = 2.0 * np.pi / n_bins
        blocked = bins > 0.0
        inflated = np.array(blocked)

        for i in range(n_bins):
            if not blocked[i]:
                continue
            dist = bins[i]
            # Half-angle occupied by the robot's body at this distance. If the
            # obstacle is closer than the safety radius it is treated as
            # blocking everything around it, to stay on the safe side
            if dist > self.safety_radius:
                half_angle = np.arcsin(self.safety_radius / dist)
            else:
                half_angle = np.pi
            spread = int(np.ceil(half_angle / bin_width))
            for offset in range(-spread, spread + 1):
                inflated[(i + offset) % n_bins] = True

        return inflated

    def select_gap_direction(self, blocked, goal_angle):
        """Chooses, among all directions the robot can fit through, the one
        whose heading is closest to the goal. Expanding outwards from the goal
        direction (rather than from the robot's current heading) makes the
        choice independent of how the robot is momentarily turned, so it does
        not oscillate. Ties are broken towards one fixed side by convention.

        Args:
            blocked (numpy.ndarray): Boolean blocked-mask as returned by inflate_obstacles.
            goal_angle (float): The direction to the goal in radians.

        Returns:
            int: The index of the chosen free bin, or `-1` if every direction is blocked.
        """
        n_bins = len(blocked)
        goal_id = self.get_bin_index(blocked, goal_angle)

        # Straight at the goal is always preferred when it is free
        if not blocked[goal_id]:
            return goal_id

        # Otherwise widen the search symmetrically around the goal direction and
        # take the first fittable direction found. The right side is checked
        # first purely by convention so the robot commits to one side instead
        # of wiggling between two equally good openings
        for offset in range(1, n_bins // 2 + 1):
            right_id = (goal_id - offset) % n_bins
            if not blocked[right_id]:
                return right_id
            left_id = (goal_id + offset) % n_bins
            if not blocked[left_id]:
                return left_id

        # Everything is blocked for the robot's width
        return -1

    def compute_heading_force(self, target_vector_local, target_dist, lidar_data):
        # Nothing to do once we are basically on top of the goal
        if target_dist < self.goal_tolerance:
            return np.zeros(2)

        # Attractive part: unit vector and angle pointing straight at the goal
        goal_dir = target_vector_local / target_dist
        goal_angle = self.vector_to_angle(goal_dir)

        # Directional bins: nearest obstacle distance if obstructed, 0 otherwise
        bins = np.array([0.0 for _ in range(self.num_bins)])
        if lidar_data:
            for angle, dist in lidar_data:
                if 0.0 < dist <= self.repulsion_radius:
                    i = self.get_bin_index(bins, angle)
                    bins[i] = min(bins[i], dist) if bins[i] > 0.0 else dist

        # Inflate obstacles by the robot's body width, then follow the gap:
        # steer towards the fittable direction closest to the goal. In a gate
        # this lines the robot up with the opening instead of a door post
        blocked = self.inflate_obstacles(bins)
        best_id = self.select_gap_direction(blocked, goal_angle)

        if best_id >= 0:
            best_dir = ((2.0 * np.pi / self.num_bins) * best_id) % (2.0 * np.pi)
            heading = self.angle_to_vector(best_dir)
            return heading * self.velocity

        # Every direction is blocked for the robot's width (e.g. wedged between
        # walls and other robots). Rather than freezing in place, back away from
        # the single nearest obstacle so the situation can free up on later ticks
        obstructed_ids = np.where(bins > 0.0)[0]
        if len(obstructed_ids) > 0:
            nearest_id = obstructed_ids[int(np.argmin(bins[obstructed_ids]))]
            nearest_dir = ((2.0 * np.pi / self.num_bins) * nearest_id) % (2.0 * np.pi)
            away = -self.angle_to_vector(nearest_dir)
            self.node.get_logger().warn("No fittable direction found, backing away from nearest obstacle. Try increasing the number of bins or reducing the safety radius.")
            return away * self.velocity

        # No obstacle information at all, just push towards the goal
        return goal_dir * self.velocity
