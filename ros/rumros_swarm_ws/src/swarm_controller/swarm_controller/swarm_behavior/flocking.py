import numpy as np
import time
from sklearn.cluster import DBSCAN
from typing import List
from geometry_msgs.msg import Twist, PoseStamped
from .swarm_behavior import SwarmBehavior
from swarm_controller.sensors.sensor_abstraction_layer import RadSensor, TRadData, PoseStampedSensor
from swarm_controller.utils.movement import force_to_twist
#import matplotlib.pyplot as plt

class FlockingBehavior(SwarmBehavior):
    """
    Implements simple flocking behavior without obstacle detection according to Algorithm 2
    in 'Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory'
    (See https://ieeexplore.ieee.org/abstract/document/1605401). Flocking is a form of
    behavior that allows the swarm to hold a certain formation while working towards a goal.
    The organization is characterized
    by three main components:
        Cohesion   - sticking closely to nearby neighbors,
        Separation - avoiding collisions with nearby neighbors,
        Alignment  - matching the velocity of nearby neighbors.
    Which concrete behavior or formation manifests among a swarm of robots depends on the
    parameters given, see the linked paper and constructor parameters for more information.
    Requires a Ranging-And-Detection type sensor.
    """
    def __init__(self, rad_sensor: RadSensor, pose_sensor: PoseStampedSensor, target_x: float, target_y: float, 
                 neighbor_radius: float = 1.5, spacing: float = 1.0, k_align: float = 1.0,
                 k_spacing: float = 2.0, k_heading: float = 0.5, cluster_epsilon: float = 0.12,
                 max_speed: float = 0.5, **kwargs):
        """
        Initializes the flocking behavior.

        Args:
            rad_sensor (RadSensor): The Ranging-And-Detection type sensor of the robot.
            pose_sensor (PoseStampedSensor): The ROS2 PoseStamped message sensor of the robot.
            target_x (float, optional): The x-coordinate of the target location.
            target_y (float, optional): The y-coordinate of the target location.
            neighbor_radius (float, optional): The radius within which neighbors are sensed. Defaults to 1.5.
            spacing (float, optional): The spacing to hold between robots. Defaults to 1.0.
            k_align (float, optional): Alignent force gain. Defaults to 1.0.
            k_spacing (float, optional): Spacing force gain. Defaults to 2.0.
            k_heading (float, optional): Heading force gain. Defaults to 0.5.
            cluster_epsilon (float, optional): Distance below which multiple sensor detections will be considered one neighbor. Defaults to 0.12.
            max_speed (float, optional): Maximum speed of the robot. Defaults to 0.5.
        """
        super().__init__('flocking', [rad_sensor, pose_sensor])

        if rad_sensor.min_range > spacing or rad_sensor.max_range < spacing or rad_sensor.min_range > neighbor_radius or rad_sensor.max_range < neighbor_radius:
            raise ValueError("The neighbor sensing radius or spacing parameters fall outside the RAD sensor's range.")
        
        self.target = np.array([target_x, target_y])
        self.last_velocity = np.zeros(2)
        self.last_cluster_positions = None
        self.last_cluster_time = time.time()
        self.neighbor_radius = neighbor_radius
        self.spacing = spacing
        self.k_align = k_align
        self.k_spacing = k_spacing
        self.k_heading = k_heading
        self.cluster_epsilon = cluster_epsilon
        self.max_speed = max_speed

    def potential(self, r: float) -> float:
        """Defines a potential function with linear spring-like potential:
        repulsive if < spacing and attractive if > spacing.

        Args:
            r (float): The distance to check.

        Returns:
            float: The evaluation result of the potential function, with numbers
            greater than 0 indicating attraction and numbers below indicating repulsion.
        """
        return r - self.spacing

    def cluster_neighbors(self, hits: List[np.ndarray]) -> List[np.ndarray]:
        """Clusters nearby hits using DBSCAN and returns the mean position of each cluster.

        Args:
            hits (List[np.ndarray]): The array of distances indicating hits of a RAD-like sensor.

        Returns:
            List[np.ndarray]: The list of cluster centers (means of clustered points).
        """
        if not hits:
            return []

        # Convert np.ndarray list to 2D np array
        points = np.vstack(hits) # shape: (n_samples, 2)

        # Run DBSCAN
        clustering = DBSCAN(eps=self.cluster_epsilon, min_samples=1).fit(points)

        labels = clustering.labels_
        clustered = []

        for label in np.unique(labels):
            cluster_points = points[labels == label]
            avg = np.mean(cluster_points, axis=0)
            clustered.append(avg)

        return clustered

    def update_clusters(self, clusters: List[np.ndarray], _time: float, current_velocity: np.ndarray):
        """Updates the internal clustering data structures with the given values.

        Args:
            clusters (List[np.ndarray]): The cluster centers.
            _time (float): The time of computation.
            current_velocity (np.ndarray): The current velocity of the robot.
        """
        self.last_cluster_positions = clusters
        self.last_cluster_time = _time
        self.last_velocity = current_velocity

    def _tick(self) -> Twist:
        twist = Twist()

        # Assume sensor index exists because super() was initialized with a RadSensor
        lidar_data: TRadData = self.sensor_data[self.rad_sensor_index]
        pose: PoseStamped = self.sensor_data[self.pose_stamped_sensor_index]
        if lidar_data:
            # Convert polar coordinates of hits to cartesian
            hits = []
            for angle, dist in lidar_data:
                if 0.0 < dist < self.neighbor_radius:
                    # Polar to cartesian
                    x = dist * np.cos(angle)
                    y = dist * np.sin(angle)

                    # Apply 90 deg counterclockwise rotation to match lidar visualization frame
                    # This is necessary because Argos' coordinate system is mismatched with the
                    # turtlebot's forward heading, likely due to its orientation bug.
                    # TODO: apply this fix only if the current robot is a turtlebot,
                    #  if other bots should also support this behavior in the future. As the
                    #  foot-bot is limited to a range of 10cm, this behavior is currently not
                    #  intended to be used with it.
                    x_rot = -y
                    y_rot = x

                    hits.append(np.array([x_rot, y_rot]))

            # For debugging: plot lidar data
            # xs, ys = zip(*hits)
            # plt.scatter(xs, ys)
            # plt.axis('equal')
            # plt.title("Lidar hits")
            # plt.show()

            # Apply clustering to nearby points (single detection per robot)
            clusters = self.cluster_neighbors(hits)

            # Store cluster positions on first iteration for velocity computation
            current_time = time.time()
            if not self.last_cluster_positions:
                self.update_clusters(clusters, current_time, np.zeros(2))
                return twist

            # Compute time delta
            dt = current_time - self.last_cluster_time
            if dt == 0: dt = 1e-3  # Prevent divide by zero, though it shouldn't happen

            # Compute spacing force
            spacing_force = np.zeros(2)
            for neighbor in clusters:
                dist = np.linalg.norm(neighbor) # Distance to neighbor
                if dist == 0:
                    continue
                direction = neighbor / dist # Normalized direction
                phi = self.potential(dist)
                spacing_force += phi * direction

            # Compute alignment force
            alignment_force = np.zeros(2)
            for curr, prev in zip(clusters, self.last_cluster_positions):
                delta = curr - prev
                vel_neighbor = delta / dt
                alignment_force += vel_neighbor - self.last_velocity
            
            # Normalize
            if len(clusters) > 0:
                alignment_force /= len(clusters)

            # Compute heading force
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
            #  if other bots should also support this behavior in the future. As the
            #  foot-bot is limited to a range of 10cm, this behavior is currently not
            #  intended to be used with it.
            S = [[ 0,  -1],
                 [1,  0]]
            rot_matrix = rot_matrix @ S
            target_vector_local = rot_matrix @ target_vector_world

            heading_force = np.zeros(2)
            if target_dist > 0.1:
                heading_force = target_vector_local / target_dist

            total_force = (
                self.k_align * alignment_force + 
                self.k_spacing * spacing_force + 
                self.k_heading * heading_force
            )

            # Clip to max velocity
            total_force = np.clip(total_force, -self.max_speed, self.max_speed)
            
            # Update state
            # Dampen alignment for smoother movement
            self.update_clusters(clusters, current_time, 0.8 * self.last_velocity + 0.2 * total_force)    

            # total_force is the desired heading angle (with it's magnitude being the desired speed)
            # This algorithm works for robots with a differential drive, so the robot has to keep
            # driving and rotating to follow the currently computed angle
            twist = force_to_twist(total_force,
                                   epsilon=0.2,
                                   max_linear=self.max_speed)
            
            # ===== DEBUG VISUALIZATION =====
            # Convert lidar hits to x and y for plotting
            # if hits:
            #     xs, ys = zip(*hits)
            #     plt.scatter(xs, ys, c='blue', label='Lidar Hits')

            # # Draw heading vector (robot forward direction)
            # plt.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='green', label='Heading (0,1)')

            # # Draw total_force vector (computed movement)
            # plt.quiver(0, 0, total_force[0], total_force[1], angles='xy', scale_units='xy', scale=1, color='red', label='Total Force')

            # # Draw target direction vector (in local frame)
            # plt.quiver(0, 0, target_vector_local[0], target_vector_local[1], angles='xy', scale_units='xy', scale=1, color='orange', label='Target Vector')

            # # Show target position in world coordinates relative to robot (transformed)
            # target_pos_local = target_vector_local
            # plt.scatter([target_pos_local[0]], [target_pos_local[1]], color='magenta', marker='x', label='Target Pos')

            # plt.axis('equal')
            # plt.legend()
            # plt.title("Robot Local Frame View")
            # plt.grid(True)
            # plt.xlabel("x (right)")
            # plt.ylabel("y (forward)")
            # plt.show()
        
        return twist
