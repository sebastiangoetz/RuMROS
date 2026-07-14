import rclpy, time
import numpy as np
from typing import List
from sklearn.cluster import DBSCAN
from rclpy.node import Node
from rumros_msgs.msg import LidarScan
import matplotlib.pyplot as plt

class LidarVisualizer(Node):
    """
    ROS 2 node for real-time visualization and clustering of LiDAR scan data.

    The node subscribes to a ``LidarScan`` topic, converts polar scan data to
    Cartesian coordinates, and renders:

    * A 2D point cloud of the LIDAR measurements.
    * Cluster centers computed using DBSCAN.

    Clustering is performed in Cartesian space using a configurable epsilon
    radius. The visualization updates once per second to limit rendering
    overhead.
    """
    def __init__(self, topic: str, epsilon: float):
        """Initializes the LiDAR visualizer node.

        Args:
            topic (str): Name of the ROS 2 topic publishing ``LidarScan`` messages.
            epsilon (float): Neighborhood radius used by DBSCAN for clustering.
        """
        super().__init__('lidar_visualizer')
        self.topic = topic
        self.cluster_epsilon = epsilon

        # Lidar data subscription
        self.lid_sub = self.create_subscription(LidarScan, self.topic, self.lidar_callback, 10)

        # Set up plots
        plt.ion()
        self.fig_scan, self.ax_scan = plt.subplots()
        self.fig_scan.canvas.manager.set_window_title(self.topic)

        self.fig_clusters, self.ax_clusters = plt.subplots()
        self.fig_clusters.canvas.manager.set_window_title('Clusters')

        self.last_update_time = 0

    def rotate_90_degrees(self, angles, distances):
        """Rotates the given LiDAR measurements by 90 degrees clockwise.

        Args:
            angles (numpy.ndarray): Array of angles.
            distances (numpy.ndarray): Array of distances.

        Returns:
            tuple: The rotated angles and distances.
        """
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        x_rot = y
        y_rot = -x
        new_angles = np.arctan2(y_rot, x_rot)
        new_distances = np.hypot(x_rot, y_rot)
        new_angles = (new_angles + 2 * np.pi) % (2 * np.pi)
        return new_angles, new_distances

    def cluster_neighbors(self, hits: List[np.ndarray]) -> List[np.ndarray]:
        """Uses DBSCAN followed by taking the mean of clustered hits for each
        cluster to compute cluster centers of LiDAR hits.

        Args:
            hits (List[np.ndarray]): An array of (x, y) tuples representing
            positions to cluster. 

        Returns:
            List[np.ndarray]: List of cluster center coordinates computed as
            the mean of each cluster.
        """
        if not hits:
            return []
        points = np.vstack(hits)
        clustering = DBSCAN(eps=self.cluster_epsilon, min_samples=1).fit(points)
        labels = clustering.labels_
        clustered = []
        for label in np.unique(labels):
            cluster_points = points[labels == label]
            avg = np.mean(cluster_points, axis=0)
            clustered.append(avg)
        return clustered
    
    def lidar_callback(self, msg: LidarScan):
        """Processes incoming LiDAR scan messages and updates visualizations.

        Args:
            msg (LidarScan): Incoming LiDAR scan message.
        """
        current_time = time.time()
        if current_time - self.last_update_time < 1.0:
            return

        self.last_update_time = current_time

        distances = (np.array(msg.datapoints) / 100.0)
        distances = np.where(distances > 0.0, distances + 0.12, distances)
        angles = np.linspace(0, 2*np.pi, msg.n, endpoint=False)

        # BUG: Currently the turtlebot3 plugin of argos has a bug where
        # the model is rotated the wrong way. This also seems to mess with
        # heading direction, the robot appears to move sideways.
        # To counteract this, a 90° clockwise rotation is applied here
        # Related: https://github.com/ilpincy/argos3-turtlebot3/issues/1
        angles, distances = self.rotate_90_degrees(angles, distances)

        # Polar to cartesian conversion
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)

        # LIDAR data plot
        self.ax_scan.clear()
        self.ax_scan.scatter(x, y, s=5, c='blue')
        self.ax_scan.set_xlim(-max(distances), max(distances))
        self.ax_scan.set_ylim(-max(distances), max(distances))
        self.ax_scan.set_aspect('equal')
        self.ax_scan.set_title('LIDAR 2D Point Cloud')
        self.ax_scan.set_xlabel('X (m)')
        self.ax_scan.set_ylabel('Y (m)')

        # Add heading indicator arrow
        self.ax_scan.arrow(0, 0, 0, 0.3,
                           head_width=0.05, head_length=0.07,
                           fc='red', ec='red', linewidth=2)

        self.fig_scan.canvas.draw()
        self.fig_scan.canvas.flush_events()

        # Clustered data plot
        self.ax_clusters.clear()
        hits = [np.array([x[i], y[i]]) for i in range(len(x)) if distances[i] > 0.0]
        clusters = self.cluster_neighbors(hits=hits)

        self.ax_clusters.scatter(x, y, s=3, c='lightgray', label='Points')
        if clusters:
            cluster_x, cluster_y = zip(*clusters)
            self.ax_clusters.scatter(cluster_x, cluster_y, s=50, c='orange', marker='x', label='Cluster Centers')

        self.ax_clusters.set_xlim(-max(distances), max(distances))
        self.ax_clusters.set_ylim(-max(distances), max(distances))
        self.ax_clusters.set_aspect('equal')
        self.ax_clusters.set_title('Clustered Points')
        self.ax_clusters.set_xlabel('X (m)')
        self.ax_clusters.set_ylabel('Y (m)')
        self.ax_clusters.legend()

        self.fig_clusters.canvas.draw()
        self.fig_clusters.canvas.flush_events()

def main(args=None):
    """Entry point for the LiDAR visualizer node.

    Args:
        args (List[str], optional): Command-line arguments passed to ``rclpy.init()``.
        Defaults to None.
    """
    rclpy.init(args=args)

    import sys
    if len(sys.argv) <= 1:
        print("Usage: ros2 run lidar_vis visualizer <lidar_topic> [<cluster_epsilon>]")
        return
    topic = sys.argv[1]
    epsilon = sys.argv[2] if len(sys.argv) > 2 else 0.2

    node = LidarVisualizer(topic, epsilon)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
