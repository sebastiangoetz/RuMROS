import numpy as np
from .sensor_abstraction_layer import RadSensor
from rumros_msgs.msg import ProximityList
from rumros_msgs.msg import LidarScan

class FootbotProximitySensor(RadSensor):
    """
    An array of 24 proximity sensors with a range of 10cm each. Received angles are in radians
    and arrangement is counter-clockwise. The values with a wrap around at the back, see the
    following simplified diagram with only 8 sensors:
    ```
    Heading ↑  (1)
             (0 rad)
                │
        (2)     │     (8)
                │
    (3) ——————— ● ——————— (7)
   (π/2)                (-π/2)

        (4)           (6)

               (5)
            (π / -π)
    ```
    See https://opentechschool-brussels.github.io/AI-for-robots-and-swarms/ref_argos.html for more details.
    """
    def __init__(self, node, topic):
        """Initializes the proximity sensor on the given topic.
        This sensor uses `ProximityList` messages.

        Args:
            node (Node): The ROS node this subscription should be associated with.
            topic (str): The ROS topic on which to listen for sensor messages.
        """
        super().__init__(node, topic, ProximityList, 0.0, 0.1)

    def sensor_callback(self, msg):
        """Processes incoming sensor messages, converting them to the intermediate
        format of the sensor abstraction layer.

        Args:
            msg (ProximityList): A RuMROS `ProximityList` message, containing an array of measured distances.
        """
        super().sensor_callback(msg) # Update timestamp

        # Convert distance to meters and map angles to [0, 2*pi]
        self._proximities = [((p.angle + 2 * np.pi) if p.angle < 0 else p.angle, p.value * 0.1) for p in msg.proximities]
        self.notify(self._proximities) # Update subscribers


class LDS01LidarSensor(RadSensor):
    """
    Adapter for an LDS-01 Lidar as used by the Turtlebot: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/.
    Minimum detection range is 12cm and maximum range is 3.5m. Received values are ordered clockwise. Maximum number of scan points
    per second is 400 (resolution ~0.9 deg) on a real-world sensor, in the simulation it can be arbitrarily high.
    """
    def __init__(self, node, topic):
        """Initializes the LiDAR sensor on the given topic.
        This sensor uses `LidarScan` messages.

        Args:
            node (Node): The ROS node this subscription should be associated with.
            topic (str): The ROS topic on which to listen for sensor messages.
        """
        super().__init__(node, topic, LidarScan, 0.12, 3.5)

    def sensor_callback(self, msg):
        """Processes incoming sensor messages, converting them to the intermediate
        format of the sensor abstraction layer.

        Args:
            msg (LidarScan): A RuMROS `LidarScan` message, containing an array
            of measured distances in cm.
        """
        super().sensor_callback(msg) # Update timestamp

        distances = np.array(msg.datapoints) / 100.0  # Convert cm to meters
        angles = np.linspace(0, 2*np.pi, msg.n, endpoint=False) # Generate angles [0, 2*pi]
    
        # BUG: Currently the turtlebot3 plugin of argos has a bug where
        # the model is rotated the wrong way. This also seems to mess with
        # heading direction. To counteract this, a 180° rotation is applied
        # Related: https://github.com/ilpincy/argos3-turtlebot3/issues/1
        angles = (angles + np.pi) % (2 * np.pi)

        # 12cm is minimum distance, use offset to correct distortion
        distances = np.where(distances > 0.0, distances + 0.12, distances)
        self._proximities = [(angles[i], distances[i]) for i in range(msg.n)]
        self.notify(self._proximities) # Update subscribers  

# Add more adapters below..