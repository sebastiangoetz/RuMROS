from abc import ABC, abstractmethod
from rclpy.node import Node
from .sensors.rad_sensor_adapter import *
from .sensors.sensor_abstraction_layer import *
from .swarm_behavior.cf_behavior import CFIdleBehavior
from .swarm_behavior.idle import IdleBehavior
from .swarm_behavior.swarm_behavior import SwarmBehavior
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from geometry_msgs.msg import Twist

class ROSHardware(ABC):
    def __init__(self, node: Node, hw_type: str, robot_id: str):
        super().__init__()

        self.node = node
        self.hw_type = hw_type
        self.robot_id = robot_id
        self.prefix = ''
        self.available_behaviors = ()
        self.idle_behavior: SwarmBehavior = IdleBehavior()
        self.movement_publisher = None

    # Common to all
    def setup_pose_sensor(self, topic: str = 'pose'):
        self.pose_sensor = PoseStampedSensor(self.node, f'/{self.prefix}{self.robot_id}/{topic}')
    
    # Setup methods for generic sensor types only
    def setup_sensors(self):
        self.setup_pose_sensor()

    def setup_actuators(self):
        # Default for Foot-Bot and Turtlebot
        self.movement_publisher = self.node.create_publisher(Twist, f'/{self.prefix}{self.robot_id}/cmd_vel', 10)

    def setup_services(self):
        pass

class Footbot(ROSHardware):
    def __init__(self, node: Node, hw_type: str, robot_id: str):
        super().__init__(node, hw_type, robot_id)
        self.prefix = 'fb'
        self.available_behaviors = (
            'idle',
            'driving',
            'random_walking',
            'diffusing',
            'flocking',
            'moving',
            'moving_oa'
        )
    
    def setup_sensors(self):
        super().setup_sensors()
        self.rad_sensor = FootbotProximitySensor(self.node, f'/{self.prefix}{self.robot_id}/proximityList')

class Turtlebot(ROSHardware):
    def __init__(self, node: Node, hw_type: str, robot_id: str):
        super().__init__(node, hw_type, robot_id)
        self.prefix = 'tb'
        self.available_behaviors = (
            'idle',
            'driving',
            'random_walking',
            'diffusing',
            'flocking',
            'moving',
            'moving_oa'
        )

    def setup_sensors(self):
        super().setup_sensors()
        self.rad_sensor = LDS01LidarSensor(self.node, f'/{self.prefix}{self.robot_id}/lidarScan')

class Crazyflie(ROSHardware):
    def __init__(self, node: Node, hw_type: str, robot_id: str):
        super().__init__(node, hw_type, robot_id)
        self.prefix = 'cf'
        self.available_behaviors = (
            'idle',
            'moving',
            'takeoff',
            'landing',
            'moving_relative'
        )
        self.idle_behavior = CFIdleBehavior()

    def setup_services(self):
        self.takeoff_client = self.node.create_client(Takeoff, f'/{self.prefix}{self.robot_id}/takeoff')
        self.goto_client = self.node.create_client(GoTo, f'/{self.prefix}{self.robot_id}/go_to')
        self.land_client = self.node.create_client(Land, f'/{self.prefix}{self.robot_id}/land')    
    

ROSHardware_REGISTRY: dict[str, ROSHardware] = {
    'footbot': Footbot,
    'turtlebot': Turtlebot,
    'crazyflie': Crazyflie
}

def make_hardware(node: Node, type_name: str, robot_id: str) -> ROSHardware:
    try:
        cls = ROSHardware_REGISTRY[type_name]
    except KeyError:
        raise ValueError(f"Unknown ROS hardware type: {type_name}")
    return cls(node, type_name, robot_id)
