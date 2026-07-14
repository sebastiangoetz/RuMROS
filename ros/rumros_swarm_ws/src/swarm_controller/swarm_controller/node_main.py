import rclpy, json
from enum import Enum
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from .swarm_behavior.swarm_behavior import SwarmBehavior
from .swarm_behavior.dispersion import DispersionBehavior
from .swarm_behavior.idle import IdleBehavior
from .swarm_behavior.flocking import FlockingBehavior
from .swarm_behavior.drive import DriveBehavior
from .swarm_behavior.moving import MovingBehavior
from .swarm_behavior.moving_with_obstacle_avoidance import MovingWithOABehavior
from .swarm_behavior.random_walk import RandomWalkBehavior
from .swarm_behavior.cf_behavior import *
from .hardware import *
from std_msgs.msg import String

class CmdMsgType(Enum):
    ACK = 0
    CHANGE_GROUP = 1
    POSITION = 2

class BehaviorController(Node):
    """
    Controls the behavior of an individual swarm member via an internal control loop that executes behavior.
    External commands can be given via /cmd_behavior/<group_id> to update the behavior of all nodes of a group.
    """
    def __init__(self, group_id: int, robot_id: int, robot_type: str):
        """Creates the RuMROS robot controller for the given robot type with the given IDs.

        Args:
            group_id (int): The numeric ID of the group the robot should work with.
            robot_id (int): The numeric ID of the robot node.
            robot_type (str, optional): The type of robot this controller steers.
            This will affect which sensors are available. Defaults to 'turtlebot'.
        """
        self.group_id = group_id
        self.hardware = make_hardware(self, robot_type, robot_id)

        super().__init__(f'sg{group_id}_{self.hardware.prefix}{robot_id}')

        # Robot-specific sensor, actuator and service setup
        self.hardware.setup_sensors()
        self.hardware.setup_actuators()
        self.hardware.setup_services()

        # Initial behavior: stand still
        self.current_behavior = self.hardware.idle_behavior

        # Subscribe to group behavior topic
        # Java+JastAdd app publish topic update messages
        # Make sure no messages are dropped and potentially added robots receive the last command
        self.qos = QoSProfile(depth=10)
        self.qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.group_behavior_sub = self.create_subscription(String, f'/swarm{group_id}/cmd_behavior', self.group_command_callback, self.qos)

        # Subscribe and publish to broadcast command topic
        cmd_topic = '/rumros/cmd_ctrl'
        ack_topic = '/rumros/cmd_ack'
        self.cmd_sub = self.create_subscription(String, cmd_topic, self.cmd_ctrl_callback, self.qos)
        self.ack_pub = self.create_publisher(String, ack_topic, self.qos)

        # Behavior tick is decoupled from sensor tick and thus can be
        # called more or less often depending on device performance.
        # Behavior tick simply accesses the latest sensor values
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(f'Initialized swarm node {robot_id} in group {group_id} of type {robot_type}')
        self.get_logger().info(f'Current state is {self.current_behavior.name}')

    # def call_service(self, client, req):
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info(f'Waiting for service for {req.__name__}...')
    #     future = client.call_async(req)
    #     spin_until_future_complete(self, future)
    #     return future.result()

    def group_command_callback(self, msg: String):
        """Function which is called on receiving group commands from the runtime model.
        Group commands always adjust the group's behavior.

        Args:
            msg (String): The ROS String message in JSON format containing the name and
            parameters of the behavior to execute.
        """
        state = json.loads(msg.data)
        name = state['name']
        parameters = state['parameters']
        if name in self.hardware.available_behaviors:
            # Switch to behavior
            self.switch_behavior(name, parameters)
        else:
            self.get_logger().warning(f'Unknown behavior: {msg.data}')

    def cmd_ctrl_callback(self, msg: String):
        """Function which is called on receiving control commands addressed to this
        robot via the cmd_ctrl topic.

        Args:
            msg (String): The ROS String message, with JSON content as created in
            `ArgosModel.connect` in the runtime model.
        """
        # Control message structure:
        # String msg key data contains JSON str
        # data contains uuid (message UUID), sender (str), receiver (str), type (int) and content (JSON str)
        # content varies depending on message type:
        # Change group message - name (group name: str), behavior (behavior name: str), parameters (list of parameters as in group commands)
        # Optionally, a change group message may contain the values of a position message in order to move there before applying the change
        # Position message - x, y, z, theta, velocity (5x float)
        data = json.loads(msg.data)

        # Handle control commands, which are addressed to individual robots
        # If sender is runtime model and this robot is addressed, react
        # By protocol, an ACK (type 0x0) with message uuid as content
        # has to be sent in response
        if data['sender'] == '__controller__' and data['receiver'] == self.hardware.prefix + str(self.hardware.robot_id):
            if data['type'] == CmdMsgType.CHANGE_GROUP.value:
                # Change group command
                self.get_logger().info(f'{self.hardware.robot_id} changing group to swarm{data["content"]["id"]}')

                # Callback for when move is finished, send ACK
                def on_move_finished():
                    self.switch_group(data['content'])
                    self.ack_pub.publish(self.create_cmd_ack(data))

                # Check for position parameters
                if 'move_parameters' in data['content']:
                    self.get_logger().info(f'{self.hardware.robot_id} moving to position {data["content"]["move_parameters"]}')

                    # Disable commands, move to group, then enable new topic and send ACK
                    self.destroy_group_sub()

                    # Upon finishing, invoke callback to perform switch
                    data['content']['move_parameters']['on_finish'] = on_move_finished

                    # Start moving (parameters in move_parameters since parameters occupied by new behavior parameters)
                    if self.hardware.hw_type == 'crazyflie':
                        self.switch_behavior(self.switch_behavior('moving', data['content']['move_parameters']))
                    else:
                        self.switch_behavior("moving_oa", data['content']['move_parameters'])
                else:
                    # Perform switch immediately
                    on_move_finished()
            elif data['type'] == CmdMsgType.POSITION.value:
                # Change position command, move to position, then ACK
                self.get_logger().info(f'{self.hardware.robot_id} moving to position {data['content']['parameters']}')
                def on_move_finished():
                    self.get_logger().info(f'{self.hardware.robot_id} finished moving, switching to idle')
                    self.switch_behavior('idle')
                    self.ack_pub.publish(self.create_cmd_ack(data))
                data['content']['parameters']['on_finish'] = on_move_finished
                if self.hardware.hw_type == 'crazyflie':
                    self.switch_behavior('moving', data['content']['parameters'])
                else:
                    self.switch_behavior('moving_oa', data['content']['parameters'])
                

    def create_cmd_ack(self, data):
        """Builds the ACK message as an answer to a cmd_ctrl message.

        Args:
            data (dict): A dictionary containing valid IDs for `sender`, `receiver`, and `uuid`.

        Returns:
            String: The ROS String message with JSON-encoded content of the ACK message data. 
        """
        ack_obj = {
            # Swap sender and receiver
            "sender": data['receiver'],
            "receiver": data['sender'],
            "type": CmdMsgType.ACK.value,
            "content": data['uuid']
        }
        
        ack_msg = String()
        ack_msg.data = json.dumps(ack_obj)
        return ack_msg

    def destroy_group_sub(self):
        """Removes the subscription to the group topic."""
        if self.group_behavior_sub:
            self.destroy_subscription(self.group_behavior_sub)

    def update_group_sub(self):
        """Recreates the group topic subscription. Useful for renewing with
        an updated `group_id`."""
        self.destroy_group_sub()
        self.group_behavior_sub = self.create_subscription(String, f'/swarm{self.group_id}/cmd_behavior', self.group_command_callback, self.qos)

    def switch_group(self, params):
        """Performs a group switch by extracting the relevant parameters from
        a group switch message, renewing the group subscription and switching
        to the group's behavior.

        Args:
            params (dict): A dictionary with the relevant parameters for
            switching, including `id` (new group ID), `behavior` (new
            group behavior) and `parameters` (new behavior parameters).
        """
        new_group_id = params['id']
        new_behavior = params['behavior']
        behavior_params = params['parameters']

        if self.group_id != new_group_id:
            self.group_id = new_group_id
        self.update_group_sub()
        self.switch_behavior(new_behavior, behavior_params)

    def switch_behavior(self, name: str, parameters = None):
        """Changes the behavior of the robot.

        Args:
            name (str): The name of the new behavior to execute.
            parameters (dict, optional): Optional keyword arguments passed to
            the new behavior object, which can differ depending on each
            individual implementation. Defaults to None.
        """
        self.get_logger().info(f'Switching to behavior {name}')

        # Stop previous behavior
        if self.current_behavior:
            self.current_behavior.stop()

        # Set and start new behavior
        if name == 'idle':
            self.current_behavior = IdleBehavior()
        elif name == 'driving':
            self.current_behavior = DriveBehavior(**parameters)
        elif name == 'random_walking':
            self.current_behavior = RandomWalkBehavior(self, **parameters) # Additionally pass node for timer functionality
        elif name == 'diffusing':
            self.current_behavior = DispersionBehavior(self.hardware.rad_sensor, **parameters)
        elif name == 'flocking':
            self.current_behavior = FlockingBehavior(self.hardware.rad_sensor, self.hardware.pose_sensor, **parameters)
        elif name == 'moving':
            # Moving is the same parameter for CF and ground robots, since semantically
            # the action is the same for any type of robot. However, the implementation
            # differs and needs to be assigned based on type.
            if self.hardware.hw_type == 'crazyflie':
                self.current_behavior = CFGoToBehavior(self, self.hardware.goto_client, **parameters)
            else:
                self.current_behavior = MovingBehavior(self, self.hardware.rad_sensor, self.hardware.pose_sensor, **parameters)
        elif name == 'moving_oa':
            self.current_behavior = MovingWithOABehavior(self, self.hardware.rad_sensor, self.hardware.pose_sensor, **parameters)
        elif name == 'takeoff':
            self.current_behavior = CFTakeoffBehavior(self, self.hardware.takeoff_client, **parameters)
        elif name == 'landing':
            self.current_behavior = CFLandBehavior(self, self.hardware.land_client, **parameters)
        elif name == 'moving_relative':
            self.current_behavior = CFGoToBehavior(self, self.hardware.goto_client, relative=True, **parameters)
        if self.current_behavior:
            self.current_behavior.start()

    def tick(self):
        """Computes and publishes a single movement command for
        the robot. This method is called periodically by a timer."""
        if self.current_behavior:
            # Move according to behavior. All behavior nodes include
            # stop functionality internally, so no need to check here
            cmd = self.current_behavior.tick()

            # For robots that act based on service calls, not command
            # topics, cmd will be None
            if cmd is None: return
            self.hardware.movement_publisher.publish(cmd)

def main(args=None):
    """Entry point of the robot controller node. Initializes the node with
    IDs from CLI arguments and starts it with idle behavior.

    Args:
        args (List[str], optional): Command-line arguments passed to ``rclpy.init()``.
    """
    rclpy.init(args=args)

    import sys
    group_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    robot_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    robot_type = sys.argv[3] if len(sys.argv) > 3 else 'turtlebot'

    node = BehaviorController(group_id, robot_id, robot_type)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
