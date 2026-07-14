from geometry_msgs.msg import Twist
import numpy as np

def force_to_twist(tagging_force: np.ndarray, epsilon: float = 0.1,
                   k_angular: float = 1.5, k_linear: float = 1.0,
                   max_linear: float = 1.0, max_angular: float = 1.0) -> Twist:
    """
    Converts a 2D heading vector in the robot's local frame to a Twist command.
    Uses angular deviation from forward vector (0, 1) to decide whether to rotate
    or move forward.

    Args:
        tagging_force (np.ndarray): A 2D numpy array (x, y) representing the deviation, heading as well as speed via magnitude.
        epsilon (float): Threshold for how much x deviation is tolerated before turning. Defaults to 0.1.
        k_angular (float): Proportional gain for angular velocity. Defaults to 1.5.
        k_linear (float): Proportional gain for linear velocity. Defaults to 1.0.
        max_linear (float): Maximum linear speed (m/s). Defaults to 1.0.
        max_angular (float): Maximum angular speed (rad/s). Defaults to 1.0.

    Returns:
        geometry_msgs.msg.Twist: The resulting velocity command.
    """
    twist = Twist()
    x, y = tagging_force

    # Compute angle between desired heading angle and forward direction
    angle = np.arctan2(x, y)

    if abs(angle) > epsilon:
        # Turn
        twist.linear.x = 0.0
        twist.angular.z = np.clip(-k_angular * angle, -max_angular, max_angular)
    else:
        # Move forward
        speed = np.linalg.norm(tagging_force)
        twist.linear.x = np.clip(k_linear * speed, -max_linear, max_linear)
        twist.angular.z = 0.0

    return twist
