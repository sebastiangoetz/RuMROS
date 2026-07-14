"""
Optional: Adapter for actuator data, though this likely will have to be implemented in the ROS-Argos bridge if desired.
"""

class ActuatorAdapter:
    def update(self, msg):
        """Update internal sensor state from raw message."""
        raise NotImplementedError

# ===== Middleware interface definitions below =====

class DifferentialDriveAdapter(ActuatorAdapter):
    """Adapts Ranging-And-Detection type sensors"""
    raise NotImplementedError
    