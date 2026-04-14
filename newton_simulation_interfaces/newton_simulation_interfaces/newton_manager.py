"""State manager for OSI simulation interface"""
import time

from simulation_interfaces.msg import SimulationState


class NewtonManager:
    """Manages the Genesis simulation scene, tracking entities, sensors, and execution state."""

    PLAYING = 1
    STOPPED = 0
    PAUSED = 2

    def __init__(self, builder):
        """Initialize the scene manager with a Genesis scene object."""
        # Simulation state
        self.current_state_code = self.PLAYING
        self.builder = builder

        # Entity tracking
        self.entities_info = {}

        # World tracking
        self.world_info = {}  # Currently loaded WorldResource
        self.latest_timestamp = None
        self.PENDING_REST = False

    @staticmethod
    def wxyz_to_xyzw(quat):
        return [quat[1], quat[2], quat[3], quat[0]]

    @staticmethod
    def xyzw_to_wxyz(quat):
        return [quat[3], quat[0], quat[1], quat[2]]

    def get_time(self):
        return time.time_ns()
