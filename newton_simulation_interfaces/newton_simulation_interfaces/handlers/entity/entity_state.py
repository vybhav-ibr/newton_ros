"""Handler for entity state services"""

from geometry_msgs.msg import Pose, Twist, Accel
from simulation_interfaces.msg import Result, EntityState
from .entity_helpers import get_entity_state, set_entity_state



class EntityStateHandler:
    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    """Handles GetEntityState and SetEntityState services"""

    def __init__(self, node, scene_manager):
        """Initialize the entity state handler."""
        self.node = node
        self.scene_manager = scene_manager

    def get_entity_state_callback(self, request, response):
        """Retrieve the current pose and velocities of an entity."""
        """Get ground truth state (pose, twist, accel) of an entity"""
        self.logger.info(f"GetEntityState service called: {request.entity}")

        response.result = Result()

        # Check if entity exists
        if request.entity not in self.scene_manager.entities_info.keys():
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response

        response.state, response.result = get_entity_state(
            request.entity, self.scene_manager
        )

        return response

    def set_entity_state_callback(self, request, response):
        """Update an entity's pose and velocities (teleportation)."""
        """Set entity state (teleport, set velocity, etc.)"""
        self.logger.info(f"SetEntityState service called: {request.entity}")

        response.result = Result()

        # Check if entity exists
        if request.entity not in self.scene_manager.entities_info.keys():
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response

        response.result = set_entity_state(
            request.entity,
            self.scene_manager,
            request.state,
            request.set_pose,
            request.set_twist,
            request.set_acceleration,
        )
        return response
