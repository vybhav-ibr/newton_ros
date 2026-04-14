"""Handler for entity metadata services"""

from simulation_interfaces.msg import Result, EntityInfo, Bounds
from .entity_helpers import get_entity_state
from geometry_msgs.msg import Vector3
import re



class EntityMetadataHandler:
    """Handles GetEntityInfo and SetEntityInfo services"""

    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    TYPE_BOX = 1

    def __init__(self, node, scene_manager):
        """Initialize the entity metadata handler."""
        self.node = node
        self.scene_manager = scene_manager

    def get_entity_info_callback(self, request, response):
        """Retrieve metadata for a specific entity."""
        """Get entity metadata (category, description, tags)"""
        self.logger.info(f"GetEntityInfo service called: {request.entity}")

        response.result = Result()

        # Check if entity exists
        if request.entity not in self.scene_manager.entities_info.keys():
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response

        # Return stored entity info
        if request.entity in self.scene_manager.entities_info.keys():
            entity_info = self.scene_manager.entities_info[request.entity]
            response.info = EntityInfo()
            response.info.description = entity_info["description"]
            response.info.tags = entity_info["tags"]
        else:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = (
                f"Entity info for the entity '{request.entity}' was not set"
            )
        response.result.result = self.RESULT_OK
        return response

    def set_entity_info_callback(self, request, response):
        """Register or update metadata for a specific entity."""
        """Set entity metadata (category, description, tags)"""
        self.logger.info(f"SetEntityInfo service called: {request.entity}")

        response.result = Result()

        # Check if entity exists
        if request.entity not in self.scene_manager.entities_info.keys():
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response

        # Store entity info
        self.scene_manager.entities_info[request.entity][
            "category"
        ] = request.info.category
        self.scene_manager.entities_info[request.entity][
            "description"
        ] = request.info.description
        self.scene_manager.entities_info[request.entity]["tags"] = request.info.tags

        response.result.result = self.RESULT_OK
        return response

    def get_entity_bounds_callback(self, request, response):
        """Calculate and return the axis-aligned bounding box of an entity."""
        """Get entity bounding box/hull"""
        self.logger.info(f"GetEntityBounds service called: {request.entity}")

        response.result = Result()

        # Check if entity exists
        if request.entity not in self.scene_manager.entities_info.keys():
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response
        else:
            entity = self.scene_manager.entities_info.get(request.entity)["entity_attr"]

        response.bounds = Bounds()
        response.bounds.type = self.TYPE_BOX
        for point in entity.get_AABB().detach().cpu().numpy()[0]:
            response.bounds.points.append(
                Vector3(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            )

        response.result.result = self.RESULT_OK
        return response

    def get_entities_callback(self, request, response):
        """List all entities that match the provided criteria."""
        """Get list of all entities matching filters"""
        self.logger.info("GetEntities service called")

        response.entities = []
        response.result = Result()
        for entity_name, entity_info in self.scene_manager.entities_info.items():
            if re.match(request.filters.filter, entity_name):
                response.entities.append(entity_name)
                continue
            if entity_info.category in request.filters.categories.category:
                response.entities.append(entity_name)
                continue
            if any(item in entity_info.tags for item in request.filters.tags.tags):
                response.entities.append(entity_name)
                continue

        response.result.result = self.RESULT_OK
        if response.entities == []:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "No entities found matching the filters"
        return response

    def get_entities_states_callback(self, request, response):
        """Retrieve states for all entities matching the provided filters."""
        """Get list of entities with their states"""
        self.logger.info("GetEntitiesStates service called")

        response.entities = []
        response.states = []
        response.result = Result()
        for entity_name, entity_info in self.scene_manager.entities_info.items():
            if re.match(request.filters.filter, entity_name):
                response.entities.append(entity_name)
                entity_sate_info, result = get_entity_state(
                    entity_name, self.scene_manager
                )
                response.states.append(entity_sate_info)
                if result.result != self.RESULT_OK:
                    response.result.result = result.result
                    response.result.error_message = result.error_message
                continue
            if entity_info.category in request.filters.categories.category:
                response.entities.append(entity_name)
                entity_sate_info, result = get_entity_state(
                    entity_name, self.scene_manager
                )
                response.states.append(entity_sate_info)
                if result.result != self.RESULT_OK:
                    response.result.result = result.result
                    response.result.error_message = result.error_message
                continue
            if any(item in entity_info.tags for item in request.filters.tags.tags):
                response.entities.append(entity_name)
                entity_sate_info, result = get_entity_state(
                    entity_name, self.scene_manager
                )
                response.states.append(entity_sate_info)
                if result.result != self.RESULT_OK:
                    response.result.result = result.result
                    response.result.error_message = result.error_message
                continue

        response.result.result = self.RESULT_OK
        if response.entities == []:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "No entities found matching the filters"
        return response
