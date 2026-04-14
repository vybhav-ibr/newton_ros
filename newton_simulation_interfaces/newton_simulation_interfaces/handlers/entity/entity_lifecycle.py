import os
import re
import uuid
from simulation_interfaces.msg import Result, EntityInfo, EntityCategory
from simulation_interfaces.srv import SpawnEntity
from newton_simulation_interfaces.newton_manager import NewtonManager
from newton_ros.newton_ros_utils import add_urdf, add_mjcf, add_usd, add_entities_info
import newton

ROS_NAME_PATTERN = r"^[A-Za-z/~][A-Za-z0-9_/]*$"


class EntityLifecycleHandler:
    """Handles SpawnEntity and DeleteEntity services"""

    # Class-level constants
    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    RESULT_INCORRECT_STATE = 3
    RESULT_OPERATION_FAILED = 4

    NAME_NOT_UNIQUE = 101
    NAME_INVALID = 102
    UNSUPPORTED_FORMAT = 103
    NO_RESOURCE = 104
    NAMESPACE_INVALID = 105
    RESOURCE_PARSE_ERROR = 106
    MISSING_ASSETS = 107
    UNSUPPORTED_ASSETS = 108
    INVALID_POSE = 109

    def __init__(self, node, newton_manager):
        """Initialize the entity lifecycle handler."""
        self.node = node
        self.newton_manager = newton_manager
        self.logger = node.get_logger()

    def spawn_entity_callback(self, request, response):
        """Spawn an entity (robot, object) from URDF/SDF/USD/MJCF"""
        self.logger.info(f"SpawnEntity service called: {request.name}")

        response.result = Result()

        # Validate scene state
        if self.newton_manager.builder is None:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Scene is not initialized"
            self.logger.critical("Scene is not initialized")
            return response

        # Check if scene is already built (model exists)
        if hasattr(self.newton_manager, "model") and self.newton_manager.model is not None:
            response.result.result = self.RESULT_INCORRECT_STATE
            response.result.error_message = (
                "Scene is already built, all entities must be added before the scene is built"
            )
            self.logger.critical(
                "Scene is already built, all entities must be added before the scene is built"
            )
            return response

        # Validate resource
        if not request.entity_resource.uri:
            response.result.result = self.NO_RESOURCE
            response.result.error_message = "Resource URI is empty"
            return response

        uri = request.entity_resource.uri
        entity_name = request.name

        # Determine entity name and handle unique naming
        if entity_name == "":
            if not request.allow_renaming:
                response.result.result = self.NAME_INVALID
                response.result.error_message = "Name is empty and allow_renaming is false"
                return response
            entity_name = "entity_" + str(uuid.uuid4())[:5]
        elif not re.fullmatch(ROS_NAME_PATTERN, entity_name):
            if not request.allow_renaming:
                response.result.result = self.NAME_INVALID
                response.result.error_message = f"Entity name '{entity_name}' is invalid"
                return response
            entity_name = f"entity_{str(uuid.uuid4())[:5]}"

        # Check for name conflicts
        if entity_name in self.newton_manager.entities_info:
            if not request.allow_renaming:
                response.result.result = self.NAME_NOT_UNIQUE
                response.result.error_message = f"Entity name '{entity_name}' already exists"
                return response
            entity_name = f"{entity_name}_{str(uuid.uuid4())[:5]}"

        # Prepare entity config for newton_ros_utils
        entity_config = {
            "source": uri,
            "pos": [
                request.initial_pose.position.x,
                request.initial_pose.position.y,
                request.initial_pose.position.z,
            ],
            "quat": [
                request.initial_pose.orientation.x,
                request.initial_pose.orientation.y,
                request.initial_pose.orientation.z,
                request.initial_pose.orientation.w,
            ],
            "namespace": request.entity_namespace,
        }

        try:
            # Load the entity using the appropriate builder function
            if uri.endswith(".urdf"):
                add_urdf(self.newton_manager.builder, entity_config)
            elif uri.endswith(".xml"):
                add_mjcf(self.newton_manager.builder, entity_config)
            elif any(uri.endswith(ext) for ext in [".usd", ".usda", ".usdc", ".usdz"]):
                add_usd(self.newton_manager.builder, entity_config)
            else:
                # Fallback to mesh loading
                mesh = newton.Mesh.create_from_file(uri)
                body = self.newton_manager.builder.add_body(
                    pos=entity_config["pos"],
                    quat=entity_config["quat"],
                )
                self.newton_manager.builder.add_shape_mesh(body=body, mesh=mesh)

        except Exception as e:
            self.logger.error(f"Failed to spawn entity: {str(e)}")
            response.result.result = self.UNSUPPORTED_FORMAT
            response.result.error_message = str(e)
            return response

        # Track the entity info in the manager
        add_entities_info(self.newton_manager.entities_info, entity_name, entity_config)

        # Add sensors structure needed by other handlers
        self.newton_manager.entities_info[entity_name]["sensors"] = {
            "cameras": [],
            "lidars": [],
            "imus": [],
            "contacts": [],
            "contact_forces": [],
        }

        response.result.result = self.RESULT_OK
        response.entity_name = entity_name

        self.logger.info(f"Entity spawned successfully: {entity_name}")
        return response
