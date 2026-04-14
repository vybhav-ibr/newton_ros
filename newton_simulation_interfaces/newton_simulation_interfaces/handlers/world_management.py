import os
from simulation_interfaces.msg import Result, WorldResource, Resource

from .newton_manager import NewtonManager
from newton_ros.newton_ros_utils import add_urdf, add_mjcf, add_usd, calculate_bounds
import newton


class WorldManagementHandler:
    """Handles LoadWorld, UnloadWorld, GetCurrentWorld, GetAvailableWorlds services"""

    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    RESULT_INCORRECT_STATE = 3
    NO_RESOURCE = 4
    UNSUPPORTED_FORMAT = 5

    def __init__(self, node, newton_manager):
        """Initialize the world management handler."""
        self.node = node
        self.newton_manager = newton_manager
        self.logger = node.get_logger()

    def load_world_callback(self, request, response):
        """Load a static world model or environment into the scene."""
        self.logger.info("LoadWorld service called")

        response.result = Result()
        response.world = WorldResource()

        # Validate scene state
        if self.newton_manager.builder is None:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Scene is not initialized"
            return response

        # Check if scene is already built (model exists)
        if hasattr(self.newton_manager, "model") and self.newton_manager.model is not None:
            response.result.result = self.RESULT_INCORRECT_STATE
            response.result.error_message = (
                "Scene is already built, the world must be added before the scene is built"
            )
            return response

        # Validate resource
        if not request.entity_resource.uri:
            response.result.result = self.NO_RESOURCE
            response.result.error_message = "uri is empty"
            return response

        uri = request.entity_resource.uri
        world_name = os.path.basename(uri).split(".")[0]

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
            "is_world": True,
        }

        try:
            # Load the world using the appropriate builder function
            if uri.endswith(".urdf"):
                add_urdf(self.newton_manager.builder, entity_config)
            elif uri.endswith(".xml"):
                add_mjcf(self.newton_manager.builder, entity_config)
            elif any(uri.endswith(ext) for ext in [".usd", ".usda", ".usdc", ".usdz"]):
                add_usd(self.newton_manager.builder, entity_config)
            else:
                # Fallback to mesh loading for other formats
                mesh = newton.Mesh.create_from_file(uri)
                body = self.newton_manager.builder.add_body(
                    pos=entity_config["pos"],
                    quat=entity_config["quat"],
                )
                self.newton_manager.builder.add_shape_mesh(body=body, mesh=mesh)

        except Exception as e:
            self.logger.error(f"Failed to load world: {str(e)}")
            response.result.result = self.UNSUPPORTED_FORMAT
            response.result.error_message = str(e)
            return response

        # Initialize world info in manager
        self.newton_manager.world_info["world_name"] = world_name
        self.newton_manager.world_info["world_resource"] = uri
        self.newton_manager.world_info["description"] = ""
        self.newton_manager.world_info["tags"] = []
        self.newton_manager.world_info["world_bounds"] = calculate_bounds(entity_config)

        response.result.result = self.RESULT_OK
        response.world = WorldResource()
        response.world.name = self.newton_manager.world_info.get("world_name", "")
        response.world.resource = Resource()
        response.world.resource.uri = self.newton_manager.world_info.get(
            "world_resource", ""
        )
        response.world.description = self.newton_manager.world_info.get(
            "description", ""
        )
        response.world.tags = self.newton_manager.world_info.get("tags", [])

        self.logger.info(f"World loaded successfully: {world_name}")
        return response

    def get_current_world_callback(self, request, response):
        """Retrieve information about the currently loaded world."""
        """Get currently loaded world info"""
        self.logger.info("GetCurrentWorld service called")

        response.result = Result()

        if self.newton_manager.world_info is None:
            response.result.result = self.NO_WORLD_LOADED
            response.result.error_message = "No world currently loaded"
            return response

        response.world = WorldResource()
        response.world.name = self.newton_manager.world_info.get("world_name", "")
        response.world.world_resource = Resource()
        response.world.world_resource.uri = self.newton_manager.world_info.get(
            "world_resource", ""
        )
        response.world.description = self.newton_manager.world_info.get(
            "description", ""
        )
        response.world.tags = self.newton_manager.world_info.get("tags", [])
        response.result.result = self.RESULT_OK
        return response
