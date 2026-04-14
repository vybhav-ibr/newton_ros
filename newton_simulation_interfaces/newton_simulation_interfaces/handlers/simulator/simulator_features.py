"""Handler for GetSimulatorFeatures service (PRIORITY 1: MANDATORY)"""

from simulation_interfaces.msg import SimulatorFeatures



class SimulatorFeaturesHandler:
    """Handles GetSimulatorFeatures service - the only MANDATORY OSI service"""

    def __init__(self, node):
        """Initialize the simulator features handler."""
        self.node = node

    def get_simulator_features_callback(self, request, response):
        """Report available OSI features and supported asset formats."""
        self.logger.info("GetSimulatorFeatures service called")

        features = SimulatorFeatures()
        # List all supported features by feature ID
        features.features = [
            0,  # SPAWNING,
            4,  # ENTITY_TAGS,
            5,  # ENTITY_BOUNDS,
            6,  # ENTITY_BOUNDS_BOX,
            8,  # ENTITY_CATEGORIES,
            10,  # ENTITY_STATE_GETTING,
            11,  # ENTITY_STATE_SETTING,
            12,  # ENTITY_INFO_GETTING,
            13,  # ENTITY_INFO_SETTING,
            20,  # SIMULATION_RESET,
            21,  # SIMULATION_RESET_TIME,
            22,  # SIMULATION_RESET_STATE,
            24,  # SIMULATION_STATE_GETTING,
            25,  # SIMULATION_STATE_SETTING,
            26,  # SIMULATION_STATE_PAUSE,
            31,  # STEP_SIMULATION_SINGLE,
            32,  # STEP_SIMULATION_MULTIPLE,
            33,  # STEP_SIMULATION_ACTION,
            40,  # WORLD_LOADING,
            42,  # WORLD_TAGS,
            44,  # WORLD_INFO_GETTING,
        ]

        # Supported spawn formats
        features.spawn_formats = [
            "urdf",
            "mjcf",
            "usd",
            "obj",
            "glb",
            "gltf",
            "stl",
            "fbx",
            "ply",
        ]  # Genesis supports these formats

        # Custom info
        features.custom_info = "Genesis Simulator - OSI Implementation v1.0"

        response.features = features
        return response
