"""
Open Simulation Interfaces (OSI) implementation for Genesis simulator.

This module provides a ROS 2 node that implements the OSI standard,
offering standardized services for simulator control, entity management,
and state access.
"""

from rclpy.node import Node

# Import all simulation_interfaces services
from simulation_interfaces.srv import (
    GetSimulatorFeatures,
    GetSimulationState,
    SetSimulationState,
    SpawnEntity,
    GetEntities,
    GetEntitiesStates,
    GetEntityState,
    SetEntityState,
    GetEntityInfo,
    SetEntityInfo,
    GetEntityBounds,
    ResetSimulation,
    StepSimulation,
    LoadWorld,
    GetCurrentWorld,
)

from newton_ros_interfaces.srv import (
    BuildSimulator,
    GetRobotOptions,
    SetRobotOptions,
    GetSensorOptions,
    SetSensorOptions,
)

# Import state manager
from .newton_manager import NewtonManager

# Import all handlers
from .handlers import (
    SimulatorFeaturesHandler,
    SimulationControlHandler,
    EntityLifecycleHandler,
    EntityStateHandler,
    EntityMetadataHandler,
    EntityOptionsHandler,
    SensorOptionsHandler,
    WorldManagementHandler,
)
import newton
import logging


class SimulationInterface(Node):
    """
    ROS 2 node implementing the Open Simulation Interfaces (OSI) standard for Genesis simulator.

    This class provides standardized ROS 2 services for simulator control, entity management,
    and state access following the OSI specification.

    The implementation is modular, with separate handler classes for each priority level:
    - Priority 1: Mandatory features (GetSimulatorFeatures)
    - Priority 2: Core simulation control (Get/SetSimulationState)
    - Priority 3: Entity lifecycle (SpawnEntity, DeleteEntity)
    - Priority 4: Entity querying (GetEntities, GetEntitiesStates)
    - Priority 5: Entity state access (Get/SetEntityState)
    - Priority 6: Entity metadata (Get/SetEntityInfo)
    - Priority 7: Simulation reset (ResetSimulation)
    - Priority 8: Simulation stepping (StepSimulation)
    - Priority 9: Optional utilities (Bounds, NamedPoses, Spawnables)
    - Priority 10: World management (Load/UnloadWorld, GetCurrentWorld, GetAvailableWorlds)
    """

    def __init__(self, builder=None, on_build_callback=None):
        """Initialize the OSI simulation interface node and its state manager."""
        super().__init__("simulation_interface")

        # Initialize state manager
        if builder is None:
            builder =  newton.ModelBuilder()
        self.newton_manager = NewtonManager(builder)
        self.on_build_callback = on_build_callback

        self.logger = self.get_logger()

        # Initialize all handlers and services
        self._init_handlers()
        self._setup_services()
        self.logger.info("Simulation Interface initialized with OSI services")

    def _init_handlers(self):
        """Initialize all service handlers"""
        self.simulator_features_handler = SimulatorFeaturesHandler(self)
        self.simulation_control_handler = SimulationControlHandler(
            self, self.newton_manager, on_build_callback=self.on_build_callback
        )
        self.entity_lifecycle_handler = EntityLifecycleHandler(self, self.newton_manager)
        self.entity_state_handler = EntityStateHandler(self, self.newton_manager)
        self.entity_metadata_handler = EntityMetadataHandler(self, self.newton_manager)
        self.entity_options_handler = EntityOptionsHandler(self, self.newton_manager)
        self.sensor_options_handler = SensorOptionsHandler(self, self.newton_manager)
        self.world_management_handler = WorldManagementHandler(self, self.newton_manager)

    def _setup_services(self):
        """Register all OSI standard and extended ROS 2 service servers."""
        """Initialize all Simulation interface service servers"""

        self.create_service(
            GetSimulatorFeatures,
            "/simulation/get_simulator_features",
            self.simulator_features_handler.get_simulator_features_callback,
        )

        self.create_service(
            GetSimulationState,
            "/simulation/get_simulation_state",
            self.simulation_control_handler.get_simulation_state_callback,
        )

        self.create_service(
            SetSimulationState,
            "/simulation/set_simulation_state",
            self.simulation_control_handler.set_simulation_state_callback,
        )

        self.create_service(
            SpawnEntity,
            "/simulation/spawn_entity",
            self.entity_lifecycle_handler.spawn_entity_callback,
        )

        self.create_service(
            BuildSimulator,
            "/simulation/build_simulator",
            self.simulation_control_handler.build_simulation_callback,
        )

        self.create_service(
            GetEntities,
            "/simulation/get_entities",
            self.entity_metadata_handler.get_entities_callback,
        )

        self.create_service(
            GetEntitiesStates,
            "/simulation/get_entities_states",
            self.entity_metadata_handler.get_entities_states_callback,
        )

        # PRIORITY 5: ENTITY STATE ACCESS
        self.create_service(
            GetEntityState,
            "/simulation/get_entity_state",
            self.entity_state_handler.get_entity_state_callback,
        )

        self.create_service(
            SetEntityState,
            "/simulation/set_entity_state",
            self.entity_state_handler.set_entity_state_callback,
        )

        # PRIORITY 6: ENTITY METADATA
        self.create_service(
            GetEntityInfo,
            "/simulation/get_entity_info",
            self.entity_metadata_handler.get_entity_info_callback,
        )

        self.create_service(
            SetEntityInfo,
            "/simulation/set_entity_info",
            self.entity_metadata_handler.set_entity_info_callback,
        )

        self.create_service(
            GetRobotOptions,
            "/simulation/get_robot_options",
            self.entity_options_handler.get_robot_options_callback,
        )

        self.create_service(
            SetRobotOptions,
            "/simulation/set_robot_options",
            self.entity_options_handler.set_robot_options_callback,
        )

        self.create_service(
            GetSensorOptions,
            "/simulation/get_sensor_options",
            self.sensor_options_handler.get_sensor_options_callback,
        )

        self.create_service(
            SetSensorOptions,
            "/simulation/set_sensor_options",
            self.sensor_options_handler.set_sensor_options_callback,
        )

        self.create_service(
            ResetSimulation,
            "/simulation/reset_simulation",
            self.simulation_control_handler.reset_simulation_callback,
        )

        self.create_service(
            StepSimulation,
            "/simulation/step_simulation",
            self.simulation_control_handler.step_simulation_callback,
        )

        self.create_service(
            GetEntityBounds,
            "/simulation/get_entity_bounds",
            self.entity_metadata_handler.get_entity_bounds_callback,
        )

        # PRIORITY 10: WORLD MANAGEMENT
        self.create_service(
            LoadWorld,
            "/simulation/load_world",
            self.world_management_handler.load_world_callback,
        )

        self.create_service(
            GetCurrentWorld,
            "/simulation/get_current_world",
            self.world_management_handler.get_current_world_callback,
        )

        self.logger.info("All available OSI services initialized")
