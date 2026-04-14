"""Handler for simulation control services"""

from simulation_interfaces.msg import SimulationState, Result
from simulation_interfaces.srv import SetSimulationState



class SimulationControlHandler:
    """Handles GetSimulationState and SetSimulationState services"""

    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    ALREADY_IN_TARGET_STATE = 101
    STATE_TRANSITION_ERROR = 102
    STATE_STOPPED = 0
    STATE_PLAYING = 1
    STATE_PAUSED = 2
    SCOPE_DEFAULT = 0
    SCOPE_ALL = 255
    SCOPE_TIME = 1
    SCOPE_STATE = 3
    SCOPE_SPAWNED = 4

    def __init__(self, node, scene_manager, on_build_callback=None):
        """Initialize the simulation control handler with node and scene context."""
        self.node = node
        self.scene_manager = scene_manager
        self.on_build_callback = on_build_callback
        self.logger = node.get_logger()

    def build_simulation_callback(self, request, response):
        """Build the Genesis scene, locking entity additions."""
        self.logger.info("BuildSimulator service called")
        response.result = Result()
        if not self.scene_manager.scene.is_built:
            self.scene_manager.scene.build()
            self.scene_manager.current_state_code = self.STATE_PAUSED
            if self.on_build_callback:
                self.on_build_callback()
            self.logger.info("Simulation built sucesfully no more entities can be added")
            response.result.result = 1
        else:
            self.logger.critical("Simulation already built cant build it again")
            response.result.result = 0
            response.result.error_message = (
                "Simulation already built cant build it again"
            )

        return response

    def step_simulation_callback(self, request, response):
        """Advance the simulation by a single step."""
        self.logger.info("StepSimulation service called")
        response.result = Result()
        if (
            not self.scene_manager.scene.is_built
            or self.scene_manager.current_state_code == self.STATE_STOPPED
        ):
            self.logger.critical("Simulation is stopped cant step it")
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Simulation is stopped cant step it"
            return response
        elif self.scene_manager.current_state_code == self.STATE_PLAYING:
            self.logger.warning("Simulation is playing cant step it")
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Simulation is playing cant step it"
            return response
        elif self.scene_manager.current_state_code == self.STATE_PAUSED:
            for _ in range(request.steps):
                self.scene_manager.scene.step()
            self.logger.info("Simulation stepped sucesfully")
            response.result.result = self.RESULT_OK
            return response
        else:
            self.logger.critical("Simulation is not in a valid state to step it")
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = (
                "Simulation is not in a valid state to step it"
            )
            return response

    def get_simulation_state_callback(self, request, response):
        """Get current simulation state (STOPPED, PLAYING, PAUSED, etc.)"""
        self.logger.info("GetSimulationState service called")

        response.state = SimulationState()
        response.state.state = self.scene_manager.current_state_code

        response.result = Result()
        response.result.result = self.RESULT_OK

        return response

    def set_simulation_state_callback(self, request, response):
        """Set simulation state (play, pause, stop)"""
        self.logger.info(f"SetSimulationState service called: {request.state.state}")

        response.result = Result()
        target_state = request.state.state

        # Check if already in target state
        if self.scene_manager.current_state_code == target_state:
            response.result.result = self.ALREADY_IN_TARGET_STATE
            response.result.error_message = "Simulation already in target state"
            return response

        # Validate state transitions
        # Cannot pause if stopped
        if (
            self.scene_manager.current_state_code == self.STATE_STOPPED
            and target_state == self.STATE_PAUSED
        ):
            response.result.result = self.STATE_TRANSITION_ERROR
            response.result.error_message = "Cannot pause when simulation is stopped"
            return response

        self.scene_manager.target_state_code = target_state

        response.result.result = self.RESULT_OK
        return response

    def reset_simulation_callback(self, request, response):
        """Reset simulation (time, state, spawned entities)"""
        self.logger.info(f"ResetSimulation service called with scope: {request.scope}")

        response.result = Result()

        # Handle default scope
        if request.scope in [self.SCOPE_DEFAULT, self.SCOPE_ALL]:
            self.scene_manager.scene.reset()
            self.scene_manager.PENDING_REST = True
            return response

        # Reset entity states
        if request.scope == self.SCOPE_STATE:
            self.logger.info("Resetting entity states")
            for entity_name, entity_info in self.scene_manager.entities_info.items():
                entity = self.scene_manager.entities[entity_name]["entity_attr"]
                entity.set_pos(entity_info["position"])
                entity.set_quat(entity_info["orientation"])
                entity.set_lin_vel(entity_info["linear_velocity"])
                entity.set_ang_vel(entity_info["angular_velocity"])
                entity.set_lin_acc(entity_info["linear_acceleration"])
                entity.set_ang_acc(entity_info["angular_acceleration"])

        # Delete spawned entities
        if request.scope == self.SCOPE_SPAWNED:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Scope spawned is not supported"
            return response
        response.result.result = self.RESULT_OK
        return response
