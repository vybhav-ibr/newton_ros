from .sensors.sensor_utils import *
from .newton_ros_robot_control import *
from .newton_ros_sensors import *
from .newton_ros_sim import *
from .newton_ros_utils import (
    modify_builder_config,
    make_solver,
    make_viewer,
    make_collider,
    get_current_timestamp,
)
from .newton_simulate_impl import(
    soft_simulate_impl,
    rigid_simulate_impl,
    hybrid_simulate_impl
)

from rclpy.node import Node
import rclpy
import yaml
import warp as wp
import newton
import logging
import time

from newton_simulation_interfaces import SimulationInterface

class NewtonRosBridge:
    """Main bridge class between Newton simulation and ROS 2."""

    def __init__(
        self,
        ros_node,
        file_path=None,
        enable_simulation_interfaces=True,
        ros_clock_node=None,
        ros_control_node=None,
    ):
        """Initialize the bridge, setting up ROS nodes, builder, and entities."""
        self.logger = logging.getLogger(__name__)
        self.device = wp.get_device()
        self.ros_node = ros_node
        self.all_nodes_to_spin = [self.ros_node]
        self.is_built = False
        if ros_clock_node is not None:
            self.ros_clock_node = ros_clock_node
            self.all_nodes_to_spin.append(self.ros_clock_node)
        else:
            self.ros_clock_node = ros_node

        if ros_control_node is not None:
            self.ros_control_node = ros_control_node
            self.all_nodes_to_spin.append(self.ros_control_node)
        else:
            self.ros_control_node = ros_node
        
        #all newton stuff
        self.builder = newton.ModelBuilder()
        self.solver = None
        self.model = None
        self.control = None
        self.collision_pipeline=None
        self.state_0 = None  # Current state
        self.state_1 = None  # Next state
        self.contacts = None
        self.viewer = None
        self.graph=None

        self.gravity_zero = wp.zeros(1, dtype=wp.vec3)
        self.gravity = wp.array(wp.vec3(0.0, 0.0, -9.81), dtype=wp.vec3)
        #add world site for frame transformer
        self.world_site = self.builder.add_site(  
            body=-1,  # Attach to world frame  
            xform=wp.transform(wp.vec3(0, 0, 0), wp.quat_identity()),  
            label="world_origin"  
        )
        if enable_simulation_interfaces:
            self.simulation_interface = SimulationInterface(
                self.builder, on_build_callback=self.initialise_robots
            )
            self.newton_manager = self.simulation_interface.newton_manager
            self.entities_info = self.newton_manager.entities_info
            self.world_info = self.newton_manager.world_info
        else:
            self.newton_manager = None
            self.entities_info = {}
            self.world_info = {}

        if file_path is not None:
            with open(file_path, "r") as file:
                self.parent_config = yaml.safe_load(file)
            if enable_simulation_interfaces:
                self.newton_manager.builder = self.builder
            
            if self.parent_config.get("builder"):
                modify_builder_config(self.builder,self.parent_config.get("builder"))
                
            if self.parent_config.get("sim"):
                sim_config = self.parent_config["sim"]
                self.fps = sim_config.get("fps", 100)
                self.sim_substeps = sim_config.get("substeps", 10)
                self.frame_dt = 1.0 / self.fps
                self.sim_dt = self.frame_dt / self.sim_substeps
                self.update_step_interval = sim_config.get("update_step_interval", 100)
            else:
                self.fps = 100
                self.sim_substeps=20
                self.frame_dt = 1.0 / self.fps
                self.sim_dt = self.frame_dt / self.sim_substeps
                self.update_step_interval = 1
            self.sim_time = 0.0
            self.sim = NewtonRosSim(self.builder)

            if self.parent_config.get("world") is not None:
                world_name = self.parent_config["world"]["name"]
                self.logger.info(f"Adding world {world_name} to newton")
                self.sim.spawn_from_config(
                    entity_config=self.parent_config["world"],
                    entity_name=world_name,
                    entities_info=self.world_info,
                )

            # -------------------------
            # Robots
            # -------------------------
            if self.parent_config.get("objects") is not None:
                for object_name, object_config in self.parent_config.get(
                    "objects", {}
                ).items():
                    self.logger.info(f"Adding object {object_name} to scene")
                    self.sim.spawn_from_config(
                        entity_config=object_config,
                        entity_name=object_name,
                        entities_info=self.entities_info,
                    )
            # -------------------------
            # Init containers (add once before loop if not already)
            # -------------------------
            self.robot_controls = {}
            self.robot_sensor_managers = {}
            # -------------------------
            # Robots
            # -------------------------
            if self.parent_config.get("robots") is not None:
                for robot_name, robot_config in self.parent_config.get("robots", {}).items():
                    namespace = robot_config.get("namespace", "")
                    self.logger.info(f"Adding robot {namespace} to newton")

                    # spawn robot
                    self.sim.spawn_from_config(
                        entity_config=robot_config,
                        entity_name=robot_name,
                        entities_info=self.entities_info,
                    )

                    # -------------------------
                    # Robot control & Sensors
                    # -------------------------
                    robot_control = NewtonRosRobotControl(
                        self.ros_control_node,
                        robot_config.get("control", {}),
                        robot_name=robot_name,
                        entities_info=self.entities_info,
                    )
                    self.robot_controls[robot_name] = robot_control

                    # -------------------------
                    # Sensor factory
                    # -------------------------
                    sensor_manager = NewtonRosSensors(
                        self.builder,
                        self.state_0,
                        namespace,
                        robot_name=robot_name,
                        entities_info=self.entities_info,
                    )
                    self.robot_sensor_managers[robot_name] = sensor_manager

                    # -------------------------
                    # Sensors
                    # -------------------------
                    if robot_config.get("sensors") is not None:
                        for sensor_config in robot_config.get("sensors", {}):
                            sensor_manager.add_sensor(sensor_config)
                    # -------------------------
                    # ROS nodes aggregation
                    # -------------------------
                    self.all_nodes_to_spin.extend(sensor_manager.all_ros_nodes)
                    if hasattr(self, "simulation_interface"):
                        self.all_nodes_to_spin.append(self.simulation_interface)
                    
                    self.logger.info(f"Registered robot '{robot_name}'")

        else:
            self.builder = None
            self.logger.warning(
                f"No config file path provided, please provide the path or manually configure the builder"
            )
        

    def build(self, build_all_components=True):
        """Finalize the newton builder and start the ROS clock."""
        if self.parent_config.get("solver",{}).get("soft") is not None:
            solver_cfg=self.parent_config.get("solver",{}).get("soft", {})
            if solver_cfg is not None and solver_cfg.get("type") == "vbd":
                self.builder.color()
        self.model = self.builder.finalize()

        self.rigid_solver = make_solver(self.model, 
                                  solver_config=self.parent_config.get("solver",{}).get("rigid"), 
                                  logger=self.logger)
        self.soft_solver = make_solver(self.model, 
                                  solver_config=self.parent_config.get("solver",{}).get("soft"), 
                                  logger=self.logger, is_soft=True)
        self.viewer = make_viewer(viewer_config=self.parent_config.get("viewer"), 
                                  logger=self.logger)
        # self.collider=make_collider(self.model, self.parent_config.get("collider"))
        
        self.control = self.model.control()
        self.state_0 = self.model.state()  # Current state
        self.state_1 = self.model.state()  # Next state
        self.contacts=self.model.contacts()
            
        if build_all_components:
            self.build_all()
        
        if wp.get_device().is_cuda:
            if self.rigid_solver is not None and self.soft_solver is not None:
                with wp.ScopedCapture() as capture:
                    hybrid_simulate_impl(self)
            elif self.rigid_solver is not None:
                with wp.ScopedCapture() as capture:
                    rigid_simulate_impl(self)
            elif self.soft_solver is not None:
                with wp.ScopedCapture() as capture:
                    soft_simulate_impl(self)

            self.graph = capture.graph
            self.logger.info("CUDA graph captured")
        else:
            self.graph = None
            self.logger.warning("Running on CPU (no CUDA graph)")
           
        self.viewer.set_model(self.model)
        self.sim.start_clock(self.ros_clock_node)
        self.is_built = True

    def build_all(self):
        """Build all robot controls and sensors after the model is finalized."""
        for robot_control in self.robot_controls.values():
            robot_control.build(self.model, self.state_0, self.control)
        for sensor_factory in self.robot_sensor_managers.values():
            sensor_factory.build(self.model, self.state_0, self.viewer)
            
    def log_all(self):
        """Build all robot controls and sensors after the model is finalized."""
        for robot_control in self.robot_controls.values():
            robot_control.log()
        for sensor_factory in self.robot_sensor_managers.values():
            sensor_factory.log()
            
    def step(self):
        # --- lifecycle control ---
        if not self.is_built:
            self.logger.critical("simulation has not been built, call build() first")
    
        if self.newton_manager is not None and self.newton_manager.PENDING_REST:
            self.newton_manager.builder.destroy()
            self.logger.critical("simulation terminated")
            raise KeyboardInterrupt
        
        if self.viewer is not None:
            if not self.viewer.is_running():
                self.logger.warning("Simulation is paused, stopping physics and ros2_control")
                return
        
        # --- simulation ---
        self.simulate()
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.log_all()
        self.viewer.end_frame()
        # --- external state sync ---
        if self.newton_manager is not None:
            self.newton_manager.latest_timestamp = get_current_timestamp()
    
    def simulate(self):
        if self.graph is not None:
            wp.capture_launch(self.graph)
        else:
            if self.rigid_solver is not None and self.soft_solver is not None:
                hybrid_simulate_impl(self)
            elif self.rigid_solver is not None:
                rigid_simulate_impl(self)
            elif self.soft_solver is not None:
                soft_simulate_impl(self)

    def initialise_robots(self):
        """Dynamically initialize robot control and sensors for newly spawned robots."""
        if self.entities_info is not None:
            for robot_name, robot_entry in self.entities_info.items():
                if robot_entry.get("initialisation_pending", False):
                    namespace = robot_entry.get("namespace", "")
                    if robot_entry.get("robot_options") is None:
                        self.logger.critical(
                            f"Robot {robot_name} has no robot options and the subscribers, publishers for this can't be added"
                        )
                        continue
                    
                    robot_control = NewtonRosRobotControl(
                        self.ros_control_node,
                        robot_entry.get("robot_options", {}).get("control", {}),
                        entities_info=self.entities_info,
                        robot_name=robot_name,
                    )
                    self.robot_controls[robot_name] = robot_control
                    
                    sensor_factory = NewtonRosSensors(
                        self.builder,
                        self.state_0,
                        namespace,
                        robot_name=robot_name,
                        entities_info=self.entities_info,
                    )
                    self.robot_sensor_managers[robot_name] = sensor_factory
                    
                    if robot_entry.get("sensor_options") is not None:
                        for sensor_config in robot_entry.get("sensor_options").get(
                            "sensors", []
                        ):
                            sensor_factory.add_sensor(sensor_config)
                        self.all_nodes_to_spin.extend(sensor_factory.all_ros_nodes)
                    else:
                        self.logger.critical(
                            f"Robot {robot_name} has no sensor options, sensor data publishers cannot be added"
                        )
                    
                    # Call build on the new entities if model is already finalized
                    if self.model is not None:
                        robot_control.build(self.model, self.state_0, self.control)
                        sensor_factory.build(self.model, self.state_0, self.viewer)
                        
                    robot_entry["initialisation_pending"] = False
