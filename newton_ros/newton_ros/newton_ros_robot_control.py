from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from .newton_ros_utils import get_current_timestamp, get_joint_names, get_dofs_idx
import logging
import warp as wp
import numpy as np
import newton
from newton.selection import ArticulationView
import re

class NewtonRosRobotControl:
    """Manages robot joint states and control commands via ROS 2 topics."""

    def __init__(
        self,
        ros_node,
        robot_config,
        entities_info=None,
        robot_name=None,
    ):
        """Initialize the robot control interface, setup publishers and subscribers."""
        self.logger = logging.getLogger(__name__)
        self.logger.info("starting robot control interfaces")
        self.model=None
        self.view = None
        self.control= None  
        self.ros_node = ros_node
        self.entities_info = entities_info
        self.robot_name = robot_name
        self.robot_config = robot_config
        self.namespace = robot_config.get("namespace", "robot")
        self.joint_names=None
        self.motor_dofs =None

        self.register_robot_options()

        self.dof_properties_set = False
        self.setup_joint_states_publisher()
        self.setup_control_subscriber()
        self.setup_joint_commands_subscriber()
        
        self.is_buit=False

    def build(self, model, state, control):
        """Build an ArticulationView for the robot to facilitate control."""
        if self.is_buit:
            return
        self.model = model
        self.state= state
        self.control= control
        self.view = ArticulationView(self.model, 
                                     self.robot_name, 
                                     exclude_joint_types=[newton.JointType.FREE,
                                                          newton.JointType.FIXED,
                                                          newton.JointType.DISTANCE],
                                     verbose=True)
        self.joint_names=self.view.joint_names
        self.is_buit=True
        
    def register_robot_options(self):
        """Register robot configuration in entities_info"""
        if self.entities_info is None or self.robot_name is None:
            return

        if self.robot_name not in self.entities_info.keys():
            self.logger.warn(
                f"Robot '{self.robot_name}' not found in entities_info, cannot register robot config."
            )
            return

        entity_entry = self.entities_info[self.robot_name]
        entity_entry["robot_options"] = self.robot_config

    def set_dofs_properties(self):
        """Apply physics properties (KP, KV, etc.) to the robot's degrees of freedom."""
        joint_properties = self.robot_config.get("joint_properties", None)
        if joint_properties is not None:
            if any("stiffness" in joint_cfg for joint_cfg in joint_properties.values()):
                self._apply_joint_property("joint_target_ke", "stiffness", "joint_target_ke")
            if any("damping" in joint_cfg for joint_cfg in joint_properties.values()):
                self._apply_joint_property("joint_target_kd", "damping", "joint_target_kd")
            if any("armature" in joint_cfg for joint_cfg in joint_properties.values()):
                self._apply_joint_property("joint_target_armature", "armature", "joint_target_armature")
            if any("friction" in joint_cfg for joint_cfg in joint_properties.values()):
                self._apply_joint_property("joint_target_friction_coeff", "friction_coeff", "joint_friction")


    def _apply_joint_property(self, attr_name, config_key, target_attr_name):
        joint_properties = self.robot_config.get("joint_properties", {})
        
        target = self.view.get_attribute(attr_name, self.model).numpy()

        for pattern, joint_cfg in joint_properties.items():
            value = joint_cfg.get(config_key, -1)
            if value <= 0:
                continue

            regex = re.compile(pattern)

            for joint_idx, joint_name in enumerate(self.joint_names):
                if regex.fullmatch(joint_name):
                    target[0, 0, joint_idx] = value

        self.view.set_attribute(target_attr_name, self.model, target)

    def setup_joint_states_publisher(self):
        """Initialize and start the joint state publisher timer."""
        self.logger.info("Joint state Publisher started")

        def timer_callback(js_publisher):
            if self.view is None:
                self.logger.warning("The build method has not been called!, skipping joint state publishing")
                return

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = get_current_timestamp()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = self.view.get_dof_positions(self.state)[0][0].numpy().tolist()
            joint_state_msg.velocity = self.view.get_dof_velocities(self.state)[0][0].numpy().tolist()
            joint_state_msg.effort = self.view.get_dof_forces(self.control)[0][0].numpy().tolist()
            js_publisher.publish(joint_state_msg)

        self.joint_state_publisher = self.ros_node.create_publisher(
            JointState,
            f'{self.namespace}/{self.robot_config.get("joint_states_topic", "joint_states")}',
            10,
        )
        self.timer = self.ros_node.create_timer(
            1.0 / self.robot_config.get("joint_states_topic_frequency", 50.0),
            lambda: timer_callback(self.joint_state_publisher),
        )

    def _trajectory_point_controller(self, point, joint_properties, dof_idx_table):
        valid = True
        pos_i, vel_i, eff_i = 0, 0, 0
        pos_vals, pos_dofs = [], []
        vel_vals, vel_dofs = [], []
        eff_vals, eff_dofs = [], []
        for joint, joint_cfg in joint_properties.items():
            if joint_cfg.get("command", "").lower() == "position":
                pos_vals.append(point.position[pos_i])
                pos_dofs.append(dof_idx_table[joint])
                pos_i += 1
            elif joint_cfg.get("command", "").lower() == "velocity":
                vel_vals.append(point.velocity[vel_i])
                vel_dofs.append(dof_idx_table[joint])
                vel_i += 1
            elif joint_cfg.get("command", "").lower() == "effort":
                eff_vals.append(msg.effort[eff_i])
                eff_dofs.append(dof_idx_table[joint])
                eff_i += 1
            else:
                self.logger.error(f"Invalid joint command type for {joint} joint")
                valid = False
        if valid:
            self._control_dofs_pos(pos_vals, pos_dofs)
            self._control_dofs_vel(vel_vals, vel_dofs)
            self._control_dofs_pos(eff_vals, eff_dofs)

    def setup_control_subscriber(self):
        """Initialize the joint trajectory (control) subscriber."""
        self.logger.info("control command subscriber started")

        def joint_control_callback(msg):
            if self.view is None:
                self.logger.warning("The build method has not been called!, skipping control subscriber callback")
                return
            
            motor_dofs = get_dofs_idx(self.joint_names, msg.joint_names)
            dof_idx_table = {}
            for k, motor_dof in enumerate(motor_dofs):
                dof_idx_table[msg.joint_names[k]] = motor_dof
            joint_properties = dict(
                sorted(self.robot_config.get("joint_properties", None).items())
            )
            if self.view is not None and not self.dof_properties_set:
                self.set_dofs_properties()
            for point in msg.points:
                self._trajectory_point_controller(
                    point, joint_properties, dof_idx_table
                )

        control_topic = self.robot_config.get(
            "joint_control_topic",
            self.robot_config.get("control_topic", "joint_control"),
        )
        control_sub = self.ros_node.create_subscription(
            JointTrajectory,
            f"{self.namespace}/{control_topic}",
            joint_control_callback,
            10,
        )
        setattr(self, f"{self.namespace}_control_subscriber", control_sub)

    def _control_dofs_pos(self, target_qpos, motor_dofs=None):
        if motor_dofs is None:
            motor_dofs = self.motor_dofs
        if target_qpos and motor_dofs:  # checks lists are non-empty
            current_qpos = self.view.get_dof_positions(self.model).numpy()
            target_qpos_np = np.array(target_qpos)  # convert list to numpy array

            for i, dof_idx in enumerate(motor_dofs):
                current_qpos[0, 0, dof_idx] = target_qpos_np[i]

            self.view.set_dof_positions(self.state, current_qpos)


    def _control_dofs_vel(self, target_qvel, motor_dofs=None):
        if motor_dofs is None:
            motor_dofs = self.motor_dofs
        if target_qvel and motor_dofs:
            current_qvel = self.view.get_dof_velocities(self.model).numpy()
            target_qvel_np = np.array(target_qvel)

            for i, dof_idx in enumerate(motor_dofs):
                current_qvel[0, 0, dof_idx] = target_qvel_np[i]

            self.view.set_dof_velocities(self.state, current_qvel)


    def _control_dofs_eff(self, target_qf, motor_dofs=None):
        if motor_dofs is None:
            motor_dofs = self.motor_dofs
        if target_qf and motor_dofs:
            current_qf = self.view.get_dof_forces(self.model).numpy()
            target_qf_np = np.array(target_qf)

            for i, dof_idx in enumerate(motor_dofs):
                current_qf[0, 0, dof_idx] = target_qf_np[i]

            self.view.set_dof_forces(self.state, current_qf)

    def setup_joint_commands_subscriber(self):
        """Initialize the joint command (direct state) subscriber."""
        self.logger.info("joint commands subscriber started")

        def joint_commands_callback(msg):
            # check and set the joint dof physics properties such as kp.kd, armanture, damping etc
            if self.view is None:
                self.logger.warning("The build method has not been called!, skipping joint command subscriber callback")
                return
            
            if self.view is not None and not self.dof_properties_set:
                self.set_dofs_properties()
                self.dof_properties_set = True
                
            motor_dofs = get_dofs_idx(self.joint_names,msg.name)
            dof_idx_table = {}
            for k, motor_dof in enumerate(motor_dofs):
                dof_idx_table[msg.name[k]] = motor_dof

            valid = True
            joint_properties = dict(
                sorted(self.robot_config.get("joint_properties", None).items())
            )
            pos_i, vel_i, eff_i = 0, 0, 0
            pos_vals, pos_dofs = [], []
            vel_vals, vel_dofs = [], []
            eff_vals, eff_dofs = [], []
            for joint, joint_cfg in joint_properties.items():
                if joint_cfg.get("command", "").lower() == "position":
                    pos_vals.append(msg.position[pos_i])
                    pos_dofs.append(dof_idx_table[joint])
                    pos_i += 1
                elif joint_cfg.get("command", "").lower() == "velocity":
                    vel_vals.append(msg.velocity[vel_i])
                    vel_dofs.append(dof_idx_table[joint])
                    vel_i += 1
                elif joint_cfg.get("command", "").lower() == "effort":
                    eff_vals.append(msg.effort[eff_i])
                    eff_dofs.append(dof_idx_table[joint])
                    eff_i += 1
                else:
                    self.logger.error(f"Invalid joint command type{joint_cfg.get('command', '')}")
                    valid = False
            if valid:
                self._control_dofs_pos(pos_vals, pos_dofs)
                self._control_dofs_vel(vel_vals, vel_dofs)
                self._control_dofs_pos(eff_vals, eff_dofs)

        joint_commands_subscriber = self.ros_node.create_subscription(
            JointState,
            f'{self.namespace}/{self.robot_config.get("joint_commands_topic", "joint_commands")}',
            joint_commands_callback,
            int(self.robot_config.get("joint_commands_topic_frequency", 50)),
        )
        setattr(
            self, f"{self.robot_name}_joint_commands_subscriber", joint_commands_subscriber
        )
    
    def log(self):
        return
