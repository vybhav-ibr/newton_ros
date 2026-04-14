"""Handler for entity metadata services"""

from simulation_interfaces.msg import Result
from newton_ros_interfaces.msg import RobotOptions, JointProperty



class EntityOptionsHandler:
    """Handles GetEntityOptions and SetEntityOptions services"""

    RESULT_OK = 1
    RESULT_NOT_FOUND = 2

    def __init__(self, node, scene_manager):
        """Initialize the entity options handler."""
        self.node = node
        self.scene_manager = scene_manager
        self.logger = node.get_logger()

    def _populate_robot_options(self, config_dict):
        """Helper to populate Robot Options message from config dict"""
        msg = RobotOptions()
        msg.joint_states_topic = str(config_dict.get("joint_states_topic", ""))
        msg.joint_states_topic_frequency = float(
            config_dict.get("joint_states_topic_frequency", 0.0)
        )
        msg.joint_commands_topic = str(config_dict.get("joint_commands_topic", ""))
        msg.joint_commands_topic_frequency = float(
            config_dict.get("joint_commands_topic_frequency", 0.0)
        )
        msg.joints_control_topic = str(config_dict.get("joints_control_topic", ""))
        msg.joints_control_topic_frequency = float(
            config_dict.get("joints_control_topic_frequency", 0.0)
        )

        joint_properties = config_dict.get("joint_properties", {})
        for j_name, j_cfg in joint_properties.items():
            jp = JointProperty()
            jp.name = str(j_name)
            jp.kp = float(j_cfg.get("kp", 0.0))
            jp.kv = float(j_cfg.get("kv", 0.0))
            jp.stiffness = float(j_cfg.get("stiffness", 0.0))
            jp.armature = float(j_cfg.get("armature", 0.0))
            jp.damping = float(j_cfg.get("damping", 0.0))
            jp.force_range = [float(x) for x in j_cfg.get("force_range", [])]
            jp.command = str(j_cfg.get("command", ""))
            msg.joint_properties.append(jp)

        return msg

    def get_robot_options_callback(self, request, response):
        """Retrieve robot-specific control and topic configurations."""
        """Get robot options message"""
        self.logger.info(f"GetRobotOptions service called: {request.entity}")

        response.result = Result()

        if request.entity not in self.scene_manager.entities_info:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response

        entity_entry = self.scene_manager.entities_info[request.entity]

        # Access robot_config
        config = entity_entry.get("robot_options")

        if config is None:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = (
                f"No robot config found for entity '{request.entity}'"
            )
            return response

        try:
            response.options = self._populate_robot_options(config)
            response.result.result = self.RESULT_OK
        except Exception as e:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Failed to serialize options: {str(e)}"
            self.logger.error(f"Error getting robot config: {e}")

        return response

    def _convert_robot_options(self, msg):
        """Helper to convert RobotOptions message to dict"""
        d = {}
        if msg.joint_states_topic:
            d["joint_states_topic"] = msg.joint_states_topic
        if msg.joint_states_topic_frequency:
            d["joint_states_topic_frequency"] = msg.joint_states_topic_frequency
        if msg.joint_commands_topic:
            d["joint_commands_topic"] = msg.joint_commands_topic
        if msg.joint_commands_topic_frequency:
            d["joint_commands_topic_frequency"] = msg.joint_commands_topic_frequency
        if msg.joints_control_topic:
            d["joints_control_topic"] = msg.joints_control_topic
        if msg.joints_control_topic_frequency:
            d["joints_control_topic_frequency"] = msg.joints_control_topic_frequency

        if msg.joint_properties:
            d["joint_properties"] = {}
            for jp in msg.joint_properties:
                jd = {}
                if jp.kp:
                    jd["kp"] = jp.kp
                if jp.kv:
                    jd["kv"] = jp.kv
                if jp.stiffness:
                    jd["stiffness"] = jp.stiffness
                if jp.armature:
                    jd["armature"] = jp.armature
                if jp.damping:
                    jd["damping"] = jp.damping
                if jp.force_range:
                    jd["force_range"] = list(jp.force_range)
                if jp.command:
                    jd["command"] = jp.command
                d["joint_properties"][jp.name] = jd
        return d

    def set_robot_options_callback(self, request, response):
        """Configure robot-specific behavior, topics, and joint properties."""
        """Set robot options from message"""
        self.logger.info(f"SetRobotOptions service called: {request.entity}")

        response.result = Result()

        if request.entity not in self.scene_manager.entities_info.keys():
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found, entity options can only be set entities which are already spawned"
            return response
        try:
            new_config = self._convert_robot_options(request.options)
            entity_entry = self.scene_manager.entities_info[request.entity]

            # Update or Set robot_config
            if entity_entry.get("robot_options") is not None:
                response.result.result = self.RESULT_NOT_FOUND
                response.result.error_message = f"Entity '{request.entity}' alraedy has entity config, entity options can only be set only once"
                return response
            else:
                entity_entry["robot_options"] = new_config
                entity_entry["initialisation_pending"] = True
            response.result.result = self.RESULT_OK

        except Exception as e:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Failed to update options: {str(e)}"
            self.logger.error(f"Error setting robot options: {e}")

        return response
