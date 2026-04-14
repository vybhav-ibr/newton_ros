
import logging

from rclpy.node import Node
from .sensor_utils import make_cv2_bridge
from rclpy.callback_groups import ReentrantCallbackGroup


class BaseSensor:
    """Base class providing common properties and registration logic for all simulation sensors."""

    def __init__(
        self,
        sensor_config,
        node,
        builder,
        namespace,
        entities_info=None,
        robot_name=None,
    ):
        """Initialize the base sensor with common configuration and simulation state."""
        self.logger = logging.getLogger(__name__)
        self.sensor_config = sensor_config
        self.node = node
        self.builder = builder
        self.namespace = namespace
        self.entities_info = entities_info
        self.robot_name = robot_name
        self.bridge = make_cv2_bridge()

        self.general_options = sensor_config.get("general_sensor_options", {})
        self.ros_options = sensor_config.get("ros_options", {})
        self.rigid_options = sensor_config.get("rigid_options", {})
        self.sensor_name = sensor_config.get("name")

        self.sensor_object = None
        self.sensor_publishers = []
        self.is_built = False  # Flag to track if the sensor has been built

    def register_sensor(self, sensor_attr, sensor_publishers):
        """Register the sensor's configuration in the entities_info for OSI services."""
        """Register sensor options in entities_info"""
        if self.entities_info is None or self.robot_name is None:
            return

        if self.robot_name not in self.entities_info:
            self.logger.warn(
                f"Robot '{self.robot_name}' not found in entities_info, cannot register sensor."
            )
            return

        entity_entry = self.entities_info[self.robot_name]

        if "sensors" not in entity_entry or not isinstance(
            entity_entry["sensors"], dict
        ):
            entity_entry["sensors"] = {
                "cameras": [],
                "lidars": [],
                "imus": [],
                "contacts": [],
                "contact_forces": [],
            }

        # Determine category based on sensor_type
        sensor_type = self.general_options.get("sensor_type", "")
        category = "others"
        if sensor_type in ["cam", "rgb", "depth", "segmentation", "rgbd"]:
            category = "cameras"
        elif sensor_type in [
            "3d_lidar",
            "grid_lidar",
            "sectional_lidar",
            "laser_scan",
            "lidar",
        ]:
            category = "lidars"
        elif sensor_type == "imu":
            category = "imus"
        elif sensor_type == "contact":
            category = "contacts"
        elif sensor_type == "contact_force":
            category = "contact_forces"

        if category not in entity_entry["sensors"]:
            entity_entry["sensors"][category] = []

        # Check if sensor already exists to avoid duplicates
        existing_sensors = entity_entry["sensors"][category]
        for i, sensor in enumerate(existing_sensors):
            if sensor.get("name") == self.sensor_name:
                existing_sensors[i] = self.sensor_config
                return
        self.sensor_config["sensor_attr"] = sensor_attr
        self.sensor_config["publishers"] = sensor_publishers
        existing_sensors.append(self.sensor_config)

    def add_sensor(self):
        """Virtual method to be overriden for creating the specific sensor object and publishers."""
        """To be implemented by subclasses"""
        raise NotImplementedError
    
    def build(self, model, state):
        """Virtual method to be overridden for building the sensor object based on the model and state."""
        """To be implemented by subclasses"""
        raise NotImplementedError
    
    def log(self):
        """Virtual method to be overridden for building the sensor object based on the model and state."""
        """To be implemented by subclasses"""
        raise NotImplementedError
