import logging

from rclpy.node import Node
from .sensors.sensor_utils import make_cv2_bridge
from .sensors import (
    CameraSensor,
    SectionalLidarSensor,
    LidarSensor,
    LaserScanSensor,
    ImuSensor,
    ContactForceSensor,
    ContactSensor,
)
import newton


class NewtonRosSensors:
    """Factory class to manage and instantiate various ROS 2 sensors for a robot."""

    def __init__(
        self,
        builder,
        state,
        namespace,
        robot_name=None,
        entities_info=None,
    ):
        """Initialize the sensor factory with simulation and robot context."""
        self.logger = logging.getLogger(__name__)
        self.logger.info("Starting all sensor data publishers")

        self.builder=builder
        self.state =state
        self.model = None
        self.view=None
        self.bridge = make_cv2_bridge()
        self.namespace = namespace
        self.entities_info = entities_info
        self.robot_name = robot_name
        self.sensors = {
            "CAM": {},
            "SECLIDAR": {},
            "LIDAR": {},
            "IMU": {},
            "CONTACT_FORCE": {},
            "CONTACT": {},
        }
        self.all_ros_nodes = []
        self.is_built = False

    def add_sensor(self, sensor_config):
        """Instantiate a specific sensor based on configuration and register its publishers."""
        sensor_name = sensor_config.get("name")
        if sensor_name is None:
            raise ValueError("Sensor name not specified, sensor options invalid")

        sensor_type = sensor_config.get("sensor_type")
        if sensor_type is None:
            raise ValueError("Sensor type not specified, sensor options invalid")

        sensor_mapping = {
            "cam": ("CAM_NODE", CameraSensor),
            "sectional_lidar": ("SECTIONAL_LIDAR_NODE", SectionalLidarSensor),
            "lidar": ("LIDAR_NODE", LidarSensor),
            "laser_scan": ("LASER_SCAN_NODE", LaserScanSensor),
            "imu": ("IMU_NODE", ImuSensor),
            "contact_force": ("CONTACT_FORCE_NODE", ContactForceSensor),
            "contact": ("CONTACT_NODE", ContactSensor),
        }

        if sensor_type not in sensor_mapping:
            self.logger.error(f"Unknown sensor type: {sensor_type}")
            return None

        node_prefix, sensor_class = sensor_mapping[sensor_type]
        node = Node(f"{node_prefix}_{self.namespace}_{sensor_name}")
        self.all_ros_nodes.append(node)

        sensor_instance = sensor_class(
            sensor_config=sensor_config,
            node=node,
            builder=self.builder,
            namespace=self.namespace,
            entities_info=self.entities_info,
            robot_name=self.robot_name,
        )

        _, _ = sensor_instance.add_sensor()

        # map type
        if sensor_type == "cam":
            sensor_key = "CAM"
        elif sensor_type in ["grid_lidar", "sectional_lidar"]:
            sensor_key = "SECLIDAR"
        elif sensor_type in ["lidar", "laser_scan"]:
            sensor_key = "LIDAR"
        elif sensor_type == "imu":
            sensor_key = "IMU"
        elif sensor_type == "contact_force":
            sensor_key = "CONTACT_FORCE"
        elif sensor_type == "contact":
            sensor_key = "CONTACT"
        else:
            sensor_key = "UNKNOWN"

        # store INSTANCE (not sensor_object)
        self.sensors[sensor_key][sensor_name] = sensor_instance
            
    def build(self, model, state, viewer):
        if self.is_built:
            self.logger.warning("sensor factory already built, skipping build()")
            return
        self.model=model
        self.state=state
        self.viewer=viewer
        self.view=newton.selection.ArticulationView(self.model, 
                                                    pattern=self.robot_name,
                                                    exclude_joint_types=[newton.JointType.FREE,
                                                                         newton.JointType.FIXED,
                                                                         newton.JointType.DISTANCE])
        for sensor_group in self.sensors.values():
            for sensor in sensor_group.values():
                if hasattr(sensor, "build"):
                    sensor.build(self.model, self.state,self.viewer)
        self.is_built = True
        
    def log(self):
        if not self.is_built:
            self.logger.warning("Must be built before logging")
            return
        for sensor_group in self.sensors.values():
            for sensor in sensor_group.values():
                if hasattr(sensor, "build"):
                    sensor.log()
