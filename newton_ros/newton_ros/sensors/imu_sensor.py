import numpy as np
from newton_ros.sensors.sensor_utils import create_site
import warp as wp
from newton.sensors import SensorIMU, SensorFrameTransform

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

from .base_sensor import BaseSensor
from ..newton_ros_utils import (
    create_qos_profile,
    get_current_timestamp,
)
import newton


class ImuSensor(BaseSensor):
    """ROS 2 IMU sensor using Newton backend."""

    def __init__(
        self,
        sensor_config,
        node,
        builder,
        namespace,
        entities_info=None,
        robot_name=None,
    ):
        super().__init__(
            sensor_config,
            node,
            builder,
            namespace,
            entities_info,
            robot_name,
        )

        self.imu = None
        self.site = None

    # -------------------------------------------------------
    # Build
    # -------------------------------------------------------
    def build(self, model, state, viewer):
        """Explicitly build the imu sensor."""
        if self.is_built:
            self.logger.warning(f"[{self.sensor_name}] Already built, skipping build()")
            return
        
        self.model=model
        self.state=state
        self.viewer=viewer
        self.viewer_valid=isinstance(self.viewer, newton.viewer.ViewerGL)
        if self.site is None:
            self.node.get_logger().error(
                f"[{self.sensor_name}] Cannot build: site not initialized"
            )
            return

        self.imu = SensorIMU(
            self.model,
            sites=[self.site],
        )
                # -------------------------
        # TF sensor
        # -------------------------
        self.tf_sensor = SensorFrameTransform(
            self.model,
            shapes=f"{self.sensor_name}_imu",
            reference_sites="world_origin",
        )

        self.is_built = True

        # register AFTER creation (fixes None registration issue)
        self.register_sensor(self.imu, self.sensor_publishers)

        self.node.get_logger().info(
            f"[{self.sensor_name}] IMU sensor built"
        )

    # -------------------------------------------------------
    # Main
    # -------------------------------------------------------
    def add_sensor(self):
        """Create site + ROS publisher + timer."""

        frame_id = self.ros_options.get("frame_id", "")
        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")
        self.attachment = self.sensor_config.get("attachment", {})

        # -------------------------
        # Resolve body
        # -------------------------
        link_name = self.attachment.get("link")
        body = self.builder.body_label.index(f"{self.robot_name}/{link_name}")
        if body is None:
            raise ValueError(f"Link '{link_name}' not found")

        # -------------------------
        # Site
        # -------------------------
        self.site = create_site(
            self.builder,
            body=body,
            pos_offset=self.rigid_options.get("pos_offset", (0, 0, 0)),
            euler_offset=self.rigid_options.get("euler_offset", (0, 0, 0)),
            label=f"{self.sensor_name}_imu",
        )

        # -------------------------
        # Publisher
        # -------------------------
        imu_qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )

        imu_pub = self.node.create_publisher(
            Imu, f"{self.namespace}/{topic}", imu_qos_profile
        )

        self.sensor_publishers = [imu_pub]

        # -------------------------
        # Timer callback
        # -------------------------
        def timer_callback():

            if not self.is_built:
                self.node.get_logger().warn(
                    f"[{self.sensor_name}] Sensor not built yet, skipping publish"
                )
                return

            if self.viewer is not None:
                if not self.viewer.is_running():
                    return

            # --- Pose ---
            self.tf_sensor.update(self.state)
            sensor_tf_np = self.tf_sensor.transforms.numpy()[0]
            self.tf = wp.transformf(*sensor_tf_np)

            self.imu.update(self.state)

            acc = self.imu.accelerometer.numpy()[0]
            gyro = self.imu.gyroscope.numpy()[0]

            imu_msg = Imu()
            imu_msg.header.frame_id = frame_id
            imu_msg.header.stamp = get_current_timestamp()

            # -------------------------
            # Orientation
            # -------------------------
            imu_msg.orientation = self.tf.q

            imu_msg.angular_velocity = Vector3(
                x=gyro[0], y=gyro[1], z=gyro[2]
            )

            imu_msg.linear_acceleration = Vector3(
                x=acc[0], y=acc[1], z=acc[2]
            )

            imu_pub.publish(imu_msg)

        # -------------------------
        # Timer
        # -------------------------
        timer = self.node.create_timer(
            1 / frequency,
            timer_callback,
        )

        setattr(self, f"{self.sensor_name}_imu_timer", timer)

        return self.imu, self.sensor_publishers

    def log(self):
        return