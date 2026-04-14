import numpy as np
from newton_ros.sensors.sensor_utils import create_site
import warp as wp

from newton.sensors import SensorFrameTransform

from geometry_msgs.msg import TransformStamped

from .base_sensor import BaseSensor
from ..newton_ros_utils import create_qos_profile


class FrameTransformer(BaseSensor):
    """ROS 2 sensor to publish frame transforms using Newton."""

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

        self.sensor_object = None
        self.ref_site = None
        self.target_site = None

    # -------------------------------------------------------
    # Build
    # -------------------------------------------------------
    def build(self, model, state):
        """Explicitly build the frame transformer sensor."""
        if self.is_built:
            self.logger.warning(f"[{self.sensor_name}] Already built, skipping build()")
            return
        
        self.model=model
        self.state=state
        
        self.sensor_object = SensorFrameTransform(
            self.model,
            shapes=[self.target_site],          # object
            reference_sites=[self.ref_site],    # reference
        )

        self.is_built = True

        self.node.get_logger().info(
            f"[{self.sensor_name}] Frame transformer built"
        )

    # -------------------------------------------------------
    # Main
    # -------------------------------------------------------
    def add_sensor(self):
        frequency = self.ros_options.get("frequency", 10.0)
        topic = self.ros_options.get("topic")

        # -------------------------
        # Resolve links
        # -------------------------
        ref_link = self.rigid_options.get("reference_link")
        target_link = self.rigid_options.get("target_link")

        ref_body = self.robot.get_body(ref_link)
        target_body = self.robot.get_body(target_link)

        if ref_body is None:
            raise ValueError(f"Reference link '{ref_link}' not found")
        if target_body is None:
            raise ValueError(f"Target link '{target_link}' not found")

        # -------------------------
        # Offsets
        # -------------------------
        ref_pos = self.rigid_options.get("reference_pos_offset", (0, 0, 0))
        ref_euler = self.rigid_options.get("reference_euler_offset", (0, 0, 0))

        tgt_pos = self.rigid_options.get("target_pos_offset", (0, 0, 0))
        tgt_euler = self.rigid_options.get("target_euler_offset", (0, 0, 0))

        # -------------------------
        # Create sites
        # -------------------------
        self.ref_site = create_site(
            ref_body,
            ref_pos,
            ref_euler,
            f"{self.sensor_name}_ref",
        )

        self.target_site = create_site(
            target_body,
            tgt_pos,
            tgt_euler,
            f"{self.sensor_name}_target",
        )

        parent_frame = self.rigid_options.get("parent_frame_id", ref_link)
        child_frame = self.rigid_options.get("child_frame_id", target_link)

        # -------------------------
        # Timer callback
        # -------------------------
        def timer_callback(pub):

            if not self.is_built:
                self.node.get_logger().warn(
                    f"[{self.sensor_name}] Sensor not built yet, skipping publish"
                )
                return

            sensor = self.sensor_object
            sensor.update(self.state)

            tf = sensor.transforms.numpy()[0]

            pos = tf[:3]
            quat = tf[3:]  # (x,y,z,w)

            msg = TransformStamped()

            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = parent_frame
            msg.child_frame_id = child_frame

            msg.transform.translation.x = float(pos[0])
            msg.transform.translation.y = float(pos[1])
            msg.transform.translation.z = float(pos[2])

            msg.transform.rotation.x = float(quat[0])
            msg.transform.rotation.y = float(quat[1])
            msg.transform.rotation.z = float(quat[2])
            msg.transform.rotation.w = float(quat[3])

            pub.publish(msg)

        # -------------------------
        # Publisher
        # -------------------------
        qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )

        pub = self.node.create_publisher(
            TransformStamped,
            f"{self.namespace}/{topic}",
            qos_profile,
        )

        self.sensor_publishers = [pub]

        # -------------------------
        # Timer
        # -------------------------
        timer = self.node.create_timer(
            1.0 / frequency,
            lambda: timer_callback(pub),
        )

        setattr(self, f"{self.sensor_name}_tf_timer", timer)

        self.register_sensor(self.sensor_object, self.sensor_publishers)

        return self.sensor_object, self.sensor_publishers