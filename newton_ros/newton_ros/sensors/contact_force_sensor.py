import numpy as np
from newton_ros.sensors.sensor_utils import create_site
import warp as wp
from newton.sensors import SensorContact

from geometry_msgs.msg import Vector3
from newton_ros_interfaces.msg import ContactForce

from .base_sensor import BaseSensor
from ..newton_ros_utils import create_qos_profile


class ContactForceSensor(BaseSensor):
    """ROS 2 sensor to measure and publish contact forces using Newton."""

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

        self.contact_force_sensor = None
        self.site = None

    def build(self, model, state):
        """Explicitly build the contact force sensor."""
        if self.is_built:
            self.logger.warning(f"[{self.sensor_name}] Already built, skipping build()")
            return
        
        self.model=model
        self.state=state
        
        if self.site is None:
            self.node.get_logger().error(
                f"[{self.sensor_name}] Cannot build: site not initialized"
            )
            return

        self.contact_force_sensor = SensorContact(
            self.model,
            sensing_sites=[self.site],
        )

        self.is_built = True
        self.node.get_logger().info(
            f"[{self.sensor_name}] Contact force sensor built"
        )

    def add_sensor(self):
        """Create site (pre-finalize) + ROS publisher + timer."""

        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        # -------------------------
        # Resolve body
        # -------------------------
        link_name = self.rigid_options.get("link")
        body = self.robot.get_body(link_name)
        if body is None:
            raise ValueError(f"Link '{link_name}' not found")

        # -------------------------
        # Site
        # -------------------------
        self.site = create_site(
            pos_offset=self.rigid_options.get("pos_offset", (0, 0, 0)),
            euler_offset=self.rigid_options.get("euler_offset", (0, 0, 0)),
            label=f"{self.sensor_name}_contact_force",
        )

        # -------------------------
        # Timer callback
        # -------------------------
        def timer_callback(contact_force_pub, link_name):

            if not self.is_built:
                self.node.get_logger().warn(
                    f"[{self.sensor_name}] Sensor not built yet, skipping publish"
                )
                return

            sensor = self.contact_force_sensor
            sensor.update(self.state)

            force = sensor.forces.numpy()[0]

            contact_msg = ContactForce()
            contact_msg.contact_force = Vector3(
                x=force[0], y=force[1], z=force[2]
            )
            contact_msg.link_name = link_name

            contact_force_pub.publish(contact_msg)

        # -------------------------
        # Publisher
        # -------------------------
        qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )

        contact_force_pub = self.node.create_publisher(
            ContactForce,
            f"{self.namespace}/{topic}",
            qos_profile,
        )

        self.sensor_publishers = [contact_force_pub]

        # -------------------------
        # Timer
        # -------------------------
        timer = self.node.create_timer(
            1 / frequency,
            lambda: timer_callback(contact_force_pub, link_name),
        )

        setattr(self, f"{self.sensor_name}_contact_timer", timer)

        self.register_sensor(self.contact_force_sensor, self.sensor_publishers)

        return self.contact_force_sensor, self.sensor_publishers