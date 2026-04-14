import numpy as np
from newton_ros.sensors.sensor_utils import create_site
import warp as wp
from newton.sensors import SensorContact

from geometry_msgs.msg import Vector3
from newton_ros_interfaces.msg import ContactForce, Contact  # <-- new msg

from .base_sensor import BaseSensor
from ..newton_ros_utils import create_qos_profile


class ContactSensor(BaseSensor):
    """ROS 2 sensor to publish contact force + contact state."""

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

        self.contact_sensor = None
        self.site = None

    def build(self, model, state):
        """Explicitly build the contact sensor."""
        if self.is_built:
            self.logger.warning(f"[{self.sensor_name}] Already built, skipping build()")
            return
        
        self.model=model
        self.state=state

        self.contact_sensor = SensorContact(
            self.model,
            sensing_sites=[self.site],
        )

        self.is_built = True
        self.node.get_logger().info(
            f"[{self.sensor_name}] Contact sensor built"
        )

    def add_sensor(self):
        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")
        contact_topic = self.ros_options.get("contact_topic", f"{topic}_state")

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
            label=f"{self.sensor_name}_contact",
        )

        # -------------------------
        # Timer callback
        # -------------------------
        def timer_callback(force_pub, contact_pub, link_name):

            if not self.is_built:
                self.node.get_logger().warn(
                    f"[{self.sensor_name}] Sensor not built yet, skipping publish"
                )
                return

            sensor = self.contact_sensor
            sensor.update(self.state)

            force = sensor.forces.numpy()[0]

            # -------------------------
            # Force message
            # -------------------------
            force_msg = ContactForce()
            force_msg.contact_force = Vector3(
                x=force[0], y=force[1], z=force[2]
            )
            force_msg.link_name = link_name
            force_pub.publish(force_msg)

            # -------------------------
            # Contact boolean
            # -------------------------
            threshold = self.sensor_config.get("contact_threshold", 1e-3)
            in_contact = (force[0]**2 + force[1]**2 + force[2]**2) > (threshold ** 2)

            contact_msg = Contact()
            contact_msg.link_name = link_name
            contact_msg.in_contact = bool(in_contact)

            contact_pub.publish(contact_msg)

        # -------------------------
        # Publishers
        # -------------------------
        qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )

        force_pub = self.node.create_publisher(
            ContactForce,
            f"{self.namespace}/{topic}",
            qos_profile,
        )

        contact_pub = self.node.create_publisher(
            Contact,
            f"{self.namespace}/{contact_topic}",
            qos_profile,
        )

        self.sensor_publishers = [force_pub, contact_pub]

        # -------------------------
        # Timer
        # -------------------------
        timer = self.node.create_timer(
            1 / frequency,
            lambda: timer_callback(force_pub, contact_pub, link_name),
        )

        setattr(self, f"{self.sensor_name}_contact_timer", timer)

        self.register_sensor(self.contact_sensor, self.sensor_publishers)

        return self.contact_sensor, self.sensor_publishers

    def log(self):
        return