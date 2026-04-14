import numpy as np
import warp as wp

from sensor_msgs.msg import PointCloud2
from newton.sensors import SensorRaycast, SensorFrameTransform
from .lidar_impl import Lidar

from .base_sensor import BaseSensor
from ..newton_ros_utils import (
    create_qos_profile,
    get_current_timestamp,
    add_gaussian_noise_vec3_1d,
    transform_points
)
from .sensor_utils import (
    create_site, 
    points_to_pcd_msg, 
    log_points,
    log_rays
)
import math
import newton

class LidarSensor(BaseSensor):
    """Newton-based 3D raycaster publishing PointCloud2."""

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

        self.site = None
        self.lidar = None
        self.tf_sensor = None
        self.points=None
        self.log_points=None
        self.rays_starts=None
        self.rays_ends=None
        self.seed=wp.int32(42)
        self.log_steps=0

    # -------------------------------------------------------
    # Build
    # -------------------------------------------------------
    def build(self, model, state, viewer):
        """Explicitly build the frame transformer sensor."""
        if self.is_built:
            self.logger.warning(f"[{self.sensor_name}] Already built, skipping build()")
            return
        
        self.model=model
        self.state=state
        self.viewer=viewer

        if self.viewer is not None and \
            isinstance(self.viewer, newton.viewer.ViewerGL):
            self.is_viewer_valid=True
        else:
            self.is_viewer_valid=False

        if self.site is None:
            self.node.get_logger().error(
                f"[{self.sensor_name}] Cannot build: site not initialized"
            )
            return

        self.tf_sensor = SensorFrameTransform(
            self.model,
            shapes=f"{self.sensor_name}_lidar",
            reference_sites="world_origin",
        )

        self.lidar = Lidar(
            model=self.model,
            hfov=np.radians(self.hfov),
            vfov=np.radians(self.vfov),
            hres=self.res[0],
            vres=self.res[1],
            min_range=self.min_range,
            max_range=self.max_range,
        )

        self.sensor_object = self.lidar

        # register AFTER creation
        self.register_sensor(self.sensor_object, self.sensor_publishers)

        self.is_built = True

        self.node.get_logger().info(
            f"[{self.sensor_name}] Lidar sensor built"
        )

    # -------------------------------------------------------
    def add_sensor(self):
        frame_id = self.ros_options.get("frame_id", "")
        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        self.hfov=self.sensor_config.get("hfov",360)
        self.vfov=self.sensor_config.get("vfov",90)
        self.res=self.sensor_config.get("res", [128,32])
        self.min_range = self.sensor_config.get("min_range", 0.05)
        self.max_range = self.sensor_config.get("max_range", 100.0)
        self.attachment = self.sensor_config.get("attachment", {})
        self.pos_offset = self.attachment.get("pos_offset", (0.0, 0.0, 0.0))
        self.euler_offset = self.attachment.get("euler_offset", (0.0, 0.0, 0.0))
        
        self.add_noise = self.sensor_config.get("add_noise", False)
        self.noise_mean = wp.float32(self.sensor_config.get("noise_mean", 0.0))
        self.noise_std = wp.float32(self.sensor_config.get("noise_std", 0.0))

        self.log_points=self.sensor_config.get("log_points", False)
        self.log_after_n_steps=self.sensor_config.get("log_after_n_steps", 5)
        self.log_point_color=self.sensor_config.get("log_point_color", (1.0,0.0,0.0))
        self.log_point_radius=self.sensor_config.get("log_point_radius", 0.005)
        self.log_rays=self.sensor_config.get("log_rays", False)
        self.log_ray_color=self.sensor_config.get("log_ray_color", (1.0,0.0,0.0))
        self.log_ray_width=self.sensor_config.get("log_ray_width", 0.005)

        # create tf_offset
        euler_offset_rad_vec = wp.vec3f(
            np.radians(
                np.array(self.euler_offset, dtype=float)
            )
        )
        q_offset = wp.quat_from_euler(euler_offset_rad_vec,0,1,2)
        pos_offset_vec = wp.vec3(
            self.pos_offset
        )
        self.offset_tf=wp.transform(pos_offset_vec, q_offset)
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
            pos_offset=(0, 0, 0),
            euler_offset=self.attachment.get("euler_offset", (0, 0, 0)),
            label=f"{self.sensor_name}_lidar",
        )

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
            PointCloud2,
            f"{self.namespace}/{topic}",
            qos_profile,
        )

        self.sensor_publishers = [pub]

        # -------------------------
        # Timer callback
        # -------------------------
        def timer_callback():
            if not self.is_built:
                self.node.get_logger().warn(
                    f"[{self.sensor_name}] Sensor not built yet, skipping publish"
                )
                return
            # --- Pose ---
            self.tf_sensor.update(self.state)

            # Get transform from sensor (convert numpy → unpack into Warp transform)
            sensor_tf_np = self.tf_sensor.transforms.numpy()[0]
            self.tf = wp.transformf(*sensor_tf_np)

            # Transform sensor tf frame by offset
            self.tf = wp.transform_multiply(self.tf,self.offset_tf)

            # --- Raycast / LiDAR update ---
            self.lidar.update(self.state, self.tf)
            # Get point cloud
            self.points=self.lidar.get_points()

            if self.add_noise:
                wp.launch(
                    add_gaussian_noise_vec3_1d,
                    dim=self.points.size,
                    inputs=[self.points, self.noise_mean, self.noise_std, self.seed],
                )

            #calculate rays only if required to visualize
            if self.is_viewer_valid and self.log_rays:
                self.rays_starts, self.rays_ends=self.lidar.get_ray_visualization_data(self.tf)

            # --- Publish ---
            msg = points_to_pcd_msg(
                self.points.numpy(),
                stamp=get_current_timestamp(),
                frame_id=frame_id,
            )
            pub.publish(msg)
        # -------------------------
        # Timer
        # -------------------------
        timer = self.node.create_timer(1 / frequency, timer_callback)
        setattr(self, f"{self.sensor_name}_sectional_lidar_timer", timer)

        return None, self.sensor_publishers
    
    def log(self):
        if (
            self.viewer is None
            or not self.is_built
            or not self.is_viewer_valid
            or not self.viewer.is_running()
        ): return

        if self.log_points and self.log_steps==self.log_after_n_steps:
            if self.points is None: return

            self.log_points=wp.empty_like(self.points)
            wp.launch(
                transform_points,
                dim=self.points.size,
                inputs=[self.points, self.log_points, self.tf],
                device="cuda",
            )

            log_points(
                self.viewer,
                f"{self.sensor_name}/points",
                self.log_points,
                self.log_point_color,
                self.log_point_radius
            )

        if self.log_rays and self.log_steps==self.log_after_n_steps:
            if self.rays_starts is None or self.rays_ends is None: return
            log_rays(
                self.viewer,
                f"{self.sensor_name}/rays",
                self.rays_starts,
                self.rays_ends,
                self.log_ray_color,
                self.log_ray_width
            )

        if self.log_steps==self.log_after_n_steps:
            self.log_steps=0
        else:
            self.log_steps+=1
    
