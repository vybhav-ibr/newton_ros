import numpy as np
import warp as wp

from sensor_msgs.msg import PointCloud2
from newton.sensors import SensorRaycast,SensorFrameTransform

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
    compute_ray_endpoints,
    depth_to_pointcloud,
    log_points,
    log_rays
)
import math
import newton

class SectionalLidarSensor(BaseSensor):
    """Depth-camera style raycast sensor publishing PointCloud2."""

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
        self.tf=None
        self.tf_sensor=None
        self.device=None
        self.points=None
        self.log_points=None
        self.rays_starts=None
        self.rays_ends=None
        self.valid=None
        self.seed=wp.int32(42)

    def build(self, model, state, viewer):
        if self.is_built:
            self.logger.warning(f"[{self.sensor_name}] Already built, skipping build()")
            return

        self.model = model
        self.state = state
        self.viewer = viewer
        if self.viewer is not None and \
            isinstance(self.viewer, newton.viewer.ViewerGL):
            self.is_viewer_valid=True
        else:
            self.is_viewer_valid=False
        self.device   = model.device

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

        # Initialize Raycast Sensor with the parent_site
        # This allows the sensor to handle transformations automatically
        self.lidar = SensorRaycast(
            model=self.model,
            camera_position=(0.0, 0.0, 0.0),   # world-space origin  
            camera_direction=(1.0, 0.0, 0.0),  # looking along +X  
            camera_up=(0.0, 0.0, 1.0),         # +Z is up  
            fov_radians=math.radians(self.fov),           # 60° vertical FOV  
            width=self.res[0],  
            height=self.res[1],  
            max_distance=self.max_range,  
        )

        self.sensor_object = self.lidar
        self.register_sensor(self.sensor_object, self.sensor_publishers)
        self.is_built = True

        self.node.get_logger().info(f"[{self.sensor_name}] Sectional lidar built")

    def add_sensor(self):
        frame_id = self.ros_options.get("frame_id", "")
        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        self.attachment = self.sensor_config.get("attachment", {})
        self.pos_offset = self.attachment.get("pos_offset", (0.0, 0.0, 0.0))
        self.euler_offset = self.attachment.get("euler_offset", (0.0, 0.0, 0.0))
        self.fov=self.sensor_config.get("fov", 60.0)
        self.res=self.sensor_config.get("res", (640, 480))
        self.min_range = self.sensor_config.get("min_range", 0.05)
        self.max_range = self.sensor_config.get("max_range", 100.0)
        
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

        link_name = self.attachment.get("link")
        try:
            body = self.builder.body_label.index(f"{self.robot_name}/{link_name}")
        except ValueError:
            raise ValueError(f"Link '{link_name}' not found for robot '{self.robot_name}'")

        # -------------------------
        # Site
        # -------------------------
        self.site = create_site(
            self.builder,
            body=body,
            pos_offset=(0, 0, 0),
            euler_offset=(0, 0, 0),
            label=f"{self.sensor_name}_lidar",
        )

        # Publisher setup
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

        self.points = wp.zeros((self.res[1], self.res[0]), dtype=wp.vec3,  device=self.device)  
        self.valid  = wp.zeros((self.res[1], self.res[0]), dtype=wp.int32, device=self.device)
        def timer_callback():
            if not self.is_built:
                return

            self.tf_sensor.update(self.state)

            # Get transform from sensor (convert numpy → unpack into Warp transform)
            sensor_tf_np = self.tf_sensor.transforms.numpy()[0]
            self.tf = wp.transformf(*sensor_tf_np)

            # Transform sensor tf frame by offset
            self.tf = wp.transform_multiply(self.tf,self.offset_tf)

            direction_vec=wp.transform_vector(self.tf, wp.vec3f((1.0, 0.0, 0.0)))
            up_vec=wp.transform_vector(self.tf, wp.vec3f((0.0, 0.0, 1.0)))
            self.lidar.update_camera_pose(  
                position=self.tf.p,  
                direction=direction_vec,   # still looking along +X  
                up=up_vec,  
            )

            self.lidar.update(self.state)
            wp.launch(  
                depth_to_pointcloud,  
                dim=(self.res[0], self.res[1]),                            # (columns, rows)  
                inputs=[  
                    self.lidar.depth_image,    
                    self.lidar.fov_scale,  
                    self.lidar.aspect_ratio,  
                    self.res[0], self.res[1],
                    self.min_range, self.max_range,
                    wp.vec3(*self.lidar.camera_direction),  
                    wp.vec3(*self.lidar.camera_up),      
                ],  
                outputs=[self.points, self.valid],  
                device=self.device,  
            )

            if self.add_noise:
                wp.launch(
                    add_gaussian_noise_vec3_1d,
                    dim=self.points.size,
                    inputs=[self.points, self.noise_mean, self.noise_std, self.seed],
                )

            # Publish
            msg = points_to_pcd_msg(
                self.points.numpy(),
                stamp=get_current_timestamp(),
                frame_id=frame_id,
            )

            self.rays_starts, self.rays_ends = self.get_ray_endpoints()
            pub.publish(msg)

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
                self.points,
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
    
    def get_ray_endpoints(self) -> tuple[wp.array, wp.array]:
        if self.lidar is None: return None, None
        sensor=self.lidar 
        H, W = self.res[1], self.res[0]  
        device = self.device  
    
        ray_starts = wp.zeros((H, W), dtype=wp.vec3, device=device)  
        ray_ends   = wp.zeros((H, W), dtype=wp.vec3, device=device)  
    
        wp.launch(  
            compute_ray_endpoints,  
            dim=(W, H),  
            inputs=[  
                wp.vec3(*sensor.camera_position),  
                wp.vec3(*sensor.camera_direction),  
                wp.vec3(*sensor.camera_up),  
                wp.vec3(*sensor.camera_right),  
                float(sensor.fov_scale),  
                float(sensor.aspect_ratio),  
                W, H,  
                sensor.depth_image,  
                self.max_range,  
                ray_starts, ray_ends,  
            ],  
            device=device,  
        )  
    
        return ray_starts, ray_ends