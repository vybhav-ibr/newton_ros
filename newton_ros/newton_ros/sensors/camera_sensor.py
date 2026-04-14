import warp as wp
import numpy as np
import math
import newton
from newton.sensors import SensorTiledCamera, SensorFrameTransform
from sensor_msgs.msg import Image

from .base_sensor import BaseSensor
from ..newton_ros_utils import(
    create_qos_profile,
    get_current_timestamp,
    add_gaussian_noise_uint32_4d,
    add_gaussian_noise_f32_4d,
    add_gaussian_noise_vec3_4d,
)
    
from .sensor_utils import (
    create_site,
    compute_flips_opengl
)
import time

class CameraSensor(BaseSensor):
    """ROS2 Camera Sensor (explicit build)."""

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
        self.cam = None
        self.tf_sensor = None
        self.tf=None
        self.added=False
        self.seed=wp.int32(42)

    # -------------------------------------------------------
    # Build (replaces lazy init)
    # -------------------------------------------------------
    def build(self, model, state, viewer):
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

        # -------------------------
        # Render config
        # -------------------------
        rcfg = SensorTiledCamera.RenderConfig()
        rcfg.enable_textures = self.enable_textures
        rcfg.enable_particles = self.enable_particles
        rcfg.enable_shadows = self.enable_shadows
        rcfg.enable_ambient_lighting = self.enable_ambient_lighting
        rcfg.enable_backface_culling = self.enable_backface_culling
        rcfg.render_order = self.render_order
        rcfg.tile_width = self.tile_width
        rcfg.tile_height = self.tile_height
        rcfg.max_distance = self.max_range

        # -------------------------
        # Camera
        # -------------------------
        self.cam = SensorTiledCamera(
            model=self.model,
            config=rcfg,
            load_textures=self.load_textures,
        )

        for light in self.lights.values():
            self.cam.utils.create_default_light(  
                enable_shadows=light["enable_shadows"],  
                direction=wp.vec3f(light["direction"]) 
            )

        # -------------------------
        # Buffers
        # -------------------------
        W, H = self.res
        self.color_buffer, self.depth_buffer, self.normal_buffer, self.seg_buffer, self.albedo_buffer = [None] * 5
        if "rgb" in self.camera_types:
            self.color_buffer = self.cam.utils.create_color_image_output(W, H, 1)
        if "depth" in self.camera_types:
            self.depth_buffer = self.cam.utils.create_depth_image_output(W, H, 1)
        if "normal" in self.camera_types:
            self.normal_buffer = self.cam.utils.create_normal_image_output(W, H, 1)
        if "segmentation" in self.camera_types:
            self.seg_buffer = self.cam.utils.create_shape_index_image_output(W, H, 1)
        if "albedo" in self.camera_types:
            self.albedo_buffer = self.cam.utils.create_albedo_image_output(W, H, 1)

        # print(self.color_buffer.dtype)
        # print(self.depth_buffer.dtype)
        # print( self.normal_buffer.dtype)
        # print( self.seg_buffer.dtype)
        # print( self.albedo_buffer.dtype)
        # -------------------------
        # Pre-allocate numpy output buffers (avoids per-frame heap allocs)
        # -------------------------
        self._np_rgb    = np.empty((H, W, 3), dtype=np.uint8)  if "rgb"          in self.camera_types else None
        self._np_depth    = np.empty((H, W, 1), dtype=np.float32)  if "depth"          in self.camera_types else None
        self._np_normal = np.empty((H, W, 3), dtype=np.uint8)  if "normal"       in self.camera_types else None
        self._np_normal_f = np.empty((H, W, 3), dtype=np.float32) if "normal"    in self.camera_types else None
        self._np_seg    = np.empty((H, W),    dtype=np.uint8)  if "segmentation" in self.camera_types else None
        self._np_seg_f  = np.empty((H, W),    dtype=np.float32) if "segmentation" in self.camera_types else None 

        if self.sensor_config.get("assign_checkerboard_material", True):
            self.cam.utils.assign_checkerboard_material_to_all_shapes()

        if self.sensor_config.get("random_color_per_shape", False):
            self.cam.utils.assign_random_colors_per_shape()

        # -------------------------
        # Rays
        # -------------------------
        self.camera_rays = self.cam.utils.compute_pinhole_camera_rays(
            self.res[0], self.res[1], math.radians(self.fov)
        )

        # -------------------------
        # TF sensor
        # -------------------------
        self.tf_sensor = SensorFrameTransform(
            self.model,
            shapes=f"{self.sensor_name}_camera",
            reference_sites="world_origin",
        )

        self.sensor_object = self.cam
        self.register_sensor(self.sensor_object, self.sensor_publishers)

        self.is_built = True
        self.node.get_logger().info(
            f"[{self.sensor_name}] Camera sensor built"
        )

    # -------------------------------------------------------
    # Main
    # -------------------------------------------------------
    def add_sensor(self):
        self._last_pub_time = time.perf_counter()
        frequency = self.ros_options.get("frequency", 1.0)
        self.frame_id = self.ros_options.get("frame_id", "")

        self.res = self.sensor_config.get("res", (320, 320))
        self.fov = self.sensor_config.get("fov", 30.0)
        self.camera_types = self.sensor_config.get("camera_types", ["rgb"])
        self.load_textures = self.sensor_config.get("load_textures", True)

        #render config
        self.render_cfg = self.sensor_config.get("render_cfg", {})
        self.enable_textures = self.render_cfg.get("enable_textures", False)
        self.enable_shadows = self.render_cfg.get("enable_shadows", False)
        self.enable_ambient_lighting = self.render_cfg.get("enable_ambient_lighting", True)
        self.enable_particles = self.render_cfg.get("enable_particles", True)
        self.enable_backface_culling = self.render_cfg.get("enable_backface_culling", True)
        render_order = self.render_cfg.get("render_order", "TILED")
        if render_order=="TILED":
            self.render_order=SensorTiledCamera.RenderOrder.TILED
        elif render_order=="PIXEL_PRIORITY":
            self.render_order=SensorTiledCamera.RenderOrder.PIXEL_PRIORITY
        elif render_order=="VIEW_PRIORITY":
            self.render_order=SensorTiledCamera.RenderOrder.VIEW_PRIORITY
        self.tile_width = self.render_cfg.get("tile_width", 16)
        self.tile_height = self.render_cfg.get("tile_height", 8)
        self.max_range = self.render_cfg.get("max_range", 1000.0)
        
        #attachment cfg
        self.attachment = self.sensor_config.get("attachment", {})
        self.pos_offset = self.attachment.get("pos_offset", (0.0, 0.0, 0.0))
        self.euler_offset = self.attachment.get("euler_offset", (0.0, 0.0, 0.0))

        # light cfg
        self.lights_cfg = self.sensor_config.get("lights", {})
        self.add_light = bool(self.lights_cfg)
        self.lights = {}
        if self.add_light:
            for light_name, light_cfg in self.lights_cfg.items():
                direction = light_cfg.get("direction", [0.0, 0.0, -1.0])
                enable_shadows = light_cfg.get("enable_shadows", False)

                self.lights[light_name] = {
                    "direction": direction,
                    "enable_shadows": enable_shadows,
                }

        #logging and noise addition
        self.add_noise = self.sensor_config.get("add_noise", False)
        self.noise_mean = wp.float32(self.sensor_config.get("noise_mean", 0.0))
        self.noise_std = wp.float32(self.sensor_config.get("noise_std", 0.0))

        # -------------------------
        # create link Site
        # -------------------------
        link_name = self.attachment.get("link")
        body = self.builder.body_label.index(f"{self.robot_name}/{link_name}")
        if body is None:
            raise ValueError(f"Link '{link_name}' not found")
        self.site = create_site(
            self.builder,
            body=body,
            pos_offset=(0, 0, 0),
            euler_offset=(0, 0, 0),
            label=f"{self.sensor_name}_camera",
        )

        # -------------------------
        # Publishers
        # -------------------------
        qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )

        self.sensor_publishers = []
        for cam_type in self.camera_types:
            topic = f"{self.namespace}/{cam_type}"
            pub = self.node.create_publisher(Image, topic, qos_profile)
            self.sensor_publishers.append(pub)

        # -------------------------
        # Timer callback
        # -------------------------
        W, H = self.res

        def timer_callback():
            if not self.is_built:
                self.node.get_logger().warn(
                    f"Sensor not built yet, skipping publish"
                )
                return

            if self.viewer is not None:
                if not self.viewer.is_running():
                    return

            # --- Pose ---
            self.tf_sensor.update(self.state)

            # Get transform from sensor (convert numpy → unpack into Warp transform)
            sensor_tf_np = self.tf_sensor.transforms.numpy()[0]
            self.tf = wp.transformf(*sensor_tf_np)

            # create q_offset
            euler_offset_rad = wp.vec3f(
                np.radians(
                    np.array(self.euler_offset, dtype=float)
                )
            )
            q_offset = wp.quat_from_euler(euler_offset_rad,0,1,2)
            pos_offset_vec = wp.vec3(
                self.pos_offset
            )

            # Transform sensor tf frame by offset
            self.tf = wp.transform_multiply(self.tf, wp.transform(pos_offset_vec, q_offset))

            tf_wp = wp.array(
                [[self.tf]],
                dtype=wp.transformf,
            )

            # --- Render ---
            self.cam.update(
                self.state,
                tf_wp,
                self.camera_rays,
                color_image=self.color_buffer,
                depth_image=self.depth_buffer,
                normal_image=self.normal_buffer,
                shape_index_image=self.seg_buffer,
                albedo_image=self.albedo_buffer,
                clear_data=SensorTiledCamera.GRAY_CLEAR_DATA,
            )

            # -------------------------
            # Publish
            # -------------------------
            stamp = get_current_timestamp()

            for pub, cam_type in zip(self.sensor_publishers, self.camera_types):
                if cam_type == "rgb":
                    # color_buffer: wp.array(dtype=uint32), shape (1, 1, H, W)
                    # Reinterpret uint32 as uint8 bytes [R, G, B, A] (little-endian).
                    # This avoids 3 bit-shift temporaries and np.stack entirely.
                    if self.add_noise:
                        wp.launch(
                            add_gaussian_noise_uint32_4d,
                            dim=self.color_buffer.shape,
                            inputs=[self.color_buffer, self.noise_mean, self.noise_std, self.seed],
                        )
                    raw = self.color_buffer.numpy()[0, 0]  # (H, W) uint32
                    np.copyto(self._np_rgb, raw.view(np.uint8).reshape(H, W, 4)[..., :3])
                    msg = self.bridge.cv2_to_imgmsg(self._np_rgb, encoding="rgb8")

                elif cam_type == "depth":
                    # depth_buffer: wp.array(dtype=float32), shape (1, 1, H, W)
                    if self.add_noise:
                        wp.launch(
                            add_gaussian_noise_f32_4d,
                            dim=self.depth_buffer.shape,
                            inputs=[self.depth_buffer, self.noise_mean, self.noise_std, self.seed],
                        )
                    self._np_depth = self.depth_buffer.numpy()[0, 0]  # (H, W) float32, no copy needed
                    msg = self.bridge.cv2_to_imgmsg(self._np_depth, encoding="32FC1")

                elif cam_type == "normal":
                    # normal_buffer: wp.array(dtype=vec3f), shape (1, 1, H, W)
                    # In-place ops on pre-allocated float buffer avoids 2 large temporaries.
                    if self.add_noise:
                        wp.launch(
                            add_gaussian_noise_vec3_4d,
                            dim=self.normal_buffer.shape,
                            inputs=[self.normal_buffer, self.noise_mean, self.noise_std, self.seed],
                        )
                    normals_raw = self.normal_buffer.numpy()[0, 0]  # (H, W, 3) float32
                    np.multiply(normals_raw, 0.5, out=self._np_normal_f)
                    self._np_normal_f += 0.5
                    self._np_normal_f *= 255.0
                    np.clip(self._np_normal_f, 0.0, 255.0, out=self._np_normal_f)
                    np.copyto(self._np_normal, self._np_normal_f, casting="unsafe")
                    msg = self.bridge.cv2_to_imgmsg(self._np_normal, encoding="rgb8")

                elif cam_type == "segmentation":
                    # seg_buffer: wp.array(dtype=uint32), shape (1, 1, H, W)
                    # In-place normalisation on pre-allocated float buffer.
                    if self.add_noise:
                        wp.launch(
                            add_gaussian_noise_uint32_4d,
                            dim=self.seg_buffer.shape,
                            inputs=[self.seg_buffer, self.noise_mean, self.noise_std, self.seed],
                        )
                    seg_raw = self.seg_buffer.numpy()[0, 0]
                    tmp = seg_raw * 37            # stays uint32
                    np.bitwise_and(tmp, 0xFF, out=self._np_seg, casting="unsafe")
                    msg = self.bridge.cv2_to_imgmsg(self._np_seg, encoding="mono8")
                else:
                    continue
                msg.header.frame_id = self.frame_id
                msg.header.stamp = stamp
                pub.publish(msg)

        # -------------------------
        # Timer
        # -------------------------
        timer = self.node.create_timer(1 / frequency, timer_callback)
        setattr(self, f"{self.sensor_name}_camera_timer", timer)

        return None, self.sensor_publishers

    def log(self):
        return