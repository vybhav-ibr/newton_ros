from simulation_interfaces.msg import Result
from newton_ros_interfaces.msg import (
    RosSensorOptions,
    GeneralSensorOptions,
    RigidSensorOptions,
    NoisySensorOptions,
    CameraOptions,
    LidarOptions,
    ImuOptions,
    ContactOptions,
    ContactForceOptions,
    GridRayCasterPattern,
    SphericalRayCasterPattern,
    DepthCameraRayCasterPattern,
)

import re


class SensorOptionsHandler:
    """Handles GetSensorOptions and SetSensorOptions services"""

    RESULT_OK = 1
    RESULT_NOT_FOUND = 2

    def __init__(self, node, scene_manager):
        """Initialize the sensor options handler."""
        self.node = node
        self.scene_manager = scene_manager
        self.logger = node.get_logger()

    # --- To Message Helpers ---

    def _populate_ros_options(self, d):
        msg = RosSensorOptions()
        msg.frame_id = str(d.get("frame_id", ""))
        msg.frequency = float(d.get("frequency", 0.0))
        msg.topic = str(d.get("topic", ""))
        msg.qos_history = str(d.get("qos_history", ""))
        msg.qos_depth = int(d.get("qos_depth", 0))
        msg.qos_reliability = str(d.get("qos_reliability", ""))
        msg.qos_durability = str(d.get("qos_durability", ""))
        return msg

    def _populate_sensor_options(self, d):
        msg = GeneralSensorOptions()
        msg.name = str(d.get("name", ""))
        msg.delay = float(d.get("delay", 0.0))
        msg.update_ground_truth_only = bool(d.get("update_ground_truth_only", False))
        msg.draw_debug = bool(d.get("draw_debug", False))
        return msg

    def _populate_rigid_options(self, d):
        msg = RigidSensorOptions()
        msg.link = str(d.get("link", ""))
        msg.pos_offset = [float(x) for x in d.get("pos_offset", [0.0, 0.0, 0.0])]
        msg.euler_offset = [float(x) for x in d.get("euler_offset", [0.0, 0.0, 0.0])]
        return msg

    def _populate_noisy_options(self, d):
        msg = NoisySensorOptions()

        # Handle scalar or list for noisy params. Message expects arrays (float64[]).
        def to_list(val):
            if isinstance(val, (list, tuple)):
                return [float(x) for x in val]
            return [float(val)]

        msg.resolution = to_list(d.get("resolution", 0.0))
        msg.bias = to_list(d.get("bias", 0.0))
        msg.noise = to_list(d.get("noise", 0.0))
        msg.random_walk = to_list(d.get("random_walk", 0.0))
        msg.jitter = float(d.get("jitter", 0.0))
        msg.interpolate = bool(d.get("interpolate", False))
        return msg

    def _populate_grid_pattern_options(self, d):
        msg = GridRayCasterPattern()
        msg.resolution = float(d.get("resolution", 0.0))
        msg.size = [float(x) for x in d.get("size", [])]
        msg.direction = [float(x) for x in d.get("direction", [])]
        return msg

    def _populate_spherical_pattern_options(self, d):
        msg = SphericalRayCasterPattern()
        msg.fov = [float(x) for x in d.get("fov", [])]
        msg.n_points = [int(x) for x in d.get("n_points", [])]
        msg.angular_resolution = [float(x) for x in d.get("angular_resolution", [])]
        msg.angles = [float(x) for x in d.get("angles", [])]
        return msg

    def _populate_depth_camera_pattern_options(self, d):
        msg = DepthCameraRayCasterPattern()
        msg.res = [float(x) for x in d.get("res", [])]
        msg.fx = float(d.get("fx", 0.0))
        msg.fy = float(d.get("fy", 0.0))
        msg.cx = float(d.get("cx", 0.0))
        msg.cy = float(d.get("cy", 0.0))
        msg.fov_horizontal = float(d.get("fov_horizontal", 0.0))
        msg.fov_vertical = float(d.get("fov_vertical", 0.0))
        return msg

    # --- From Message Helpers ---

    def _convert_ros_options(self, msg):
        d = {}
        d["frame_id"] = msg.frame_id
        d["frequency"] = msg.frequency
        d["topic"] = msg.topic
        d["qos_history"] = msg.qos_history
        d["qos_depth"] = msg.qos_depth
        d["qos_reliability"] = msg.qos_reliability
        d["qos_durability"] = msg.qos_durability
        return d

    def _convert_sensor_options(self, msg):
        d = {}
        d["name"] = msg.name
        d["sensor_type"] = msg.sensor_type
        d["delay"] = msg.delay
        d["update_ground_truth_only"] = msg.update_ground_truth_only
        d["draw_debug"] = msg.draw_debug
        return d

    def _convert_rigid_options(self, msg):
        d = {}
        d["link"] = msg.link
        d["pos_offset"] = list(msg.pos_offset)
        d["euler_offset"] = list(msg.euler_offset)
        return d

    def _convert_noisy_options(self, msg):
        d = {}
        d["resolution"] = list(msg.resolution)
        d["bias"] = list(msg.bias)
        d["noise"] = list(msg.noise)
        d["random_walk"] = list(msg.random_walk)
        d["jitter"] = msg.jitter
        d["interpolate"] = msg.interpolate
        return d

    def _convert_grid_pattern_options(self, msg):
        if msg is None:
            return None
        d = {}
        if msg.resolution:
            d["resolution"] = msg.resolution
        if msg.size:
            d["size"] = list(msg.size)
        if msg.direction:
            d["direction"] = list(msg.direction)
        return d

    def _convert_spherical_pattern_options(self, msg):
        if msg is None:
            return None
        d = {}
        if msg.fov:
            d["fov"] = list(msg.fov)
        if msg.n_points:
            d["n_points"] = list(msg.n_points)
        if msg.angular_resolution:
            d["angular_resolution"] = list(msg.angular_resolution)
        if msg.angles:
            d["angles"] = list(msg.angles)
        return d

    def _convert_depth_camera_pattern_options(self, msg):
        if msg is None:
            return None
        d = {}
        if msg.res:
            d["res"] = list(msg.res)
        if msg.fx:
            d["fx"] = msg.fx
        if msg.fy:
            d["fy"] = msg.fy
        if msg.cx:
            d["cx"] = msg.cx
        if msg.cy:
            d["cy"] = msg.cy
        if msg.fov_horizontal:
            d["fov_horizontal"] = msg.fov_horizontal
        if msg.fov_vertical:
            d["fov_vertical"] = msg.fov_vertical
        return d

    def set_sensor_options_callback(self, request, response):
        """Configure various sensors (camera, lidar, imu, etc.) on an entity."""
        """Set sensor options from structured messages"""
        self.logger.info(f"SetSensorOptions service called: {request.entity}")

        response.result = Result()

        if request.entity not in self.scene_manager.entities_info:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = f"Entity '{request.entity}' not found"
            return response

        new_sensors_config = {
            "cameras": [],
            "lidars": [],
            "imus": [],
            "contacts": [],
            "contact_forces": [],
        }

        # Process Cameras
        for cam in request.cameras:
            d = {}
            d["ros_options"] = self._convert_ros_options(cam.ros_options)
            d["general_sensor_options"] = self._convert_sensor_options(cam.options)
            d["rigid_options"] = self._convert_rigid_options(cam.rigid_options)

            if cam.camera_types:
                d["camera_types"] = list(cam.camera_types)
            if cam.res:
                d["res"] = list(cam.res)
            if cam.fov:
                d["fov"] = cam.fov
            if cam.near:
                d["near"] = cam.near
            if cam.far:
                d["far"] = cam.far
            if cam.aperture:
                d["aperture"] = cam.aperture
            d["denoise"] = cam.denoise
            if cam.spp:
                d["spp"] = cam.spp
            if cam.model:
                d["model"] = cam.model
            if cam.focus_dist:
                d["focus_dist"] = cam.focus_dist
            d["gui"] = cam.gui
            new_sensors_config["cameras"].append(d)

        # Process Lidars
        for lidar in request.lidars:
            d = {}
            d["ros_options"] = self._convert_ros_options(lidar.ros_options)
            d["general_sensor_options"] = self._convert_sensor_options(lidar.options)
            d["rigid_options"] = self._convert_rigid_options(lidar.rigid_options)
            d["grid_pattern"] = self._convert_grid_pattern_options(
                lidar.grid_pattern_options
            )
            d["spherical_pattern"] = self._convert_spherical_pattern_options(
                lidar.spherical_pattern_options
            )
            d["depth_camera_pattern"] = self._convert_depth_camera_pattern_options(
                lidar.depth_camera_pattern_options
            )

            num_pattern = sum(
                [
                    d["grid_pattern"] is not None,
                    d["spherical_pattern"] is not None,
                    d["depth_camera_pattern"] is not None,
                ]
            )

            # Validate: Only one pattern should be set
            if num_pattern != 1:
                response.success = False
                response.message = "Error: Exactly one pattern must be provided (Grid, Spherical, or DepthCamera). lidar files with more than one pattern are ignored"
                break

            if lidar.min_range:
                d["min_range"] = lidar.min_range
            if lidar.max_range:
                d["max_range"] = lidar.max_range
            if lidar.no_hit_value:
                d["no_hit_value"] = lidar.no_hit_value
            if lidar.return_points_in_world_frame:
                d["return_points_in_world_frame"] = lidar.return_points_in_world_frame
            if lidar.draw_point_radius:
                d["draw_point_radius"] = lidar.draw_point_radius
            if lidar.ray_start_color:
                d["ray_start_color"] = list(lidar.ray_start_color)
            if lidar.ray_hit_color:
                d["ray_hit_color"] = list(lidar.ray_hit_color)
            if lidar.add_noise:
                d["add_noise"] = lidar.add_noise
            if lidar.noise_mean:
                d["noise_mean"] = lidar.noise_mean
            if lidar.noise_std:
                d["noise_std"] = lidar.noise_std
            new_sensors_config["lidars"].append(d)

        # Process IMUs
        for imu in request.imus:
            d = {}
            d["ros_options"] = self._convert_ros_options(imu.ros_options)
            d["general_sensor_options"] = self._convert_sensor_options(imu.options)
            d["rigid_options"] = self._convert_rigid_options(imu.rigid_options)
            d["noisy_options"] = self._convert_noisy_options(imu.noisy_options)

            if imu.acc_resolution:
                d["acc_resolution"] = imu.acc_resolution
            if imu.acc_cross_axis_coupling:
                d["acc_cross_axis_coupling"] = list(imu.acc_cross_axis_coupling)
            if imu.acc_bias:
                d["acc_bias"] = list(imu.acc_bias)
            if imu.acc_noise:
                d["acc_noise"] = list(imu.acc_noise)
            if imu.acc_random_walk:
                d["acc_random_walk"] = list(imu.acc_random_walk)

            if imu.gyro_resolution:
                d["gyro_resolution"] = imu.gyro_resolution
            if imu.gyro_cross_axis_coupling:
                d["gyro_cross_axis_coupling"] = list(imu.gyro_cross_axis_coupling)
            if imu.gyro_bias:
                d["gyro_bias"] = list(imu.gyro_bias)
            if imu.gyro_noise:
                d["gyro_noise"] = list(imu.gyro_noise)
            if imu.gyro_random_walk:
                d["gyro_random_walk"] = list(imu.gyro_random_walk)

            if imu.debug_acc_color:
                d["debug_acc_color"] = list(imu.debug_acc_color)
            if imu.debug_acc_scale:
                d["debug_acc_scale"] = imu.debug_acc_scale
            if imu.debug_gyro_color:
                d["debug_gyro_color"] = list(imu.debug_gyro_color)
            if imu.debug_gyro_scale:
                d["debug_gyro_scale"] = imu.debug_gyro_scale
            new_sensors_config["imus"].append(d)

        # Process Contacts
        for contact in request.contacts:
            d = {}
            d["ros_options"] = self._convert_ros_options(contact.ros_options)
            d["general_sensor_options"] = self._convert_sensor_options(contact.options)
            d["rigid_options"] = self._convert_rigid_options(contact.rigid_options)

            if contact.debug_sphere_radius:
                d["debug_sphere_radius"] = contact.debug_sphere_radius
            if contact.debug_color:
                d["debug_color"] = list(contact.debug_color)
            new_sensors_config["contacts"].append(d)

        # Process Contact Forces
        for cf in request.contact_forces:
            d = {}
            d["ros_options"] = self._convert_ros_options(cf.ros_options)
            d["general_sensor_options"] = self._convert_sensor_options(cf.options)
            d["rigid_options"] = self._convert_rigid_options(cf.rigid_options)
            d["noisy_options"] = self._convert_noisy_options(cf.noisy_options)

            if cf.min_force:
                d["min_force"] = list(cf.min_force)
            if cf.max_force:
                d["max_force"] = list(cf.max_force)
            if cf.debug_color:
                d["debug_color"] = list(cf.debug_color)
            if cf.debug_scale:
                d["debug_scale"] = cf.debug_scale
            new_sensors_config["contact_forces"].append(d)

        has_sensors = any(len(v) > 0 for v in new_sensors_config.values())
        if not has_sensors:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "No valid sensor configurations provided"
            return response

        # Update the entity info
        entity_entry = self.scene_manager.entities_info[request.entity]

        if isinstance(entity_entry, dict):
            if "sensors" not in entity_entry or not isinstance(
                entity_entry["sensors"], dict
            ):
                entity_entry["sensors"] = {}
            entity_entry["sensors"].update(new_sensors_config)
        else:
            if not hasattr(entity_entry, "sensors"):
                entity_entry.sensors = {}

            if isinstance(entity_entry.sensors, dict):
                entity_entry.sensors.update(new_sensors_config)
            else:
                # Fallback
                entity_entry.sensors = new_sensors_config
        
        if isinstance(entity_entry, dict):
            entity_entry["initialisation_pending"] = True
        else:
            entity_entry.initialisation_pending = True

        total_sensors = sum(len(v) for v in new_sensors_config.values())
        self.logger.info(
            f"Updated sensor config for {request.entity} with {total_sensors} sensors"
        )
        response.result.result = self.RESULT_OK
        return response

    # def get_sensor_options_callback(self, request, response):
    #     """Retrieve existing sensor configurations for entities matching a regex."""
    #     """Get sensor options as structured messages"""
    #     self.logger.info(
    #         f"GetSensorOptions service called: entity_filter: {request.entity}, "
    #         f"names: {request.names}, sensor_types: {request.sensor_types}"
    #     )

    #     response.result = Result()

    #     all_target_sensors = []
    #     entity_found = False

    #     try:
    #         entity_regex = re.compile(request.entity)
    #     except re.error:
    #         entity_regex = re.compile(re.escape(request.entity))

    #     for entity_name, entity_info in self.scene_manager.entities_info.items():
    #         if entity_regex.match(entity_name):
    #             entity_found = True
    #             sensors_data = entity_info.get("sensors")
    #             if sensors_data is None or not isinstance(sensors_data, dict):
    #                 continue

    #             for category in [
    #                 "cameras",
    #                 "lidars",
    #                 "imus",
    #                 "contacts",
    #                 "contact_forces",
    #             ]:
    #                 if category in sensors_data:
    #                     for sensor_config in sensors_data[category]:
    #                         s_name = sensor_config.get("general_sensor_options", {}).get("name", "")
    #                         s_type = (
    #                             sensor_config.get("general_sensor_options", {})
    #                             .get("sensor_type", "")
    #                         )

    #                         if s_name not in request.names:
    #                             continue
    #                         if (
    #                             s_type not in request.sensor_types
    #                         ):
    #                             continue

    #                         all_target_sensors.append(sensor_config)

    #     if not entity_found:
    #         response.result.result = self.RESULT_NOT_FOUND
    #         response.result.error_message = (
    #             f"No entities found matching pattern '{request.entity}'"
    #         )
    #         return response

    #     if not all_target_sensors:
    #         response.result.result = self.RESULT_OK # Not an error if no sensors exist but entity exists
    #         return response

    #     try:
    #         for config in all_target_sensors:
    #             s_type = config.get("general_sensor_options").get("sensor_type", "")

    #             # Camera
    #             if s_type in ["cam", "rgb", "depth", "segmentation", "rgbd"]:
    #                 msg = CameraOptions()
    #                 msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
    #                 msg.options = self._populate_sensor_options(config.get("general_sensor_options", {}))
    #                 msg.rigid_options = self._populate_rigid_options(config.get("rigid_options", {}))

    #                 if "camera_types" in config:
    #                     msg.camera_types = [str(t) for t in config["camera_types"]]
    #                 if "res" in config:
    #                     msg.res = [int(x) for x in config["res"]]
    #                 if "fov" in config:
    #                     msg.fov = float(config["fov"])  # Scalar
    #                 if "near" in config:
    #                     msg.near = float(config["near"])
    #                 if "far" in config:
    #                     msg.far = float(config["far"])
    #                 if "aperture" in config:
    #                     msg.aperture = float(config["aperture"])
    #                 if "denoise" in config:
    #                     msg.denoise = bool(config["denoise"])
    #                 if "spp" in config:
    #                     msg.spp = int(config["spp"])
    #                 if "model" in config:
    #                     msg.model = str(config["model"])
    #                 if "focus_dist" in config:
    #                     msg.focus_dist = float(config["focus_dist"])
    #                 if "gui" in config:
    #                     msg.gui = bool(config["gui"])
    #                 response.cameras.append(msg)

    #             # Lidar
    #             elif s_type in [
    #                 "3d_lidar",
    #                 "grid_lidar",
    #                 "sectional_lidar",
    #                 "laser_scan",
    #                 "lidar",
    #             ]:
    #                 msg = LidarOptions()
    #                 msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
    #                 msg.options = self._populate_sensor_options(config.get("general_sensor_options", {}))
    #                 msg.rigid_options = self._populate_rigid_options(config.get("rigid_options", {}))
    #                 msg.grid_pattern_options = self._populate_grid_pattern_options(config.get("grid_pattern", {}))
    #                 msg.spherical_pattern_options = self._populate_spherical_pattern_options(config.get("spherical_pattern", {}))
    #                 msg.depth_camera_pattern_options = self._populate_depth_camera_pattern_options(config.get("depth_camera_pattern", {}))

    #                 if "min_range" in config:
    #                     msg.min_range = float(config["min_range"])
    #                 if "max_range" in config:
    #                     msg.max_range = float(config["max_range"])
    #                 if "no_hit_value" in config:
    #                     msg.no_hit_value = float(config["no_hit_value"])
    #                 if "return_points_in_world_frame" in config:
    #                     msg.return_points_in_world_frame = bool(config["return_points_in_world_frame"])
    #                 if "draw_point_radius" in config:
    #                     msg.draw_point_radius = float(config["draw_point_radius"])
    #                 if "ray_start_color" in config:
    #                     msg.ray_start_color = [float(x) for x in config["ray_start_color"]]
    #                 if "ray_hit_color" in config:
    #                     msg.ray_hit_color = [float(x) for x in config["ray_hit_color"]]
    #                 if "add_noise" in config:
    #                     msg.add_noise = bool(config["add_noise"])
    #                 if "noise_mean" in config:
    #                     msg.noise_mean = float(config["noise_mean"])
    #                 if "noise_std" in config:
    #                     msg.noise_std = float(config["noise_std"])
    #                 response.lidars.append(msg)

    #             # Imu
    #             elif s_type == "imu":
    #                 msg = ImuOptions()
    #                 msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
    #                 msg.options = self._populate_sensor_options(config.get("general_sensor_options", {}))
    #                 msg.rigid_options = self._populate_rigid_options(config.get("rigid_options", {}))
    #                 msg.noisy_options = self._populate_noisy_options(config.get("noisy_options", {}))

    #                 if "acc_resolution" in config:
    #                     msg.acc_resolution = float(config["acc_resolution"])
    #                 if "acc_cross_axis_coupling" in config:
    #                     msg.acc_cross_axis_coupling = [
    #                         float(x) for x in config["acc_cross_axis_coupling"]
    #                     ]
    #                 if "acc_bias" in config:
    #                     msg.acc_bias = [float(x) for x in config["acc_bias"]]
    #                 if "acc_noise" in config:
    #                     msg.acc_noise = [float(x) for x in config["acc_noise"]]
    #                 if "acc_random_walk" in config:
    #                     msg.acc_random_walk = [
    #                         float(x) for x in config["acc_random_walk"]
    #                     ]

    #                 if "gyro_resolution" in config:
    #                     msg.gyro_resolution = float(config["gyro_resolution"])
    #                 if "gyro_cross_axis_coupling" in config:
    #                     msg.gyro_cross_axis_coupling = [
    #                         float(x) for x in config["gyro_cross_axis_coupling"]
    #                     ]
    #                 if "gyro_bias" in config:
    #                     msg.gyro_bias = [float(x) for x in config["gyro_bias"]]
    #                 if "gyro_noise" in config:
    #                     msg.gyro_noise = [float(x) for x in config["gyro_noise"]]
    #                 if "gyro_random_walk" in config:
    #                     msg.gyro_random_walk = [
    #                         float(x) for x in config["gyro_random_walk"]
    #                     ]

    #                 if "debug_acc_color" in config:
    #                     msg.debug_acc_color = [
    #                         float(x) for x in config["debug_acc_color"]
    #                     ]
    #                 if "debug_acc_scale" in config:
    #                     msg.debug_acc_scale = float(config["debug_acc_scale"])
    #                 if "debug_gyro_color" in config:
    #                     msg.debug_gyro_color = [
    #                         float(x) for x in config["debug_gyro_color"]
    #                     ]
    #                 if "debug_gyro_scale" in config:
    #                     msg.debug_gyro_scale = float(config["debug_gyro_scale"])
    #                 response.imus.append(msg)

    #             # Contact
    #             elif s_type == "contact":
    #                 msg = ContactOptions()
    #                 msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
    #                 msg.options = self._populate_sensor_options(config.get("general_sensor_options", {}))
    #                 msg.rigid_options = self._populate_rigid_options(config.get("rigid_options", {}))

    #                 if "debug_sphere_radius" in config:
    #                     msg.debug_sphere_radius = float(config["debug_sphere_radius"])
    #                 if "debug_color" in config:
    #                     msg.debug_color = [float(x) for x in config["debug_color"]]
    #                 response.contacts.append(msg)

    #             # Contact Force
    #             elif s_type == "contact_force":
    #                 msg = ContactForceOptions()
    #                 msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
    #                 msg.options = self._populate_sensor_options(config.get("general_sensor_options", {}))
    #                 msg.rigid_options = self._populate_rigid_options(config.get("rigid_options", {}))
    #                 msg.noisy_options = self._populate_noisy_options(config.get("noisy_options", {}))

    #                 if "min_force" in config:
    #                     msg.min_force = [float(x) for x in config["min_force"]]
    #                 if "max_force" in config:
    #                     msg.max_force = [float(x) for x in config["max_force"]]
    #                 if "debug_color" in config:
    #                     msg.debug_color = [float(x) for x in config["debug_color"]]
    #                 if "debug_scale" in config:
    #                     msg.debug_scale = float(config["debug_scale"])
    #                 response.contact_forces.append(msg)

    #         response.result.result = self.RESULT_OK

    #     except Exception as e:
    #         response.result.result = self.RESULT_NOT_FOUND
    #         response.result.error_message = f"Failed to process config: {str(e)}"
    #         self.logger.error(f"Error processing sensor config: {e}")
    #     return response
    def get_sensor_options_callback(self, request, response):
        """Retrieve existing sensor configurations for entities matching a regex."""
        self.logger.info(
            f"GetSensorOptions service called: entity_filter={request.entity}, "
            f"names={request.names}, sensor_types={request.sensor_types}"
        )

        response.result = Result()
        all_target_sensors = []
        entity_found = False

        try:
            entity_regex = re.compile(request.entity)
        except re.error:
            entity_regex = re.compile(re.escape(request.entity))

        for entity_name, entity_info in self.scene_manager.entities_info.items():
            if not entity_regex.match(entity_name):
                continue

            entity_found = True
            sensors_data = entity_info.get("sensors")
            if not isinstance(sensors_data, dict):
                continue

            for sensor_list in sensors_data.values():
                for sensor_config in sensor_list:
                    gen = sensor_config.get("general_sensor_options", {})
                    s_name = gen.get("name", "")
                    s_type = gen.get("sensor_type", "")

                    if request.names and s_name not in request.names:
                        continue

                    if request.sensor_types and s_type not in request.sensor_types:
                        continue

                    all_target_sensors.append(sensor_config)

        if not entity_found:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = (
                f"No entities found matching pattern '{request.entity}'"
            )
            return response

        if not all_target_sensors:
            response.result.result = self.RESULT_OK
            return response

        try:
            for config in all_target_sensors:
                s_type = config.get("general_sensor_options", {}).get("sensor_type", "")

                if s_type in ["cam", "rgb", "depth", "segmentation", "rgbd"]:
                    self._process_camera(config, response)

                elif s_type in [
                    "3d_lidar",
                    "grid_lidar",
                    "sectional_lidar",
                    "laser_scan",
                    "lidar",
                ]:
                    self._process_lidar(config, response)

                elif s_type == "imu":
                    self._process_imu(config, response)

                elif s_type == "contact":
                    self._process_contact(config, response)

                elif s_type == "contact_force":
                    self._process_contact_force(config, response)

            response.result.result = self.RESULT_OK

        except Exception as e:
            self.logger.error(f"Error processing sensor config: {e}")
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = str(e)

        return response

    def _process_camera(self, config, response):
        msg = CameraOptions()
        msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
        msg.options = self._populate_sensor_options(
            config.get("general_sensor_options", {})
        )
        msg.rigid_options = self._populate_rigid_options(
            config.get("rigid_options", {})
        )

        if "camera_types" in config:
            msg.camera_types = [str(t) for t in config["camera_types"]]
        if "res" in config:
            msg.res = [int(x) for x in config["res"]]
        if "fov" in config:
            msg.fov = float(config["fov"])  # Scalar
        if "near" in config:
            msg.near = float(config["near"])
        if "far" in config:
            msg.far = float(config["far"])
        if "aperture" in config:
            msg.aperture = float(config["aperture"])
        if "denoise" in config:
            msg.denoise = bool(config["denoise"])
        if "spp" in config:
            msg.spp = int(config["spp"])
        if "model" in config:
            msg.model = str(config["model"])
        if "focus_dist" in config:
            msg.focus_dist = float(config["focus_dist"])
        if "gui" in config:
            msg.gui = bool(config["gui"])

        response.cameras.append(msg)

    def _process_lidar(self, config, response):
        msg = LidarOptions()
        msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
        msg.options = self._populate_sensor_options(
            config.get("general_sensor_options", {})
        )
        msg.rigid_options = self._populate_rigid_options(
            config.get("rigid_options", {})
        )
        msg.grid_pattern_options = self._populate_grid_pattern_options(
            config.get("grid_pattern", {})
        )
        msg.spherical_pattern_options = self._populate_spherical_pattern_options(
            config.get("spherical_pattern", {})
        )
        msg.depth_camera_pattern_options = self._populate_depth_camera_pattern_options(
            config.get("depth_camera_pattern", {})
        )

        for key in [
            "min_range",
            "max_range",
            "no_hit_value",
            "draw_point_radius",
            "noise_mean",
            "noise_std",
        ]:
            if key in config:
                setattr(msg, key, float(config[key]))

        if "return_points_in_world_frame" in config:
            msg.return_points_in_world_frame = bool(
                config["return_points_in_world_frame"]
            )

        if "ray_start_color" in config:
            msg.ray_start_color = [float(x) for x in config["ray_start_color"]]
        if "ray_hit_color" in config:
            msg.ray_hit_color = [float(x) for x in config["ray_hit_color"]]

        if "add_noise" in config:
            msg.add_noise = bool(config["add_noise"])

        response.lidars.append(msg)

    def _process_imu(self, config, response):
        msg = ImuOptions()
        msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
        msg.options = self._populate_sensor_options(
            config.get("general_sensor_options", {})
        )
        msg.rigid_options = self._populate_rigid_options(
            config.get("rigid_options", {})
        )
        msg.noisy_options = self._populate_noisy_options(
            config.get("noisy_options", {})
        )

        vector_fields = [
            "acc_cross_axis_coupling",
            "acc_bias",
            "acc_noise",
            "acc_random_walk",
            "gyro_cross_axis_coupling",
            "gyro_bias",
            "gyro_noise",
            "gyro_random_walk",
        ]

        scalar_fields = ["acc_resolution", "gyro_resolution"]

        for f in vector_fields:
            if f in config:
                setattr(msg, f, [float(x) for x in config[f]])

        for f in scalar_fields:
            if f in config:
                setattr(msg, f, float(config[f]))

        response.imus.append(msg)

    def _process_contact(self, config, response):
        msg = ContactOptions()
        msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
        msg.options = self._populate_sensor_options(
            config.get("general_sensor_options", {})
        )
        msg.rigid_options = self._populate_rigid_options(
            config.get("rigid_options", {})
        )

        if "debug_sphere_radius" in config:
            msg.debug_sphere_radius = float(config["debug_sphere_radius"])
        if "debug_color" in config:
            msg.debug_color = [float(x) for x in config["debug_color"]]

        response.contacts.append(msg)

    def _process_contact_force(self, config, response):
        msg = ContactForceOptions()
        msg.ros_options = self._populate_ros_options(config.get("ros_options", {}))
        msg.options = self._populate_sensor_options(
            config.get("general_sensor_options", {})
        )
        msg.rigid_options = self._populate_rigid_options(
            config.get("rigid_options", {})
        )
        msg.noisy_options = self._populate_noisy_options(
            config.get("noisy_options", {})
        )

        if "min_force" in config:
            msg.min_force = [float(x) for x in config["min_force"]]
        if "max_force" in config:
            msg.max_force = [float(x) for x in config["max_force"]]
        if "debug_color" in config:
            msg.debug_color = [float(x) for x in config["debug_color"]]
        if "debug_scale" in config:
            msg.debug_scale = float(config["debug_scale"])

        response.contact_forces.append(msg)
