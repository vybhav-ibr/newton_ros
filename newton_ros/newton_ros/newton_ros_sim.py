import logging

import warp as wp
import numpy as np
from rosgraph_msgs.msg import Clock
from .newton_ros_utils import (
    add_entities_info,
    add_ground_plane,
    add_mjcf,
    add_primitive_box,
    add_primitive_capsule,
    add_primitive_cone,
    add_primitive_cylinder,
    add_primitive_ellipsoid,
    add_primitive_sphere,
    add_urdf,
    add_usd,
    get_current_timestamp,
    make_shape_cfg,
    calculate_bounds,
    make_terrain_mesh,
    make_xform,
)
import newton


class NewtonRosSim:
    """Handles core Newton simulation operations like making solvers and spawning entities."""

    def __init__(self, builder):
        """Initialize the simulator wrapper with scene reference and configuration."""
        self.logger = logging.getLogger(__name__)
        self.builder = builder
        self.STOP_SIMULATOR = False

    def spawn_from_config(self, entity_config, entity_name, entities_info):
        """Create and add an entity (robot, object, or world) to the simulation."""
        if entity_config.get("type") =="plane":
            add_ground_plane(self.builder, entity_config)
        elif entity_config.get("type") == "terrain":
            terrain_mesh=make_terrain_mesh(entity_config)
            self.builder.add_shape_mesh(
                body=-1,
                mesh=terrain_mesh,
                xform=make_xform(entity_config),
                cfg=make_shape_cfg(entity_config),
            )
        elif entity_config.get("type") == "sphere":
            add_primitive_sphere(self.builder, entity_config)
        elif entity_config.get("type") == "ellipsoid":
            add_primitive_ellipsoid(self.builder, entity_config)
        elif entity_config.get("type") == "box":
            add_primitive_box(self.builder, entity_config)
        elif entity_config.get("type") == "capsule":                
            add_primitive_capsule(self.builder, entity_config)
        elif entity_config.get("type") == "cylinder":
            add_primitive_cylinder(self.builder, entity_config)
        elif entity_config.get("type") == "cone":
            add_primitive_cone(self.builder, entity_config)
        elif entity_config.get("type") =="mesh":
            mesh = newton.Mesh.create_from_file(entity_config["source"])  
            body = self.builder.add_body(pos=entity_config.get("pos", (0, 0, 0)), mass=entity_config.get("mass", 1.0))  
            self.builder.add_shape_mesh(body=body, mesh=mesh)            # Add mesh shape to body  
        elif entity_config.get("type") =="urdf":
            add_urdf(self.builder, entity_config)
        elif entity_config.get("type") == "mjcf":
            add_mjcf(self.builder, entity_config)
        elif entity_config.get("type") == "usd":
            add_usd(self.builder, entity_config)
        else:
            raise ValueError(f"Unsupported entity type '{entity_config.get('type')}' for entity '{entity_name}'")
        
        if entity_config.get("is_world", False):
            bounds = calculate_bounds(entity_config)
            entities_info[entity_name] = {
                "world_name": entity_name,
                "world_bounds": bounds,
            }
        else:
            add_entities_info(entities_info, entity_name, entity_config)


    def start_clock(self, ros_node):
        """Start a ROS 2 clock publisher synchronized with simulation time."""

        def timer_callback(clock_publisher):
            cur_t=get_current_timestamp()
            msg = Clock()
            msg.clock.sec = cur_t.sec
            msg.clock.nanosec = cur_t.nanosec
            clock_publisher.publish(msg)

        self.pub = ros_node.create_publisher(Clock, "/clock", 50)
        # Timer fires every 0.005 seconds
        self.timer = ros_node.create_timer(0.005, lambda: timer_callback(self.pub))