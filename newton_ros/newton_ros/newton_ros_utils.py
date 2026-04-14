import time

import numpy as np
from builtin_interfaces.msg import Time

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)
import newton
from newton.solvers import (
    SolverFeatherstone,
    SolverImplicitMPM,
    SolverKamino,
    SolverMuJoCo,
    SolverSemiImplicit,
    SolverStyle3D,
    SolverVBD,
    SolverXPBD,
)
import newton
import warp as wp
from newton._src.geometry.utils import remesh_ftetwild
from pytetwild import tetrahedralize    


def create_qos_profile(
    history: str = "keep_last",
    depth: int = 10,
    reliability: str = "reliable",
    durability: str = "volatile",
) -> QoSProfile:
    """Create a ROS 2 QoSProfile from string parameters for history, reliability, and durability."""
    """
    Create a QoSProfile from string parameters.
    Allowed values (case-insensitive):
    - history: "keep_last", "keep_all"
    - reliability: "reliable", "best_effort"
    - durability: "transient_local", "volatile"
    """
    qos = QoSProfile(depth=depth)

    hist = history.lower()
    if hist == "keep_all":
        qos.history = QoSHistoryPolicy.KEEP_ALL
    elif hist == "keep_last":
        qos.history = QoSHistoryPolicy.KEEP_LAST
    else:
        raise ValueError(f"Invalid history policy: '{history}'")

    rel = reliability.lower()
    if rel == "reliable":
        qos.reliability = QoSReliabilityPolicy.RELIABLE
    elif rel == "best_effort":
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
    else:
        raise ValueError(f"Invalid reliability policy: '{reliability}'")

    dur = durability.lower()
    if dur == "transient_local":
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    elif dur == "volatile":
        qos.durability = QoSDurabilityPolicy.VOLATILE
    else:
        raise ValueError(f"Invalid durability policy: '{durability}'")

    return qos


def quat_angle_difference(q1, q2):
    """
    Compute the angle difference (in radians) between two unit quaternions.
    q1 and q2 must be 4D arrays: [x, y, z, w]
    """
    q1 = np.array(q1, dtype=np.float64)
    q2 = np.array(q2, dtype=np.float64)

    # Normalize to ensure unit quaternions
    q1 /= np.linalg.norm(q1)
    q2 /= np.linalg.norm(q2)

    # Compute the dot product
    dot = np.dot(q1, q2)

    # Clamp the dot product to [-1, 1] to avoid numerical issues
    dot = np.clip(dot, -1.0, 1.0)

    # Angle = 2 * arccos(|dot|) in radians
    angle = 2 * np.arccos(np.abs(dot))
    return angle


def get_entity(entities_info, name=None):
    """Retrieve an entity from a list by its unique name."""
    if name is not None:
        if entities_info.get(name) is not None:
            return entities_info[name]["entity_attr"]


def get_dofs_idx(all_joint_names, target_joint_names):
    """Map a list of joint names to their corresponding local DOF indices in the robot."""
    motor_dofs = []
    if target_joint_names is not None:
        for target_joint_name in target_joint_names:
            # if joint_name == "root_joint":
            #     continue
            for joint_index, joint_name in enumerate(all_joint_names):
                if joint_name == target_joint_name:
                    motor_dofs.append(joint_index)
        return motor_dofs


def get_links_idx(robot, link_names):
    """Map a list of link names to their corresponding local indices in the robot."""
    links_idx_local = []
    if link_names is not None:
        for link_name in link_names:
            for link in robot.links:
                if link.name == link_name:
                    links_idx_local.append(link.idx_local)
        return links_idx_local


def get_current_timestamp():
    t_ns = time.time_ns()
    timestamp = Time()
    timestamp.sec = t_ns // 1_000_000_000
    timestamp.nanosec = t_ns % 1_000_000_000
    return timestamp


def get_joint_names(robot):
    """Retrieve all non-root joint names and their corresponding local DOF indices."""
    joint_names = []
    dofs_idx_local = []
    for joint in robot.joints:
        if "root_joint" not in joint.name:
            joint_names.append(joint.name)
            dofs_idx_local.append(joint.dofs_idx_local[0])
    return joint_names, dofs_idx_local

def make_viewer(viewer_config, logger):
    """Initialize a Newton Viewer object using parameters from a configuration dictionary."""
    if viewer_config is None:
        logger.info("No viewer config found, using headless mode")
        return newton.viewer.ViewerNull()
        
    viewer_type = viewer_config.get("type", "gl")

    if viewer_type == "gl":
        logger.info("Initializing GL viewer")
        return newton.viewer.ViewerGL(
            width=viewer_config.get("width", 1920),
            height=viewer_config.get("height", 1080),
            vsync=viewer_config.get("vsync", False),
            headless=viewer_config.get("headless", False),
        )
    else:
        logger.info("No viewer config found, using headless mode")
        return newton.viewer.ViewerNull()
    
def make_solver(model, solver_config, logger, is_soft=False):
    """Initialize a Newton Solver object using parameters from a configuration dictionary."""
    if solver_config is None:
        logger.warning(f"No solver config provided, defaulting to 'featherstone'")
        if is_soft:
            return None
        return SolverFeatherstone(model)
    
    solver_type = solver_config.get('type', 'featherstone')

    if solver_type == 'featherstone':
        logger.info("Initializing featherstone solver")
        return SolverFeatherstone(
            model,
            angular_damping=solver_config.get('angular_damping', 0.05),
            update_mass_matrix_interval=solver_config.get('update_mass_matrix_interval', 1),
            friction_smoothing=solver_config.get('friction_smoothing', 1.0),
            use_tile_gemm=solver_config.get('use_tile_gemm', False),
            fuse_cholesky=solver_config.get('fuse_cholesky', True),
        )

    elif solver_type == 'mujoco':
        logger.info("Initializing mujoco solver")
        return SolverMuJoCo(
            model,
            separate_worlds=solver_config.get("separate_worlds", None),
            njmax=solver_config.get("njmax", None),
            nconmax=solver_config.get("nconmax", None),
            iterations=solver_config.get("iterations", None),
            ls_iterations=solver_config.get("ls_iterations", None),
            ccd_iterations=solver_config.get("ccd_iterations", None),
            sdf_iterations=solver_config.get("sdf_iterations", None),
            sdf_initpoints=solver_config.get("sdf_initpoints", None),
            solver=solver_config.get("solver", None),
            integrator=solver_config.get("integrator", None),
            cone=solver_config.get("cone", None),
            jacobian=solver_config.get("jacobian", None),
            impratio=solver_config.get("impratio", None),
            tolerance=solver_config.get("tolerance", None),
            ls_tolerance=solver_config.get("ls_tolerance", None),
            ccd_tolerance=solver_config.get("ccd_tolerance", None),
            density=solver_config.get("density", None),
            viscosity=solver_config.get("viscosity", None),
            wind=solver_config.get("wind", None),
            magnetic=solver_config.get("magnetic", None),
            use_mujoco_cpu=solver_config.get("use_mujoco_cpu", False),
            disable_contacts=solver_config.get("disable_contacts", False),
            update_data_interval=solver_config.get("update_data_interval", 1),
            save_to_mjcf=solver_config.get("save_to_mjcf", None),
            ls_parallel=solver_config.get("ls_parallel", False),
            use_mujoco_contacts=solver_config.get("use_mujoco_contacts", True),
            include_sites=solver_config.get("include_sites", True),
            skip_visual_only_geoms=solver_config.get("skip_visual_only_geoms", True),
        )

    elif solver_type == 'semi_implicit':
        logger.info("Initializing Semi-Implicit solver")
        return SolverSemiImplicit(
            model,
            angular_damping=solver_config.get("angular_damping", 0.05),
            friction_smoothing=solver_config.get("friction_smoothing", 1.0),
            joint_attach_ke=solver_config.get("joint_attach_ke", 1.0e4),
            joint_attach_kd=solver_config.get("joint_attach_kd", 1.0e2),
            enable_tri_contact=solver_config.get("enable_tri_contact", True),
        )

    elif solver_type == 'xpbd':
        logger.info("Initializing XPBD solver")
        return SolverXPBD(
            model,
            iterations=solver_config.get("iterations", 2),
            soft_body_relaxation=solver_config.get("soft_body_relaxation", 0.9),
            soft_contact_relaxation=solver_config.get("soft_contact_relaxation", 0.9),
            joint_linear_relaxation=solver_config.get("joint_linear_relaxation", 0.7),
            joint_angular_relaxation=solver_config.get("joint_angular_relaxation", 0.4),
            joint_linear_compliance=solver_config.get("joint_linear_compliance", 0.0),
            joint_angular_compliance=solver_config.get("joint_angular_compliance", 0.0),
            rigid_contact_relaxation=solver_config.get("rigid_contact_relaxation", 0.8),
            rigid_contact_con_weighting=solver_config.get("rigid_contact_con_weighting", True),
            angular_damping=solver_config.get("angular_damping", 0.0),
            enable_restitution=solver_config.get("enable_restitution", False),
        )

    elif solver_type == 'vbd':
        logger.info("Initializing vbd solver")
        return SolverVBD(
            model,
            iterations=solver_config.get("iterations", 10),
            friction_epsilon=solver_config.get("friction_epsilon", 1e-2),
            integrate_with_external_rigid_solver=solver_config.get("integrate_with_external_rigid_solver", False),
            # Particle parameters
            particle_enable_self_contact=solver_config.get("particle_enable_self_contact", False),
            particle_self_contact_radius=solver_config.get("particle_self_contact_radius", 0.2),
            particle_self_contact_margin=solver_config.get("particle_self_contact_margin", 0.2),
            particle_conservative_bound_relaxation=solver_config.get("particle_conservative_bound_relaxation", 0.85),
            particle_vertex_contact_buffer_size=solver_config.get("particle_vertex_contact_buffer_size", 32),
            particle_edge_contact_buffer_size=solver_config.get("particle_edge_contact_buffer_size", 64),
            particle_collision_detection_interval=solver_config.get("particle_collision_detection_interval", 0),
            particle_edge_parallel_epsilon=solver_config.get("particle_edge_parallel_epsilon", 1e-5),
            particle_enable_tile_solve=solver_config.get("particle_enable_tile_solve", True),
            particle_topological_contact_filter_threshold=solver_config.get("particle_topological_contact_filter_threshold", 2),
            particle_rest_shape_contact_exclusion_radius=solver_config.get("particle_rest_shape_contact_exclusion_radius", 0.0),
            particle_external_vertex_contact_filtering_map=solver_config.get("particle_external_vertex_contact_filtering_map", None),
            particle_external_edge_contact_filtering_map=solver_config.get("particle_external_edge_contact_filtering_map", None),
            # Rigid body parameters
            rigid_avbd_beta=solver_config.get("rigid_avbd_beta", 1.0e5),
            rigid_avbd_gamma=solver_config.get("rigid_avbd_gamma", 0.99),
            rigid_contact_k_start=solver_config.get("rigid_contact_k_start", 1.0e2),
            rigid_joint_linear_k_start=solver_config.get("rigid_joint_linear_k_start", 1.0e4),
            rigid_joint_angular_k_start=solver_config.get("rigid_joint_angular_k_start", 1.0e1),
            rigid_joint_linear_ke=solver_config.get("rigid_joint_linear_ke", 1.0e9),
            rigid_joint_angular_ke=solver_config.get("rigid_joint_angular_ke", 1.0e9),
            rigid_joint_linear_kd=solver_config.get("rigid_joint_linear_kd", 1.0e-2),
            rigid_joint_angular_kd=solver_config.get("rigid_joint_angular_kd", 0.0),
            rigid_body_contact_buffer_size=solver_config.get("rigid_body_contact_buffer_size", 64),
            rigid_body_particle_contact_buffer_size=solver_config.get("rigid_body_particle_contact_buffer_size", 256),
            rigid_enable_dahl_friction=solver_config.get("rigid_enable_dahl_friction", False),
        )

    elif solver_type == 'style3d':
        logger.info("Initializing style3d solver")
        return SolverStyle3D(
            model,
            iterations=solver_config.get("iterations", 10),
            linear_iterations=solver_config.get("linear_iterations", 10),
            drag_spring_stiff=solver_config.get("drag_spring_stiff", 1e2),
            enable_mouse_dragging=solver_config.get("enable_mouse_dragging", False),
        )

    elif solver_type == 'implicit_mpm':
        logger.info("Initializing Implicit MPM solver")
        impl_mpm_solver_config = SolverImplicitMPM.Config()
        impl_mpm_solver_config.max_iterations = solver_config.get("max_iterations", 250)
        impl_mpm_solver_config.tolerance = solver_config.get("tolerance", 1.0e-4)
        impl_mpm_solver_config.solver = solver_config.get("solver", "gauss-seidel")
        impl_mpm_solver_config.warmstart_mode = solver_config.get("warmstart_mode", "auto")
        impl_mpm_solver_config.collider_velocity_mode = solver_config.get("collider_velocity_mode", "forward")

        # grid
        impl_mpm_solver_config.voxel_size = solver_config.get("voxel_size", 0.1)
        impl_mpm_solver_config.grid_type = solver_config.get("grid_type", "sparse")
        impl_mpm_solver_config.grid_padding = solver_config.get("grid_padding", 0)
        impl_mpm_solver_config.max_active_cell_count = solver_config.get("max_active_cell_count", -1)
        impl_mpm_solver_config.transfer_scheme = solver_config.get("transfer_scheme", "apic")
        impl_mpm_solver_config.integration_scheme = solver_config.get("integration_scheme", "pic")

        # material / background
        impl_mpm_solver_config.critical_fraction = solver_config.get("critical_fraction", 0.0)
        impl_mpm_solver_config.air_drag = solver_config.get("air_drag", 1.0)

        # experimental
        impl_mpm_solver_config.collider_normal_from_sdf_gradient = solver_config.get("collider_normal_from_sdf_gradient", False)
        impl_mpm_solver_config.collider_basis = solver_config.get("collider_basis", "Q1")
        impl_mpm_solver_config.strain_basis = solver_config.get("strain_basis", "P0")
        impl_mpm_solver_config.velocity_basis = solver_config.get("velocity_basis", "Q1")
        return SolverImplicitMPM(model, config=impl_mpm_solver_config)

    elif solver_type == 'kamino':
        logger.info("Initializing Kamino solver")
        kaminio_solver_config = SolverKamino.Config()
        kaminio_solver_config.sparse_jacobian = solver_config.get("sparse_jacobian", False)
        kaminio_solver_config.sparse_dynamics = solver_config.get("sparse_dynamics", False)
        kaminio_solver_config.use_collision_detector = solver_config.get("use_collision_detector", False)
        kaminio_solver_config.use_fk_solver = solver_config.get("use_fk_solver", False)

        kaminio_solver_config.collision_detector = solver_config.get("collision_detector", None)
        kaminio_solver_config.constraints = solver_config.get("constraints", None)
        kaminio_solver_config.dynamics = solver_config.get("dynamics", None)
        kaminio_solver_config.padmm = solver_config.get("padmm", None)
        kaminio_solver_config.fk = solver_config.get("fk", None)

        kaminio_solver_config.rotation_correction = solver_config.get("rotation_correction", "twopi")
        kaminio_solver_config.integrator = solver_config.get("integrator", "euler")
        kaminio_solver_config.angular_velocity_damping = solver_config.get("angular_velocity_damping", 0.0)

        kaminio_solver_config.collect_solver_info = solver_config.get("collect_solver_info", False)
        kaminio_solver_config.compute_solution_metrics = solver_config.get("compute_solution_metrics", False)
        return SolverKamino(model, config=kaminio_solver_config)

    else:
        logger.warning(f"Unknown solver type '{solver_type}', defaulting to 'featherstone'")
        if is_soft:
            return None
        return SolverFeatherstone(model)

def make_collider(model, collider_config):
    return newton.CollisionPipeline(
        model,
        reduce_contacts=collider_config.get("reduce_contacts", True),
        rigid_contact_max=collider_config.get("rigid_contact_max", None),
        max_triangle_pairs=collider_config.get("max_triangle_pairs", 1000000),
        shape_pairs_filtered=collider_config.get("shape_pairs_filtered", None),
        soft_contact_max=collider_config.get("soft_contact_max", None),
        soft_contact_margin=collider_config.get("soft_contact_margin", None),
        requires_grad=collider_config.get("requires_grad", None),
        broad_phase=collider_config.get("broad_phase", None),
    )
    
def make_shape_cfg(cfg_dict):
    if cfg_dict is None:
        return None
    return newton.ModelBuilder.ShapeConfig(
        density=cfg_dict.get("density", 1000.0),
        ke=cfg_dict.get("ke", 2.5e3),
        kd=cfg_dict.get("kd", 100.0),
        kf=cfg_dict.get("kf", 1000.0),
        ka=cfg_dict.get("ka", 0.0),
        mu=cfg_dict.get("mu", 1.0),
        restitution=cfg_dict.get("restitution", 0.0),
        mu_torsional=cfg_dict.get("mu_torsional", 0.005),
        mu_rolling=cfg_dict.get("mu_rolling", 0.0001),
        margin=cfg_dict.get("margin", 0.0),
        gap=cfg_dict.get("gap", None),
        is_solid=cfg_dict.get("is_solid", True),
        collision_group=cfg_dict.get("collision_group", 1),
        collision_filter_parent=cfg_dict.get("collision_filter_parent", True),
        has_shape_collision=cfg_dict.get("has_shape_collision", True),
        has_particle_collision=cfg_dict.get("has_particle_collision", True),
        is_visible=cfg_dict.get("is_visible", True),
        is_site=cfg_dict.get("is_site", False),
        sdf_narrow_band_range=tuple(cfg_dict.get("sdf_narrow_band_range", (-0.1, 0.1))),
        sdf_target_voxel_size=cfg_dict.get("sdf_target_voxel_size", None),
        sdf_max_resolution=cfg_dict.get("sdf_max_resolution", None),
        sdf_texture_format=cfg_dict.get("sdf_texture_format", "uint16"),
        is_hydroelastic=cfg_dict.get("is_hydroelastic", False),
        kh=cfg_dict.get("kh", 1.0e10),
    )

def make_xform(entity_config):
    pos_offset = wp.vec3(entity_config.get("pos", (0, 0, 0)))
    if entity_config.get("quat"):
        quat_offset=entity_config.get("quat")
    else:
        if entity_config.get("euler"):
            quat_offset=q = wp.quat_from_euler(wp.vec3f(entity_config.get("euler")), 0, 1, 2)
        else:
            quat_offset=wp.quat_identity()
    return wp.transform(p=pos_offset,q=quat_offset)
    
def make_terrain_mesh(terrain_cfg):
    # --- core solver_config ---
    grid_size = tuple(terrain_cfg.get("grid_size", (1, 1)))
    block_size = tuple(terrain_cfg.get("block_size", (1.0, 1.0)))

    terrain_types = terrain_cfg.get("terrain_types", ["flat"])

    # --- terrain solver_config (per-type dict) ---
    terrain_solver_config = terrain_cfg.get("terrain_solver_config", {})

    # normalize tuples (yaml lists → tuples where needed)
    for k, v in terrain_solver_config.items():
        if isinstance(v, dict):
            for kk, vv in v.items():
                if isinstance(vv, list):
                    v[kk] = tuple(vv)

    # --- optional globals ---
    seed = terrain_cfg.get("seed", None)
    horizontal_scale = terrain_cfg.get("horizontal_scale", None)
    vertical_scale = terrain_cfg.get("vertical_scale", None)
    compute_inertia = terrain_cfg.get("compute_inertia", False)

    # --- build kwargs cleanly ---
    kwargs = dict(
        grid_size=grid_size,
        block_size=block_size,
        terrain_types=terrain_types,
        terrain_solver_config=terrain_solver_config,
        compute_inertia=compute_inertia,
    )

    if seed is not None:
        kwargs["seed"] = seed
    if horizontal_scale is not None:
        kwargs["horizontal_scale"] = horizontal_scale
    if vertical_scale is not None:
        kwargs["vertical_scale"] = vertical_scale

    # --- create terrain ---
    terrain_obj = newton.Mesh.create_terrain(**kwargs)

    return terrain_obj

def extract_common_entity_kwargs(entity_config):
    """
    Extract arguments common to URDF, MJCF, and USD model builders.
    """
    return {
        "source": entity_config["source"],
        "xform": make_xform(entity_config),
        "floating": entity_config.get("floating", None),
        "base_joint": entity_config.get("base_joint", None),
        "parent_body": entity_config.get("parent_body", -1),
        "force_show_colliders": entity_config.get("force_show_colliders", False),
        "enable_self_collisions": entity_config.get("enable_self_collisions", True),
        "collapse_fixed_joints": entity_config.get("collapse_fixed_joints", False),
        "mesh_maxhullvert": entity_config.get("mesh_maxhullvert", None),
        "override_root_xform": entity_config.get("override_root_xform", False),
    }

def add_usd(builder, entity_config):
    builder.add_usd(
        only_load_enabled_rigid_bodies=entity_config.get("only_load_enabled_rigid_bodies", False),
        only_load_enabled_joints=entity_config.get("only_load_enabled_joints", True),
        joint_drive_gains_scaling=entity_config.get("joint_drive_gains_scaling", 1.0),
        verbose=entity_config.get("verbose", False),
        ignore_paths=entity_config.get("ignore_paths", None),
        apply_up_axis_from_stage=entity_config.get("apply_up_axis_from_stage", False),
        root_path=entity_config.get("root_path", "/"),
        joint_ordering=entity_config.get("joint_ordering", "dfs"),
        bodies_follow_joint_ordering=entity_config.get("bodies_follow_joint_ordering", True),
        skip_mesh_approximation=entity_config.get("skip_mesh_approximation", False),
        load_sites=entity_config.get("load_sites", True),
        load_visual_shapes=entity_config.get("load_visual_shapes", True),
        hide_collision_shapes=entity_config.get("hide_collision_shapes", False),
        parse_mujoco_options=entity_config.get("parse_mujoco_options", True),
        schema_resolvers=entity_config.get("schema_resolvers", None),
        force_position_velocity_actuation=entity_config.get("force_position_velocity_actuation", False),
        **extract_common_entity_kwargs(entity_config)
    )
    
def add_mjcf(builder, entity_config):
    builder.add_mjcf(
        armature_scale=entity_config.get("armature_scale", 1.0),
        scale=entity_config.get("scale", 1.0),
        hide_visuals=entity_config.get("hide_visuals", False),
        parse_visuals_as_colliders=entity_config.get("parse_visuals_as_colliders", False),
        parse_meshes=entity_config.get("parse_meshes", True),
        parse_sites=entity_config.get("parse_sites", True),
        parse_visuals=entity_config.get("parse_visuals", True),
        parse_mujoco_options=entity_config.get("parse_mujoco_options", True),
        up_axis=entity_config.get("up_axis", "Z"),
        ignore_names=entity_config.get("ignore_names", []),
        ignore_classes=entity_config.get("ignore_classes", []),
        visual_classes=entity_config.get("visual_classes", ["visual"]),
        collider_classes=entity_config.get("collider_classes", ["collision"]),
        no_class_as_colliders=entity_config.get("no_class_as_colliders", True),
        ignore_inertial_definitions=entity_config.get("ignore_inertial_definitions", False),
        verbose=entity_config.get("verbose", False),
        skip_equality_constraints=entity_config.get("skip_equality_constraints", False),
        convert_3d_hinge_to_ball_joints=entity_config.get("convert_3d_hinge_to_ball_joints", False),
        ctrl_direct=entity_config.get("ctrl_direct", False),
        path_resolver=entity_config.get("path_resolver", None),
        **extract_common_entity_kwargs(entity_config)
    )

def add_urdf(builder, entity_config):
    builder.add_urdf(
        scale=entity_config.get("scale", 1.0),
        hide_visuals=entity_config.get("hide_visuals", False),
        parse_visuals_as_colliders=entity_config.get("parse_visuals_as_colliders", False),
        up_axis=entity_config.get("up_axis", "Z"),
        ignore_inertial_definitions=entity_config.get("ignore_inertial_definitions", False),
        joint_ordering=entity_config.get("joint_ordering", "dfs"),
        bodies_follow_joint_ordering=entity_config.get("bodies_follow_joint_ordering", True),
        force_position_velocity_actuation=entity_config.get("force_position_velocity_actuation", False),
        **extract_common_entity_kwargs(entity_config)
    )

def add_primitive_ellipsoid(builder, entity_config):
    if (entity_config or {}).get("cfg", {}).get("is_soft", False):
        surface_mesh = newton.Mesh.create_ellipsoid(
            rx=entity_config.get("rx", 1.0),
            ry=entity_config.get("ry", 0.75),
            rz=entity_config.get("rz", 0.5))
        #Tetrahedralize the surface mesh 
        tet_vertices, tet_faces = remesh_ftetwild(  
            surface_mesh.vertices,  
            surface_mesh.indices.reshape(-1, 3),  
            edge_length_fac=0.1,  # controls tet density  
        )    
        tet_mesh = newton.TetMesh(tet_vertices, tet_faces.flatten())  
        xform=make_xform(entity_config)
        # Add as softbody
        soft_cfg= entity_config.get("cfg", {})
        add_soft_mesh(builder, tet_mesh, soft_cfg, xform)
    else:
        if not entity_config.get("fixed", True):
            body=builder.add_body(
                xform=make_xform(entity_config), 
                label=entity_config.get("label", None))
            xform=None
            label=None
        else:
            body=entity_config.get("body",-1)
            xform=make_xform(entity_config)
            label=entity_config.get("label", None)
        builder.add_shape_ellipsoid(
            body=body,
            xform=xform,
            rx=entity_config.get("rx", 1.0),
            ry=entity_config.get("ry", 0.75),
            rz=entity_config.get("rz", 0.5),
            cfg=make_shape_cfg(entity_config.get("cfg", None)),
            as_site=entity_config.get("as_site", False),
            color=entity_config.get("color", None),
            label=label,
            custom_attributes=entity_config.get("custom_attributes", None),
        )
    
def add_primitive_box(builder, entity_config):
    if (entity_config or {}).get("cfg", {}).get("is_soft", False):
        surface_mesh = newton.Mesh.create_box(
            hx=entity_config.get("hx", 0.5),
            hy=entity_config.get("hy", 0.5),
            hz=entity_config.get("hz", 0.5))
        #Tetrahedralize the surface mesh 
        tet_vertices, tet_faces = remesh_ftetwild(  
            surface_mesh.vertices,  
            surface_mesh.indices.reshape(-1, 3),  
            edge_length_fac=0.1,  # controls tet density  
        )    
        tet_mesh = newton.TetMesh(tet_vertices, tet_faces.flatten())  
        xform=make_xform(entity_config)
        # Add as softbody
        soft_cfg= entity_config.get("cfg", {})
        add_soft_mesh(builder, tet_mesh, soft_cfg, xform)
    else:
        if not entity_config.get("fixed", True):
            body=builder.add_body(
                xform=make_xform(entity_config), 
                label=entity_config.get("label", None))
            xform=None
            label=None
        else:
            body=entity_config.get("body",-1)
            xform=make_xform(entity_config)
            label=entity_config.get("label", None)
        builder.add_shape_box(
            body=body,
            xform=xform,
            hx=entity_config.get("hx", 0.5),
            hy=entity_config.get("hy", 0.5),
            hz=entity_config.get("hz", 0.5),
            cfg=make_shape_cfg(entity_config.get("cfg", None)),
            as_site=entity_config.get("as_site", False),
            color=entity_config.get("color", None),
            label=label,
            custom_attributes=entity_config.get("custom_attributes", None),
        )
    
def add_primitive_capsule(builder, entity_config):
    if (entity_config or {}).get("cfg", {}).get("is_soft", False):
        surface_mesh = newton.Mesh.create_capsule(
            radius=entity_config.get("radius", 1.0),
            half_height=entity_config.get("half_height", 1.0))  
        #Tetrahedralize the surface mesh 
        tet_vertices, tet_indices = tetrahedralize(  
            surface_mesh.vertices,  
            surface_mesh.indices.reshape(-1, 3),  
            edge_length_fac=0.1,  
        )   
        tet_mesh = newton.TetMesh(tet_vertices, tet_indices.flatten())    
        xform=make_xform(entity_config)
        # Add as softbody
        soft_cfg= entity_config.get("cfg", {})
        add_soft_mesh(builder, tet_mesh, soft_cfg, xform)
    else:
        if not entity_config.get("fixed", True):
            body=builder.add_body(
                xform=make_xform(entity_config), 
                label=entity_config.get("label", None))
            xform=None
            label=None
        else:
            body=entity_config.get("body",-1)
            xform=make_xform(entity_config)
            label=entity_config.get("label", None)
        builder.add_shape_capsule(
            body=body,
            xform=xform,
            radius=entity_config.get("radius", 1.0),
            half_height=entity_config.get("half_height", 0.5),
            cfg=make_shape_cfg(entity_config.get("cfg", None)),
            as_site=entity_config.get("as_site", False),
            color=entity_config.get("color", None),
            label=label,
            custom_attributes=entity_config.get("custom_attributes", None),
        )
    
def add_primitive_cylinder(builder, entity_config):
    if (entity_config or {}).get("cfg", {}).get("is_soft", False):
        surface_mesh = newton.Mesh.create_cylinder(
            radius=entity_config.get("radius", 1.0),
            half_height=entity_config.get("half_height", 1.0))  
        #Tetrahedralize the surface mesh 
        tet_vertices, tet_faces = remesh_ftetwild(  
            surface_mesh.vertices,  
            surface_mesh.indices.reshape(-1, 3),  
            edge_length_fac=0.1,  # controls tet density  
        )    
        tet_mesh = newton.TetMesh(tet_vertices, tet_faces.flatten())  
        xform=make_xform(entity_config)
        # Add as softbody
        soft_cfg= entity_config.get("cfg", {})
        add_soft_mesh(builder, tet_mesh, soft_cfg, xform)
    else:
        if not entity_config.get("fixed", True):
            body=builder.add_body(
                xform=make_xform(entity_config), 
                label=entity_config.get("label", None))
            xform=None
            label=None
        else:
            body=entity_config.get("body",-1)
            xform=make_xform(entity_config)
            label=entity_config.get("label", None)
        builder.add_shape_cylinder(
            body=body,
            xform=xform,
            radius=entity_config.get("radius", 1.0),
            half_height=entity_config.get("half_height", 0.5),
            cfg=make_shape_cfg(entity_config.get("cfg", None)),
            as_site=entity_config.get("as_site", False),
            color=entity_config.get("color", None),
            label=label,
            custom_attributes=entity_config.get("custom_attributes", None),
        )
    
def add_primitive_cone(builder, entity_config):
    if (entity_config or {}).get("cfg", {}).get("is_soft", False):
        surface_mesh = newton.Mesh.create_cone(
            radius=entity_config.get("radius", 1.0),
            half_height=entity_config.get("half_height", 1.0))  
        #Tetrahedralize the surface mesh 
        tet_vertices, tet_faces = remesh_ftetwild(  
            surface_mesh.vertices,  
            surface_mesh.indices.reshape(-1, 3),  
            edge_length_fac=0.1,  # controls tet density  
        )    
        tet_mesh = newton.TetMesh(tet_vertices, tet_faces.flatten())  
        xform=make_xform(entity_config)
        # Add as softbody
        soft_cfg= entity_config.get("cfg", {})
        add_soft_mesh(builder, tet_mesh, soft_cfg, xform)
    else:
        if not entity_config.get("fixed", True):
            body=builder.add_body(
                xform=make_xform(entity_config), 
                label=entity_config.get("label", None))
            xform=None
            label=None
        else:
            body=entity_config.get("body",-1)
            xform=make_xform(entity_config)
            label=entity_config.get("label", None)
        builder.add_shape_cone(
            body=body,
            xform=xform,
            radius=entity_config.get("radius", 1.0),
            half_height=entity_config.get("half_height", 0.5),
            cfg=make_shape_cfg(entity_config.get("cfg", None)),
            as_site=entity_config.get("as_site", False),
            color=entity_config.get("color", None),
            label=label,
            custom_attributes=entity_config.get("custom_attributes", None),
        )
    
def add_primitive_sphere(builder, entity_config):
    if (entity_config or {}).get("cfg", {}).get("is_soft", False):
        surface_mesh = newton.Mesh.create_sphere(
            radius=entity_config.get("radius", 1.0))  
        #Tetrahedralize the surface mesh 
        tet_vertices, tet_faces = remesh_ftetwild(  
            surface_mesh.vertices,  
            surface_mesh.indices.reshape(-1, 3),  
            edge_length_fac=0.1,  # controls tet density  
        )    
        tet_mesh = newton.TetMesh(tet_vertices, tet_faces.flatten())  
        xform=make_xform(entity_config)
        # Add as softbody
        soft_cfg= entity_config.get("cfg", {})
        add_soft_mesh(builder, tet_mesh, soft_cfg, xform)
    else:
        if not entity_config.get("fixed", True):  
            body = builder.add_body(  
                xform=make_xform(entity_config),   
                label=entity_config.get("label", None))  
            # For dynamic bodies, shape transform should be identity (centered on body)  
            xform = wp.transform()  
            label = None  
        else:  
            body = entity_config.get("body", -1)  
            xform = make_xform(entity_config)  
            label = entity_config.get("label", None)  
        builder.add_shape_sphere(
            body=body,
            xform=xform,
            radius=entity_config.get("radius", 1.0),
            cfg=make_shape_cfg(entity_config.get("cfg", None)),
            as_site=entity_config.get("as_site", False),
            color=entity_config.get("color", None),
            label=label,
            custom_attributes=entity_config.get("custom_attributes", None),
        )
    
def add_ground_plane(builder, entity_config):
    builder.add_ground_plane(
        height=entity_config.get("height", 0.0),
        cfg=make_shape_cfg(entity_config.get("cfg", None)),
        color=entity_config.get("color", (0.125, 0.125, 0.15)),
        label=entity_config.get("label", None),
    )

def add_soft_mesh(builder, tet_mesh, soft_cfg, xform):
    builder.add_soft_mesh(  
        pos=xform.p,  
        rot=xform.q,
        vel=(0.0,0.0,0.0),  
        scale=soft_cfg.get("scale", 1.0),    
        mesh=tet_mesh,  
        density=soft_cfg.get("density", None),  
        k_mu=soft_cfg.get("k_mu", None),  
        k_lambda=soft_cfg.get("k_lambda", None),  
        k_damp=soft_cfg.get("k_damp", None),
        tri_ke= soft_cfg.get("tri_ke", 0.0),
        tri_ka= soft_cfg.get("tri_ka", 0.0),
        tri_kd= soft_cfg.get("tri_kd", 0.0),
        tri_drag= soft_cfg.get("tri_drag", 0.0),
        tri_lift= soft_cfg.get("tri_lift", 0.0),
        add_surface_mesh_edges= soft_cfg.get("add_surface_mesh_edges", True),
        edge_ke= soft_cfg.get("edge_ke", 0.0),
        edge_kd= soft_cfg.get("edge_kd", 0.0),
        particle_radius= soft_cfg.get("particle_radius", None),  
    )

def calculate_bounds(cfg):
    return [
        range(int(-50), int(50)),
        range(int(-50), int(50)),
        range(int(0), 100),
    ]
        

        
def add_entities_info(entities_info, entity_name, entity_config, initialisation_pending=False):
    if entities_info is not None:
        entities_info[entity_name] = {}
        entities_info[entity_name]["initial_pos"] = entity_config.get(
            "pos", [0, 0, 0]
        )
        entities_info[entity_name]["initial_quat"] = entity_config.get(
            "quat", [1, 0, 0, 0]
        )
        entities_info[entity_name]["resource"] = entity_config.get(
            "source", None
        )
        entities_info[entity_name]["namespace"] = entity_config.get(
            "namespace", None
        )
        entities_info[entity_name]["initialisation_pending"] = initialisation_pending
        entities_info[entity_name]["description"] = entity_config.get(
            "description", ""
        )
        entities_info[entity_name]["tags"] = entity_config.get("tags", [])
        entities_info[entity_name]["category"] = entity_config.get(
            "category", 1
        )
            
def modify_builder_config(builder, cfg: dict):
    """
    Apply overrides from a config dict to a builder's default config objects.
    Only the provided keys in the config dict are applied.
    """

    # --- SHAPE CONFIG ---
    # Get YAML-provided shape overrides
    shape_yaml = cfg.get("default_shape_cfg", {})

    # Grab builder's current default_shape_cfg
    shape_cfg = builder.default_shape_cfg

    # Apply overrides if present
    if "density" in shape_yaml:
        shape_cfg.density = float(shape_yaml["density"])
    if "ke" in shape_yaml:
        shape_cfg.ke = float(shape_yaml["ke"])
    if "kd" in shape_yaml:
        shape_cfg.kd = float(shape_yaml["kd"])
    if "kf" in shape_yaml:
        shape_cfg.kf = float(shape_yaml["kf"])
    if "ka" in shape_yaml:
        shape_cfg.ka = float(shape_yaml["ka"])

    if "mu" in shape_yaml:
        shape_cfg.mu = float(shape_yaml["mu"])
    if "restitution" in shape_yaml:
        shape_cfg.restitution = float(shape_yaml["restitution"])
    if "mu_torsional" in shape_yaml:
        shape_cfg.mu_torsional = float(shape_yaml["mu_torsional"])
    if "mu_rolling" in shape_yaml:
        shape_cfg.mu_rolling = float(shape_yaml["mu_rolling"])

    if "margin" in shape_yaml:
        shape_cfg.margin = float(shape_yaml["margin"])
    if "gap" in shape_yaml:
        raw = shape_yaml["gap"]
        shape_cfg.gap = None if raw is None else float(raw)

    if "is_solid" in shape_yaml:
        shape_cfg.is_solid = bool(shape_yaml["is_solid"])
    if "collision_group" in shape_yaml:
        shape_cfg.collision_group = int(shape_yaml["collision_group"])
    if "collision_filter_parent" in shape_yaml:
        shape_cfg.collision_filter_parent = bool(shape_yaml["collision_filter_parent"])
    if "has_shape_collision" in shape_yaml:
        shape_cfg.has_shape_collision = bool(shape_yaml["has_shape_collision"])
    if "has_particle_collision" in shape_yaml:
        shape_cfg.has_particle_collision = bool(shape_yaml["has_particle_collision"])

    if "is_visible" in shape_yaml:
        shape_cfg.is_visible = bool(shape_yaml["is_visible"])
    if "is_site" in shape_yaml:
        shape_cfg.is_site = bool(shape_yaml["is_site"])

    if "sdf_narrow_band_range" in shape_yaml:
        shape_cfg.sdf_narrow_band_range = tuple(shape_yaml["sdf_narrow_band_range"])
    if "sdf_target_voxel_size" in shape_yaml:
        raw = shape_yaml["sdf_target_voxel_size"]
        shape_cfg.sdf_target_voxel_size = None if raw is None else float(raw)
    if "sdf_max_resolution" in shape_yaml:
        raw = shape_yaml["sdf_max_resolution"]
        shape_cfg.sdf_max_resolution = None if raw is None else int(raw)
    if "sdf_texture_format" in shape_yaml:
        shape_cfg.sdf_texture_format = str(shape_yaml["sdf_texture_format"])

    if "is_hydroelastic" in shape_yaml:
        shape_cfg.is_hydroelastic = bool(shape_yaml["is_hydroelastic"])
    if "kh" in shape_yaml:
        shape_cfg.kh = float(shape_yaml["kh"])

    # Assign back to builder
    builder.default_shape_cfg = shape_cfg

    # --- JOINT CONFIG ---
    joint_yaml = cfg.get("default_joint_cfg", {})

    joint_cfg = builder.default_joint_cfg

    if "axis" in joint_yaml:
        joint_cfg.axis = tuple(joint_yaml["axis"])
    if "limit_lower" in joint_yaml:
        joint_cfg.limit_lower = float(joint_yaml["limit_lower"])
    if "limit_upper" in joint_yaml:
        joint_cfg.limit_upper = float(joint_yaml["limit_upper"])
    if "limit_ke" in joint_yaml:
        joint_cfg.limit_ke = float(joint_yaml["limit_ke"])
    if "limit_kd" in joint_yaml:
        joint_cfg.limit_kd = float(joint_yaml["limit_kd"])

    if "target_pos" in joint_yaml:
        joint_cfg.target_pos = float(joint_yaml["target_pos"])
    if "target_vel" in joint_yaml:
        joint_cfg.target_vel = float(joint_yaml["target_vel"])
    if "target_ke" in joint_yaml:
        joint_cfg.target_ke = float(joint_yaml["target_ke"])
    if "target_kd" in joint_yaml:
        joint_cfg.target_kd = float(joint_yaml["target_kd"])

    if "armature" in joint_yaml:
        joint_cfg.armature = float(joint_yaml["armature"])
    if "effort_limit" in joint_yaml:
        joint_cfg.effort_limit = float(joint_yaml["effort_limit"])
    if "velocity_limit" in joint_yaml:
        joint_cfg.velocity_limit = float(joint_yaml["velocity_limit"])
    if "friction" in joint_yaml:
        joint_cfg.friction = float(joint_yaml["friction"])

    if "actuator_mode" in joint_yaml:
        joint_cfg.actuator_mode = str(joint_yaml["actuator_mode"])

    # Assign back to builder
    builder.default_joint_cfg = joint_cfg

@wp.kernel
def transform_points(
    points_in: wp.array(dtype=wp.vec3),
    points_out: wp.array(dtype=wp.vec3),
    tf: wp.transform,
):
    i = wp.tid()
    points_out[i] = wp.transform_point(tf, points_in[i])

@wp.kernel
def add_gaussian_noise_f32_3d(
    data: wp.array(dtype=wp.float32, ndim=3),
    mean: wp.float32,
    std: wp.float32,
    seed: wp.int32,
):
    i, j, k = wp.tid()

    flat_i = (((i * data.shape[1] + j) * data.shape[2] + k) * data.shape[3])
    state = wp.rand_init(seed, flat_i)

    data[i, j, k] += mean + std * wp.randn(state)


@wp.kernel
def add_gaussian_noise_f32_4d(
    data: wp.array(dtype=wp.float32, ndim=4),
    mean: wp.float32,
    std: wp.float32,
    seed: wp.int32,
):
    i, j, k, l = wp.tid()

    flat_i = (((i * data.shape[1] + j) * data.shape[2] + k) * data.shape[3] + l)
    state = wp.rand_init(seed, flat_i)

    data[i, j, k, l] += mean + std * wp.randn(state)


@wp.kernel
def add_gaussian_noise_uint32_4d(
    data: wp.array(dtype=wp.uint32, ndim=4),
    mean: wp.float32,
    std: wp.float32,
    seed: wp.int32,
):
    i, j, k, l = wp.tid()

    flat_i = (((i * data.shape[1] + j) * data.shape[2] + k) * data.shape[3] + l)
    state = wp.rand_init(seed, flat_i)

    noise = mean + std * wp.randn(state)
    val = wp.float32(data[i, j, k, l]) + noise

    val = wp.clamp(val, 0.0, 4294967295.0)
    data[i, j, k, l] = wp.uint32(val)

@wp.kernel
def add_gaussian_noise_vec3_1d(
    data: wp.array(dtype=wp.vec3, ndim=1),
    mean: wp.float32,
    std: wp.float32,
    seed: wp.int32,
):
    i = wp.tid()

    state = wp.rand_init(seed, i)

    data[i] += wp.vec3(
        mean + std * wp.randn(state),
        mean + std * wp.randn(state),
        mean + std * wp.randn(state),
    )

@wp.kernel
def add_gaussian_noise_vec3_4d(
    data: wp.array(dtype=wp.vec3, ndim=4),
    mean: wp.float32,
    std: wp.float32,
    seed: wp.int32,
):
    i, j, k, l = wp.tid()

    flat_i = (((i * data.shape[1] + j) * data.shape[2] + k) * data.shape[3] + l)
    state = wp.rand_init(seed, flat_i)

    data[i, j, k, l] += wp.vec3(
        mean + std * wp.randn(state),
        mean + std * wp.randn(state),
        mean + std * wp.randn(state),
    )