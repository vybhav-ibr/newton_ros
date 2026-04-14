import math
import numpy as np
import warp as wp
import newton
from newton.sensors import SensorRaycast


# ─────────────────────────────────────────────────────────────────────
# Kernel 1: Equirectangular reprojection → LOCAL sensor frame
# ─────────────────────────────────────────────────────────────────────

@wp.kernel
def equirect_reproject_kernel(
    depth:        wp.array3d(dtype=float),
    face_fwd:     wp.array(dtype=wp.vec3),   # world-space face forward vectors
    face_up:      wp.array(dtype=wp.vec3),   # world-space face up vectors
    face_right:   wp.array(dtype=wp.vec3),   # world-space face right vectors
    cam_quat:     wp.quat,                   # Local → World rotation of the sensor
    fov_scale:    float,                     # tan(face_fov / 2), = 1.0 for 90°
    face_res:     int,                       # pixel resolution of each cube face
    eq_width:     int,                       # equirect horizontal resolution (hres)
    eq_height:    int,                       # equirect vertical resolution   (vres)
    horiz_fov:    float,                     # horizontal FOV in radians
    vert_fov:     float,                     # vertical   FOV in radians
    min_dist:     float,                     #Minimum distance, avoid undesired intersections
    # ── outputs ──────────────────────────────────────────────────────
    eq_points:    wp.array(dtype=wp.vec3),   # 3-D hit points  in LOCAL frame
    eq_distances: wp.array(dtype=float),     # Euclidean distance to each hit
    eq_hit_mask:  wp.array(dtype=wp.int32),  # 1 = valid hit, 0 = miss
    eq_rays:      wp.array(dtype=wp.vec3),   # unit ray dir    in LOCAL frame
):
    tid = wp.tid()
    col = tid % eq_width
    row = tid // eq_width

    # ── 1. Build unit ray in LOCAL sensor frame ───────────────────────
    # Convention: +X forward, +Y left, +Z up  (standard for lidar)
    theta = (float(col) + 0.5) / float(eq_width)  * horiz_fov - horiz_fov * 0.5
    phi   = vert_fov * 0.5 - (float(row) + 0.5) / float(eq_height) * vert_fov

    cos_phi = wp.cos(phi)
    ray_local = wp.normalize(wp.vec3(
        cos_phi * wp.cos(theta),
        cos_phi * wp.sin(theta),
        wp.sin(phi),
    ))
    eq_rays[tid] = ray_local          # store LOCAL ray for get_rays()

    # ── 2. Rotate ray into World space for cubemap lookup ─────────────
    ray_world = wp.quat_rotate(cam_quat, ray_local)

    # ── 3. Face selection (dot against world-space face forwards) ─────
    best_face = 0
    best_dot  = -1.0
    for f in range(6):
        d = wp.dot(ray_world, face_fwd[f])
        if d > best_dot:
            best_dot  = d
            best_face = f

    fwd_w   = face_fwd[best_face]
    up_w    = face_up[best_face]
    right_w = face_right[best_face]

    # ── 4. Project onto selected face → pixel index ───────────────────
    dot_fwd = wp.dot(ray_world, fwd_w)
    cam_x   = wp.dot(ray_world, right_w) / dot_fwd
    cam_y   = wp.dot(ray_world, up_w)    / dot_fwd

    ndc_x = cam_x / fov_scale
    ndc_y = cam_y / fov_scale

    res_f  = float(face_res)
    frac_x = (ndc_x + 1.0) * 0.5 * res_f - 0.5
    frac_y = (1.0  - ndc_y) * 0.5 * res_f - 0.5

    # Nearest-neighbour sampling (avoids depth bleeding)
    ix = int(wp.clamp(wp.round(frac_x), 0.0, res_f - 1.0))
    iy = int(wp.clamp(wp.round(frac_y), 0.0, res_f - 1.0))

    d_val = depth[best_face, iy, ix]
    eq_distances[tid] = d_val

    # ── 5. Perspective-correct reconstruction ─────────────────────────
    # Re-derive the exact world direction that pixel (ix, iy) was cast along.
    pix_ndc_x = (2.0 * float(ix) + 1.0) / res_f - 1.0
    pix_ndc_y = 1.0 - (2.0 * float(iy) + 1.0) / res_f

    persp_dir_w = wp.normalize(
        right_w * (pix_ndc_x * fov_scale)
      + up_w    * (pix_ndc_y * fov_scale)
      + fwd_w
    )

    # ── 6. Output in LOCAL frame ──────────────────────────────────────
    if d_val > min_dist:
        persp_dir_local = wp.quat_rotate_inv(cam_quat, persp_dir_w)
        eq_points[tid]   = persp_dir_local * d_val
        eq_distances[tid] = d_val
        eq_hit_mask[tid]  = 1
    else:
        eq_points[tid]    = wp.vec3(0.0, 0.0, 0.0)
        eq_distances[tid] = 0.0
        eq_hit_mask[tid]  = 0


# ─────────────────────────────────────────────────────────────────────
# Kernel 2: GPU stream compaction (atomic counter)
# ─────────────────────────────────────────────────────────────────────

@wp.kernel
def compact_hits_atomic(
    points:  wp.array(dtype=wp.vec3),
    mask:    wp.array(dtype=wp.int32),
    out_pts: wp.array(dtype=wp.vec3),
    counter: wp.array(dtype=wp.int32),
):
    tid = wp.tid()
    if mask[tid] == 1:
        idx = wp.atomic_add(counter, 0, 1)
        out_pts[idx] = points[tid]


import warp as wp

@wp.kernel
def compute_ray_endpoints(
    pts_local: wp.array(dtype=wp.vec3),
    rays_local: wp.array(dtype=wp.vec3),
    mask: wp.array(dtype=wp.int32),
    origin: wp.vec3,
    quat: wp.quat,
    max_range: float,
    ray_starts: wp.array(dtype=wp.vec3),
    ray_ends: wp.array(dtype=wp.vec3),
):
    i = wp.tid()

    # All rays start at origin
    ray_starts[i] = origin

    if mask[i] == 1:
        # Hit case: rotate local hit point and translate
        p_world = wp.quat_rotate(quat, pts_local[i])
        ray_ends[i] = p_world + origin
    else:
        # Miss case: rotate direction, scale by max range, translate
        r_world = wp.quat_rotate(quat, rays_local[i])
        ray_ends[i] = r_world * max_range + origin

# ─────────────────────────────────────────────────────────────────────
# Lidar class
# ─────────────────────────────────────────────────────────────────────

class Lidar:
    """
    Cubemap-based Lidar sensor for Newton physics simulations.

    All output (points, rays) is expressed in the LOCAL frame of the sensor,
    i.e. relative to the origin and orientation supplied to update().

    Parameters
    ----------
    model    : Newton ModelBuilder-finalised model.
    hfov     : Horizontal field of view in radians (default: 2π = 360°).
    vfov     : Vertical   field of view in radians (default: π/2 = 90°).
    hres     : Number of horizontal samples in the equirectangular grid.
    vres     : Number of vertical   samples in the equirectangular grid.
    max_dist : Maximum raycast range in metres.
    """

    # Standard 90° cubemap face directions expressed in the sensor-LOCAL frame.
    # Convention: +X forward, +Y left, +Z up.
    _BASE_FACES = [
        (np.array([ 1,  0,  0], np.float32), np.array([0,  0,  1], np.float32)),  # +X
        (np.array([-1,  0,  0], np.float32), np.array([0,  0,  1], np.float32)),  # -X
        (np.array([ 0,  1,  0], np.float32), np.array([0,  0,  1], np.float32)),  # +Y
        (np.array([ 0, -1,  0], np.float32), np.array([0,  0,  1], np.float32)),  # -Y
        (np.array([ 0,  0,  1], np.float32), np.array([0, -1,  0], np.float32)),  # +Z
        (np.array([ 0,  0, -1], np.float32), np.array([0,  1,  0], np.float32)),  # -Z
    ]

    def __init__(
        self,
        model,
        hfov: float = 2.0 * math.pi,
        vfov: float = math.pi / 2.0,
        hres: int   = 160,
        vres: int   = 80,
        min_range: float = 0.10,
        max_range: float = 50.0,
    ):
        self.model    = model
        self.device   = model.device
        self.hfov     = hfov
        self.vfov     = vfov
        self.hres     = hres
        self.vres     = vres
        self.max_range = max_range
        self.min_range = min_range
        self.N_eq     = hres * vres

        # ── Face resolution ───────────────────────────────────────────
        # Match the angular pixel density of the equirect grid so we
        # never under-sample the cube faces.
        # pixels_per_rad (horizontal) = hres / hfov
        # face spans π/2 rad → face_res = (hres / hfov) * (π/2)
        self.face_res  = max(1, int(round((hres / hfov) * (math.pi / 2.0))))
        self.face_fov  = math.pi / 2.0          # all faces are 90°
        self.fov_scale = math.tan(self.face_fov * 0.5)   # = 1.0 for 90°

        # ── Newton SensorRaycast instances (one per cube face) ────────
        # Initial orientations don't matter; update() overwrites them.
        self._sensors = [
            SensorRaycast(
                model,
                camera_position=np.zeros(3, np.float32),
                camera_direction=fwd.copy(),
                camera_up=up.copy(),
                fov_radians=self.face_fov,
                width=self.face_res,
                height=self.face_res,
                max_distance=max_range,
            )
            for fwd, up in self._BASE_FACES
        ]

        # ── GPU buffers (allocated once, reused every update) ─────────
        self._eq_points_wp = wp.zeros(self.N_eq, dtype=wp.vec3,  device=self.device)
        self._eq_dist_wp   = wp.zeros(self.N_eq, dtype=float,    device=self.device)
        self._eq_mask_wp   = wp.zeros(self.N_eq, dtype=wp.int32, device=self.device)
        self._eq_rays_wp   = wp.zeros(self.N_eq, dtype=wp.vec3,  device=self.device)
        self._counter_wp   = wp.zeros(1,         dtype=wp.int32, device=self.device)
        self._hits_wp      = wp.zeros(self.N_eq, dtype=wp.vec3,  device=self.device)

    # ── Private helpers ───────────────────────────────────────────────

    @staticmethod
    def _wp_vec3_to_np(v) -> np.ndarray:
        """Safely convert a wp.vec3 (or any 3-element sequence) to float32 ndarray."""
        return np.array([float(v[0]), float(v[1]), float(v[2])], dtype=np.float32)

    @staticmethod
    def _rotate_np_by_quat(q: wp.quat, v: np.ndarray) -> np.ndarray:
        """Rotate a numpy float32 vector by a warp quaternion; return numpy float32."""
        result = wp.quat_rotate(q, wp.vec3(float(v[0]), float(v[1]), float(v[2])))
        return np.array([float(result[0]), float(result[1]), float(result[2])], dtype=np.float32)

    # ── Public API ────────────────────────────────────────────────────

    def update(self, state, tf) -> None:
        """
        Fire all rays and update internal buffers.
        """
        pos_np = self._wp_vec3_to_np(tf.p)
        quat   = tf.q           

        fwd_list, up_list, right_list = [], [], []

        for i, (base_fwd_np, base_up_np) in enumerate(self._BASE_FACES):
            # Rotate each base face direction into world space
            w_fwd_np = self._rotate_np_by_quat(quat, base_fwd_np)
            w_up_np  = self._rotate_np_by_quat(quat, base_up_np)
            w_right_np = np.cross(w_fwd_np, w_up_np)

            self._sensors[i].update_camera_pose(  
                position=pos_np,  
                direction=w_fwd_np,  
                up=w_up_np,  
            )  
            self._sensors[i].update(state)

            fwd_list.append(wp.vec3(*w_fwd_np))  
            up_list.append(wp.vec3(*w_up_np))       
            right_list.append(wp.vec3(*w_right_np))

        fwd_wp   = wp.array(fwd_list,   dtype=wp.vec3, device=self.device)
        up_wp    = wp.array(up_list,    dtype=wp.vec3, device=self.device)
        right_wp = wp.array(right_list, dtype=wp.vec3, device=self.device)

        depth_np = np.stack([s.get_depth_image_numpy() for s in self._sensors], axis=0)
        depth_wp = wp.array(depth_np, dtype=float, device=self.device)

        # ─────────────────────────────────────────────────────────────────
        # Kernel Launch
        # ─────────────────────────────────────────────────────────────────
        # Combines inputs and outputs into a single list. Some newer versions 
        # of Warp silently drop the `outputs=` argument, leaving your arrays filled with 0s.
        wp.launch(
            equirect_reproject_kernel,
            dim=self.N_eq,
            inputs=[
                depth_wp,
                fwd_wp, up_wp, right_wp,
                quat,
                self.fov_scale,
                self.face_res,
                self.hres, self.vres,
                self.hfov, self.vfov,
                self.min_range,
                self._eq_points_wp,  # Moved outputs into the main inputs list
                self._eq_dist_wp,
                self._eq_mask_wp,
                self._eq_rays_wp,
            ],
            device=self.device,
        )
        self.num_hits = int(self._counter_wp.numpy()[0])
        self.num_invalid = self.N_eq - self.num_hits

    def get_points(self, valid_only: bool = True) -> wp.array:
        """
        3-D hit points in the LOCAL sensor frame.

        Parameters
        ----------
        valid_only : If True  → returns only the N_hits valid points (compact).
                     If False → returns the full hres×vres grid; misses are (0,0,0).
        """
        if not valid_only:
            return self._eq_points_wp

        self._counter_wp.zero_()
        wp.launch(
            compact_hits_atomic,
            dim=self.N_eq,
            inputs=[self._eq_points_wp, self._eq_mask_wp,
                    self._hits_wp, self._counter_wp],
            device=self.device,
        )
        num_hits = int(self._counter_wp.numpy()[0])
        if num_hits > 0:
            return self._hits_wp[:num_hits]
        return wp.array([], dtype=wp.vec3, device=self.device)

    def get_valid(self) -> wp.array:
        """
        Hit mask: wp.array of int32, length hres*vres.
        1 = valid return, 0 = miss / beyond max_dist.
        Updated by every call to update().
        """
        return self._eq_mask_wp

    def get_ray_visualization_data(self, tf):
        num_rays = self._eq_points_wp.shape[0]

        origin = wp.vec3(float(tf.p[0]), float(tf.p[1]), float(tf.p[2]))
        quat = tf.q

        # Allocate outputs on device
        ray_starts_wp = wp.empty(num_rays, dtype=wp.vec3)
        ray_ends_wp = wp.empty(num_rays, dtype=wp.vec3)

        wp.launch(
            compute_ray_endpoints,
            dim=num_rays,
            inputs=[
                self._eq_points_wp,
                self._eq_rays_wp,
                self._eq_mask_wp,
                origin,
                quat,
                self.max_range,
            ],
            outputs=[ray_starts_wp, ray_ends_wp],
        )

        # Move back to numpy only once
        return ray_starts_wp.numpy(), ray_ends_wp.numpy()

    def get_distances(self) -> wp.array:
        """
        Euclidean distance to each hit, length hres*vres.
        0.0 for misses.  Updated by every call to update().
        """
        return self._eq_dist_wp

    # ── Diagnostics ───────────────────────────────────────────────────

    def __repr__(self) -> str:
        return (
            f"Lidar(hfov={math.degrees(self.hfov):.1f}°, "
            f"vfov={math.degrees(self.vfov):.1f}°, "
            f"hres={self.hres}, vres={self.vres}, "
            f"face_res={self.face_res}, "
            f"max_dist={self.max_range}m)"
        )