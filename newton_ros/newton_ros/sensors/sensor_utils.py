from sensor_msgs.msg import PointCloud2, PointField, LaserScan
import numpy as np
import warp as wp
from cv_bridge import CvBridge


def make_cv2_bridge():
    """Return a CvBridge instance for converting between OpenCV and ROS images."""
    return CvBridge()

@wp.kernel  
def depth_to_pointcloud(  
    depth:          wp.array2d(dtype=float),  
    fov_scale:      float,  
    aspect_ratio:   float,  
    width:          int,  
    height:         int,  
    min_dist:       float,  
    max_dist:       float,  
    cam_fwd:        wp.vec3,   # camera forward vector (defines target frame)  
    cam_up:         wp.vec3,   # camera up vector     (defines target frame)  
    points:         wp.array2d(dtype=wp.vec3),  
    valid:          wp.array2d(dtype=wp.int32),  
):  
    px, py = wp.tid()  
  
    d = depth[py, px]  
  
    if d <= 0.0 or d < min_dist or d > max_dist:  
        points[py, px] = wp.vec3(0.0, 0.0, 0.0)  
        valid[py, px]  = 0  
        return  
  
    # ── Sensor-frame ray (X=right, Y=up, Z=forward) ──────────────────  
    ndc_x = (2.0 * float(px) + 1.0) / float(width)  - 1.0  
    ndc_y = 1.0 - (2.0 * float(py) + 1.0) / float(height)  
  
    cam_x = ndc_x * fov_scale * aspect_ratio  
    cam_y = ndc_y * fov_scale  
    cam_z = 1.0  
  
    pt = wp.normalize(wp.vec3(cam_x, cam_y, cam_z)) * d  
  
    # ── Build orthonormal basis from fwd + up ─────────────────────────  
    # Matches _compute_camera_basis: right = cross(fwd, up)  
    right    = wp.normalize(wp.cross(cam_fwd, cam_up))  
    up_ortho = wp.normalize(wp.cross(right, cam_fwd))  
  
    # ── Rotate: sensor local (X=right, Y=up, Z=fwd) → target frame ───  
    # pt.x is along right, pt.y is along up, pt.z is along forward  
    points[py, px] = right * pt[0] + up_ortho * pt[1] + cam_fwd * pt[2]  
    valid[py, px]  = 1

def points_to_pcd_msg(input_pc, stamp, frame_id):
    """Serialize a numpy point cloud array into a ROS 2 PointCloud2 message."""
    type_mappings = [
        (PointField.INT8, np.dtype("int8")),
        (PointField.UINT8, np.dtype("uint8")),
        (PointField.INT16, np.dtype("int16")),
        (PointField.UINT16, np.dtype("uint16")),
        (PointField.INT32, np.dtype("int32")),
        (PointField.UINT32, np.dtype("uint32")),
        (PointField.FLOAT32, np.dtype("float32")),
        (PointField.FLOAT64, np.dtype("float64")),
    ]
    nptype_to_pftype = {nptype: pftype for pftype, nptype in type_mappings}
    arr = input_pc.astype(np.float32)
    cloud_arr = np.core.records.fromarrays(arr.T, names="x,y,z", formats="f4,f4,f4")
    cloud_arr = np.atleast_2d(cloud_arr)
    # Create PointCloud2 message
    msg = PointCloud2()
    msg.height = cloud_arr.shape[0]
    msg.width = cloud_arr.shape[1]
    msg.is_bigendian = False
    msg.point_step = cloud_arr.dtype.itemsize
    msg.row_step = msg.point_step * cloud_arr.shape[1]
    msg.is_dense = all(
        np.isfinite(cloud_arr[name]).all() for name in cloud_arr.dtype.names
    )
    msg.data = cloud_arr.tobytes()

    if stamp is not None:
        msg.header.stamp = stamp
    if frame_id is not None:
        msg.header.frame_id = frame_id

    # Define PointFields
    msg.fields = []
    for name in cloud_arr.dtype.names:
        np_type, offset = cloud_arr.dtype.fields[name]
        pf = PointField()
        pf.name = name
        if np_type.subdtype:
            item_type, shape = np_type.subdtype
            pf.count = int(np.prod(shape))
            np_type = item_type
        else:
            pf.count = 1
        pf.datatype = nptype_to_pftype[np_type]
        pf.offset = offset
        msg.fields.append(pf)

    return msg

wp.kernel  
def compute_ray_endpoints(  
    camera_position:  wp.vec3,  
    camera_direction: wp.vec3,  
    camera_up:        wp.vec3,  
    camera_right:     wp.vec3,  
    fov_scale:        float,  
    aspect_ratio:     float,  
    width:            int,  
    height:           int,  
    depth_image:      wp.array2d(dtype=float),  
    max_distance:     float,          # <-- new parameter  
    ray_starts: wp.array2d(dtype=wp.vec3),  
    ray_ends:   wp.array2d(dtype=wp.vec3),  
):  
    px, py = wp.tid()  
  
    resolution = wp.vec2(float(width), float(height))  
    origin, direction = ray_for_pixel(  
        camera_position, camera_direction,  
        camera_up, camera_right,  
        fov_scale, aspect_ratio,  
        resolution, px, py,  
    )  
  
    depth = depth_image[py, px]  
    # Use max_distance for rays that didn't hit anything  
    d = depth if depth > 0.0 else max_distance  
  
    ray_starts[py, px] = origin  
    ray_ends[py, px]   = origin + direction * d

def log_points(viewer, path, points, color, radius):
    radii=wp.full(len(points),
                        radius,
                        dtype=wp.float32)
    colors=wp.full(len(points),
                        color,
                        dtype=wp.vec3f)
    viewer.log_points(  
        path,  
        points=points,  
        radii=radii,  
        colors=colors,  
    )

def log_rays(viewer, path, rays_starts, rays_ends, color, width):
    colors=wp.full(len(rays_starts),
                        color,
                        dtype=wp.vec3f)
    viewer.log_lines(  
        path,  
        starts=wp.array(rays_starts, dtype=wp.vec3f),  
        ends=wp.array(rays_ends, dtype=wp.vec3f),  
        colors=colors,  
        width=width,  
    )
    
def create_site(builder,body, pos_offset, euler_offset, label):
    euler_rad = np.radians(euler_offset)
    cr, sr = np.cos(euler_rad[0] * 0.5), np.sin(euler_rad[0] * 0.5)
    cp, sp = np.cos(euler_rad[1] * 0.5), np.sin(euler_rad[1] * 0.5)
    cy, sy = np.cos(euler_rad[2] * 0.5), np.sin(euler_rad[2] * 0.5)

    quat = wp.quat(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

    xform = wp.transform(wp.vec3(*pos_offset), quat)

    return builder.add_site(body, xform=xform, label=label)

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]])

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[ c,0, s],
                     [ 0,1, 0],
                     [-s,0, c]])

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],
                     [s, c,0],
                     [0, 0,1]])

def euler_xyz_to_R(roll, pitch, yaw):
    r, p, y = np.deg2rad([roll, pitch, yaw])
    return Rx(r) @ Ry(p) @ Rz(y)


def compute_flips_opengl(euler_deg):
    R = euler_xyz_to_R(*euler_deg)

    # Your camera basis
    right_cam = np.array([0.0, -1.0, 0.0])
    up_cam    = np.array([0.0,  0.0, 1.0])

    # Rotate into world
    right_w = R @ right_cam
    up_w    = R @ up_cam

    # OpenGL axes
    gl_right = np.array([1.0, 0.0, 0.0])
    gl_up    = np.array([0.0, 1.0, 0.0])

    # Flip checks
    hflip = np.dot(right_w, gl_right) < 0
    vflip = np.dot(up_w, gl_up) < 0

    return hflip, vflip
