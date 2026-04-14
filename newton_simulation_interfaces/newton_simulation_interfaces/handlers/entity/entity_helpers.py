from simulation_interfaces.msg import EntityState
from simulation_interfaces.msg import Result
from newton_simulation_interfaces.newton_manager import NewtonManager
import torch
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Vector3


def get_entity_state(entity_name, newton_manager):
    """Fetch the full physics state (pose, twist, acceleration) of an entity."""
    entity_state = EntityState()
    if newton_manager.entities_info.get(entity_name) is None:
        return entity_state, Result(
            result=0,
            error_message=f"Entity with name {entity_name} not found in the scene",
        )
    else:
        entity = newton_manager.entities_info.get(entity_name)["entity_attr"]
    entity_state.header.frame_id = entity.links[0].name

    if (
        newton_manager.latest_timestamp is None
        or newton_manager.current_state_code != newton_manager.PLAYING
    ):
        return entity_state, Result(
            result=3,
            error_message="Scene is not in a running state, it is either not built or paused",
        )
    else:
        entity_state.header.stamp = newton_manager.latest_timestamp

    def check_validity(entries):
        for entry in entries:
            if entry is None:
                return False
            elif torch.isnan(entry).any() or torch.isinf(entry).any():
                return False
        return True

    if not check_validity(
        [
            entity.get_pos(),
            entity.get_quat(),
            entity.get_vel(),
            entity.get_ang(),
            entity.get_links_acc(links_idx_local=0),
            entity.get_links_acc_ang(links_idx_local=0),
        ]
    ):
        return Result(
            result=4, error_message="Entity state is not available or invalid"
        )
    else:
        pos = entity.get_pos().detach().cpu().numpy()[0]
        entity_state.pose.position = Point(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
        )
        quat = NewtonManager.wxyz_to_xyzw(entity.get_quat()[0])
        entity_state.pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
        )
        vel = entity.get_vel()[0].detach().cpu().numpy()
        entity_state.twist.linear = Vector3(
            x=float(vel[0]), y=float(vel[1]), z=float(vel[2])
        )
        ang_vel = entity.get_ang()[0].detach().cpu().numpy()
        entity_state.twist.angular = Vector3(
            x=float(ang_vel[0]), y=float(ang_vel[1]), z=float(ang_vel[2])
        )

        lin_acc = entity.get_links_acc(links_idx_local=0)[0][0].detach().cpu().numpy()
        entity_state.acceleration.linear = Vector3(
            x=float(lin_acc[0]), y=float(lin_acc[1]), z=float(lin_acc[2])
        )
        ang_acc = (
            entity.get_links_acc_ang(links_idx_local=0)[0][0].detach().cpu().numpy()
        )
        entity_state.acceleration.angular = Vector3(
            x=float(ang_acc[0]), y=float(ang_acc[1]), z=float(ang_acc[2])
        )
    return entity_state, Result(result=1)


def set_entity_state(
    entity_name,
    scene_manager,
    entity_state,
    set_pos=True,
    set_twist=True,
    set_acceleration=False,
):
    """Apply a physics state update (teleport or velocity change) to an entity."""
    if scene_manager.entities_info.get(entity_name) is None:
        return Result(
            result=0,
            error_message=f"Entity with name {entity_name} not found in the scene",
        )
    else:
        entity = scene_manager.entities_info.get(entity_name)["entity_attr"]

    if scene_manager.current_state_code != scene_manager.PLAYING:
        return Result(
            result=3,
            error_message="Scene is not in a running state, it is either not built or paused",
        )

    def check_validity(entries):
        for entry in entries:
            if entry is None:
                return False
            elif np.isnan(entry).any() or np.isinf(entry).any():
                return False
        return True

    target_position = np.array(
        [
            entity_state.pose.position.x,
            entity_state.pose.position.y,
            entity_state.pose.position.z,
        ]
    )
    target_orientation = np.array(
        [
            entity_state.pose.orientation.w,
            entity_state.pose.orientation.x,
            entity_state.pose.orientation.y,
            entity_state.pose.orientation.z,
        ]
    )
    target_vel_lin = np.array(
        [
            entity_state.twist.linear.x,
            entity_state.twist.linear.y,
            entity_state.twist.linear.z,
        ]
    )
    target_vel_ang = np.array(
        [
            entity_state.twist.angular.x,
            entity_state.twist.angular.y,
            entity_state.twist.angular.z,
        ]
    )

    if not check_validity(
        [
            target_position,
            target_orientation,
            target_vel_lin,
            target_vel_ang,
        ]
    ):
        return Result(
            result=4, error_message="Entity state is not available or invalid"
        )
    bounds = next(iter(scene_manager.world_info.values()))["world_bounds"]
    pos_valid = (
        entity_state.pose.position.x in bounds[0]
        and entity_state.pose.position.y in bounds[1]
        and entity_state.pose.position.z in bounds[2]
    )

    quat_valid = np.isclose(np.linalg.norm(target_orientation), 1.0)
    try:
        if set_pos:
            if pos_valid and quat_valid:
                entity.set_pos(target_position)
                entity.set_quat(SceneManager.xyzw_to_wxyz(target_orientation))
            else:
                return Result(result=101, error_message="Pose provided is not valid")
        if set_twist:
            entity.set_lin_vel(target_vel_lin)
            entity.set_ang_vel(target_vel_ang)
        if set_acceleration:
            return Result(
                result=4,
                error_message="Acceleration is not supported for static objects",
            )
    except Exception as e:
        return Result(result=4, error_message=f"Failed to set entity state: {str(e)}")

    return Result(result=1)
