import numpy as np
from typing import List
import quaternion

def quaternion_rotate_vector(
    quat: quaternion.quaternion, v: np.ndarray
) -> np.ndarray:
    r"""Rotates a vector by a quaternion
    Args:
        quaternion: The quaternion to rotate by
        v: The vector to rotate
    Returns:
        np.ndarray: The rotated vector
    """
    vq = quaternion.quaternion(0, 0, 0, 0)
    vq.imag = v
    return (quat * vq * quat.inverse()).imag

def cartesian_to_polar(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi



def quat_to_xy_heading(quat):
    direction_vector = np.array([0, 0, -1])

    heading_vector = quaternion_rotate_vector(quat, direction_vector)

    phi = cartesian_to_polar(-heading_vector[2], heading_vector[0])[1]
    return np.array([phi], dtype=np.float32)



def HeadingSensor(rotation):
    rotation_world_agent = rotation
    if isinstance(rotation_world_agent, quaternion.quaternion):
        return quat_to_xy_heading(rotation_world_agent.inverse())
    else:
        raise ValueError("Agent's roation was not a quaternion")


def EpisodicCompassSensor(epi_start_rotation, rotation):
    rotation_world_agent = rotation
    roation_world_start = epi_start_rotation
    if isinstance(rotation_world_agent, quaternion.quaternion):
        return  quat_to_xy_heading(rotation_world_agent.inverse() * roation_world_start)
    else:
        raise ValueError("Agent's rotation was not a quaternion")

def EpisodicGPSSensor(epi_start_position, epi_start_rotation, position, dim):
    origin = np.array(epi_start_position, dtype = np.float32)
    rotation_world_start = epi_start_rotation
    agent_position = position
    agent_position = quaternion_rotate_vector(rotation_world_start.inverse(), agent_position-origin)
    if dim == 2:
        return np.array(
            [-agent_position[2], agent_position[0]], dtype = np.float32
        )
    else:
        return agent_position.astype(np.float32)

