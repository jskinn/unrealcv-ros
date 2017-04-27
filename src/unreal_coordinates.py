import numpy as np


_TORAD = np.pi / 180.0
_TODEG = 180 / np.pi


def unreal_quat2euler(w, x, y, z):
    """
    Convert a quaternion in unreal space to euler angles.
    Based on FQuat::Rotator in
    Engine/Source/Runtime/Core/Private/Math/UnrealMath.cpp ln 536
    which is in turn based on
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    :param w:
    :param x:
    :param y:
    :param z:
    :return:
    """
    SINGULARITY_THRESHOLD = 0.4999995

    singularity_test = z * x - w * y
    yaw_y = 2 * (w * z + x * y)
    yaw_x = 1 - 2 * (y * y + z * z)

    yaw = np.arctan2(yaw_y, yaw_x) * _TODEG
    if singularity_test < -SINGULARITY_THRESHOLD:
        pitch = -90
        roll = _clamp_axis(-yaw - 2 * np.arctan2(x, w) * _TODEG)
    elif singularity_test > SINGULARITY_THRESHOLD:
        pitch = 90
        roll = _clamp_axis(yaw - 2 * np.arctan2(x, w) * _TODEG)
    else:
        pitch = np.arcsin(2 * singularity_test) / _TORAD
        roll = np.arctan2(-2 * (w * x + y * z), (1 - 2 * (x * x + y * y))) * _TODEG
    return roll, pitch, yaw


def transform_to_unreal(pose):
    """
    Swap the coordinate frames from the ROS standard coordinate frame to the one used by unreal
    :param pose: A point, as any 3-length indexable, 
    :return: A tuple containing location and rotation (in euler angles)
    """
    if hasattr(pose, 'position'):
        location = pose.position
    elif len(pose) >= 3:
        location = (pose[0], pose[1], pose[2])
    else:
        location = (0, 0, 0)

    if hasattr(pose, 'orientation'):
        rotation = pose.orientation
    elif hasattr(pose, 'rotation'):
        rotation = pose.rotation
    else:
        rotation = (1, 0, 0, 0)

    # Invert y axis
    location = (location[0], -location[1], location[0])
    rotation = (rotation[0], rotation[1], -rotation[2], rotation[3])

    # Invert rotation for left-handed switch
    rotation = (np.array((1, -1, -1, -1)) * rotation) / np.dot(rotation, rotation)

    return (location, unreal_quat2euler(rotation[0], rotation[1], rotation[2], rotation[3]))


def _clamp_axis(angle):
    """
    Clamp an angle to range -180 to 180
    :param angle: 
    :return: 
    """
    angle %= 360
    if angle < -180:
        angle += 360
    return angle
