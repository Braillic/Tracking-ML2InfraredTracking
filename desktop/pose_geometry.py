"""Coordinate transforms shared by the tracking core and adapters; no transport."""
from __future__ import annotations

import cv2
import numpy as np
from tracking_types import CameraIntrinsics

# OpenCV Y-down to Unity Y-up; shouldFlipTexture already changes image convention.
_OPENCV_TO_UNITY_CAM = np.diag([1.0, -1.0, 1.0]).astype(np.float64)

def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a Unity xyzw quaternion."""
    m = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if (not np.isfinite(m).all() or not np.allclose(m.T @ m, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(m), 1., atol=1e-5)):
        raise ValueError("Quaternion conversion requires a proper rotation, not a reflection")
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array((x, y, z, w), dtype=np.float64)
    quat /= max(np.linalg.norm(quat), 1e-12)
    return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])


def opencv_rvec_tvec_to_cam_T_object(
    rvec: np.ndarray,
    tvec: np.ndarray,
    *,
    depth_vertically_flipped: bool = True,
    convert_object_axes: bool = False,
) -> np.ndarray:
    """OpenCV object→camera (rvec/tvec) → 4x4 Unity camera←object matrix.

    When the streamed depth is vertically flipped (ML shouldFlipTexture=true) and
    cy has been adjusted to match, OpenCV's +Y already points the same way as
    Unity camera +Y — do **not** apply the Y-flip matrix C or motion mirrors.

    Only for an unflipped image is:
        R_u = C @ R_cv @ C, t_u = C @ t_cv, C = diag(1, -1, 1).
        This requires convert_object_axes=True; C @ R alone is a reflection.

    convert_object_axes conjugates with C as well (C @ R @ C); keep False unless
    model points were authored with a matching Y flip.
    """
    r_cv, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    t_cv = np.asarray(tvec, dtype=np.float64).reshape(3)

    if depth_vertically_flipped:
        # Flipped image + corrected cy: OpenCV frame already matches Unity cam axes.
        r_u = r_cv
        t_u = t_cv
        if convert_object_axes:
            # Still allow an explicit object-axis conjugate if the caller wants it,
            # but use C only on the object side: R' = C @ R @ C, t' = C @ t.
            c = _OPENCV_TO_UNITY_CAM
            r_u = c @ r_cv @ c
            t_u = c @ t_cv
    else:
        c = _OPENCV_TO_UNITY_CAM
        if convert_object_axes:
            r_u = c @ r_cv @ c
            t_u = c @ t_cv
        else:
            raise ValueError("Unflipped input requires --convert-object-axes to define a proper rotation")

    cam_t_object = np.eye(4, dtype=np.float64)
    cam_t_object[:3, :3] = r_u
    cam_t_object[:3, 3] = t_u
    return cam_t_object


def camera_matrix_for_vertically_flipped_image(
    camera_matrix: np.ndarray, image_height: int
) -> np.ndarray:
    """Adjust cy when depth was flipped for Unity (shouldFlipTexture=true).

    Magic Leap GetSensorData(..., shouldFlipTexture=true) vertically flips the
    plane bytes before we stream them. Intrinsics stay in the unflipped sensor
    frame, so OpenCV PnP needs cy' = H - 1 - cy to match GetSensorPose.
    """
    k = np.asarray(camera_matrix, dtype=np.float64).copy()
    k[1, 2] = float(image_height) - 1.0 - k[1, 2]
    return k


def unity_trs_matrix(position_xyz: np.ndarray, rotation_xyzw: np.ndarray) -> np.ndarray:
    """Build a Unity TRS matrix from position (xyz) and quaternion (xyzw)."""
    x, y, z, w = np.asarray(rotation_xyzw, dtype=np.float64).reshape(4)
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm < 1e-8 or not np.isfinite(position_xyz).all():
        raise ValueError("Invalid sensor pose")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    rotation = np.array(
        (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        ),
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    return matrix


def opencv_camera_pose_to_unity_world(
    rvec: np.ndarray,
    tvec: np.ndarray,
    sensor_position: np.ndarray,
    sensor_rotation_xyzw: np.ndarray,
    *,
    depth_vertically_flipped: bool = True,
    convert_object_axes: bool = False,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Compose OpenCV PnP with the depth frame's Unity sensor pose.

    world_T_object = world_T_sensor * cam_T_object
    """
    world_t_sensor = unity_trs_matrix(sensor_position, sensor_rotation_xyzw)
    cam_t_object = opencv_rvec_tvec_to_cam_T_object(
        rvec,
        tvec,
        depth_vertically_flipped=depth_vertically_flipped,
        convert_object_axes=convert_object_axes,
    )
    world_t_object = world_t_sensor @ cam_t_object
    position = (
        float(world_t_object[0, 3]),
        float(world_t_object[1, 3]),
        float(world_t_object[2, 3]),
    )
    rotation = rotation_matrix_to_quaternion_xyzw(world_t_object[:3, :3])
    return position, rotation


def pose_camera_matrix(
    intrinsics: CameraIntrinsics | None,
    image_height: int,
    *,
    depth_vertically_flipped: bool,
) -> np.ndarray | None:
    """Camera matrix used for live PnP (includes flip-cy adjustment when needed)."""
    if intrinsics is None:
        return None
    camera_matrix = intrinsics.camera_matrix
    if depth_vertically_flipped:
        camera_matrix = camera_matrix_for_vertically_flipped_image(camera_matrix, image_height)
    return camera_matrix

