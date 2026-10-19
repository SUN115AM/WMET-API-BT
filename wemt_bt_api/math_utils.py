from __future__ import annotations

import math

import numpy as np


def normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion cannot have zero length.")
    return q / norm


def euler_to_quaternion(euler_degrees: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.radians(np.asarray(euler_degrees, dtype=float))

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return normalize_quaternion(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ]
        )
    )


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array(
        [
            [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2],
        ],
        dtype=float,
    )


def rotation_matrix_to_euler(rotation_matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation_matrix, dtype=float)
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    singular = sy < 1e-6

    if singular:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = 0.0
    else:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])

    return np.degrees(np.array([roll, pitch, yaw], dtype=float))


def quaternion_to_euler(quaternion: np.ndarray) -> np.ndarray:
    return rotation_matrix_to_euler(quaternion_to_rotation_matrix(quaternion))


def make_homogeneous_matrix(quaternion: np.ndarray, position_mm: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_to_rotation_matrix(quaternion)
    matrix[:3, 3] = np.asarray(position_mm, dtype=float)
    return matrix
