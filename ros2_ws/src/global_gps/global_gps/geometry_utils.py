"""
Geometry utilities for the ground-plane localizer.

All functions operate in the camera's 3-D coordinate frame unless the
docstring states otherwise.
"""

import numpy as np


def fit_plane_svd(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Fit a plane to a set of 3-D points using SVD.

    Parameters
    ----------
    points:
        Array of shape (N, 3) containing the 3-D points to fit.

    Returns
    -------
    normal:
        Unit normal vector of the best-fit plane (shape (3,)).
    d:
        Plane offset such that ``normal @ p + d == 0`` for any point *p*
        on the plane.
    """
    centroid = np.mean(points, axis=0)
    _, _, Vt = np.linalg.svd(points - centroid)
    normal: np.ndarray = Vt[-1]
    normal /= np.linalg.norm(normal)
    d: float = float(-normal.dot(centroid))
    return normal, d


def project_point_to_plane(
    p: np.ndarray,
    normal: np.ndarray,
    d: float,
) -> np.ndarray:
    """Project a 3-D point onto a plane defined by (normal, d).

    Parameters
    ----------
    p:
        Point to project, shape (3,).
    normal:
        Unit normal of the plane, shape (3,).
    d:
        Plane offset (``normal @ p + d == 0`` on the plane).

    Returns
    -------
    Projected point on the plane, shape (3,).
    """
    dist = normal.dot(p) + d
    return p - dist * normal


def build_world_transform(
    origin: np.ndarray,
    x_point: np.ndarray,
    y_point: np.ndarray,
    normal: np.ndarray,
) -> np.ndarray:
    """Build a 4x4 homogeneous transform from camera frame to world frame.

    The world frame is defined by three corner anchor markers:

    * ``origin``  — world origin (0, 0)
    * ``x_point`` — a point along the positive world-X axis
    * ``y_point`` — a point roughly along the positive world-Y axis
                    (Gram–Schmidt orthogonalised against X)

    The world Z axis is the ground-plane normal.

    Parameters
    ----------
    origin, x_point, y_point:
        3-D positions of three corner anchors in camera frame, shape (3,).
    normal:
        Unit normal of the ground plane in camera frame, shape (3,).

    Returns
    -------
    T_world_from_cam:
        4x4 homogeneous matrix mapping camera-frame coordinates to
        world-frame coordinates.
    """
    X = x_point - origin
    X /= np.linalg.norm(X)

    Y = y_point - origin
    Y -= X * np.dot(Y, X)          # Gram–Schmidt: remove X component
    Y /= np.linalg.norm(Y)

    Z = normal

    T_cam_from_world = np.eye(4)
    T_cam_from_world[:3, 0] = X
    T_cam_from_world[:3, 1] = Y
    T_cam_from_world[:3, 2] = Z
    T_cam_from_world[:3, 3] = origin

    return np.linalg.inv(T_cam_from_world)
