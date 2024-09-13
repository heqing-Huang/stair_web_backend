"""
# File       : tools.py
# Time       ：2022/12/09 11:27
# Author     ：CR_X
# version    ：python 3.8
# Description：
"""
import math
import numpy as np
from stair_rebar_layout.models import Point


def rotation_matrix_from_vectors(vec1, vec2):
    """Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (
        vec2 / np.linalg.norm(vec2)
    ).reshape(3)
    v = np.cross(a, b)
    if any(v):  # if not all zeros then
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))

    else:
        return np.eye(3)  # cross of all zeros only occurs on identical directions


def get_z(slope: float, y_0: float, z_0: float, y: float) -> float:
    """
    已知斜率和一点坐标，根据y值求解该直线上z坐标
    :param slope: 斜率
    :param y_0: 固定y坐标
    :param z_0: 固定z坐标
    :param y: 动态y坐标
    :return:
    """
    z = z_0 + slope * (y - y_0)
    return z


def get_y(slope: float, y_0: float, z_0: float, z) -> float:
    """
    已知斜率和一点坐标，根据z值求解该直线上y坐标
    :param slope: 斜率
    :param y_0: 固定y坐标
    :param z_0: 固定z坐标
    :param z: 动态z坐标
    :return:
    """
    y = y_0 + (z - z_0) / slope
    return y


def rotation_3d(position: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """
    工具函数,用于依仗特定轴旋转特定向量特定角度
    position:原坐标[x, y, z]
    axis:旋转的坐标轴[ex, ey, ez]
    angle: 旋转弧度
    """

    ex, ey, ez = axis
    ex, ey, ez = [x / np.sqrt(ex**2 + ey**2 + ez**2) for x in axis]  # 归一化
    s, c = (
        np.sin(angle),
        np.cos(angle),
    )
    matrix1 = np.array(
        [
            [ex**2, ex * ey, ex * ez],
            [ey * ex, ey**2, ey * ez],
            [ex * ez, ey * ez, ez**2],
        ]
    )
    matrix2 = np.array(
        [[c, -ez * s, ey * s], [ez * s, c, -ex * s], [-ey * s, ex * s, c]]
    )
    matrix = (1 - c) * matrix1 + matrix2
    return matrix.dot(np.array(position).reshape(3, 1)).reshape(1, 3)[0]


def radian_bt_vectors(vector1: np.ndarray, vector2: np.ndarray) -> float:
    """
    计算两个向量之间的弧度值

    :param vector1:
    :param vector2:
    :return:
    """
    a_norm = np.linalg.norm(vector1)  # 范数
    b_norm = np.linalg.norm(vector2)  # 范数
    a_dot_b = vector1.dot(vector2)  # 向量点乘
    radian = np.arccos(a_dot_b / (a_norm * b_norm))  #
    return radian


def get_bounding_box_rotation_vertex(
    point: Point, radian: float, length_x: float, length_y: float, length_z: float
) -> Point:
    """
    计算三维包围框旋转后的角点：
    :param point:几何包围框的形状中心
    :param radian:旋转角度--弧度
    :param length_x:  x方向的长度
    :param length_y:  y方向的长度
    :param length_z:  z方向的长度
    :return:
    """
    add_y = -length_y / 2
    add_z = -length_z / 2
    total_length = math.sqrt(add_y**2 + add_z**2)
    initial_center_vector = np.asarray(
        [0, add_y / total_length, add_z / total_length]
    )  # 局部坐标系下轴心到原点的向量
    after_rotate_center_vector = rotation_3d(
        initial_center_vector, np.asarray([1, 0, 0]), radian
    )  # 旋转后轴心到原点的向量
    global_vertex = after_rotate_center_vector * total_length + np.asarray(
        [point.x - length_x / 2, point.y, point.z]
    )
    fact_point = Point(x=global_vertex[0], y=global_vertex[1], z=global_vertex[2])
    return fact_point
