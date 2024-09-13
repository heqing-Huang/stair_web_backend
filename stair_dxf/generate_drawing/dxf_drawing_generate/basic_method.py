"""
    DXF绘图基本方法：封装基本模块
"""
import math

import ezdxf
from ezdxf.layouts import Modelspace  # 模型空间
from ezdxf import document
from typing import List, Tuple
import copy
import numpy as np
import uuid  # 给对象赋予不同的名称


def radian_bt_vectors(vector1: np.ndarray, vector2: np.ndarray) -> float:
    """
    计算两个向量之间的弧度值

    :param vector1:
    :param vector2:
    :return:
    """
    a_norm = np.linalg.norm(vector1)
    b_norm = np.linalg.norm(vector2)
    a_dot_b = vector1.dot(vector2)
    radian = np.arccos(a_dot_b / (a_norm * b_norm))
    return radian


def rotation_3d(position: np.asarray, axis: np.asarray, angle: float) -> np.ndarray:
    """
    工具函数,用于依仗特定轴旋转特定向量特定角度，此处必须用np.asarray，不然容易报错。
    position:原坐标(x, y, z)
    axis:旋转的坐标轴(ex, ey, ez)
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


def calculate_normal_vector(
    point_1: Tuple[float], point_2: Tuple[float], theta: float
) -> np.array:
    """
    开始计算两个元组与平面向外叉乘的法向量
    :param point_1:
    :param point_2:
    :param theta:
    :return:
    """
    vector_1 = np.array(
        np.array(list(point_2)) - np.array(list(point_1))
    ) / np.linalg.norm(np.array(list(point_2)) - np.array(list(point_1)))
    vector_0 = rotation_3d(np.asarray(vector_1), np.asarray([0, 0, 1]), theta)
    vector_2 = np.array([0, 0, 1])
    normal = np.cross(vector_0, vector_2)
    normal_ = np.array(normal) / np.linalg.norm(normal)
    return normal_


def get_uuid() -> str:
    """
    a string value to add in block's name

    Returns:
        str
    """
    return uuid.uuid4().hex


def get_uuid_block(
    block_name: str, dxf_file: ezdxf.document.Drawing
) -> ezdxf.layouts.blocklayout.BlockLayout:
    """
    create block,with uuid

    Args:
        block_name:
        dxf_file:

    Returns:
        the block shape_entity
    """
    block: ezdxf.layouts.blocklayout.BlockLayout = dxf_file.blocks.new(
        f"{block_name}_{get_uuid()}"
    )
    return block


def draw_box(msp: Modelspace, width: int, height: int):
    """
    绘制长方形
    :param msp: 模型空间
    :param width: 图纸的宽度
    :param height: 图纸的高度
    :return:
    """
    msp.add_lwpolyline([(0, 0), (width, 0), (width, height), (0, height)], close=True)
    msp.add_line((0, 0), (width, height))  # 矩形框对角线
    msp.add_line((0, height), (width, 0))  # 矩形框对角线
    text = f"Box {width}x{height} drawing units"
    msp.add_text(text, height=0.1, dxfattribs=dict(style="Arial")).set_placement(
        (0.1, 0.1)
    )


def adjust_drawing_scale(point: List[float], scale: float) -> Tuple[float]:
    """
    调整图纸的比例
    :param point: [float,float,float]
    :param scale: float
    :return:
    """
    for num in range(len(point)):
        point[num] *= scale
    final_point = tuple(point)
    return final_point


def draw_polyline(msp: Modelspace, point: List[Tuple]):
    """
    通过已知点绘制多边形
    :param msp: 模型空间
    :param point: 点集合
    :return:
    """
    msp.add_line(point[0], point[1])  # 添加直线
    return msp


def transform_point_from_yz_to_xy(point: List[float]):
    """
    转换坐标点：由平面图形yz转换到xy平面
    :param point: 坐标点
    :return:
    """
    transform_matrix = np.array([[0, 0, 1], [0, 1, 0], [0, 0, 0]])
    current_point = np.array(copy.deepcopy(point)).T  # 转置该点
    result = np.matmul(transform_matrix, current_point).T
    return result.tolist()


def transform_point_from_yz_to_xy_s(point: List[float]):
    """
    转换坐标点：由平面图形yz转换到xy平面:y->x,z->y
    :param point: 坐标点
    :return:
    """
    transform_matrix = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
    current_point = np.array(copy.deepcopy(point)).T  # 转置该点
    result = np.matmul(transform_matrix, current_point).T
    return result.tolist()


def transform_point_from_xy_to_yx(point: List[float]):
    """
    转换坐标点：由平面图形xy转换到yx平面:x->y,y->x
    :param point: 坐标点
    :return:
    """
    transform_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 0]])
    current_point = np.array(copy.deepcopy(point)).T  # 转置该点
    result = np.matmul(transform_matrix, current_point).T
    return result.tolist()


def transform_point_from_xz_to_xy(point: List[float]):
    """
    转换坐标点：由平面图形yz转换到xy平面
    :param point: 坐标点
    :return:
    """
    transform_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, 0, 0]])
    current_point = np.array(copy.deepcopy(point)).T  # 转置该点
    result = np.matmul(transform_matrix, current_point).T
    return result.tolist()


def rotation_point_from_base_to_theta(
    point: List[float], base: List[float], theta: float
):
    """
    将坐标点由基点逆时针旋转theta角度
    :param point: 待旋转的点
    :param base: 坐下角度基点
    :param theta: 逆时针方向的旋转角度
    :return:
    """
    local_coor_m = np.array(point) - np.array(base)  # 局部坐标点矩阵表示方法
    local_coor_p = local_coor_m.tolist()  # 局部坐标点的表示
    rotation_matrix = np.array(
        [
            [1, 0, 0],
            [0, math.cos(theta), -math.sin(theta)],
            [0, math.sin(theta), math.cos(theta)],
        ]
    )  # 旋转矩阵
    current_point = np.array(copy.deepcopy(local_coor_p)).T  # 转置该点
    local_coor_n = np.matmul(rotation_matrix, current_point).T  # 旋转后的局部坐标点
    global_point = np.array(base) + local_coor_n
    return global_point.tolist()


def get_vector_of_two_points(point_1: Tuple[float], point_2: Tuple[float]):
    """
    获取两点形成的标准化向量
    :param point_1: Tuple[float]
    :param point_2: Tuple[float]
    :return:
    """
    vector = np.array(
        np.array(list(point_2)) - np.array(list(point_1))
    ) / np.linalg.norm(np.array(list(point_2)) - np.array(list(point_1)))
    return vector


def get_circle_radius_and_center(points: List[List[Tuple]]):
    """
    计算圆的半径和圆心
    :param points: [(),()]
    :return:
    """
    sum_x = 0
    sum_y = 0
    sum_z = 0
    for point in points:
        point_1 = copy.deepcopy(point)
        sum_x += point_1[0]
        sum_y += point_1[1]
        sum_y += point_1[2]
    num = len(points)
    point_0 = [points[0][0], points[0][1], points[0][2]]
    center = [sum_x / num, sum_y / num, sum_z / num]
    radius = round(
        np.linalg.norm(np.array(center) - np.array(point_0)), 1
    )  # 圆心与圆上各点的距离
    origin = (round(center[0], 0), round(center[1], 0), round(center[2], 0))
    return origin, radius


def get_two_vector_angle(vector_1: np.array, vector_2: np.array):
    """
    计算两向量的夹角
    :param vector_1:
    :param vector_2:
    :return:
    """
    length_1 = np.linalg.norm(vector_1)  # 计算长度1
    length_2 = np.linalg.norm(vector_2)  # 计算长度2
    cos_theta = np.dot(vector_1, vector_2) / (length_1 * length_2)
    theta = np.arccos(cos_theta)  # 弧度制
    return theta
