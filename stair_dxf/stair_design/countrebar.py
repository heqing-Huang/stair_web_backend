"""
Project:计算预埋件和钢筋的位置
Author: zhangchao
Date: 2022/8/24

在进行几何运算过程中，可以参考线性代数的知识来解决
"""
import copy

from stair_dxf.stair_design.datas import (
    Point,
    RebarData,
    HoleReinRebar,
    HoistRebar,
    RoundHeadHangingNailInformation,
    EmbeddedAnchorInformation,
    RailingInformation,
    RebarConfig,
    LadderBeamAndSlabInformation,
    NodeInformation,
    ConnectElementShapeInfo,
    RailingEmbeddedRabbet,
    ConnectionHoleInformation,
)
from stair_dxf.stair_design.tools import rebar_mandrel_radius
from typing import Dict, List
import math
import numpy as np


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
    vector_1 = np.asarray([0, add_y, add_z])
    total_length = np.linalg.norm(vector_1)
    initial_center_vector = vector_1 / total_length  # 局部坐标系下轴心到原点的向量
    after_rotate_center_vector = rotation_3d(
        initial_center_vector, np.asarray([1, 0, 0]), -radian
    )  # 旋转后轴心到原点的向量,顺时针旋转为负
    global_vertex = after_rotate_center_vector * total_length + np.asarray(
        [point.x - length_x / 2, point.y, point.z]
    )
    fact_point = Point(x=global_vertex[0], y=global_vertex[1], z=global_vertex[2])
    return fact_point


class BottomLongitudinalRebar(object):
    """
    下部纵筋的钢筋坐标计算
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.bottom_rebar_init_point = self.rebar_for_bim.bottom_rebar
        single_rebar = self.bottom_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算下部纵筋的钢筋数据模型
        :return:
        """
        number = len(self.bottom_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.bottom_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class TopLongitudinalRebar(object):
    """
    顶部纵筋的钢筋坐标计算
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.top_rebar_init_point = self.rebar_for_bim.top_rebar
        single_rebar = self.top_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算顶部纵筋的钢筋数据模型
        :return:
        """
        number = len(self.top_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.top_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class MidDistributionRebar(object):
    """
    中部分布筋钢筋计算
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.mid_rebar_init_point = self.rebar_for_bim.mid_rebar
        single_rebar = self.mid_rebar_init_point[0]
        self.diameter = self.struct_book.d_3_actual
        self.rebar_config = RebarConfig()
        self.cover = self.slab_struct.construction.concrete_cover_thickness
        self.b0 = self.detail_slab.geometric_detailed.width

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算中部分布筋的钢筋数据模型:单层钢筋坐标点----为了生成图纸的规范性，特在此处增加数据的变化，保证图纸的合理，但与具体生产有出入。
        :return:
        """
        number = int(len(self.mid_rebar_init_point) / 2)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.mid_rebar_init_point[2 * i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                if point_.x < self.b0 / 2:
                    point_.x = self.cover + 0.5 * self.diameter
                else:
                    point_.x = self.b0 - (self.cover + 0.5 * self.diameter)
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_double_rebar_model(self) -> List[List[Point]]:
        """
        得到双层中部分布筋模型
        :return:List[List[Point]]---[钢筋1-1，钢筋1-2，钢筋2-1，钢筋2-2，...,]
        """
        rebar_loc = self.get_rebar_model()  # 获取钢筋的数据
        rebar_diam = self.get_rebar_diameter()  # 获取钢筋的直径数据
        rebar_sets = []
        for num in range(len(rebar_loc)):
            points = copy.deepcopy(rebar_loc[num])
            point_1 = copy.deepcopy([points[0].x, points[0].y, points[0].z])
            point_2 = copy.deepcopy([points[1].x, points[1].y, points[1].z])
            point_3 = copy.deepcopy([points[2].x, points[2].y, points[2].z])
            point_4 = copy.deepcopy([points[3].x, points[3].y, points[3].z])
            vec_1 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
                np.array(point_2) - np.array(point_1)
            )
            vec_2 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
                np.array(point_3) - np.array(point_2)
            )
            vec_vert = np.cross(vec_1, vec_2)  # 叉乘得到法向量
            vert_stad = vec_vert / np.linalg.norm(vec_vert)  # 法向量标准化
            # 左侧钢筋坐标计算
            point_1_left = np.array(point_1) + vert_stad * (rebar_diam + 1) / 2
            point_2_left = np.array(point_2) + vert_stad * (rebar_diam + 1) / 2
            point_3_left = np.array(point_3) + vert_stad * (rebar_diam + 1) / 2
            point_4_left = np.array(point_4) + vert_stad * (rebar_diam + 1) / 2
            rebar_left = [
                point_1_left.tolist(),
                point_2_left.tolist(),
                point_3_left.tolist(),
                point_4_left.tolist(),
            ]  # 左侧钢筋数据
            point_left = []
            for i in range(len(rebar_left)):
                point_: Point = Point(0, 0, 0)
                _point = copy.deepcopy(rebar_left[i])
                point_.x = _point[0]
                point_.y = _point[1]
                point_.z = _point[2]
                point_left.append(point_)
            # 右侧钢筋坐标计算
            point_1_right = np.array(point_1) + vert_stad * (-rebar_diam - 1) / 2
            point_2_right = np.array(point_2) + vert_stad * (-rebar_diam - 1) / 2
            point_3_right = np.array(point_3) + vert_stad * (-rebar_diam - 1) / 2
            point_4_right = np.array(point_4) + vert_stad * (-rebar_diam - 1) / 2
            rebar_right = [
                point_2_right.tolist(),
                point_1_right.tolist(),
                point_4_right.tolist(),
                point_3_right.tolist(),
            ]  # 右侧钢筋数据
            point_right = []
            for j in range(len(rebar_right)):
                point_: Point = Point(0, 0, 0)
                _point = copy.deepcopy(rebar_right[j])
                point_.x = _point[0]
                point_.y = _point[1]
                point_.z = _point[2]
                point_right.append(point_)
            rebar_sets.append(point_left)
            rebar_sets.append(point_right)
        return rebar_sets

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class BottomEdgeLongitudinalRebar(object):
    """
    底端边缘纵筋的钢筋数据
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.bottom_edge_rebar_init_point = self.rebar_for_bim.bottom_rein_rebar
        single_rebar = self.bottom_edge_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算底端边缘纵筋的钢筋数据模型
        :return:
        """
        number = len(self.bottom_edge_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.bottom_edge_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_radius(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol

    def get_rebar_spacing(self):
        """
        :return:
        """
        spacing = self.detail_slab.rebar_detailed.bottom_edge_longitudinal_rebar.spacing
        return spacing


class TopEdgeLongitudinalRebar(object):
    """
    顶端边缘纵筋的钢筋坐标
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.top_edge_rebar_init_point = self.rebar_for_bim.top_rein_rebar
        single_rebar = self.top_edge_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算底端边缘纵筋的钢筋数据模型
        :return:
        """
        number = len(self.top_edge_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.top_edge_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_radius(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol

    def get_rebar_spacing(self):
        """
        :return:
        """
        spacing = self.detail_slab.rebar_detailed.top_edge_longitudinal_rebar.spacing
        return spacing


class BottomEdgeStirrup(object):
    """
    底端边缘箍筋的坐标
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.bottom_edge_stirrup_rebar_init_point = (
            self.rebar_for_bim.bottom_edge_stirrup_rebar
        )
        single_rebar = self.bottom_edge_stirrup_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算底端边缘纵筋的钢筋数据模型
        :return:
        """
        number = len(self.bottom_edge_stirrup_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.bottom_edge_stirrup_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points) - 1):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol

    def get_rebar_spacing(self):
        """
        :return:
        """
        spacing = self.detail_slab.rebar_detailed.bottom_edge_stirrup.spacing
        return spacing


class TopEdgeStirrup(object):
    """
    顶端边缘箍筋的坐标
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.top_edge_stirrup_rebar_init_point = (
            self.rebar_for_bim.top_edge_stirrup_rebar
        )
        single_rebar = self.top_edge_stirrup_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算底端边缘纵筋的钢筋数据模型
        :return:
        """
        number = len(self.top_edge_stirrup_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.top_edge_stirrup_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points) - 1):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol

    def get_rebar_spacing(self):
        """
        :return:
        """
        spacing = self.detail_slab.rebar_detailed.top_edge_stirrup.spacing
        return spacing


class HoleReinforceRebar(object):
    """
    销键或孔洞加强筋的钢筋数据
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.hole_rebar_init_point = self.rebar_for_bim.hole_rebar
        single_rebar = self.hole_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算孔洞加强筋的钢筋数据模型
        :return:
        """
        number = len(self.hole_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.hole_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_radius(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class HoistingReinforceLongitudinalRebar(object):  # zc2022/9/23  TODO
    """
    吊点加强纵筋的钢筋数据
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.lifting_longitudinal_rebar_init_point = (
            self.rebar_for_bim.lifting_longitudinal_rebar
        )
        single_rebar = self.lifting_longitudinal_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算吊点加强纵筋的钢筋数据模型
        :return:
        """
        number = len(self.lifting_longitudinal_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.lifting_longitudinal_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class HoistingReinforcePointRebar(object):  # 2022/9/23
    """
    吊点加强点筋的钢筋数据
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.lifting_point_rebar_init_point = self.rebar_for_bim.lifting_point_rebar
        single_rebar = self.lifting_point_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算吊装点筋的钢筋数据模型
        :return:
        """
        number = len(self.lifting_point_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.lifting_point_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_radius(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class TopEdgeReinforceRebar(object):
    """
    上部边缘加强筋筋的钢筋数据
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.top_edge_rein_rebar_init_point = self.rebar_for_bim.top_edge_rein_rebar
        single_rebar = self.top_edge_rein_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算吊装点筋的钢筋数据模型
        :return:
        """
        number = len(self.top_edge_rein_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.top_edge_rein_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class BottomEdgeReinforceRebar(object):
    """
    下部边缘加强筋筋的钢筋数据
    """

    def __init__(
        self, slab_design, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_rebar_datas()  # 产生钢筋初始数据

    def generate_rebar_datas(self):
        """
        产生钢筋初始数
        :return:
        """
        self.bottom_edge_rein_rebar_init_point = (
            self.rebar_for_bim.bottom_edge_rein_rebar
        )
        single_rebar = self.bottom_edge_rein_rebar_init_point[0]
        self.diameter = 2 * single_rebar.radius
        self.rebar_config = RebarConfig()

    def get_rebar_model(self) -> List[List[Point]]:
        """
        计算吊装点筋的钢筋数据模型
        :return:
        """
        number = len(self.bottom_edge_rein_rebar_init_point)
        rebar_result = []  # 钢筋数据结果
        for i in range(number):
            current_rebar = self.bottom_edge_rein_rebar_init_point[i]
            current_rebar_poly_points = current_rebar.poly.points  # 当前钢筋的轨迹
            current_rebar = []
            for num in range(len(current_rebar_poly_points)):
                current_point = current_rebar_poly_points[num]
                point_ = Point(x=0, y=0, z=0)
                point_.x = current_point.x
                point_.y = current_point.y
                point_.z = current_point.z
                current_rebar.append(point_)
            rebar_result.append(current_rebar)
        return rebar_result

    def get_rebar_diameter(self):
        """
        获取钢筋的直径
        :return:
        """
        return self.diameter

    def get_bending_diameter(self):
        """
        获取钢筋的弯箍半径
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        steel_grade = self.rebar_config.rebar_sets[rebar_level].level
        mandarel_radius = rebar_mandrel_radius(self.diameter / 2, steel_grade)
        return mandarel_radius

    def get_rebar_config_info(self):
        """
        获取钢筋的配置信息
        :return:
        """
        rebar_level = self.slab_struct.material.rebar_name
        rebar_info = self.rebar_config.rebar_sets[rebar_level]
        rebar_symbol = rebar_info.symbol
        return rebar_symbol


class HoistingEmbeddedPartsLoc(object):
    """
    吊装预埋件定位坐标
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.lifting_parameter = self.detail_book.lifting_parameter  # 吊装参数
        self.lifting_type = self.detail_slab.inserts_detailed.lifting_type
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_hoist_embedded_part_loc(self) -> List[Point]:
        """
        计算吊装预埋件的坐标点,标志点为企口下边缘
        :return:
        """
        lifting_position_a = self.detail_slab.inserts_detailed.lifting_position.a
        lifting_position_b = self.detail_slab.inserts_detailed.lifting_position.b
        lifting_position_c = float(
            self.detail_slab.inserts_detailed.lifting_position.c
        )  # 吊装预埋件左侧横向边距
        lifting_position_d = float(
            self.detail_slab.inserts_detailed.lifting_position.d
        )  # 吊装预埋件右侧横向边距
        edge_a = self.lb_d + (lifting_position_a - 0.5) * self.tabu_b  # 吊装预埋件顶端纵向边距
        edge_b = self.lb_d + (lifting_position_b - 0.5) * self.tabu_b  # 吊装预埋件底端纵向边距
        edge_c = lifting_position_c  # 吊装预埋件左侧横向边距
        edge_d = lifting_position_d  # 吊装预埋件右侧横向边距
        top_h = self.h2 + lifting_position_a * self.tabu_h
        bottom_h = self.h2 + lifting_position_b * self.tabu_h

        if self.lifting_type.value == 0:  # ROUNDING_HEAD = 0 #    ANCHOR = 1
            embedded_part_height = self.lifting_parameter.radius  # 吊装预埋件直径  # 埋入深度
        else:  # self.lifting_type.value == 1:
            embedded_part_height = self.lifting_parameter.m_length  # 埋入深度
        rebar_x = [edge_c, self.b0 - edge_d]
        rebar_y = [edge_b, edge_a]  # zc改
        rebar_z = [bottom_h - embedded_part_height, top_h - embedded_part_height]
        embedded_part = [
            Point(x=rebar_x[0], y=rebar_y[0], z=rebar_z[0]),
            Point(x=rebar_x[0], y=rebar_y[1], z=rebar_z[1]),
            Point(x=rebar_x[1], y=rebar_y[0], z=rebar_z[0]),
            Point(x=rebar_x[1], y=rebar_y[1], z=rebar_z[1]),
        ]
        return embedded_part

    def get_hoist_embedded_part_info(self):
        """
        获取吊装预埋件信息：埋件种类，具体规格
        :return:
        """
        hoist_info = {}
        hoist_shape = {}
        hoist_info[
            "type"
        ] = self.lifting_type.value  # 吊装预埋件类型 ROUNDING_HEAD = 0 #    ANCHOR = 1
        if self.lifting_type.value == 0:
            name = self.lifting_parameter.name
            hoist_info["name"] = name
            hoist_data = self.lifting_parameter  # 吊装预埋件信息
            hoist_shape["top_diameter"] = hoist_data.top_diameter
            hoist_shape["top_height"] = hoist_data.top_height
            hoist_shape["top_adjacent_height"] = hoist_data.top_adjacent_height
            hoist_shape["middle_diameter"] = hoist_data.middle_diameter
            hoist_shape["middle_height"] = hoist_data.middle_height
            hoist_shape["bottom_adjacent_height"] = hoist_data.bottom_adjacent_height
            hoist_shape["bottom_diameter"] = hoist_data.bottom_diameter
            hoist_shape["bottom_height"] = hoist_data.bottom_height
            hoist_shape["radius"] = hoist_data.radius
        else:  # self.lifting_type.value == 1:
            name = self.lifting_parameter.name
            hoist_info["name"] = name
            hoist_data = self.lifting_parameter  # 吊装预埋件信息
            hoist_shape["m_diameter"] = hoist_data.m_diameter
            hoist_shape["m_length"] = hoist_data.m_length
            hoist_shape["o_diameter"] = hoist_data.o_diameter
            hoist_shape["length"] = hoist_data.length
            hoist_shape["s_diameter"] = hoist_data.s_diameter
            hoist_shape["l_p"] = hoist_data.l_p
            hoist_shape["a"] = hoist_data.a
            hoist_shape["e_diameter"] = hoist_data.e_diameter
            hoist_shape["g"] = hoist_data.g
            hoist_shape["b"] = hoist_data.b

        hoist_info["specification"] = hoist_shape
        return hoist_info

    def get_single_hoist_bounding_box_vertex(self, point: Point) -> List[List[Point]]:
        """
        计算单个吊装预埋件的角点包围框
        :return:
        """
        single_hoist_model = []  # 单个吊装件模型
        if self.lifting_type.value == 0:
            hoist_information = self.lifting_parameter  # 获取圆头埋件信息
            semi_circle_r = hoist_information.radius  # 顶部半球半径
            top_d = hoist_information.top_diameter  # 顶部直径
            top_height = (
                hoist_information.top_height + hoist_information.top_adjacent_height
            )  # 顶部总高度
            mid_d = hoist_information.middle_diameter  # 中部直径
            mid_height = hoist_information.middle_height  # 中部高度
            bottom_d = hoist_information.bottom_diameter  # 底部直径
            bottom_height = (
                hoist_information.bottom_height
                + hoist_information.bottom_adjacent_height
            )  # 底部高度

            # 企口包围框
            semicircle_rabbet_model = [
                Point(x=point.x - semi_circle_r, y=point.y - semi_circle_r, z=point.z),
                Point(
                    x=point.x + semi_circle_r,
                    y=point.y + semi_circle_r,
                    z=point.z + semi_circle_r,
                ),
            ]
            # 顶部包围框
            top_model = [
                Point(
                    x=point.x - top_d / 2, y=point.y - top_d / 2, z=point.z - top_height
                ),
                Point(x=point.x + top_d / 2, y=point.y + top_d / 2, z=point.z),
            ]
            # 中部包围框
            mid_model = [
                Point(
                    x=point.x - mid_d / 2,
                    y=point.y - mid_d / 2,
                    z=point.z - top_height - mid_height,
                ),
                Point(
                    x=point.x + top_d / 2, y=point.y + top_d / 2, z=point.z - mid_height
                ),
            ]
            # 底部包围框
            bottom_model = [
                Point(
                    x=point.x - bottom_d / 2,
                    y=point.y - bottom_d / 2,
                    z=point.z - top_height - mid_height - bottom_height,
                ),
                Point(
                    x=point.x + bottom_d / 2,
                    y=point.y + bottom_d / 2,
                    z=point.z - top_height - mid_height,
                ),
            ]
            single_hoist_model.append(semicircle_rabbet_model)
            single_hoist_model.append(top_model)
            single_hoist_model.append(mid_model)
            single_hoist_model.append(bottom_model)

        else:
            hoist_information = self.lifting_parameter  # 获取圆头埋件信息
            top_d = hoist_information.m_diameter  # 顶部企口直径
            top_height = hoist_information.m_length  # 顶部企口高度
            bottom_d = hoist_information.o_diameter  # 底部锚栓直径
            bottom_height = hoist_information.length  # 底部锚栓长度或高度
            rebar_d = hoist_information.s_diameter  # 锚固钢筋直径
            rebar_length = hoist_information.l_p  # 锚固钢筋的长度
            rebar_relative_loc = (
                hoist_information.length - hoist_information.a
            )  # 钢筋相对锚栓的相对位置

            # 顶部企口模型
            top_rabbet_model = [
                Point(x=point.x - top_d / 2, y=point.y - top_d / 2, z=point.z),
                Point(
                    x=point.x + top_d / 2, y=point.y + top_d / 2, z=point.z + top_height
                ),
            ]
            bottom_model = [
                Point(
                    x=point.x - bottom_d / 2,
                    y=point.y - bottom_d / 2,
                    z=point.z - bottom_height,
                ),
                Point(x=point.x + bottom_d / 2, y=point.y + bottom_d / 2, z=point.z),
            ]
            anchor_rebar_model = [
                Point(
                    x=point.x - rebar_d / 2,
                    y=point.y - rebar_length / 2,
                    z=point.z - rebar_relative_loc - rebar_d / 2,
                ),
                Point(
                    x=point.x + rebar_d / 2,
                    y=point.y + rebar_length / 2,
                    z=point.z - rebar_relative_loc + rebar_d / 2,
                ),
            ]
            single_hoist_model.append(top_rabbet_model)
            single_hoist_model.append(bottom_model)
            single_hoist_model.append(anchor_rebar_model)

        return single_hoist_model

    def get_all_hoist_bounding_box_vertexs(self) -> List[List[List[Point]]]:
        """
        计算所有吊装预埋件包围框的角点
        :return:
        """
        hoist_points = self.get_hoist_embedded_part_loc()  # 获取吊装预埋件标志点
        hoist_bounding_box_vertexs = []
        for hoist_point in hoist_points:
            single_bounding_box = self.get_single_hoist_bounding_box_vertex(hoist_point)
            hoist_bounding_box_vertexs.append(single_bounding_box)
        return hoist_bounding_box_vertexs


class StepSlotLoc(object):
    """
    计算防滑槽的位置信息
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        # AUTOMATIC--0,MANUAL--1,NO--2
        self.step_slot_mode = (
            self.detail_slab.construction_detailed.step_slot_design_mode
        )  # 防滑槽设计模式
        self.step_slot_shape = self.detail_slab.construction_detailed.step_slot  # 防滑槽形状
        self.step_slot_position = (
            self.detail_slab.construction_detailed.step_slot_position
        )  # 防滑槽位置
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_step_slot_location(self):
        """
        计算防滑槽的位置信息
        :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]]
        """
        total_width = self.step_slot_shape.a + self.step_slot_shape.b  # 防滑槽的宽度
        edge_y_1 = self.step_slot_position.c3 + total_width / 2  # 组内边缘防滑槽面中心距y向边缘距离
        step_locs = []  # 踏步定位信息
        init_loc = [self.step_slot_position.c1, self.lb_d + edge_y_1, self.h2]  # 初始位置
        tabu_w = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        for num in range(self.n):
            point_: Point = Point(0, 0, 0)
            point_.x = init_loc[0]
            point_.y = init_loc[1] + num * tabu_w
            point_.z = init_loc[2] + (num + 1) * tabu_h
            step_locs.append(point_)
        return step_locs

    def get_step_slot_configuration(self) -> Dict:
        """
        计算防滑槽的配置信息
        :return: Dict
        """
        step_info = {}  # 预埋件信息库
        if self.step_slot_mode.value != 2:
            step_info["design_mode"] = self.step_slot_mode.value
            step_info["a"] = self.step_slot_shape.a
            step_info["b"] = self.step_slot_shape.b
            step_info["c"] = self.step_slot_shape.c
            step_info["d"] = self.step_slot_shape.d
            step_info["e"] = self.step_slot_shape.e
            step_info["width"] = self.step_slot_shape.a + self.step_slot_shape.b
            step_info["spacing"] = (
                self.step_slot_shape.a + self.step_slot_shape.b + self.step_slot_shape.c
            )  # 组内防滑槽中心间距
            step_info["length"] = (
                self.b0 - self.step_slot_position.c1 - self.step_slot_position.c2
            )  # 防滑槽的长度
            step_info["direction"] = [1, 0, 0]  # 拉伸方向
        return step_info


class WaterDripLoc(object):
    """
    滴水线槽的位置信息
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.water_drip_mode = (
            self.detail_slab.construction_detailed.water_drip_design_mode
        )  # 滴水线槽设计模式:1---MANUAL, 2--NO
        self.water_drip_shape = (
            self.detail_slab.construction_detailed.water_drip_shape
        )  # 滴水线槽形状:trapezoid梯形截面---0,semicircle半圆形截面---1
        self.water_drip_layout = (
            self.detail_slab.construction_detailed.water_drip_layout
        )  # 防滑槽位置  上--0，下--1，都有--2
        self.water_drip_parameter = (
            self.detail_slab.construction_detailed.water_drip
        )  # 滴水线槽参数
        self.water_drip_position = (
            self.detail_slab.construction_detailed.water_drip_position
        )  # 滴水线槽参数
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_water_drip_location(self):
        """
        滴水线槽坐标位置
        :return:
        """
        water_loc = []  # 滴水线槽的定位信息
        edge_a1 = self.water_drip_position.a1 + self.water_drip_parameter.b / 2  # 边距
        edge_a2 = self.water_drip_position.a2 + self.water_drip_parameter.b / 2  # 边距
        edge_a3 = self.water_drip_position.a3 + self.water_drip_parameter.b / 2  # 边距
        total_length = self.lb_d + self.ln + self.lt_d  # 楼梯总长
        area_height = self.h2 + self.height - self.h1  # 楼梯部分高度
        total_width_1 = self.b0 + self.b2
        total_width_2 = self.b0 + self.b1
        if self.lb_d > self.l1b:
            h_0 = (self.lb_d - self.l1b) * self.tan
            # 底部滴水线槽关键节点
            point_bottom = [
                [0, edge_a2, 0],
                [edge_a3, edge_a2, 0],
                [edge_a3, self.lb_d, 0],
                [edge_a3, self.lb_d, h_0],
                [edge_a3, total_length - self.l1t, area_height],
                [edge_a3, total_length - edge_a1, area_height],
                [0, total_length - edge_a1, area_height],
            ]  # 左侧点集合
            # 顶部滴水线槽关键节点
            point_top = [
                [total_width_1, edge_a2, 0],
                [self.b0 - edge_a3, edge_a2, 0],
                [self.b0 - edge_a3, self.lb_d, 0],
                [self.b0 - edge_a3, self.lb_d, h_0],
                [self.b0 - edge_a3, total_length - self.l1t, area_height],
                [self.b0 - edge_a3, total_length - edge_a1, area_height],
                [total_width_2, total_length - edge_a1, area_height],
            ]  # 右侧点集合
        else:
            # 底部滴水线槽关键节点
            point_bottom = [
                [0, edge_a2, 0],
                [edge_a3, edge_a2, 0],
                [edge_a3, self.l1b, 0],
                [edge_a3, total_length - self.l1t, area_height],
                [edge_a3, total_length - edge_a1, area_height],
                [0, total_length - edge_a1, area_height],
            ]  # 左侧点集合
            # 顶部滴水线槽关键节点
            point_top = [
                [total_width_1, edge_a2, 0],
                [self.b0 - edge_a3, edge_a2, 0],
                [self.b0 - edge_a3, self.l1b, 0],
                [self.b0 - edge_a3, total_length - self.l1t, area_height],
                [self.b0 - edge_a3, total_length - edge_a1, area_height],
                [total_width_2, total_length - edge_a1, area_height],
            ]  # 右侧点集合

        if self.water_drip_mode.value == 1:  # 若为人工输入
            if self.water_drip_layout == 0:  # 仅上侧布置滴水线槽
                for num in range(len(point_top)):
                    point_ = copy.deepcopy(point_top[num])
                    point_c: Point = Point(0, 0, 0)
                    point_c.x = point_[0]
                    point_c.y = point_[1]
                    point_c.z = point_[2]
                    water_loc.append(point_c)  #
            elif self.water_drip_layout == 1:  # 仅下侧布置滴水线槽
                for num in range(len(point_bottom)):
                    point_ = copy.deepcopy(point_bottom[num])
                    point_c: Point = Point(0, 0, 0)
                    point_c.x = point_[0]
                    point_c.y = point_[1]
                    point_c.z = point_[2]
                    water_loc.append(point_c)  #
            else:  # 两侧布置滴水线槽
                point_t_ = copy.deepcopy(point_top)
                point_b_ = copy.deepcopy(point_bottom)
                point_t_b = [point_t_, point_b_]  # 底部和顶部点集合
                for num in range(len(point_t_b)):
                    point_ = copy.deepcopy(point_t_b[num])
                    current_loc = []  # 单个滴水线槽
                    for num_p in range(len(point_)):
                        _point = copy.deepcopy(point_[num_p])
                        point_c: Point = Point(0, 0, 0)
                        point_c.x = _point[0]
                        point_c.y = _point[1]
                        point_c.z = _point[2]
                        current_loc.append(point_c)
                    water_loc.append(current_loc)  # 所有滴水线槽坐标点
        else:  # 无滴水线槽,无需设置坐标点
            pass
        return water_loc

    def get_water_drip_configuration(self):
        """
        滴水线槽配置信息
        :return:
        """
        water_drip_info = {}  # 滴水线槽配置信息
        if self.water_drip_mode.value != 2:  # 有滴水线槽
            water_drip_info["design_mode"] = self.water_drip_mode.value  # 滴水线槽设计模式
            water_drip_info[
                "design_location"
            ] = self.water_drip_layout.value  # 滴水线槽设置部位
            water_drip_info["design_shape"] = self.water_drip_shape.value  # 滴水线槽设置形状
            water_drip_info["a"] = self.water_drip_parameter.a  # 若为圆，则为半径
            water_drip_info["b"] = self.water_drip_parameter.b  #
            water_drip_info["c"] = self.water_drip_parameter.c  #
        return water_drip_info


class DemoldEmbeddedPartsLoc(object):
    """
    脱模预埋件位置
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.demolding_design_mode = (
            self.detail_slab.inserts_detailed.demolding_design_mode
        )  # 脱模方式
        self.demolding_parameter = self.detail_book.demolding_parameter  # 脱模参数
        self.demolding_type = self.detail_slab.inserts_detailed.demolding_type  # 脱模类型
        self.pouring_way = self.detail_slab.inserts_detailed.pouring_way  # 脱模方式
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_demold_embedded_part_loc(self) -> List[Point]:
        """
        计算脱模预埋件的坐标点
        :return:
        """
        # 获取信息
        demold_embedded_type = self.demolding_type.value  # 脱模预埋件类型
        demold_embedded_name = self.demolding_parameter.name  # 脱模预埋件名称,影响预埋件的埋入深度
        depth = 0  # 初始化埋入深度值
        if demold_embedded_type == 1:  # 若为预埋锚栓
            depth = self.demolding_parameter.m_length
        else:  # demold_embedded_type == 0:  # 若为圆头吊钉
            depth = self.demolding_parameter.radius
        # 侧面脱模预埋件的定位
        demolding_position_a = self.detail_slab.inserts_detailed.demolding_position.a
        demolding_position_b = self.detail_slab.inserts_detailed.demolding_position.b
        demolding_position_c = self.detail_slab.inserts_detailed.demolding_position.c
        demolding_position_d = self.detail_slab.inserts_detailed.demolding_position.d
        demolding_position_t = self.detail_slab.inserts_detailed.demolding_position.t
        demold_c_x = self.b0 - depth  # 侧面脱模预埋件x坐标
        demold_c_t_y = (
            self.lb_d + self.lt_d + self.ln - demolding_position_a
        )  # 顶端侧面脱模预埋件y坐标
        demold_c_b_y = demolding_position_b  # 底端侧面脱模预埋件y坐标
        demold_c_b_z = (
            demolding_position_b - self.l1b
        ) * self.tan + demolding_position_t / self.cos  # 底端侧面脱模预埋件z坐标
        demold_c_t_z = (
            self.lb_d + self.lt_d + self.ln - self.l1b - demolding_position_a
        ) * self.tan + demolding_position_t / self.cos  # 顶端侧面脱模预埋件z坐标
        # 卧式踏步平板侧脱模预埋件的定位
        demold_w_l_x = demolding_position_c  # 卧式左侧底面脱模预埋件x坐标
        demold_w_r_x = self.b0 - demolding_position_d  # 卧式右侧底面脱模预埋件x坐标
        demold_w_t_y = (
            self.lb_d + self.lt_d + self.ln - demolding_position_a
        )  # 卧式顶端底面脱模预埋件y坐标
        demold_w_b_y = demolding_position_b  # 卧式底端底面脱模预埋件y坐标
        demold_w_t_z = (
            self.lb_d + self.lt_d + self.ln - self.l1b - demolding_position_a
        ) * self.tan + depth / self.cos
        demold_w_b_z = (demolding_position_b - self.l1b) * self.tan + depth / self.cos
        demold_result = []
        if (
            self.pouring_way.value == 1
        ):  # 立式浇筑立式脱模 0 立式浇筑卧式脱模，1  # 立式浇筑立式脱模，2  # 卧式浇筑卧式脱模
            demold_result = [
                Point(x=demold_c_x, y=demold_c_b_y, z=demold_c_b_z),
                Point(x=demold_c_x, y=demold_c_t_y, z=demold_c_t_z),
            ]
        elif self.pouring_way.value == 2:  # 卧式浇筑卧式脱模
            demold_result = [
                Point(x=demold_w_l_x, y=demold_w_b_y, z=demold_w_b_z),
                Point(x=demold_w_l_x, y=demold_w_t_y, z=demold_w_t_z),
                Point(x=demold_w_r_x, y=demold_w_b_y, z=demold_w_b_z),
                Point(x=demold_w_r_x, y=demold_w_t_y, z=demold_w_t_z),
            ]
        else:  # 立式浇筑卧式脱模
            demold_result = [
                Point(x=demold_c_x, y=demold_c_b_y, z=demold_c_b_z),
                Point(x=demold_c_x, y=demold_c_t_y, z=demold_c_t_z),
                Point(x=demold_w_l_x, y=demold_w_b_y, z=demold_w_b_z),
                Point(x=demold_w_l_x, y=demold_w_t_y, z=demold_w_t_z),
                Point(x=demold_w_r_x, y=demold_w_b_y, z=demold_w_b_z),
                Point(x=demold_w_r_x, y=demold_w_t_y, z=demold_w_t_z),
            ]
        return demold_result

    def get_demold_part_info(self):
        """
        得到脱模预埋件的信息: 用于OCC计算
        :return:
        """
        demold_info = {}
        demold_shape = {}
        # 浇筑方式
        demold_embedded_type = self.demolding_type  # 脱模预埋件类型
        demold_embedded_name = self.demolding_parameter.name  # 脱模预埋件名称,影响预埋件的埋入深度
        demold_info["pouring_way"] = self.pouring_way.value
        demold_info["type"] = demold_embedded_type.value  # ROUNDING_HEAD=0,ANCHOR=1
        demold_info["name"] = demold_embedded_name  # 脱模预埋件名称
        if demold_info["type"] == 1:  # 若为预埋锚栓
            demold_data = self.demolding_parameter
            demold_shape["m_diameter"] = demold_data.m_diameter
            demold_shape["m_length"] = demold_data.m_length
            demold_shape["o_diameter"] = demold_data.o_diameter
            demold_shape["length"] = demold_data.length
            demold_shape["s_diameter"] = demold_data.s_diameter
            demold_shape["l_p"] = demold_data.l_p
            demold_shape["a"] = demold_data.a
            demold_shape["e_diameter"] = demold_data.e_diameter
            demold_shape["g"] = demold_data.g
            demold_shape["b"] = demold_data.b
        else:  # 若为圆头吊钉
            demold_data = self.demolding_parameter
            demold_shape["top_diameter"] = demold_data.top_diameter
            demold_shape["top_height"] = demold_data.top_height
            demold_shape["top_adjacent_height"] = demold_data.top_adjacent_height
            demold_shape["middle_diameter"] = demold_data.middle_diameter
            demold_shape["middle_height"] = demold_data.middle_height
            demold_shape["bottom_adjacent_height"] = demold_data.bottom_adjacent_height
            demold_shape["bottom_diameter"] = demold_data.bottom_diameter
            demold_shape["bottom_height"] = demold_data.bottom_height

        demold_info["specification"] = demold_shape
        return demold_info

    def get_single_bottom_xie_demold_embedded_points(self, point):
        """
        计算底部斜向单个脱模件的底部标志点和包围框,标志点靠近x=0侧面
        :param point:
        :return:List[List[Point,List[int]]]
        """
        demold_model = []
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        standard_vector = np.array(
            [
                0,
                tabu_b / math.sqrt(tabu_b**2 + tabu_h**2),
                tabu_h / math.sqrt(tabu_b**2 + tabu_h**2),
            ]
        )
        radian = radian_bt_vectors(
            np.array([0, 1, 0]), np.array(standard_vector)
        )  # 旋转角
        if self.demolding_type.value == 1:  # 预埋锚栓
            demold_information = self.demolding_parameter  # 获取预埋锚栓信息
            top_d = demold_information.m_diameter  # 顶部企口直径
            top_height = demold_information.m_length  # 顶部企口高度
            bottom_d = demold_information.o_diameter  # 底部锚栓直径
            bottom_height = demold_information.length  # 底部锚栓长度或高度
            rebar_d = demold_information.s_diameter  # 锚固钢筋直径
            rebar_length = demold_information.l_p  # 锚固钢筋的长度
            rebar_relative_loc = (
                demold_information.length - demold_information.a
            )  # 钢筋相对锚栓的相对位置

            # 企口部分
            add_y_0 = -top_d / 2  #
            add_z_0 = -top_height
            length_ = math.sqrt(add_y_0**2 + add_z_0**2)
            curr_vector = np.asarray(
                [0, add_y_0 / length_, add_z_0 / length_]
            )  # 局部坐标向量
            vector_initial = rotation_3d(
                curr_vector, np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_top = (
                np.array([point.x - top_d / 2, point.y, point.z])
                + vector_initial * length_
            )
            # 企口部分
            top_rabbet_model = [
                Point(
                    x=vector_symbol_point_top[0],
                    y=vector_symbol_point_top[1],
                    z=vector_symbol_point_top[2],
                )
            ]
            top_rabbet_length = [top_d, top_d, top_height]  # 企口包围框
            top_rabbet_model.append(top_rabbet_length)
            # 预留锚栓部分
            length_1 = bottom_d / 2
            vector_initial = rotation_3d(
                np.asarray([0, -1, 0]), np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_bottom = (
                np.array([point.x - bottom_d / 2, point.y, point.z])
                + vector_initial * length_1
            )
            bottom_anchor_model = [
                Point(
                    x=vector_symbol_point_bottom[0],
                    y=vector_symbol_point_bottom[1],
                    z=vector_symbol_point_bottom[2],
                )
            ]
            bottom_anchor_length = [bottom_d, bottom_d, bottom_height]  # 预留锚栓包围框
            bottom_anchor_model.append(bottom_anchor_length)
            # 锚固钢筋部分
            add_y_3 = -rebar_length / 2
            add_z_3 = rebar_relative_loc - rebar_d / 2
            length_3 = math.sqrt(add_y_3**2 + add_z_3**2)
            curr_vector_3 = np.asarray(
                [0, add_y_3 / length_3, add_z_3 / length_3]
            )  # 局部坐标向量
            vector_initial_3 = rotation_3d(
                curr_vector_3, np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_rebar = (
                np.array([point.x - rebar_d / 2, point.y, point.z])
                + vector_initial_3 * length_3
            )
            anchor_rebar_model = [
                Point(
                    x=vector_symbol_point_rebar[0],
                    y=vector_symbol_point_rebar[1],
                    z=vector_symbol_point_rebar[2],
                )
            ]
            anchor_rebar_length = [rebar_d, rebar_length, rebar_d]  # 锚固钢筋包围框
            anchor_rebar_model.append(anchor_rebar_length)
            # 将脱模预埋件的各部件添加
            demold_model.append(top_rabbet_model)
            demold_model.append(bottom_anchor_model)
            demold_model.append(anchor_rebar_model)

        else:  # 圆头吊钉
            demold_information = self.demolding_parameter  # 获取预埋锚栓信息
            semi_circle_r = demold_information.radius  # 顶部半球半径
            top_d = demold_information.top_diameter  # 顶部直径
            top_height = (
                demold_information.top_height + demold_information.top_adjacent_height
            )  # 顶部总高度
            mid_d = demold_information.middle_diameter  # 中部直径
            mid_height = demold_information.middle_height  # 中部高度
            bottom_d = demold_information.bottom_diameter  # 底部直径
            bottom_height = (
                demold_information.bottom_height
                + demold_information.bottom_adjacent_height
            )  # 底部高度
            # 企口
            add_y_0 = -semi_circle_r  #
            add_z_0 = -semi_circle_r
            length_0 = math.sqrt(add_y_0**2 + add_z_0**2)
            curr_vector = np.asarray(
                [0, add_y_0 / length_0, add_z_0 / length_0]
            )  # 局部坐标向量
            vector_initial_0 = rotation_3d(
                curr_vector, np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_1 = (
                np.array([point.x - semi_circle_r, point.y, point.z])
                + vector_initial_0 * length_0
            )
            rabbet_length = [
                2 * semi_circle_r,
                2 * semi_circle_r,
                semi_circle_r,
            ]  # 企口包围框的边长
            rabbet_model = [
                Point(
                    x=vector_symbol_point_1[0],
                    y=vector_symbol_point_1[1],
                    z=vector_symbol_point_1[2],
                )
            ]
            rabbet_model.append(rabbet_length)
            # 吊钉顶部
            top_length = [top_d, top_d, top_height]  # 圆头吊钉顶部包围框
            length_1 = top_d / 2
            vector_initial_1 = rotation_3d(
                np.asarray([0, -1, 0]), np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_top = (
                np.array([point.x - top_d / 2, point.y, point.z])
                + vector_initial_1 * length_1
            )
            top_model = [
                Point(
                    x=vector_symbol_point_top[0],
                    y=vector_symbol_point_top[1],
                    z=vector_symbol_point_top[2],
                )
            ]
            top_model.append(top_length)
            # 吊钉中部
            middle_length = [mid_d, mid_d, mid_height]  # 圆头吊钉中部包围框
            add_y_2 = -mid_d / 2
            add_z_2 = top_height
            length_2 = math.sqrt(add_y_2**2 + add_z_2**2)
            curr_vector_2 = np.asarray(
                [0, add_y_2 / length_2, add_z_2 / length_2]
            )  # 局部坐标向量
            vector_initial_2 = rotation_3d(
                curr_vector_2, np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_mid = (
                np.array([point.x - mid_d / 2, point.y, point.z])
                + vector_initial_2 * length_2
            )
            mid_model = [
                Point(
                    x=vector_symbol_point_mid[0],
                    y=vector_symbol_point_mid[1],
                    z=vector_symbol_point_mid[2],
                )
            ]
            mid_model.append(middle_length)
            # 吊钉底部
            bottom_length = [bottom_d, bottom_d, bottom_height]  # 圆头吊钉底部包围框
            add_y_3 = -bottom_d / 2
            add_z_3 = top_height + mid_height
            length_3 = math.sqrt(add_y_3**2 + add_z_3**2)
            curr_vector_3 = np.asarray(
                [0, add_y_3 / length_3, add_z_3 / length_3]
            )  # 局部坐标向量
            vector_initial_3 = rotation_3d(
                curr_vector_3, np.asarray([1, 0, 0]), radian
            )  # 旋转后的向量
            vector_symbol_point_bottom = (
                np.array([point.x - bottom_d / 2, point.y, point.z])
                + vector_initial_3 * length_3
            )
            bottom_model = [
                Point(
                    x=vector_symbol_point_bottom[0],
                    y=vector_symbol_point_bottom[1],
                    z=vector_symbol_point_bottom[2],
                )
            ]
            bottom_model.append(bottom_length)
            # 将脱模预埋件的各部件添加
            demold_model.append(rabbet_model)
            demold_model.append(top_model)
            demold_model.append(mid_model)
            demold_model.append(bottom_model)
        return demold_model

    def get_single_side_anchor_rebar_bounding_box(
        self, point: Point
    ):  # zc2022/9/23改 TODO
        """
        计算侧面斜向预埋锚栓钢筋的包围框
        :return: List[Point,List]
        """
        demold_name = self.demolding_parameter.name
        demold_information = self.demolding_parameter  # 获取预埋锚栓信息
        rebar_d = demold_information.s_diameter  # 锚固钢筋直径
        rebar_length = demold_information.l_p  # 锚固钢筋的长度
        rebar_relative_loc = (
            demold_information.length - demold_information.a
        )  # 钢筋相对锚栓的相对位置
        anchor_rebar_bounding_box = [rebar_d, rebar_length, rebar_d]  # 锚固钢筋的包围框
        point_center = Point(x=point.x + rebar_relative_loc, y=point.y, z=point.z)
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        standard_vector = np.array(
            [
                0,
                tabu_b / math.sqrt(tabu_b**2 + tabu_h**2),
                tabu_h / math.sqrt(tabu_b**2 + tabu_h**2),
            ]
        )
        radian = radian_bt_vectors(
            np.array([0, 1, 0]), np.array(standard_vector)
        )  # 旋转角

        point_vertex = get_bounding_box_rotation_vertex(
            point_center, radian, rebar_d, rebar_length, rebar_d
        )
        component_model = [point_vertex, anchor_rebar_bounding_box]
        return component_model

    def get_all_side_anchor_embedded_rebar_bounding_box(self):  # zc2022/9/23改
        """
        计算所有侧面脱模埋件为预埋锚栓的锚固钢筋的包围框
        :return:List[List[Point,List]]
        """
        side_rebar_model = []
        demold_points = self.get_demold_embedded_part_loc()  #
        if self.pouring_way.value == 1:  # 立式浇筑立式脱模
            if self.demolding_type.value == 1:  # 脱模埋件为预埋锚栓
                for point in demold_points:
                    bounding_box = self.get_single_side_anchor_rebar_bounding_box(point)
                    side_rebar_model.append(bounding_box)
        elif self.pouring_way.value == 2:  # 卧式浇筑卧式脱模
            pass
        elif self.pouring_way.value == 0:  # 立式浇筑卧式脱模
            if self.demolding_type.value == 1:  # 脱模埋件为预埋锚栓
                points = [demold_points[0], demold_points[1]]
                for point in points:
                    bounding_box = self.get_single_side_anchor_rebar_bounding_box(point)
                    side_rebar_model.append(bounding_box)
        return side_rebar_model

    def get_all_bottom_xie_demold_embedded_points(self):
        """
        计算所有底部脱模预埋件的标志点和包围框
        :return: List[List[List[Point,List[int]]]]
        """
        bottom_xie_model = []
        bottom_xie_symbol_point = self.get_demold_embedded_part_loc()  # 预埋件的定位
        if self.pouring_way.value == 1:  # 立式浇筑立式脱模
            bottom_xie_model = []
        elif self.pouring_way.value == 2:  # 卧式浇筑卧式脱模
            demold_bottom_points = bottom_xie_symbol_point
            for demold_point in demold_bottom_points:
                demold_symbol_bounding_box = (
                    self.get_single_bottom_xie_demold_embedded_points(demold_point)
                )
                bottom_xie_model.append(demold_symbol_bounding_box)
        elif self.pouring_way.value == 0:  # 立式浇筑卧式脱模
            demold_bottom_points = [
                bottom_xie_symbol_point[2],
                bottom_xie_symbol_point[3],
                bottom_xie_symbol_point[4],
                bottom_xie_symbol_point[5],
            ]
            for demold_point in demold_bottom_points:
                demold_symbol_bounding_box = (
                    self.get_single_bottom_xie_demold_embedded_points(demold_point)
                )
                bottom_xie_model.append(demold_symbol_bounding_box)

        return bottom_xie_model

    def get_single_side_demold_bounding_box(self, point: Point) -> List[List[Point]]:
        """
        获取所有侧面脱模预埋件包围框,需要明确侧面的脱模件锚固钢筋的方向是平行于y轴，还是梯段板倾斜方向平行？
        当前书写的是水平方向的锚固钢筋
        :return:[左下角点，右上角点]
        """
        single_demold_model = []  # 单个脱模件模型
        if self.demolding_type.value == 0:  # 圆头吊钉
            demold_name = self.demolding_parameter.name
            demold_information = self.demolding_parameter  # 获取圆头埋件信息
            semi_circle_r = demold_information.radius  # 顶部半球半径
            top_d = demold_information.top_diameter  # 顶部直径
            top_height = (
                demold_information.top_height + demold_information.top_adjacent_height
            )  # 顶部总高度
            mid_d = demold_information.middle_diameter  # 中部直径
            mid_height = demold_information.middle_height  # 中部高度
            bottom_d = demold_information.bottom_diameter  # 底部直径
            bottom_height = (
                demold_information.bottom_height
                + demold_information.bottom_adjacent_height
            )  # 底部高度

            # 企口包围框角点
            semicircle_rabbet_model = [
                Point(
                    x=point.x - semi_circle_r,
                    y=point.y - semi_circle_r,
                    z=point.z - semi_circle_r,
                ),
                Point(x=point.x, y=point.y + semi_circle_r, z=point.z + semi_circle_r),
            ]
            # 顶部包围框角点
            top_model = [
                Point(x=point.x, y=point.y - top_d / 2, z=point.z - top_d / 2),
                Point(
                    x=point.x + top_height, y=point.y + top_d / 2, z=point.z + top_d / 2
                ),
            ]
            # 中部包围框角点
            mid_model = [
                Point(
                    x=point.x + top_height, y=point.y - mid_d / 2, z=point.z - mid_d / 2
                ),
                Point(
                    x=point.x + top_height + mid_height,
                    y=point.y + top_d / 2,
                    z=point.z + mid_d / 2,
                ),
            ]
            # 底部包围框角点
            bottom_model = [
                Point(
                    x=point.x + top_height + mid_height,
                    y=point.y - bottom_d / 2,
                    z=point.z - bottom_d / 2,
                ),
                Point(
                    x=point.x + top_height + mid_height + bottom_height,
                    y=point.y + bottom_d / 2,
                    z=point.z - bottom_d / 2,
                ),
            ]
            single_demold_model.append(semicircle_rabbet_model)
            single_demold_model.append(top_model)
            single_demold_model.append(mid_model)
            single_demold_model.append(bottom_model)

        elif self.demolding_type.value == 1:
            demold_name = self.demolding_parameter.name
            demold_information = self.demolding_parameter  # 获取预埋锚栓信息
            top_d = demold_information.m_diameter  # 顶部企口直径
            top_height = demold_information.m_length  # 顶部企口高度
            bottom_d = demold_information.o_diameter  # 底部锚栓直径
            bottom_height = demold_information.length  # 底部锚栓长度或高度

            # 顶部企口角点模型
            top_rabbet_model = [
                Point(
                    x=point.x - top_height, y=point.y - top_d / 2, z=point.z - top_d / 2
                ),
                Point(x=point.x, y=point.y + top_d / 2, z=point.z + top_d / 2),
            ]
            # 底部预埋锚栓模型
            bottom_model = [
                Point(x=point.x, y=point.y - bottom_d / 2, z=point.z - bottom_d / 2),
                Point(
                    x=point.x + bottom_height,
                    y=point.y + bottom_d / 2,
                    z=point.z + bottom_d / 2,
                ),
            ]
            # 侧面为锚固钢筋的包围框见 get_all_side_anchor_embedded_rebar_bounding_box()函数
            single_demold_model.append(top_rabbet_model)
            single_demold_model.append(bottom_model)

        return single_demold_model

    def get_all_side_demold_embedded_bounding_box_vertexs(
        self,
    ) -> List[List[List[Point]]]:
        """
        计算所有侧面的脱模预埋件(除锚固钢筋外)包围框
        :return:
        """
        side_model = []
        demold_symbol_point = self.get_demold_embedded_part_loc()  # 预埋件的定位
        if self.pouring_way.value == 1:  # 立式浇筑立式脱模
            for side_point in demold_symbol_point:
                demold_bounding_box = self.get_single_side_demold_bounding_box(
                    side_point
                )
                side_model.append(demold_bounding_box)
        elif self.pouring_way.value == 2:  # 卧式浇筑卧式脱模
            side_model = []
        elif self.pouring_way.value == 0:  # 立式浇筑卧式脱模
            side_symbol_points = [demold_symbol_point[0], demold_symbol_point[1]]
            for side_point in side_symbol_points:
                demold_bounding_box = self.get_single_side_demold_bounding_box(
                    side_point
                )
                side_model.append(demold_bounding_box)
        return side_model


class RailingEmbeddedPart(object):
    """
    栏杆预埋件定位信息
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.railing_design_mode = (
            self.detail_slab.inserts_detailed.rail_design_mode
        )  # 栏杆埋件设计模式
        self.railing_parameter = self.detail_book.rail_parameter  # 栏杆埋件参数
        self.railing_layout = (
            self.detail_slab.inserts_detailed.rail_layout
        )  # 栏杆埋件布局: ONLY_RIGHT--0,ONLY_LEFT--1,BOTH--2
        self.rail_number = self.detail_slab.inserts_detailed.rail_number  # 栏杆布置位置
        self.rail_position = self.detail_slab.inserts_detailed.rail_position  # 栏杆定位
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_rail_embedded_part_loc(self) -> List[Point]:
        """
        计算栏杆预埋件的坐标点：栏杆上表面,栏杆布置侧为 上楼梯方向的左右侧
        :return:
        """
        rail_result = []
        if self.railing_design_mode.value != 2:
            rail_loc = self.rail_number  # 栏杆所在的阶数
            x_a = self.rail_position.a  # 横向边距
            z_d = self.railing_parameter.depth  # 栏杆埋入深度
            rail_name = self.railing_parameter.name  # 栏杆预埋件型号
            rail_b = self.rail_position.b  # 栏杆预埋件的一边边长
            # 栏杆布置左右侧
            rebar_x = []
            if self.railing_layout.value == 0:
                rebar_x.append(self.b0 - 0.5 * rail_b - x_a)
            elif self.railing_layout.value == 1:
                rebar_x.append(0.5 * rail_b + x_a)
            elif self.railing_layout.value == 2:
                rebar_x.append(0.5 * rail_b + x_a)
                rebar_x.append(self.b0 - 0.5 * rail_b - x_a)
            rebar_y = []
            for i in rail_loc:
                rebar_y.append(self.lb_d + (i - 0.5) * self.tabu_b)
            rebar_z = []
            for i in rail_loc:
                rebar_z.append(self.h2 - z_d + i * self.tabu_h)
            for i in range(len(rebar_x)):
                for j in range(len(rail_loc)):
                    rail_result.append(Point(x=rebar_x[i], y=rebar_y[j], z=rebar_z[j]))
        return rail_result

    def get_rail_embedded_rabbet_loc(self):
        """
        获取栏杆预埋件企口坐标点
        :return:
        """
        rail_loc = self.get_rail_embedded_part_loc()  # 获取栏杆预埋件坐标点
        return rail_loc

    def get_rail_rabbet_config(self):
        """
        获取栏杆预埋件企口信息
        :return:
        """
        rabbet_info = {}
        if self.railing_design_mode.value != 2:
            rail_config = self.get_rail_datas()  # 获取栏杆预埋件配置信息
            rail_x = rail_config.b
            rail_y = rail_config.a
            rabbet_height = self.railing_parameter.depth  # 企口高度
            rabbet_extend_width = self.railing_parameter.length  # 企口边缘延伸长度
            rabbet_info["rail_x"] = rail_x
            rabbet_info["rail_y"] = rail_y
            rabbet_info["height"] = rabbet_height
            rabbet_info["extend_width"] = rabbet_extend_width
        return rabbet_info

    def get_rail_datas(self):
        """
        得到栏杆预埋件的类型
        :return:
        """
        rail_name = self.railing_parameter.name  # 计算栏杆埋件的规格名称
        single_rail_data = self.railing_parameter  # 获取栏杆预埋件的数据
        return single_rail_data

    def get_single_rail_vertex(self, point: Point) -> List[List[Point]]:
        """
        计算单个栏杆预埋件拆分成四个包围框角点的函数:焊板+C型钢筋(标志点分别为竖向钢筋连接点1,3、水平钢筋形状中心2)
        :return:
        """
        rail_name = self.railing_parameter.name  # 计算栏杆埋件的规格名称
        single_rail_data = self.railing_parameter  # 获取栏杆预埋件的数据
        rail_a = single_rail_data.a
        rail_b = single_rail_data.b
        rail_c = single_rail_data.c
        rail_d = single_rail_data.d
        rail_t = single_rail_data.t
        rail_fi = single_rail_data.fi  # 直径
        single_rail_model = []
        # 焊板角点包围框
        solder_block = [
            Point(x=point.x - rail_b / 2, y=point.y - rail_a / 2, z=point.z - rail_t),
            Point(x=point.x + rail_b / 2, y=point.y + rail_a / 2, z=point.z),
        ]
        single_rail_model.append(solder_block)
        add_x = rail_b - 2 * rail_c  # C型钢筋x方向偏移
        add_y = rail_a - 2 * rail_c
        point_1 = Point(
            x=point.x - add_x / 2, y=point.y - add_y / 2, z=point.z - rail_t
        )  # 竖向钢筋
        point_3 = Point(
            x=point.x - add_x / 2, y=point.y + add_y / 2, z=point.z - rail_t
        )  # 竖向钢筋
        point_2 = Point(
            x=point.x - add_x / 2, y=point.y, z=point.z - rail_t - rail_d
        )  # 水平段钢筋
        vertical_rebar_1 = [
            Point(
                x=point_1.x - rail_fi / 2,
                y=point_1.y - rail_fi / 2,
                z=point_1.z - rail_d - rail_fi / 2,
            ),
            Point(x=point_1.x + rail_fi / 2, y=point_1.y + rail_fi / 2, z=point_1.z),
        ]
        vertical_rebar_2 = [
            Point(
                x=point_2.x - rail_fi / 2,
                y=point_2.y - add_y / 2 + rail_fi / 2,
                z=point_2.z - rail_fi / 2,
            ),
            Point(
                x=point_2.x + rail_fi / 2,
                y=point_2.y + add_y / 2 - rail_fi / 2,
                z=point_2.z + rail_fi / 2,
            ),
        ]
        vertical_rebar_3 = [
            Point(
                x=point_3.x - rail_fi / 2,
                y=point_3.y - rail_fi / 2,
                z=point_3.z - rail_d - rail_fi / 2,
            ),
            Point(x=point_3.x + rail_fi / 2, y=point_3.y + rail_fi / 2, z=point_3.z),
        ]
        rebar_model = [vertical_rebar_1, vertical_rebar_2, vertical_rebar_3]  # C型钢筋模型
        rebar_x = [0, add_x]
        for x in rebar_x:
            for rebar in rebar_model:
                rebar_: List[Point] = []
                _point_1 = Point(x=0, y=0, z=0)
                _point_2 = Point(x=0, y=0, z=0)
                _point_1.x = rebar[0].x + x
                _point_1.y = rebar[0].y
                _point_1.z = rebar[0].z
                _point_2.x = rebar[1].x + x
                _point_2.y = rebar[1].y
                _point_2.z = rebar[1].z
                rebar_.append(_point_1)
                rebar_.append(_point_2)
                single_rail_model.append(rebar_)
        return single_rail_model

    def get_all_rail_bounding_box_vertex(self) -> List[List[List[Point]]]:
        """
        计算所有栏杆预埋件形成的包围框角点数据
        :return:
        """
        rail_points = self.get_rail_embedded_part_loc()  # 获取栏杆预埋件所有坐标点信息
        all_rail_bounding_box_vertexs = []  # 所有栏杆埋件形成角点包围框信息
        for rail_point in rail_points:
            single_rail_bounding_box_vertex = self.get_single_rail_vertex(rail_point)
            all_rail_bounding_box_vertexs.append(single_rail_bounding_box_vertex)
        return all_rail_bounding_box_vertexs


class InternalCorner(object):
    """
    楼梯踏步阴角数据
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length

    def get_internal_corner_location(self):
        """
        获取楼梯踏步阴角坐标点
        :return:
        """
        point_0 = [0, self.lb_d, self.h2]
        tabu_h = self.tabu_h  # 踏步高度
        tabu_b = self.tabu_b  # 踏步宽度
        corner_loc = []
        for num in range(self.n):
            curr_point = Point(0, 0, 0)
            curr_point.x = point_0[0]
            curr_point.y = point_0[1] + num * tabu_b
            curr_point.z = point_0[2] + num * tabu_h
            corner_loc.append(curr_point)
        return corner_loc

    @staticmethod
    def get_internal_corner_config():
        """
        获取楼梯踏步阴角配置信息
        :return:
        """
        internal_info = {}
        internal_info["radius"] = 10
        return internal_info


class ExternalCorner(object):
    """
    楼梯踏步阳角数据
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length

    def get_external_corner_location(self):
        """
        获取楼梯踏步阳角坐标点
        :return:
        """
        point_0 = [0, self.lb_d, self.h2]
        tabu_h = self.tabu_h  # 踏步高度
        tabu_b = self.tabu_b  # 踏步宽度
        corner_loc = []
        for num in range(self.n):
            curr_point = Point(0, 0, 0)
            curr_point.x = point_0[0]
            curr_point.y = point_0[1] + num * tabu_b
            curr_point.z = point_0[2] + (num + 1) * tabu_h
            corner_loc.append(curr_point)
        return corner_loc

    @staticmethod
    def get_external_corner_config():
        """
        获取楼梯踏步阳角配置信息
        :return:
        """
        external_info = {}
        external_info["side"] = 10
        return external_info


class HoleLocation(object):
    """
    预留孔洞定位信息
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  #

    def generate_basic_datas(self):
        """
        产生楼梯基础数据
        :return:
        """
        self.top_hole_position = (
            self.detail_slab.construction_detailed.top_hole_position
        )  # 顶部孔洞位置
        self.bottom_hole_position = (
            self.detail_slab.construction_detailed.bottom_hole_position
        )  # 底部孔洞位置
        self.top_hole_type = (
            self.detail_slab.construction_detailed.top_hole_type
        )  # 顶部孔洞类型
        self.bottom_hole_type = (
            self.detail_slab.construction_detailed.bottom_hole_type
        )  # 底部孔洞类型
        self.top_hole_shape = self.detail_slab.construction_detailed.top_hole  # 顶部孔洞形状
        self.bottom_hole_shape = (
            self.detail_slab.construction_detailed.bottom_hole
        )  # 顶部孔洞形状
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_hole_loc(self) -> List[Point]:
        """
        计算销键或连接孔洞的位置,以孔洞的下边缘的圆心为标志点
        :return:
        """
        # 顶端孔洞边距
        top_right_long_a1 = self.top_hole_position.a1  # 顶端右侧纵向边距
        top_right_horizon_b1 = self.top_hole_position.b1  # 顶端右侧横向边距
        top_left_long_a2 = self.top_hole_position.a2  # 顶端左侧纵向边距
        top_left_horizon_b2 = self.top_hole_position.b2  # 顶端左侧横向边距
        # 底端孔洞边距
        bottom_right_long_a3 = self.bottom_hole_position.a3  # 底端右侧纵向边距
        bottom_right_horizon_b3 = self.bottom_hole_position.b3  # 底端右侧横向边距
        bottom_left_long_a4 = self.bottom_hole_position.a4  # 底端左侧纵向边距
        bottom_left_horizon_b4 = self.bottom_hole_position.b4  # 底端左侧横向边距

        total_l = self.lb_d + self.ln + self.lt_d  # 纵向总长
        total_h = self.h2 + self.height  # 楼梯总高
        hole_model = [
            Point(x=bottom_left_horizon_b4, y=bottom_left_long_a4, z=0),
            Point(x=self.b0 - bottom_right_horizon_b3, y=bottom_right_long_a3, z=0),
            Point(
                x=top_left_horizon_b2, y=total_l - top_left_long_a2, z=total_h - self.h1
            ),
            Point(
                x=self.b0 - top_right_horizon_b1,
                y=total_l - top_right_long_a1,
                z=total_h - self.h1,
            ),
        ]  # 由底端到顶端，由左到向右
        return hole_model

    def get_hole_config(self):
        """
        获得孔洞直径配置信息
        :return:
        """
        hole_info = {}
        hole_info[
            "top_type"
        ] = (
            self.top_hole_type.value
        )  # 存储顶部孔洞类型  HoleType.FIXED_HINGE =0 , HoleType.SLIDING_HINGE=1
        hole_info["bottom_type"] = self.bottom_hole_type.value  # 存储底部孔洞类型
        if self.bottom_hole_type.value == 0:  # 底端孔洞为固定铰支座
            hole_info["bottom_top_dia"] = self.bottom_hole_shape.fix_hinge_c2
            hole_info["bottom_bottom_dia"] = self.bottom_hole_shape.fix_hinge_d2
        else:
            hole_info["bottom_top_dia"] = self.bottom_hole_shape.sliding_hinge_c1
            hole_info["bottom_bottom_dia"] = self.bottom_hole_shape.sliding_hinge_f1
            hole_info["bottom_top_dia_1"] = self.bottom_hole_shape.sliding_hinge_e1
            hole_info["bottom_bottom_dia_1"] = self.bottom_hole_shape.sliding_hinge_d1
            hole_info["bottom_h"] = self.bottom_hole_shape.sliding_hinge_h1
        if self.top_hole_type.value == 1:  # 顶端孔洞为滑动铰支座
            hole_info["top_top_dia"] = self.top_hole_shape.sliding_hinge_c1
            hole_info["top_bottom_dia"] = self.top_hole_shape.sliding_hinge_f1
            hole_info["top_top_dia_1"] = self.top_hole_shape.sliding_hinge_e1
            hole_info["top_bottom_dia_1"] = self.top_hole_shape.sliding_hinge_d1
            hole_info["top_h"] = self.top_hole_shape.sliding_hinge_h1
        else:
            hole_info["top_top_dia"] = self.top_hole_shape.fix_hinge_c2
            hole_info["top_bottom_dia"] = self.top_hole_shape.fix_hinge_d2
        return hole_info

    def get_hole_geometry_shape(self):
        """
        获取楼梯孔洞几何形状
        :return:
        """
        hole_info = {}  # 孔洞信息
        fixed_info = {}  # 固定铰节点
        slide_info = {}  # 滑动铰节点
        if self.bottom_hole_type.value == 0:  # 底端孔洞为固定铰支座
            fixed_info["c2"] = self.bottom_hole_shape.fix_hinge_c2
            fixed_info["d2"] = self.bottom_hole_shape.fix_hinge_d2
        else:
            slide_info["c1"] = self.bottom_hole_shape.sliding_hinge_c1
            slide_info["f1"] = self.bottom_hole_shape.sliding_hinge_f1
            slide_info["e1"] = self.bottom_hole_shape.sliding_hinge_e1
            slide_info["d1"] = self.bottom_hole_shape.sliding_hinge_d1
            slide_info["h1"] = self.bottom_hole_shape.sliding_hinge_h1
        if self.top_hole_type.value == 1:  # 顶端孔洞为滑动铰支座
            slide_info["c1"] = self.top_hole_shape.sliding_hinge_c1
            slide_info["f1"] = self.top_hole_shape.sliding_hinge_f1
            slide_info["e1"] = self.top_hole_shape.sliding_hinge_e1
            slide_info["d1"] = self.top_hole_shape.sliding_hinge_d1
            slide_info["h1"] = self.top_hole_shape.sliding_hinge_h1
        else:
            fixed_info["c2"] = self.top_hole_shape.fix_hinge_c2
            fixed_info["d2"] = self.top_hole_shape.fix_hinge_d2
        hole_info["fixed_hinge"] = fixed_info
        hole_info["slide_hinge"] = slide_info
        return hole_info

    def get_hole_bounding_box(self) -> List[List[float]]:
        """
        计算孔洞的包围框,描述格式：[bottom=[max_x,max_y,max_z]，top=[max_x,max_y,max_z]]，即包围盒的边长
        :return:
        """
        # 初始化数据
        max_x = 0
        max_y = 0
        hole_bounding_box = []
        # 底端孔洞的包围框
        if self.bottom_hole_type.value == 0:  # 底端孔洞为固定铰支座
            max_x = self.bottom_hole_shape.fix_hinge_c2
            max_y = max_x
        elif self.bottom_hole_type.value == 1:  # 底端孔洞为滑动铰支座
            max_x = self.bottom_hole_shape.sliding_hinge_c1
            max_y = max_x
        max_z = self.h2
        hole_bounding_box.append([max_x, max_y, max_z])
        # 顶端孔洞的包围框
        if self.top_hole_type.value == 0:  # 顶端孔洞为固定铰支座
            max_x = self.top_hole_shape.fix_hinge_c2
            max_y = max_x
        elif self.top_hole_type.value == 1:  # 顶端孔洞为滑动铰支座
            max_x = self.top_hole_shape.sliding_hinge_c1
            max_y = max_x
            max_z = self.h2
        hole_bounding_box.append([max_x, max_y, max_z])
        return hole_bounding_box

    def get_bounding_box_vertex(self) -> List[List[Point]]:
        """
        获取包围盒的两顶点坐标，格式：[[左下角=Point(min_x,min_y,min_z)，右上角=Point(max_x,max_y,max_z)]]
        :return:
        """
        hole_loc = self.get_hole_loc()
        hole_bounding_box = self.get_hole_bounding_box()
        hole_vertex_model = []
        for i in range(len(hole_loc)):
            hole_model: List[Point] = []
            if i == 0 or i == 1:  # 底端两孔洞选择底端孔洞的尺寸数据
                point: Point = hole_loc[i]
                length = hole_bounding_box[0]  #
                hole_model.append(
                    Point(
                        x=point.x - length[0] / 2, y=point.y - length[1] / 2, z=point.z
                    )
                )
                hole_model.append(
                    Point(
                        x=point.x + length[0] / 2,
                        y=point.y + length[1] / 2,
                        z=point.z + length[2],
                    )
                )
            else:  # 顶端孔洞供选择顶端孔洞的尺寸数据
                point: Point = hole_loc[i]
                length = hole_bounding_box[1]  #
                hole_model.append(
                    Point(
                        x=point.x - length[0] / 2, y=point.y - length[1] / 2, z=point.z
                    )
                )
                hole_model.append(
                    Point(
                        x=point.x + length[0] / 2,
                        y=point.y + length[1] / 2,
                        z=point.z + length[2],
                    )
                )
            hole_vertex_model.append(hole_model)

        return hole_vertex_model


class LadderBeamAndSlabLoc(object):
    """
    楼梯顶部和底部平台梁和平台板的位置信息
    """

    def __init__(self, slab_design, detail_slab) -> None:
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.beam_slab_info = LadderBeamAndSlabInformation()  # 平台梁和板信息
        self.generate_basic_datas()

    def generate_basic_datas(self):
        """
        产生基本数据
        :return:
        """
        self.node_design_mode = (
            self.detail_slab.construction_detailed.joint_design_mode
        )  # 节点设计模式
        self.top_node = self.detail_slab.construction_detailed.top_joint  # 顶部节点
        self.bottom_node = self.detail_slab.construction_detailed.bottom_joint  # 底部节点
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length  # 顶端上边长
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length  # 底端上边长
        self.ln = self.slab_struct.geometric.clear_span  # 净跨
        self.h = self.slab_struct.geometric.height  # 楼梯高度
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness

    def get_stair_bottom_beam_loc(self):
        """
        获取楼梯底部平台梁的位置信息:点序列为顺时针方向
        :return:[Point,Point,....]
        """
        # 节点类型尺寸
        bottom_node_a = self.bottom_node.a
        bottom_node_b = self.bottom_node.b
        bottom_node_c = self.bottom_node.c
        # 获取底端平台梁尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        bottom_info = beam_and_slab_info["bottom"]
        beam_width = bottom_info.beam_width
        beam_area_width = bottom_info.beam_area_width
        beam_height = bottom_info.beam_height
        # 获取楼梯基本数据
        h2 = self.h2  # 获取楼梯底端板厚度
        # 生成基本坐标点
        point_1 = Point(0, 0, 0)
        point_1.y = -bottom_node_a
        point_1.z = h2 - bottom_node_b
        point_2 = Point(0, 0, 0)
        point_2.y = -bottom_node_a
        point_2.z = -bottom_node_c
        point_3 = Point(0, 0, 0)
        point_3.y = beam_area_width - bottom_node_a
        point_3.z = -bottom_node_c
        point_4 = Point(0, 0, 0)
        point_4.y = beam_area_width - bottom_node_a
        point_4.z = -(beam_height - h2)
        point_5 = Point(0, 0, 0)
        point_5.y = -(beam_width - beam_area_width + bottom_node_a)
        point_5.z = -(beam_height - h2)
        point_6 = Point(0, 0, 0)
        point_6.y = -(beam_width - beam_area_width + bottom_node_a)
        point_6.z = h2 - bottom_node_b
        profile_points = [point_1, point_2, point_3, point_4, point_5, point_6]
        return profile_points

    def get_stair_bottom_beam_stretch_length(self):
        """
        获取楼梯底部平台梁拉伸长度
        :return:List[float]
        """
        # 获取底端平台梁尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        bottom_info = beam_and_slab_info["bottom"]
        beam_length = bottom_info.beam_length
        stretch_length = self.b0 + self.b2
        return [stretch_length, 0, 0]

    def get_stair_top_beam_loc(self):
        """
        获取楼梯顶部平台梁的位置信息：点为顺时针方向
        :return:[Point,Point,...]
        """
        # 节点类型尺寸
        top_node_a = self.top_node.a
        top_node_b = self.top_node.b
        top_node_c = self.top_node.c
        # 获取顶端平台梁尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        top_info = beam_and_slab_info["top"]
        beam_width = top_info.beam_width
        beam_area_width = top_info.beam_area_width
        beam_height = top_info.beam_height
        # 获取楼梯基本数据
        h1 = self.h1  # 获取楼梯顶端板厚度
        h2 = self.h2  # 获取楼梯底端板厚度
        h = self.h  # 楼梯净高
        lb = self.lb_d  # 楼梯底端上边长
        ln = self.ln  # 楼梯净跨
        lt = self.lt_d  # 楼梯顶端上边长
        total_H = h + h2  # 楼梯总高
        total_L = lb + ln + lt  # 楼梯总长
        # 生成基本坐标点
        point_1 = Point(0, 0, 0)
        point_1.y = total_L + top_node_a
        point_1.z = total_H - top_node_b
        point_2 = Point(0, 0, 0)
        point_2.y = total_L + (beam_width - beam_area_width + top_node_a)
        point_2.z = total_H - top_node_b
        point_3 = Point(0, 0, 0)
        point_3.y = total_L + (beam_width - beam_area_width + top_node_a)
        point_3.z = total_H - beam_height
        point_4 = Point(0, 0, 0)
        point_4.y = total_L - (beam_area_width - top_node_a)
        point_4.z = total_H - beam_height
        point_5 = Point(0, 0, 0)
        point_5.y = total_L - (beam_area_width - top_node_a)
        point_5.z = total_H - (h1 + top_node_c)
        point_6 = Point(0, 0, 0)
        point_6.y = total_L + top_node_a
        point_6.z = total_H - (h1 + top_node_c)
        profile_points = [point_1, point_2, point_3, point_4, point_5, point_6]
        return profile_points

    def get_stair_top_beam_stretch_length(self):
        """
        获取楼梯顶部平台梁拉伸长度
        :return:List[float]
        """
        # 获取顶端平台梁尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        bottom_info = beam_and_slab_info["top"]
        beam_length = bottom_info.beam_length
        stretch_length = self.b0 + self.b1
        return [stretch_length, 0, 0]

    def get_stair_bottom_slab_loc(self):
        """
        获取楼梯底部平台板坐标点
        :return: [Point,Point,....]
        """
        # 节点类型尺寸
        bottom_node_a = self.bottom_node.a
        bottom_node_b = self.bottom_node.b
        bottom_node_c = self.bottom_node.c
        # 获取楼梯基本数据
        h2 = self.h2  # 楼梯底端板厚度
        # 获取底端平台梁尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        bottom_info = beam_and_slab_info["bottom"]
        beam_width = bottom_info.beam_width
        beam_area_width = bottom_info.beam_area_width
        beam_height = bottom_info.beam_height
        # 获取底端平台板尺寸信息
        slab_length = bottom_info.slab_length
        slab_width = bottom_info.slab_width
        slab_thickness = bottom_info.slab_thickness
        # 平台板轮廓点
        point_1 = Point(0, 0, 0)
        point_1.y = -(bottom_node_a + beam_width - beam_area_width)
        point_1.z = h2 - bottom_node_b
        point_2 = Point(0, 0, 0)
        point_2.y = -(bottom_node_a + beam_width - beam_area_width)
        point_2.z = h2 - bottom_node_b - slab_thickness
        point_3 = Point(0, 0, 0)
        point_3.y = -(bottom_node_a + beam_width - beam_area_width + slab_length)
        point_3.z = h2 - bottom_node_b - slab_thickness
        point_4 = Point(0, 0, 0)
        point_4.y = -(bottom_node_a + beam_width - beam_area_width + slab_length)
        point_4.z = h2 - bottom_node_b
        profile_points = [point_1, point_2, point_3, point_4]
        return profile_points

    def get_stair_bottom_slab_stretch_length(self):
        """
        获取楼梯底部平台板拉伸长度
        :return:List[float]
        """
        # 获取底端平台板尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        bottom_info = beam_and_slab_info["top"]
        beam_length = bottom_info.beam_length
        stretch_length = self.b0 + self.b2
        return [stretch_length, 0, 0]

    def get_stair_top_slab_loc(self):
        """
        获取楼梯顶部板坐标点
        :return:List[Point,Point,Point,...]
        """
        # 节点类型尺寸
        top_node_a = self.top_node.a
        top_node_b = self.top_node.b
        top_node_c = self.top_node.c
        # 获取顶端平台梁尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        top_info = beam_and_slab_info["top"]
        beam_width = top_info.beam_width
        beam_area_width = top_info.beam_area_width
        beam_height = top_info.beam_height
        # 获取顶端平台板尺寸信息
        slab_width = top_info.slab_width
        slab_thickness = top_info.slab_thickness
        slab_length = top_info.slab_length
        # 获取楼梯基本数据
        h1 = self.h1  # 获取楼梯顶端板厚度
        h2 = self.h2  # 获取楼梯底端板厚度
        h = self.h  # 楼梯净高
        lb = self.lb_d  # 楼梯底端上边长
        ln = self.ln  # 楼梯净跨
        lt = self.lt_d  # 楼梯顶端上边长
        total_H = h + h2  # 楼梯总高
        total_L = lb + ln + lt  # 楼梯总长
        # 平台楼梯轮廓点
        point_1 = Point(0, 0, 0)
        point_1.y = total_L + top_node_a + (beam_width - beam_area_width)
        point_1.z = total_H - top_node_b
        point_2 = Point(0, 0, 0)
        point_2.y = total_L + top_node_a + (beam_width - beam_area_width) + slab_length
        point_2.z = total_H - top_node_b
        point_3 = Point(0, 0, 0)
        point_3.y = total_L + top_node_a + (beam_width - beam_area_width) + slab_length
        point_3.z = total_H - top_node_b - slab_thickness
        point_4 = Point(0, 0, 0)
        point_4.y = total_L + top_node_a + (beam_width - beam_area_width)
        point_4.z = total_H - top_node_b - slab_thickness
        profile_points = [point_1, point_2, point_3, point_4]
        return profile_points

    def get_stair_top_slab_stretch_length(self):
        """
        获取楼梯顶部平台板拉伸长度
        :return:List[float]
        """
        # 获取底端平台板尺寸信息
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets
        bottom_info = beam_and_slab_info["top"]
        beam_length = bottom_info.beam_length
        stretch_length = self.b0 + self.b1
        return [stretch_length, 0, 0]


class ConnectEmbeddedPartLoc(object):
    """
    连接预埋件的位置信息:楼梯孔洞与结构梁连接部分
    """

    def __init__(self, slab_design, detail_slab, struct_book, detail_book) -> None:
        self.slab_struct = slab_design
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.beam_slab_info = LadderBeamAndSlabInformation()  # 平台梁和板信息
        self.connect_element_info = ConnectElementShapeInfo()  # 连接部件信息
        self.generate_basic_datas()

    def generate_basic_datas(self):
        """
        产生基本数据
        :return:
        """
        self.top_hole_position = (
            self.detail_slab.construction_detailed.top_hole_position
        )  # 顶部孔洞位置
        self.bottom_hole_position = (
            self.detail_slab.construction_detailed.bottom_hole_position
        )  # 底部孔洞位置
        self.top_hole_type = (
            self.detail_slab.construction_detailed.top_hole_type
        )  # 顶部孔洞类型
        self.bottom_hole_type = (
            self.detail_slab.construction_detailed.bottom_hole_type
        )  # 底部孔洞类型
        self.top_hole_shape = self.detail_slab.construction_detailed.top_hole  # 顶部孔洞形状
        self.bottom_hole_shape = (
            self.detail_slab.construction_detailed.bottom_hole
        )  # 顶部孔洞形状
        self.node_design_mode = (
            self.detail_slab.construction_detailed.joint_design_mode
        )  # 节点设计模式
        self.top_node = self.detail_slab.construction_detailed.top_joint  # 顶部节点
        self.bottom_node = self.detail_slab.construction_detailed.bottom_joint  # 底部节点
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detail_slab.geometric_detailed.top_thickness
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length
        self.l1t = self.detail_book.top_bottom_length
        self.l1b = self.detail_book.bottom_bottom_length
        self.ln = self.slab_struct.geometric.clear_span
        self.height = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_stair_connect_hole_loc(self):
        """
        获取连接孔洞坐标点:以上侧点为标志点
        :return: {"bottom_hole":bottom_hole_loc,"top_hole":top_hole_loc}
        """
        hole_loc_info = {}
        # 顶端孔洞边距
        # 顶端孔洞边距
        top_right_long_a1 = self.top_hole_position.a1  # 顶端右侧纵向边距
        top_right_horizon_b1 = self.top_hole_position.b1  # 顶端右侧横向边距
        top_left_long_a2 = self.top_hole_position.a2  # 顶端左侧纵向边距
        top_left_horizon_b2 = self.top_hole_position.b2  # 顶端左侧横向边距
        # 底端孔洞边距
        bottom_right_long_a3 = self.bottom_hole_position.a3  # 底端右侧纵向边距
        bottom_right_horizon_b3 = self.bottom_hole_position.b3  # 底端右侧横向边距
        bottom_left_long_a4 = self.bottom_hole_position.a4  # 底端左侧纵向边距
        bottom_left_horizon_b4 = self.bottom_hole_position.b4  # 底端左侧横向边距

        total_l = self.lb_d + self.ln + self.lt_d  # 纵向总长
        total_h = self.h2 + self.height  # 楼梯总高
        bottom_hole_model = [
            Point(x=bottom_left_horizon_b4, y=bottom_left_long_a4, z=self.h2),
            Point(
                x=self.b0 - bottom_right_horizon_b3, y=bottom_right_long_a3, z=self.h2
            ),
        ]  # 由底端到顶端，由左到向右
        top_hole_model = [
            Point(x=top_left_horizon_b2, y=total_l - top_left_long_a2, z=total_h),
            Point(
                x=self.b0 - top_right_horizon_b1,
                y=total_l - top_right_long_a1,
                z=total_h,
            ),
        ]
        hole_loc_info["bottom_hole_loc"] = bottom_hole_model  # 底部孔洞模型
        hole_loc_info["top_hole_loc"] = top_hole_model  # 顶部孔洞模型
        return hole_loc_info

    def get_stair_connect_element_local_info(self):
        """
        获取楼梯连接预埋件各部件局部坐标点
        :return:
        """
        connect_info = {}  # 连接预埋件信息
        top_connect_element_shape = self.connect_element_info.connect_info[
            "top_shape"
        ]  # 顶部形状
        bottom_connect_element_shape = self.connect_element_info.connect_info[
            "bottom_shape"
        ]  # 底部形状
        connect_info["top_shape"] = top_connect_element_shape  # 顶部连接形状
        connect_info["bottom_shape"] = bottom_connect_element_shape  # 底部连接形状
        return connect_info

    def get_stair_connect_rebar_local_location(self):
        """
        获取楼梯连接钢筋局部坐标位置
        :return: List[List[Point,Point,...],List[Point,Point,...]]
        """
        top_connect_element_shape = self.connect_element_info.connect_info[
            "top_shape"
        ]  # 顶部形状
        bottom_connect_element_shape = self.connect_element_info.connect_info[
            "bottom_shape"
        ]  # 底部形状
        top_rebar_anchor_diameter = (
            top_connect_element_shape.rebar_anchor_diameter
        )  # 顶部锚固钢筋直径
        top_rebar_anchor_a = top_connect_element_shape.rebar_anchor_a  # 顶部锚固钢筋长度段a
        top_rebar_anchor_b = top_connect_element_shape.rebar_anchor_b  # 顶部锚固钢筋长度段b
        top_edge_t = top_connect_element_shape.edge_t  # 顶部锚固钢筋顶部边距（混凝土上边缘）
        bottom_rebar_anchor_diameter = (
            bottom_connect_element_shape.rebar_anchor_diameter
        )  # 底部锚固钢筋直径
        bottom_rebar_anchor_a = (
            bottom_connect_element_shape.rebar_anchor_a
        )  # 底部锚固钢筋长度段a
        bottom_rebar_anchor_b = (
            bottom_connect_element_shape.rebar_anchor_b
        )  # 顶部锚固钢筋长度段b
        bottom_edge_t = bottom_connect_element_shape.edge_t  # 底部锚固钢筋顶部边距（混凝土上边缘）
        rebar_model = {}  # 连接锚固钢筋模型
        top_rebar_model = [
            Point(x=0, y=0, z=-top_edge_t),
            Point(
                x=0,
                y=0,
                z=-(self.h1 + top_rebar_anchor_a - top_rebar_anchor_diameter / 2),
            ),
            Point(
                x=0,
                y=(top_rebar_anchor_b - top_rebar_anchor_diameter / 2),
                z=-(self.h1 + top_rebar_anchor_a - top_rebar_anchor_diameter / 2),
            ),
        ]
        bottom_rebar_model = [
            Point(x=0, y=0, z=-bottom_edge_t),
            Point(
                x=0,
                y=0,
                z=-(self.h2 + bottom_rebar_anchor_a - bottom_rebar_anchor_diameter / 2),
            ),
            Point(
                x=0,
                y=-(bottom_rebar_anchor_b - bottom_rebar_anchor_diameter / 2),
                z=-(self.h2 + bottom_rebar_anchor_a - bottom_rebar_anchor_diameter / 2),
            ),
        ]
        rebar_model["top_rebar"] = top_rebar_model
        rebar_model["bottom_rebar"] = bottom_rebar_model
        return rebar_model

    def get_stair_connect_nut_local_location(self):
        """
        获取楼梯连接螺纹局部坐标位置信息：以垫片的最下侧为参考坐标点
        :return: Dict{"key":Point,"key":Point}
        """
        # 存取螺母信息
        nut_model = {}
        # 获取基本数据
        top_connect_element_shape = self.connect_element_info.connect_info[
            "top_shape"
        ]  # 顶部形状
        bottom_connect_element_shape = self.connect_element_info.connect_info[
            "bottom_shape"
        ]  # 底部形状
        top_edge_t = top_connect_element_shape.edge_t  # 顶部锚固钢筋边距
        top_edge_m = top_connect_element_shape.edge_m  # 顶部螺母距离钢筋边距
        top_nut_t = top_connect_element_shape.nut_thickness  # 顶部螺母厚度
        bottom_edge_t = bottom_connect_element_shape.edge_t  # 底部锚固钢筋边距
        bottom_edge_m = bottom_connect_element_shape.edge_m  # 底部螺母距离钢筋边距
        bottom_nut_t = bottom_connect_element_shape.nut_thickness  # 底部螺母厚度
        top_nut_loc = Point(x=0, y=0, z=-(top_edge_t + top_edge_m + top_nut_t))
        bottom_nut_loc = Point(
            x=0, y=0, z=-(bottom_edge_t + bottom_edge_m + bottom_nut_t)
        )
        nut_model["top_nut"] = top_nut_loc
        nut_model["bottom_nut"] = bottom_nut_loc
        return nut_model

    def get_stair_connect_shim_local_location(self):
        """
        获取楼梯连接垫片局部坐标点:以垫片的最下侧为参考坐标点
        :return: Dict{"key":Point],"key":Point}
        """
        # 存取垫片信息
        shim_model = {}
        # 获取基本数据
        top_connect_element_shape = self.connect_element_info.connect_info[
            "top_shape"
        ]  # 顶部形状
        bottom_connect_element_shape = self.connect_element_info.connect_info[
            "bottom_shape"
        ]  # 底部形状
        top_edge_t = top_connect_element_shape.edge_t  # 顶部锚固钢筋边距
        top_edge_m = top_connect_element_shape.edge_m  # 顶部螺母距离钢筋边距
        top_nut_t = top_connect_element_shape.nut_thickness  # 顶部螺母厚度
        top_shim_t = top_connect_element_shape.shim_thickness  # 顶部点pain厚度
        bottom_edge_t = bottom_connect_element_shape.edge_t  # 底部锚固钢筋边距
        bottom_edge_m = bottom_connect_element_shape.edge_m  # 底部螺母距离钢筋边距
        bottom_nut_t = bottom_connect_element_shape.nut_thickness  # 底部螺母厚度
        bottom_shim_t = bottom_connect_element_shape.shim_thickness  # 底部垫片厚度
        top_shim_loc = Point(
            x=0, y=0, z=-(top_edge_t + top_edge_m + top_nut_t + top_shim_t)
        )
        bottom_shim_loc = Point(
            x=0, y=0, z=-(bottom_edge_t + bottom_edge_m + bottom_nut_t + bottom_shim_t)
        )
        shim_model["top_shim"] = top_shim_loc
        shim_model["bottom_shim"] = bottom_shim_loc
        return shim_model
