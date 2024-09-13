"""
# File       : rebar_data.py
# Time       ：2022/9/09 11:27
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""
import math
import numpy as np
from typing import List

from dc_rebar import Rebar, IndexedPolyCurve
from stair_detailed.models import DetailedDesign, DetailedDesignResult

from stair_structure.model import (
    StructuralDesign,
    StructuralDesignResult,
    RebarParameter,
)
from stair_structure import __version__ as structure_v
from .Fcl_models import Rebar_fcl
from .tools import rotation_matrix_from_vectors, get_y


class RebarData(object):
    def __init__(
        self,
        structure_design: StructuralDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design: DetailedDesign,
        detailed_design_result: DetailedDesignResult,
    ):
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result

        self.project_ID = self.structure_design.stair_id.project_ID
        self.stair_ID = self.structure_design.stair_id.stair_ID
        if structure_v > "0.1.4":
            self.rebar_parameter = RebarParameter.by_name(
                structure_design.material.rebar_name
            )
        else:
            self.rebar_parameter = self.structure_design_result.rebar_parameter
        self.hole_rebar_diameter = (
            self.detailed_design.rebar_detailed.hole_reinforce_rebar.diameter
        )  # 7 销键加强筋的直径
        self.lifting_rebar_diameter = (
            self.detailed_design.rebar_detailed.lifting_reinforce_rebar.diameter
        )  # 8 吊装预埋件纵向加强筋的直径
        self.bottom_edge_reinforce_rebar_diameter = (
            self.detailed_design.rebar_detailed.bottom_edge_reinforce_rebar.diameter
        )  # 11 下部边缘加强筋
        self.top_edge_reinforce_rebar_diameter = (
            self.detailed_design.rebar_detailed.top_edge_reinforce_rebar.diameter
        )  # 10 上部边缘加强筋

        self.thickness = self.structure_design.geometric.thickness  # 梯段板厚度

        self.bottom_rebar_diameter = int(
            self.structure_design_result.d_1_actual
        )  # 1 下部纵筋的直径
        self.bottom_rebar_spacing = (
            self.structure_design_result.spacing_1_actual
        )  # 1 下部纵筋的间距
        self.top_rebar_diameter = int(
            self.structure_design_result.d_2_actual
        )  # 2 上部纵筋的直径
        self.top_rebar_spacing = (
            self.structure_design_result.spacing_2_actual
        )  # 2 上部纵筋的间距

        self.bottom_edge_stirrup_diameter = (
            self.detailed_design.rebar_detailed.bottom_edge_stirrup.diameter
        )  # 6 底端边缘箍筋的直径
        self.bottom_edge_stirrup_spacing = (
            self.detailed_design.rebar_detailed.bottom_edge_stirrup.spacing
        )  # 6 底端边缘箍筋的间距
        self.top_edge_stirrup_diameter = (
            self.detailed_design.rebar_detailed.top_edge_stirrup.diameter
        )  # 9 顶端边缘箍筋的直径
        self.top_edge_stirrup_spacing = (
            self.detailed_design.rebar_detailed.top_edge_stirrup.spacing
        )  # 9 顶端边缘箍筋的间距

        self.bottom_edge_longitudinal_rebar_diameter = (
            self.detailed_design.rebar_detailed.bottom_edge_longitudinal_rebar.diameter
        )  # 4 底端边缘纵筋的钢筋直径
        self.bottom_edge_longitudinal_rebar_spacing = (
            self.detailed_design.rebar_detailed.bottom_edge_longitudinal_rebar.spacing
        )  # 4 底端边缘纵筋的钢筋间距

        self.top_edge_longitudinal_rebar_diameter = (
            self.detailed_design.rebar_detailed.top_edge_longitudinal_rebar.diameter
        )  # 5 顶端边缘纵筋的钢筋直径
        self.top_edge_longitudinal_rebar_spacing = (
            self.detailed_design.rebar_detailed.top_edge_longitudinal_rebar.spacing
        )  # 5 顶底端边缘纵筋的钢筋间距
        self.mid_distribution_rebar_diameter = int(
            self.structure_design_result.d_3_actual
        )  # 中部分布筋的直径
        self.mid_distribution_rebar_spacing = (
            self.structure_design_result.spacing_3_actual
        )  # 中部分布筋的间距

        self.steps_number = self.structure_design.geometric.steps_number
        self.steps_h = self.structure_design_result.steps_h
        self.steps_b = self.structure_design_result.steps_b
        self.width = self.detailed_design.geometric_detailed.width
        self.top_b = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.bottom_b = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.bottom_thickness = self.detailed_design.geometric_detailed.bottom_thickness
        self.top_thickness = self.detailed_design.geometric_detailed.top_thickness
        self.bottom_top_length = (
            self.detailed_design.geometric_detailed.bottom_top_length
        )
        self.top_bottom_length = self.detailed_design_result.top_bottom_length
        self.top_top_length = self.detailed_design.geometric_detailed.top_top_length
        self.bottom_bottom_length = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.height = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.cover = self.structure_design.construction.concrete_cover_thickness
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.angle = round(math.acos(self.cos) / math.pi * 180, 2)

        self.lifting_parameter = self.detailed_design_result.lifting_parameter
        self.lifting_type = self.detailed_design.inserts_detailed.lifting_type

    def get_hole_rebar(self):
        start_distance = 50  # 销键加强筋的起步间距
        LENGTH = 280  # 孔洞加强筋平直段长度
        RADIUS = 60  # 孔洞加强筋弯曲半径
        hole_rebar_objs = []  # fcl模型
        hole_a1 = (
            self.detailed_design.construction_detailed.top_hole_position.a1
        )  # 顶端右侧孔洞纵向边距
        hole_b1 = (
            self.detailed_design.construction_detailed.top_hole_position.b1
        )  # 顶端右侧孔洞横向边距
        hole_a2 = (
            self.detailed_design.construction_detailed.top_hole_position.a2
        )  # 顶端左侧孔洞纵向边距
        hole_b2 = (
            self.detailed_design.construction_detailed.top_hole_position.b2
        )  # 顶端左侧孔洞横向边距
        hole_a3 = (
            self.detailed_design.construction_detailed.bottom_hole_position.a3
        )  # 底端右侧孔洞纵向边距
        hole_b3 = (
            self.detailed_design.construction_detailed.bottom_hole_position.b3
        )  # 底端右侧孔洞横向边距
        hole_a4 = (
            self.detailed_design.construction_detailed.bottom_hole_position.a4
        )  # 底端左侧孔洞纵向边距
        hole_b4 = (
            self.detailed_design.construction_detailed.bottom_hole_position.b4
        )  # 底端左侧孔洞横向边距

        top_rebar_x = [hole_b2, self.width - hole_b1]
        top_rebar_y = [self.l_total - hole_a2, self.l_total - hole_a1]
        top_rebar_z = [
            self.h_total - self.top_thickness + start_distance,
            self.h_total - start_distance,
        ]
        bottom_rebar_x = [hole_b4, self.width - hole_b3]
        bottom_rebar_y = [hole_a4, hole_a3]
        bottom_rebar_z = [
            max(
                start_distance,
                self.cover
                + self.bottom_edge_longitudinal_rebar_diameter
                + 1
                + self.bottom_edge_reinforce_rebar_diameter
                + 1
                + int(math.ceil(self.hole_rebar_diameter / 2)),
            ),
            self.bottom_thickness - start_distance,
        ]

        top_hole_rebar_fcl = np.array(
            [
                [-RADIUS, -LENGTH, 0],
                [-RADIUS, RADIUS, 0],
                [RADIUS, RADIUS, 0],
                [RADIUS, -LENGTH, 0],
            ]
        )
        bottom_hole_rebar_fcl = np.array(
            [
                [-RADIUS, LENGTH, 0],
                [-RADIUS, -RADIUS, 0],
                [RADIUS, -RADIUS, 0],
                [RADIUS, LENGTH, 0],
            ]
        )
        original_direction = np.array([[0, 0, 1]])
        for z in top_rebar_z:  # 对底端销键加强筋的z坐标
            for x, y in zip(top_rebar_x, top_rebar_y):  # zip___一一对应，打包成包含元组的列表
                rebar_points = top_hole_rebar_fcl + np.array([x, y, z])
                for i in range(len(rebar_points) - 1):
                    rebar_l = np.sqrt(
                        np.sum(
                            np.square(
                                np.array(
                                    [
                                        rebar_points[i + 1][0] - rebar_points[i][0],
                                        rebar_points[i + 1][1] - rebar_points[i][1],
                                        rebar_points[i + 1][2] - rebar_points[i][2],
                                    ]
                                )
                            )
                        )
                    )
                    final_direction = np.array(
                        [
                            rebar_points[i + 1][0] - rebar_points[i][0],
                            rebar_points[i + 1][1] - rebar_points[i][1],
                            rebar_points[i + 1][2] - rebar_points[i][2],
                        ]
                    )
                    transformation = rotation_matrix_from_vectors(
                        original_direction, final_direction
                    )
                    position = np.array(
                        [
                            (rebar_points[i + 1][0] + rebar_points[i][0]) / 2,
                            (rebar_points[i + 1][1] + rebar_points[i][1]) / 2,
                            (rebar_points[i + 1][2] + rebar_points[i][2]) / 2,
                        ]
                    )
                    hole_rebar_objs.append(
                        Rebar_fcl(
                            diameter=self.hole_rebar_diameter,
                            length=rebar_l,
                            transformation=transformation,
                            position=position,
                        )
                    )
        for z in bottom_rebar_z:
            for x, y in zip(bottom_rebar_x, bottom_rebar_y):
                rebar_points = bottom_hole_rebar_fcl + np.array([x, y, z])
                for i in range(len(rebar_points) - 1):
                    rebar_l = np.sqrt(
                        np.sum(
                            np.square(
                                np.array(
                                    [
                                        rebar_points[i + 1][0] - rebar_points[i][0],
                                        rebar_points[i + 1][1] - rebar_points[i][1],
                                        rebar_points[i + 1][2] - rebar_points[i][2],
                                    ]
                                )
                            )
                        )
                    )
                    final_direction = np.array(
                        [
                            rebar_points[i + 1][0] - rebar_points[i][0],
                            rebar_points[i + 1][1] - rebar_points[i][1],
                            rebar_points[i + 1][2] - rebar_points[i][2],
                        ]
                    )
                    transformation = rotation_matrix_from_vectors(
                        original_direction, final_direction
                    )
                    position = np.array(
                        [
                            (rebar_points[i + 1][0] + rebar_points[i][0]) / 2,
                            (rebar_points[i + 1][1] + rebar_points[i][1]) / 2,
                            (rebar_points[i + 1][2] + rebar_points[i][2]) / 2,
                        ]
                    )
                    hole_rebar_objs.append(
                        Rebar_fcl(
                            diameter=self.hole_rebar_diameter,
                            length=rebar_l,
                            transformation=transformation,
                            position=position,
                        )
                    )
        hole_rebar_BIM = []
        top_hole_rebar_BIM = np.array(
            [
                [-RADIUS, -LENGTH, 0],  # 半圆的三个控制点
                [-RADIUS, 0, 0],
                [0, RADIUS, 0],
                [RADIUS, 0, 0],
                [RADIUS, -LENGTH, 0],
            ]
        )
        bottom_hole_rebar_BIM = np.array(
            [
                [-RADIUS, LENGTH, 0],
                [-RADIUS, 0, 0],
                [0, -RADIUS, 0],
                [RADIUS, 0, 0],
                [RADIUS, LENGTH, 0],
            ]
        )
        for z in top_rebar_z:
            for x, y in zip(top_rebar_x, top_rebar_y):
                rebar_points = (top_hole_rebar_BIM + np.array([x, y, z])).tolist()
                hole_rebar_BIM.append(
                    Rebar(
                        radius=int(self.hole_rebar_diameter / 2),
                        poly=IndexedPolyCurve(
                            points=rebar_points, segments=[[1, 2], [2, 3, 4], [4, 5]]
                        ),
                    )
                )
        for z in bottom_rebar_z:
            for x, y in zip(bottom_rebar_x, bottom_rebar_y):
                rebar_points = (bottom_hole_rebar_BIM + np.array([x, y, z])).tolist()
                hole_rebar_BIM.append(
                    Rebar(
                        radius=int(self.hole_rebar_diameter / 2),
                        poly=IndexedPolyCurve(
                            points=rebar_points, segments=[[1, 2], [2, 3, 4], [4, 5]]
                        ),
                    )
                )
        return hole_rebar_objs, hole_rebar_BIM

    def get_lifting_longitudinal_rebar(self):
        """
        吊点加强纵筋的钢筋数据，五点描述
        :return:
        """
        EDGE = 150  # 吊点纵筋斜段长度
        TOP_EDGE = 10  # 钢筋距离顶端企口下边缘的距离,防腐
        lifting_position_a = self.detailed_design.inserts_detailed.lifting_position.a
        lifting_position_b = self.detailed_design.inserts_detailed.lifting_position.b
        lifting_position_c = float(
            self.detailed_design.inserts_detailed.lifting_position.c
        )  # 吊装预埋件左侧横向边距
        lifting_position_d = float(
            self.detailed_design.inserts_detailed.lifting_position.d
        )  # 吊装预埋件右侧横向边距
        if self.lifting_type.value == 0:  # ROUNDING_HEAD = 0    ANCHOR = 1
            middle_diameter = self.lifting_parameter.middle_diameter  # 吊装预埋件 中部直径
            rabbet_radius = self.lifting_parameter.radius  # 埋入深度 企口
            rebar_top_edge = max(rabbet_radius + TOP_EDGE, self.cover)  # 纵筋边到上边缘的距离

        else:
            middle_diameter = self.lifting_parameter.o_diameter
            rabbet_length = self.lifting_parameter.m_length  # 埋入深度
            rebar_top_edge = max(rabbet_length + TOP_EDGE, self.cover)  # 纵筋边到上边缘的距离

        rebar_b = (
            (self.steps_b + self.thickness / self.sin)
            - (rebar_top_edge + 0.5 * self.lifting_rebar_diameter) / self.tan
            - (self.cover + 0.5 * self.lifting_rebar_diameter) / self.sin
            - (self.cover + 0.5 * self.lifting_rebar_diameter)
        )  # 吊点加强纵筋水平段，若楼梯梯段板厚度小，去掉中部分布筋直径占用的吊点纵筋空间

        rebar_h = rebar_b * self.tan  # 吊点加强纵筋竖直段

        top_h = self.bottom_thickness + lifting_position_a * self.steps_h
        bottom_h = self.bottom_thickness + lifting_position_b * self.steps_h
        rebar_x = [
            lifting_position_c
            - 0.5 * middle_diameter
            - 0.5 * self.lifting_rebar_diameter,
            lifting_position_c
            + 0.5 * middle_diameter
            + 0.5 * self.lifting_rebar_diameter,
            self.width
            - lifting_position_d
            - 0.5 * middle_diameter
            - 0.5 * self.lifting_rebar_diameter,
            self.width
            - lifting_position_d
            + 0.5 * middle_diameter
            + 0.5 * self.lifting_rebar_diameter,
        ]  # x方向钢筋中心点
        rebar_y = [
            self.bottom_top_length
            + (lifting_position_a - 0.5) * self.steps_b
            - (0.5 * self.steps_b - self.cover - 0.5 * self.lifting_rebar_diameter),
            self.bottom_top_length
            + (lifting_position_b - 0.5) * self.steps_b
            - (0.5 * self.steps_b - self.cover - 0.5 * self.lifting_rebar_diameter),
        ]  # y方向钢筋中心点（从过了保护层开始）
        rebar_z = [
            top_h - (rebar_top_edge + 0.5 * self.lifting_rebar_diameter),
            bottom_h - (rebar_top_edge + 0.5 * self.lifting_rebar_diameter),
        ]

        rebar_model = np.array(
            [
                [
                    0,
                    -(EDGE - self.lifting_rebar_diameter / 2) * self.cos,
                    -(rebar_h + (EDGE - self.lifting_rebar_diameter / 2) * self.sin),
                ],
                [0, 0, -rebar_h],
                [0, 0, 0],
                [0, rebar_b, 0],
                [
                    0,
                    rebar_b + (EDGE - self.lifting_rebar_diameter / 2) * self.cos,
                    (EDGE - self.lifting_rebar_diameter / 2) * self.sin,
                ],
            ]
        )
        lifting_longitudinal_rebar_BIM = []
        lifting_longitudinal_rebar_objs = []
        original_direction = np.array([[0, 0, 1]])
        for x in rebar_x:
            for y, z in zip(rebar_y, rebar_z):
                rebar_points = (rebar_model + np.array([x, y, z])).tolist()
                lifting_longitudinal_rebar_BIM.append(
                    Rebar(
                        radius=int(self.lifting_rebar_diameter / 2),
                        poly=IndexedPolyCurve(rebar_points),
                    )
                )
                for i in range(len(rebar_points) - 1):
                    rebar_l = np.sqrt(
                        np.sum(
                            np.square(
                                np.array(
                                    [
                                        rebar_points[i + 1][0] - rebar_points[i][0],
                                        rebar_points[i + 1][1] - rebar_points[i][1],
                                        rebar_points[i + 1][2] - rebar_points[i][2],
                                    ]
                                )
                            )
                        )
                    )
                    final_direction = np.array(
                        [
                            rebar_points[i + 1][0] - rebar_points[i][0],
                            rebar_points[i + 1][1] - rebar_points[i][1],
                            rebar_points[i + 1][2] - rebar_points[i][2],
                        ]
                    )
                    transformation = rotation_matrix_from_vectors(
                        original_direction, final_direction
                    )
                    position = np.array(
                        [
                            (rebar_points[i + 1][0] + rebar_points[i][0]) / 2,
                            (rebar_points[i + 1][1] + rebar_points[i][1]) / 2,
                            (rebar_points[i + 1][2] + rebar_points[i][2]) / 2,
                        ]
                    )
                    lifting_longitudinal_rebar_objs.append(
                        Rebar_fcl(
                            diameter=self.lifting_rebar_diameter,
                            length=rebar_l,
                            transformation=transformation,
                            position=position,
                        )
                    )

        return lifting_longitudinal_rebar_objs, lifting_longitudinal_rebar_BIM

    def get_lifting_point_rebar(self):
        """
        吊点横向加强筋（x向）的钢筋数据，两点描述
        :return:
        """
        TOP_EDGE = 10  # 钢筋距离顶端企口下边缘的距离,防腐

        if self.lifting_type.value == 0:  # ROUNDING_HEAD = 0    ANCHOR = 1
            rabbet_radius = self.lifting_parameter.radius  # 埋入深度 企口
            rebar_top_edge = max(rabbet_radius + TOP_EDGE, self.cover)  # 纵筋边到上边缘的距离

        else:
            rabbet_length = self.lifting_parameter.m_length  # 埋入深度
            rebar_top_edge = max(rabbet_length + TOP_EDGE, self.cover)  # 纵筋边到上边缘的距离

        lifting_position_a = self.detailed_design.inserts_detailed.lifting_position.a
        lifting_position_b = self.detailed_design.inserts_detailed.lifting_position.b
        lifting_position_c = float(
            self.detailed_design.inserts_detailed.lifting_position.c
        )  # 吊装预埋件左侧横向边距
        lifting_position_d = float(
            self.detailed_design.inserts_detailed.lifting_position.d
        )  # 吊装预埋件右侧横向边距
        top_h = self.bottom_thickness + lifting_position_a * self.steps_h
        bottom_h = self.bottom_thickness + lifting_position_b * self.steps_h

        rebar_y = [
            self.bottom_top_length
            + (lifting_position_a - 0.5) * self.steps_b
            - (0.5 * self.steps_b - self.cover - 1.5 * self.lifting_rebar_diameter),
            self.bottom_top_length
            + (lifting_position_b - 0.5) * self.steps_b
            - (0.5 * self.steps_b - self.cover - 1.5 * self.lifting_rebar_diameter),
        ]
        rebar_z = [
            top_h - (rebar_top_edge + 1.5 * self.lifting_rebar_diameter),
            bottom_h - (rebar_top_edge + 1.5 * self.lifting_rebar_diameter),
        ]

        rebar_model = np.array([[self.cover, 0, 0], [self.width - self.cover, 0, 0]])
        lifting_point_rebar_BIM = []
        lifting_point_rebar_objs = []
        original_direction = np.array([[0, 0, 1]])
        for y, z in zip(rebar_y, rebar_z):
            rebar_points = (rebar_model + np.array([0, y, z])).tolist()
            lifting_point_rebar_BIM.append(
                Rebar(
                    radius=int(self.lifting_rebar_diameter / 2),
                    poly=IndexedPolyCurve(rebar_points),
                )
            )
            for i in range(len(rebar_points) - 1):
                rebar_l = np.sqrt(
                    np.sum(
                        np.square(
                            np.array(
                                [
                                    rebar_points[i + 1][0] - rebar_points[i][0],
                                    rebar_points[i + 1][1] - rebar_points[i][1],
                                    rebar_points[i + 1][2] - rebar_points[i][2],
                                ]
                            )
                        )
                    )
                )
                final_direction = np.array(
                    [
                        rebar_points[i + 1][0] - rebar_points[i][0],
                        rebar_points[i + 1][1] - rebar_points[i][1],
                        rebar_points[i + 1][2] - rebar_points[i][2],
                    ]
                )
                transformation = rotation_matrix_from_vectors(
                    original_direction, final_direction
                )
                position = np.array(
                    [
                        (rebar_points[i + 1][0] + rebar_points[i][0]) / 2,
                        (rebar_points[i + 1][1] + rebar_points[i][1]) / 2,
                        (rebar_points[i + 1][2] + rebar_points[i][2]) / 2,
                    ]
                )
                lifting_point_rebar_objs.append(
                    Rebar_fcl(
                        diameter=self.lifting_rebar_diameter,
                        length=rebar_l,
                        transformation=transformation,
                        position=position,
                    )
                )

        return lifting_point_rebar_objs, lifting_point_rebar_BIM

    def get_bottom_edge_reinforce_rebar(self):
        """
        下部边缘加强筋的钢筋数据，3点描述
        :return:
        """

        y_0 = (
            self.bottom_bottom_length
            - (self.cover + 0.5 * self.bottom_edge_reinforce_rebar_diameter) / self.sin
        )  # 底端y坐标点
        z_0 = 0  # 底端z坐标点
        z_1 = self.cover + 0.5 * self.bottom_edge_reinforce_rebar_diameter
        y_1 = get_y(self.tan, y_0, z_0, z_1)
        z_2 = self.h_total - self.top_thickness / 2  # 无特殊依据，定制的钢筋终止点。
        y_2 = get_y(self.tan, y_0, z_0, z_2)
        rebar_model = np.array([[0, self.cover, z_1], [0, y_1, z_1], [0, y_2, z_2]])

        rebar_x = [
            self.cover + 0.5 * self.bottom_edge_reinforce_rebar_diameter,
            self.width - (self.cover + 0.5 * self.bottom_edge_reinforce_rebar_diameter),
        ]
        bottom_edge_reinforce_rebar = []
        for x in rebar_x:
            rebar_points = (rebar_model + np.array([x, 0, 0])).tolist()
            bottom_edge_reinforce_rebar.append(rebar_points)

        return bottom_edge_reinforce_rebar

    def get_top_edge_reinforce_rebar(self):
        """
        上部边缘加强筋的钢筋数据，4点描述
        :return:
        """

        y_0 = (
            self.bottom_bottom_length
            - (
                self.thickness
                - self.cover
                - 0.5 * self.top_edge_reinforce_rebar_diameter
            )
            / self.sin
        )  # 底端y坐标点
        z_0 = 0  # 底端z坐标点
        z_1 = (
            self.cover
            + 0.5 * self.top_edge_reinforce_rebar_diameter
            + (self.bottom_edge_longitudinal_rebar_diameter + 1)
            + (
                max(
                    self.bottom_rebar_diameter,  # 下部纵筋
                    self.bottom_edge_stirrup_diameter,  # 底部边缘箍筋
                    self.bottom_edge_reinforce_rebar_diameter,
                )
                + 1
            )
        )  # 底部边缘加强筋
        y_1 = get_y(self.tan, y_0, z_0, z_1)
        z_2 = self.h_total - (
            self.cover
            + (self.top_edge_longitudinal_rebar_diameter + 1)
            + (self.top_edge_stirrup_diameter + 1)
            + 0.5 * self.top_edge_reinforce_rebar_diameter
        )  #
        y_2 = get_y(self.tan, y_0, z_0, z_2)
        rebar_model = np.array(
            [
                [0, self.cover, z_1],
                [0, y_1, z_1],
                [0, y_2, z_2],
                [0, self.l_total - self.cover, z_2],
            ]
        )
        rebar_x = [
            self.cover + 0.5 * self.top_edge_reinforce_rebar_diameter,
            self.width - (self.cover + 0.5 * self.top_edge_reinforce_rebar_diameter),
        ]
        top_edge_reinforce_rebar = []
        for x in rebar_x:
            rebar_points = (rebar_model + np.array([x, 0, 0])).tolist()
            top_edge_reinforce_rebar.append(rebar_points)

        return top_edge_reinforce_rebar

    def get_bottom_rebar(self):
        """
        下部纵筋的钢筋坐标计算，3点描述
        :return:
        """
        start_distance = 40  # 起步间距
        y_0 = (
            self.bottom_bottom_length
            - (self.cover + 0.5 * self.bottom_rebar_diameter) / self.sin
        )  # 底端y坐标点
        z_0 = 0  # 底端z坐标点
        z_1 = self.cover + 0.5 * self.bottom_rebar_diameter
        y_1 = get_y(self.tan, y_0, z_0, z_1)
        z_2 = self.h_total - self.top_thickness / 2  # 无特殊依据，定制的钢筋终止点。
        y_2 = get_y(self.tan, y_0, z_0, z_2)
        rebar_model = np.array([[0, self.cover, z_1], [0, y_1, z_1], [0, y_2, z_2]])
        rebar_number = (
            math.ceil((self.width - 2 * start_distance) / self.bottom_rebar_spacing) + 1
        )  # 向上取整
        rebar_spacing = (self.width - 2 * start_distance) / (rebar_number - 1)
        rebar_x: List[float] = []  # 下部纵筋x坐标
        for i in range(rebar_number):
            if i == 0:
                rebar_x.append(start_distance)
            elif i == rebar_number - 1:
                rebar_x.append(self.width - start_distance)
            else:
                rebar_x.append(start_distance + i * rebar_spacing)

        bottom_rebar = []
        for x in rebar_x:
            rebar_points = (rebar_model + np.array([x, 0, 0])).tolist()
            bottom_rebar.append(rebar_points)

        return bottom_rebar

    def get_top_rebar(self):
        """
        上部纵筋的钢筋数据，4点描述
        :return:
        """
        start_distance = 35  # 起步间距
        y_0 = (
            self.bottom_bottom_length
            - (self.thickness - self.cover - 0.5 * self.top_rebar_diameter) / self.sin
        )  # 底端y坐标点
        z_0 = 0  # 底端z坐标点
        z_1 = (
            self.cover
            + 0.5 * self.top_rebar_diameter
            + (self.bottom_edge_longitudinal_rebar_diameter + 1)
            + (
                max(
                    self.bottom_rebar_diameter,
                    self.bottom_edge_stirrup_diameter,
                    self.bottom_edge_reinforce_rebar_diameter,
                )
                + 1
            )
        )
        y_1 = get_y(self.tan, y_0, z_0, z_1)
        z_2 = self.h_total - (
            self.cover
            + (self.top_edge_longitudinal_rebar_diameter + 1)
            + (self.top_edge_stirrup_diameter + 1)
            + 0.5 * self.top_rebar_diameter
        )
        y_2 = get_y(self.tan, y_0, z_0, z_2)
        rebar_model = np.array(
            [
                [0, self.cover, z_1],
                [0, y_1, z_1],
                [0, y_2, z_2],
                [0, self.l_total - self.cover, z_2],
            ]
        )
        rebar_number = (
            math.ceil((self.width - 2 * start_distance) / self.top_rebar_spacing) + 1
        )
        rebar_spacing = (self.width - 2 * start_distance) / (rebar_number - 1)
        rebar_x: List[float] = []  # 下部纵筋x坐标
        for i in range(rebar_number):
            if i == 0:
                rebar_x.append(start_distance)
            elif i == rebar_number - 1:
                rebar_x.append(self.width - start_distance)
            else:
                rebar_x.append(start_distance + i * rebar_spacing)
        top_rebar = []
        for x in rebar_x:
            rebar_points = (rebar_model + np.array([x, 0, 0])).tolist()
            top_rebar.append(rebar_points)

        return top_rebar

    def get_bottom_edge_stirrup_rebar(self):
        """
        底部边缘箍筋的钢筋坐标计算，4点描述
        :return:
        """
        start_distance = 60  # 底端边缘箍筋的起步间距
        rebar_range_y = min(
            self.bottom_top_length, self.bottom_bottom_length
        )  # 防止底部边缘箍筋出筋
        rebar_number = (
            math.ceil(
                (self.width + self.bottom_b - 2 * start_distance)
                / self.bottom_edge_stirrup_spacing
            )
            + 1
        )  # 底端边缘箍筋的数量
        rebar_spacing = (self.width + self.bottom_b - 2 * start_distance) / (
            rebar_number - 1
        )  # 底端边缘箍筋的实际间距

        rebar_model = np.array(
            [
                [
                    0,
                    self.cover + 0.5 * self.bottom_edge_stirrup_diameter,
                    self.cover + 0.5 * self.bottom_edge_stirrup_diameter,
                ],
                [
                    0,
                    rebar_range_y
                    - (self.cover + 0.5 * self.bottom_edge_stirrup_diameter),
                    self.cover + 0.5 * self.bottom_edge_stirrup_diameter,
                ],
                [
                    0,
                    rebar_range_y
                    - (self.cover + 0.5 * self.bottom_edge_stirrup_diameter),
                    self.bottom_thickness
                    - (self.cover + 0.5 * self.bottom_edge_stirrup_diameter),
                ],
                [
                    0,
                    self.cover + 0.5 * self.bottom_edge_stirrup_diameter,
                    self.bottom_thickness
                    - (self.cover + 0.5 * self.bottom_edge_stirrup_diameter),
                ],
                [
                    0,
                    self.cover + 0.5 * self.bottom_edge_stirrup_diameter,
                    self.cover + 0.5 * self.bottom_edge_stirrup_diameter,
                ],
            ]
        )
        rebar_x: List[float] = []  # 下部纵筋x坐标
        for i in range(rebar_number):
            if i == 0:
                rebar_x.append(start_distance)
            elif i == rebar_number - 1:
                rebar_x.append(self.width + self.bottom_b - start_distance)
            else:
                rebar_x.append(start_distance + i * rebar_spacing)

        bottom_edge_stirrup_rebar = []
        for x in rebar_x:
            rebar_points = (rebar_model + np.array([x, 0, 0])).tolist()
            bottom_edge_stirrup_rebar.append(rebar_points)
        return bottom_edge_stirrup_rebar

    def get_top_edge_stirrup_rebar(self):
        """
        顶端边缘箍筋的钢筋坐标计算，五点描述
        :return:
        """
        start_distance = 60  # 顶端边缘箍筋的起步间距
        rebar_number = (
            math.ceil(
                (self.width + self.top_b - 2 * start_distance)
                / self.top_edge_stirrup_spacing
            )
            + 1
        )  # 顶端边缘箍筋的数量
        rebar_spacing = (self.width + self.top_b - 2 * start_distance) / (
            rebar_number - 1
        )  # 顶端边缘箍筋的实际间距
        rebar_model = np.array(
            [
                [
                    0,
                    self.bottom_top_length
                    + self.ln
                    + self.cover
                    + 0.5 * self.top_edge_stirrup_diameter,
                    self.h_total
                    - self.top_thickness
                    + (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                ],
                [
                    0,
                    self.l_total - (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                    self.h_total
                    - self.top_thickness
                    + (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                ],
                [
                    0,
                    self.l_total - (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                    self.h_total - (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                ],
                [
                    0,
                    self.bottom_top_length
                    + self.ln
                    + self.cover
                    + 0.5 * self.top_edge_stirrup_diameter,
                    self.h_total - (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                ],
                [
                    0,
                    self.bottom_top_length
                    + self.ln
                    + self.cover
                    + 0.5 * self.top_edge_stirrup_diameter,
                    self.h_total
                    - self.top_thickness
                    + (self.cover + 0.5 * self.top_edge_stirrup_diameter),
                ],
            ]
        )
        rebar_x: List[float] = []  # 下部纵筋x坐标
        for i in range(rebar_number):
            if i == 0:
                rebar_x.append(start_distance)
            elif i == rebar_number - 1:
                rebar_x.append(self.width + self.top_b - start_distance)
            else:
                rebar_x.append(start_distance + i * rebar_spacing)

        top_edge_stirrup_rebar = []
        for x in rebar_x:
            rebar_points = (rebar_model + np.array([x, 0, 0])).tolist()
            top_edge_stirrup_rebar.append(rebar_points)
        return top_edge_stirrup_rebar

    def get_bottom_rein_rebar(self):
        """
        底端边缘纵筋的钢筋模型，两点描述
        :return:
        """
        start_distance = 40  # 底端边缘纵筋的起步距离
        rebar_range_y = min(
            self.bottom_top_length, self.bottom_bottom_length
        )  # 避免底部边缘纵筋和底部边缘箍筋出筋
        rebar_number = (
            math.ceil(
                (rebar_range_y - 2 * start_distance)
                / self.bottom_edge_longitudinal_rebar_spacing
            )
            + 1
        )  # 底端边缘纵筋的数量
        rebar_spacing = (rebar_range_y - 2 * start_distance) / (
            rebar_number - 1
        )  # 底端边缘纵筋的实际间距
        diameter_down = (
            max(
                self.bottom_rebar_diameter,
                self.bottom_edge_stirrup_diameter,
                self.bottom_edge_reinforce_rebar_diameter,
            )
            + 1
        )
        diameter_up = self.top_rebar_diameter + 1

        rebar_model = np.array([[self.cover, 0, 0], [self.width - self.cover, 0, 0]])
        rebar_y: List[float] = []  # 下部纵筋y坐标
        for i in range(rebar_number):
            if i == 0:
                rebar_y.append(start_distance)
            elif i == rebar_number - 1:
                rebar_y.append(rebar_range_y - start_distance)
            else:
                rebar_y.append(start_distance + i * rebar_spacing)
        rebar_z = [
            self.cover
            + diameter_down
            + 0.5 * self.bottom_edge_longitudinal_rebar_diameter,
            self.bottom_thickness
            - (
                self.cover
                + diameter_up
                + 0.5 * self.bottom_edge_longitudinal_rebar_diameter
            ),
        ]

        bottom_rein_rebar = []
        for y in rebar_y:
            for z in rebar_z:
                rebar_points = (rebar_model + np.array([0, y, z])).tolist()
                bottom_rein_rebar.append(rebar_points)
        return bottom_rein_rebar

    def get_top_rein_rebar(self):
        """
        顶端边缘纵筋的钢筋模型，两点描述
        :return:
        """
        start_distance = 40  # 顶端边缘纵筋的起步距离
        rebar_number = (
            math.ceil(
                (self.top_top_length - 2 * start_distance)
                / self.top_edge_longitudinal_rebar_spacing
            )
            + 1
        )  # 底端边缘纵筋的数量
        rebar_spacing = (self.top_top_length - 2 * start_distance) / (
            rebar_number - 1
        )  # 底端边缘纵筋的实际间距

        rebar_model = np.array([[self.cover, 0, 0], [self.width - self.cover, 0, 0]])
        rebar_y: List[float] = []  # 上部纵筋y坐标
        for i in range(rebar_number):
            if i == 0:
                rebar_y.append(self.bottom_top_length + self.ln + start_distance)
            elif i == rebar_number - 1:
                rebar_y.append(self.l_total - start_distance)
            else:
                rebar_y.append(
                    self.bottom_top_length
                    + self.ln
                    + start_distance
                    + i * rebar_spacing
                )
        rebar_z = [
            self.h_total
            - self.top_thickness
            + self.cover
            + self.top_edge_stirrup_diameter
            + 1
            + 0.5 * self.top_edge_longitudinal_rebar_diameter,
            self.h_total
            - (
                self.cover
                + self.top_edge_stirrup_diameter
                + 1
                + 0.5 * self.top_edge_longitudinal_rebar_diameter
            ),
        ]

        top_rein_rebar = []
        for y in rebar_y:
            for z in rebar_z:
                rebar_points = (rebar_model + np.array([0, y, z])).tolist()
                top_rein_rebar.append(rebar_points)
        return top_rein_rebar

    def get_mid_rebar(self):
        """
        中部分布筋钢筋
        :return:
        """
        start_distance = 40  # 中部分布筋的起步间距
        # 该处钢筋直径进行修改
        diameter_bottom = (
            max(self.bottom_rebar_diameter, self.bottom_edge_reinforce_rebar_diameter)
            + 1
        )
        diameter_top = (
            max(self.top_rebar_diameter, self.top_edge_reinforce_rebar_diameter) + 1
        )

        # 计算坐标点
        y_0 = (
            self.bottom_bottom_length
            - (
                self.cover
                + diameter_bottom
                + 0.5 * self.mid_distribution_rebar_diameter
            )
            / self.sin
        )  # 底端y坐标点
        z_0 = 0  # 底端z坐标点
        x_1 = self.cover + 0.5 * self.mid_distribution_rebar_diameter
        x_2 = self.cover + 1.5 * self.mid_distribution_rebar_diameter
        z_1 = (
            (self.cover + diameter_bottom + 0.5 * self.mid_distribution_rebar_diameter)
            / self.tan
            + start_distance
        ) * self.sin
        y_1 = get_y(self.tan, y_0, z_0, z_1)
        mid_distribution_height = (
            self.thickness
            - 2 * self.cover
            - diameter_bottom
            - diameter_top
            - self.mid_distribution_rebar_diameter
        )  # 中部分布筋净高 需要增加本身的直径

        y_2 = y_1 - mid_distribution_height * self.sin
        z_2 = z_1 + mid_distribution_height * self.cos
        x_3 = self.width - (self.cover + 0.5 * self.mid_distribution_rebar_diameter)
        x_4 = self.width - (self.cover + 1.5 * self.mid_distribution_rebar_diameter)

        rebar_model_bottom = np.array(
            [[x_2, y_2, z_2], [x_2, y_1, z_1], [x_3, y_1, z_1], [x_3, y_2, z_2]]
        )
        rebar_model_top = np.array(
            [[x_1, y_1, z_1], [x_1, y_2, z_2], [x_4, y_2, z_2], [x_4, y_1, z_1]]
        )
        rebar_number = (
            math.ceil(
                ((self.h_total - self.top_thickness) / self.sin - 2 * start_distance)
                / self.mid_distribution_rebar_spacing
            )
            + 1
        )
        rebar_spacing = (
            (self.h_total - self.top_thickness) / self.sin - 2 * start_distance
        ) / (rebar_number - 1)

        rebar_y: List[float] = []  # 下部纵筋y坐标
        rebar_z: List[float] = []  # 下部纵筋z坐标

        for i in range(rebar_number):
            rebar_y.append(i * rebar_spacing * self.cos)
            rebar_z.append(i * rebar_spacing * self.sin)

        mid_distribution_rebar = []
        for i in range(rebar_number):
            rebar_points = (
                rebar_model_bottom + np.array([0, rebar_y[i], rebar_z[i]])
            ).tolist()
            mid_distribution_rebar.append(rebar_points)
            rebar_points = (
                rebar_model_top + np.array([0, rebar_y[i], rebar_z[i]])
            ).tolist()
            mid_distribution_rebar.append(rebar_points)

        return mid_distribution_rebar
