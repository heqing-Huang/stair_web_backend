"""
# File       : collision_detection.py
# Time       ：2022/9/21 10:00
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""
import copy
import math
from typing import List

import fcl
import numpy as np
from .Fcl_models import Agent, Rebar_fcl, Box_fcl, Diagonal_fcl, Cylinder_fcl
from .tools import rotation_matrix_from_vectors
from stair_detailed.models import (
    DetailedDesign,
    DetailedDesignResult,
    LiftingType,
    DemoldingType,
    RailLayout,
    PouringWay,
)
from stair_structure.model import StructuralDesign, StructuralDesignResult
from .models import Point


class StairObstacle:
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
        self.bottom_bottom_length = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.height = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = self.structure_design.construction.concrete_cover_thickness - 1

        self.top_hole_type = self.detailed_design.construction_detailed.top_hole_type
        self.bottom_hole_type = (
            self.detailed_design.construction_detailed.bottom_hole_type
        )
        self.step_slot_mode = (
            self.detailed_design.construction_detailed.step_slot_design_mode
        )
        self.water_drop_mode = (
            self.detailed_design.construction_detailed.water_drip_design_mode
        )
        self.water_drop_layout = (
            self.detailed_design.construction_detailed.water_drip_layout
        )  # 上 0  下 1  都有 2
        self.rail_design_mode = self.detailed_design.inserts_detailed.rail_design_mode
        self.rail_layout = self.detailed_design.inserts_detailed.rail_layout
        self.rail_parameter = self.detailed_design_result.rail_parameter
        self.lifting_parameter = self.detailed_design_result.lifting_parameter
        self.lifting_type = self.detailed_design.inserts_detailed.lifting_type
        self.demolding_parameter = (
            self.detailed_design_result.demolding_parameter
        )  # 脱模预埋件参数
        self.demolding_type = (
            self.detailed_design.inserts_detailed.demolding_type
        )  # 脱模预埋件类型
        self.pouring_way = (
            self.detailed_design.inserts_detailed.pouring_way
        )  # 楼梯的浇筑方式,影响预埋件的位置及数量

    def get_cover(self) -> List:
        """
        计算楼梯保护层厚度形成的边界障碍物
        要求：1.顶端和底端挑耳除外；2.以楼梯立面图右下角为初始点(0,0,0)，逆时针方向前进。
        :return:
        """

        # 添加底端板三角点
        geometry_points = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, float(self.bottom_thickness)],
            [0.0, float(self.bottom_top_length), float(self.bottom_thickness)],
        ]
        # 添加踏步角点
        for i in range(self.steps_number - 1):
            geometry_points.append(
                [
                    0.0,
                    float(self.bottom_top_length + i * self.steps_b),
                    float(self.bottom_thickness + (i + 1) * self.steps_h),
                ]
            )
            geometry_points.append(
                [
                    0.0,
                    float(self.bottom_top_length + (i + 1) * self.steps_b),
                    float(self.bottom_thickness + (i + 1) * self.steps_h),
                ]
            )
        geometry_points = geometry_points + [
            [
                0.0,
                float(self.bottom_top_length + self.ln),
                float(self.h_total),
            ],  # 添加顶端板右上角点
            [0.0, float(self.l_total), float(self.h_total)],
        ]  # 添加顶端板左上角点
        cover_objs = []
        for i in range(int((len(geometry_points) - 1) / 2)):
            cover_objs.append(
                Box_fcl(
                    x=self.width,
                    y=self.cover,
                    z=geometry_points[2 * i + 1][2] - geometry_points[2 * i][2],
                    position=np.array(
                        [
                            self.width / 2,
                            (
                                geometry_points[2 * i + 1][1]
                                + self.cover
                                + geometry_points[2 * i][1]
                            )
                            / 2,
                            (geometry_points[2 * i + 1][2] + geometry_points[2 * i][2])
                            / 2,
                        ]
                    ),
                )
            )
            cover_objs.append(
                Box_fcl(
                    x=self.width,
                    y=geometry_points[2 * i + 2][1] - geometry_points[2 * i + 1][1],
                    z=self.cover,
                    position=np.array(
                        [
                            self.width / 2,
                            (
                                geometry_points[2 * i + 2][1]
                                + geometry_points[2 * i + 1][1]
                            )
                            / 2,
                            (
                                geometry_points[2 * i + 2][2]
                                + geometry_points[2 * i + 1][2]
                                - self.cover
                            )
                            / 2,
                        ]
                    ),
                )
            )

        cover_objs.append(
            Box_fcl(
                x=float(self.width),
                y=float(self.cover),
                z=float(self.top_thickness),
                position=np.array(
                    [
                        self.width / 2,
                        (2 * self.l_total - self.cover) / 2,
                        (2 * self.h_total - self.top_thickness) / 2,
                    ]
                ),
            )
        )
        cover_objs.append(
            Box_fcl(
                x=float(self.width),
                y=float(self.top_bottom_length),
                z=float(self.cover),
                position=np.array(
                    [
                        self.width / 2,
                        (2 * self.l_total - self.top_bottom_length) / 2,
                        (2 * self.h_total - 2 * self.top_thickness + self.cover) / 2,
                    ]
                ),
            )
        )
        cover_objs.append(
            Box_fcl(
                x=float(self.width),
                y=float(self.bottom_bottom_length),
                z=float(self.cover),
                position=np.array(
                    [self.width / 2, self.bottom_bottom_length / 2, self.cover / 2]
                ),
            )
        )
        cover_objs.append(
            Box_fcl(
                x=float(self.cover + 200),
                y=float(self.l_total),
                z=float(self.h_total),
                position=np.array(
                    [(-200 + self.cover) / 2, self.l_total / 2, self.h_total / 2]
                ),
            )
        )
        cover_objs.append(
            Box_fcl(
                x=float(self.cover + max(self.top_b, self.bottom_b) + 200),
                y=float(self.l_total),
                z=float(self.h_total),
                position=np.array(
                    [
                        (
                            2 * self.width
                            - self.cover
                            + (max(self.top_b, self.bottom_b))
                            + 200
                        )
                        / 2,
                        self.l_total / 2,
                        self.h_total / 2,
                    ]
                ),
            )
        )
        original_direction = np.array([0, 0, 1])
        final_direction = np.array(
            [
                0,
                self.l_total - self.top_bottom_length - self.bottom_bottom_length,
                self.h_total - self.top_thickness,
            ]
        )
        transformation = rotation_matrix_from_vectors(
            original_direction, final_direction
        )
        cover_objs.append(
            Diagonal_fcl(
                x=float(self.width),
                y=float(self.cover),
                z=np.sqrt(np.sum(np.square(final_direction))),
                transformation=transformation,
                position=np.array(
                    [
                        self.width / 2,
                        (
                            self.l_total
                            - self.top_bottom_length
                            + self.bottom_bottom_length
                        )
                        / 2,
                        (self.h_total - self.top_thickness) / 2,
                    ]
                ),
            )
        )
        # 增加顶端板右侧障碍物空间
        original_direction_1 = np.array([0, 0, 1])
        final_direction_1 = np.array(
            [
                0,
                self.l_total
                - self.detailed_design.geometric_detailed.top_top_length
                - self.detailed_design.geometric_detailed.bottom_top_length,
                self.h_total - self.steps_h - self.bottom_thickness,
            ]
        )
        transformation_1 = rotation_matrix_from_vectors(
            original_direction_1, final_direction_1
        )
        cover_objs.append(
            Diagonal_fcl(
                x=float(self.width),
                y=float(self.cover),
                z=np.sqrt(np.sum(np.square(final_direction_1))),
                transformation=transformation_1,
                position=np.array(
                    [
                        self.width / 2,
                        (
                            self.l_total
                            - self.detailed_design.geometric_detailed.top_top_length
                            + self.detailed_design.geometric_detailed.bottom_top_length
                        )
                        / 2,
                        (self.h_total - self.steps_h + self.bottom_thickness) / 2,
                    ]
                ),
            )
        )
        return cover_objs

    def get_hole(self) -> List:
        """
        获取包围盒的两顶点坐标，格式：[[左下角=Point(min_x,min_y,min_z)，右上角=Point(max_x,max_y,max_z)]]
        :return:
        """
        hole_objs = []
        # 顶端孔洞边距
        top_right_long_a1 = (
            self.detailed_design.construction_detailed.top_hole_position.a1
        )  # 顶端右侧纵向边距
        top_right_horizon_b1 = (
            self.detailed_design.construction_detailed.top_hole_position.b1
        )  # 顶端右侧横向边距
        top_left_long_a2 = (
            self.detailed_design.construction_detailed.top_hole_position.a2
        )  # 顶端左侧纵向边距
        top_left_horizon_b2 = (
            self.detailed_design.construction_detailed.top_hole_position.b2
        )  # 顶端左侧横向边距
        # 底端孔洞边距
        bottom_right_long_a3 = (
            self.detailed_design.construction_detailed.bottom_hole_position.a3
        )  # 底端右侧纵向边距
        bottom_right_horizon_b3 = (
            self.detailed_design.construction_detailed.bottom_hole_position.b3
        )  # 底端右侧横向边距
        bottom_left_long_a4 = (
            self.detailed_design.construction_detailed.bottom_hole_position.a4
        )  # 底端左侧纵向边距
        bottom_left_horizon_b4 = (
            self.detailed_design.construction_detailed.bottom_hole_position.b4
        )  # 底端左侧横向边距

        top_hole_length = self.detailed_design.geometric_detailed.top_thickness
        bottom_hole_length = self.detailed_design.geometric_detailed.bottom_thickness

        if (
            self.top_hole_type.value == 0
        ):  # HoleType.FIXED_HINGE =0 , HoleType.SLIDING_HINGE=1
            top_radius = (
                self.detailed_design.construction_detailed.top_hole.fix_hinge_c2 / 2
            )
        else:
            top_radius = (
                self.detailed_design.construction_detailed.top_hole.sliding_hinge_c1 / 2
            )
        if (
            self.bottom_hole_type.value == 0
        ):  # HoleType.FIXED_HINGE =0 , HoleType.SLIDING_HINGE=1
            bottom_radius = (
                self.detailed_design.construction_detailed.bottom_hole.fix_hinge_c2 / 2
            )
        else:
            bottom_radius = (
                self.detailed_design.construction_detailed.bottom_hole.sliding_hinge_c1
                / 2
            )

        hole_objs.append(
            Cylinder_fcl(
                radius=bottom_radius,
                length=bottom_hole_length,
                position=np.array(
                    [
                        bottom_left_horizon_b4,
                        bottom_left_long_a4,
                        0 + bottom_hole_length / 2,
                    ]
                ),
            )
        )
        hole_objs.append(
            Cylinder_fcl(
                radius=bottom_radius,
                length=bottom_hole_length,
                position=np.array(
                    [
                        self.width - bottom_right_horizon_b3,
                        bottom_right_long_a3,
                        0 + bottom_hole_length / 2,
                    ]
                ),
            )
        )
        hole_objs.append(
            Cylinder_fcl(
                radius=top_radius,
                length=top_hole_length,
                position=np.array(
                    [
                        top_left_horizon_b2,
                        self.l_total - top_left_long_a2,
                        self.h_total - top_hole_length / 2,
                    ]
                ),
            )
        )
        hole_objs.append(
            Cylinder_fcl(
                radius=top_radius,
                length=top_hole_length,
                position=np.array(
                    [
                        self.width - top_right_horizon_b1,
                        self.l_total - top_right_long_a1,
                        self.h_total - top_hole_length / 2,
                    ]
                ),
            )
        )
        return hole_objs

    def get_lifting(self):
        lifting_objs = []
        lifting_position_a = self.detailed_design.inserts_detailed.lifting_position.a
        lifting_position_b = self.detailed_design.inserts_detailed.lifting_position.b
        lifting_position_c = float(
            self.detailed_design.inserts_detailed.lifting_position.c
        )  # 吊装预埋件左侧横向边距
        lifting_position_d = float(
            self.detailed_design.inserts_detailed.lifting_position.d
        )  # 吊装预埋件右侧横向边距
        edge_a = (
            self.bottom_top_length + (lifting_position_a - 0.5) * self.steps_b
        )  # 吊装预埋件顶端纵向边距
        edge_b = (
            self.bottom_top_length + (lifting_position_b - 0.5) * self.steps_b
        )  # 吊装预埋件底端纵向边距
        top_h = self.bottom_thickness + lifting_position_a * self.steps_h  # 下部吊装件坐标
        bottom_h = self.bottom_thickness + lifting_position_b * self.steps_h  # 上部吊装件坐标
        position_x = [lifting_position_c, float(self.width - lifting_position_d)]
        position_y = [float(edge_b), float(edge_a)]
        position_z = [float(bottom_h), float(top_h)]
        lifting_positions = [
            [position_x[0], position_y[0], position_z[0]],
            [position_x[0], position_y[1], position_z[1]],
            [position_x[1], position_y[0], position_z[0]],
            [position_x[1], position_y[1], position_z[1]],
        ]
        if self.lifting_type.value == 0:  # ROUNDING_HEAD = 0 #    ANCHOR = 1
            rabbet_radius = self.lifting_parameter.radius  # 顶部半球半径
            top_diameter = self.lifting_parameter.top_diameter  # 顶部直径
            top_height = (
                self.lifting_parameter.top_height
                + self.lifting_parameter.top_adjacent_height
            )  # 顶部总高度
            middle_diameter = self.lifting_parameter.middle_diameter  # 中部直径
            middle_height = self.lifting_parameter.middle_height  # 中部高度
            bottom_diameter = self.lifting_parameter.bottom_diameter  # 底部直径
            bottom_height = (
                self.lifting_parameter.bottom_height
                + self.lifting_parameter.bottom_adjacent_height
            )  # 底部高度
            height_0 = (
                rabbet_radius - self.lifting_parameter.top_height
            ) / 2  # 吊钉的最上端距离台阶面的距离
            for lifting_position in lifting_positions:
                lifting_objs.append(
                    Cylinder_fcl(
                        radius=rabbet_radius,
                        length=rabbet_radius,
                        position=np.array(lifting_position)
                        - np.array([0.0, 0.0, rabbet_radius / 2]),
                    )
                )
                lifting_objs.append(
                    Cylinder_fcl(
                        radius=top_diameter / 2,
                        length=top_height,
                        position=np.array(lifting_position)
                        - np.array([0.0, 0.0, height_0 + top_height / 2]),
                    )
                )
                lifting_objs.append(
                    Cylinder_fcl(
                        radius=middle_diameter / 2,
                        length=middle_height,
                        position=np.array(lifting_position)
                        - np.array(
                            [0.0, 0.0, height_0 + top_height + middle_height / 2]
                        ),
                    )
                )
                lifting_objs.append(
                    Cylinder_fcl(
                        radius=bottom_diameter / 2,
                        length=bottom_height,
                        position=np.array(lifting_position)
                        - np.array(
                            [
                                0.0,
                                0.0,
                                height_0
                                + top_height
                                + middle_height
                                + bottom_height / 2,
                            ]
                        ),
                    )
                )
        else:  # self.lifting_type.value == 1:
            rabbet_length = float(self.lifting_parameter.m_length)  # 企口的深度
            rabbet_diameter = float(self.lifting_parameter.m_diameter)  # 企口的直径
            o_diameter = float(self.lifting_parameter.o_diameter)  # 锚栓直径，最外侧
            e_diameter = float(self.lifting_parameter.e_diameter)  # 螺纹公称直径
            g = float(self.lifting_parameter.g)  # 螺纹嵌入深度
            b = float(self.lifting_parameter.b)  # 螺纹嵌入箭头长度
            length = float(self.lifting_parameter.length)  # 锚栓长度
            s_diameter = float(self.lifting_parameter.s_diameter)  # 卡槽直径，即小钢筋直径
            a = float(self.lifting_parameter.a)  # 卡槽边距
            l_p = float(self.lifting_parameter.l_p)  # 卡槽长度
            transformation = rotation_matrix_from_vectors(
                np.array([[0, 0, 1]]), np.array([[1, 0, 0]])
            )
            for lifting_position in lifting_positions:
                lifting_objs.append(
                    Cylinder_fcl(
                        radius=rabbet_diameter / 2,
                        length=rabbet_length,
                        position=np.array(lifting_position)
                        - np.array([0.0, 0.0, rabbet_length / 2]),
                    )
                )
                lifting_objs.append(
                    Cylinder_fcl(
                        radius=o_diameter / 2,
                        length=length,
                        position=np.array(lifting_position)
                        - np.array([0.0, 0.0, rabbet_length + length / 2]),
                    )
                )
                lifting_objs.append(
                    Rebar_fcl(
                        diameter=s_diameter,
                        length=l_p,
                        transformation=transformation,
                        position=np.array(lifting_position)
                        - np.array([0.0, 0.0, rabbet_length + length - a]),
                    )
                )
        return lifting_objs

    def get_demolding(self):
        demolding_objs = []
        if self.pouring_way.value != 2:  # 0 立式浇筑卧式脱模，1  # 立式浇筑立式脱模，2  # 卧式浇筑卧式脱模
            # 顶端侧面脱模预埋件y坐标
            demolding_a = float(
                self.detailed_design.inserts_detailed.demolding_position.a
            )  # 脱模埋件顶端纵向边距
            demolding_b = float(
                self.detailed_design.inserts_detailed.demolding_position.b
            )  # 脱模埋件底端纵向边距
            demolding_c = float(
                self.detailed_design.inserts_detailed.demolding_position.c
            )  # 脱模埋件左侧横向边距
            demolding_d = float(
                self.detailed_design.inserts_detailed.demolding_position.d
            )  # 脱模埋件右侧横向边距
            demolding_t = float(
                self.detailed_design.inserts_detailed.demolding_position.t
            )  # 脱模埋件厚度方向边距
            # 顶端侧面脱模预埋件y坐标
            demolding_t_y = self.l_total - demolding_a
            # 顶端侧面脱模预埋件z坐标
            demolding_t_z = (
                self.l_total - self.bottom_bottom_length - demolding_a
            ) * self.tan + demolding_t / self.cos
            # 底端侧面脱模预埋件y坐标
            demolding_b_y = demolding_b
            # 底端侧面脱模预埋件z坐标
            demolding_b_z = (
                demolding_b - self.bottom_bottom_length
            ) * self.tan + demolding_t / self.cos
            demolding_positions = [
                [self.width, demolding_t_y, demolding_t_z],
                [self.width, demolding_b_y, demolding_b_z],
            ]
            if self.demolding_type.value == 0:  # ROUNDING_HEAD = 0    ANCHOR = 1
                rabbet_radius = self.lifting_parameter.radius  # 顶部半球半径
                top_diameter = self.lifting_parameter.top_diameter  # 顶部直径
                top_height = (
                    self.lifting_parameter.top_height
                    + self.lifting_parameter.top_adjacent_height
                )  # 顶部总高度
                middle_diameter = self.lifting_parameter.middle_diameter  # 中部直径
                middle_height = self.lifting_parameter.middle_height  # 中部高度
                bottom_diameter = self.lifting_parameter.bottom_diameter  # 底部直径
                bottom_height = (
                    self.lifting_parameter.bottom_height
                    + self.lifting_parameter.bottom_adjacent_height
                )  # 底部高度
                height_0 = (
                    rabbet_radius - self.lifting_parameter.top_height
                ) / 2  # 吊钉的最上端距离台阶面的距离z
                transformation = rotation_matrix_from_vectors(
                    np.array([[0, 0, 1]]), np.array([[-1, 0, 0]])
                )

                for demolding_position in demolding_positions:
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=2 * rabbet_radius,
                            length=rabbet_radius,
                            transformation=transformation,
                            position=np.array(demolding_position)
                            - np.array([rabbet_radius / 2.0, 0.0, 0.0]),
                        )
                    )
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=top_diameter,
                            length=top_height,
                            transformation=transformation,
                            position=np.array(demolding_position)
                            - np.array([height_0 + top_height / 2, 0.0, 0.0]),
                        )
                    )
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=middle_diameter,
                            length=middle_height,
                            transformation=transformation,
                            position=np.array(demolding_position)
                            - np.array(
                                [height_0 + top_height + middle_height / 2, 0.0, 0.0]
                            ),
                        )
                    )
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=bottom_diameter,
                            length=bottom_height,
                            transformation=transformation,
                            position=np.array(demolding_position)
                            - np.array(
                                [
                                    height_0
                                    + top_height
                                    + middle_height
                                    + bottom_height / 2,
                                    0.0,
                                    0.0,
                                ]
                            ),
                        )
                    )
            else:  # self.demolding_type.value == 1:
                rabbet_length = float(self.lifting_parameter.m_length)  # 企口的深度
                rabbet_diameter = float(self.lifting_parameter.m_diameter)  # 企口的直径
                o_diameter = float(self.lifting_parameter.o_diameter)  # 锚栓直径，最外侧
                e_diameter = float(self.lifting_parameter.e_diameter)  # 螺纹公称直径
                g = float(self.lifting_parameter.g)  # 螺纹嵌入深度
                b = float(self.lifting_parameter.b)  # 螺纹嵌入箭头长度
                length = float(self.lifting_parameter.length)  # 锚栓长度
                s_diameter = float(self.lifting_parameter.s_diameter)  # 卡槽直径，即小钢筋直径
                a = float(self.lifting_parameter.a)  # 卡槽边距
                l_p = float(self.lifting_parameter.l_p)  # 卡槽长度
                transformation = rotation_matrix_from_vectors(
                    np.array([[0, 0, 1]]), np.array([[-1, 0, 0]])
                )
                transformation_rebar = rotation_matrix_from_vectors(
                    np.array([[0, 0, 1]]), np.array([[0, self.cos, self.sin]])
                )
                for demolding_position in demolding_positions:
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=rabbet_diameter,
                            length=rabbet_length,
                            transformation=transformation,
                            position=np.array(demolding_position)
                            - np.array([rabbet_length / 2, 0.0, 0.0]),
                        )
                    )
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=o_diameter,
                            length=length,
                            transformation=transformation,
                            position=np.array(demolding_position)
                            - np.array([rabbet_length + length / 2, 0.0, 0.0]),
                        )
                    )
                    demolding_objs.append(
                        Rebar_fcl(
                            diameter=s_diameter,
                            length=l_p,
                            transformation=transformation_rebar,
                            position=np.array(demolding_position)
                            - np.array([rabbet_length + length - a, 0.0, 0.0]),
                        )
                    )
        return demolding_objs

    def get_railing(self):
        railing_objs = []
        if self.rail_design_mode.value == 1:  # MANUAL = 1    NO = 2
            # 获取栏杆预埋件的数据
            rail_a = self.rail_parameter.a
            rail_b = self.rail_parameter.b
            rail_c = self.rail_parameter.c
            rail_d = self.rail_parameter.d
            rail_t = self.rail_parameter.t
            rail_fi = self.rail_parameter.fi  # 直径
            rabbet_depth = float(self.rail_parameter.depth)  # 埋入深度
            rabbet_add_d = float(self.rail_parameter.length)  # 底边长度

            rail_number = self.detailed_design.inserts_detailed.rail_number  # 栏杆所在的阶数
            x_a = self.detailed_design.inserts_detailed.rail_position.a  # 横向边距
            y_b = self.detailed_design.inserts_detailed.rail_position.b  # 纵向距离台阶
            # 栏杆布置左右侧
            position_x = []
            if self.rail_layout.value == 0:  # ONLY_RIGHT = 0 ONLY_LEFT = 1   BOTH = 2
                position_x.append(float(self.width - x_a))
            elif self.rail_layout.value == 1:
                position_x.append(float(x_a))
            else:
                position_x.append(float(x_a))
                position_x.append(float(self.width - x_a))
            position_y = []
            for i in rail_number:
                position_y.append(
                    float(self.bottom_top_length + i * self.steps_b - y_b)
                )  # 边界点？中心点？
            position_z = []
            for i in rail_number:
                position_z.append(float(self.bottom_thickness + i * self.steps_h))
            railing_positions = []
            for i in range(len(position_x)):
                for j in range(len(rail_number)):
                    railing_positions.append(
                        [position_x[i], position_y[j], position_z[j]]
                    )
            transformation = rotation_matrix_from_vectors(
                np.array([[0, 0, 1]]), np.array([[0, 1, 0]])
            )
            for railing_position in railing_positions:
                railing_objs.append(
                    Box_fcl(
                        x=2 * rabbet_add_d + rail_b,
                        y=2 * rabbet_add_d + rail_a,
                        z=rabbet_depth,
                        position=np.array(railing_position)
                        - np.array([0.0, 0.0, rabbet_depth / 2]),
                    )
                )
                railing_objs.append(
                    Box_fcl(
                        x=rail_b,
                        y=rail_a,
                        z=rail_t,
                        position=np.array(railing_position)
                        - np.array([0.0, 0.0, rabbet_depth + rail_t / 2]),
                    )
                )
                railing_objs.append(
                    Rebar_fcl(
                        diameter=rail_fi,
                        length=rail_d,
                        transformation=np.eye(3),
                        position=np.array(railing_position)
                        + np.array(
                            [
                                -(rail_b / 2 - rail_c),
                                rail_a / 2 - rail_c,
                                -(rabbet_depth + rail_t + rail_d / 2),
                            ]
                        ),
                    )
                )
                railing_objs.append(
                    Rebar_fcl(
                        diameter=rail_fi,
                        length=rail_a - 2 * rail_c,
                        transformation=transformation,
                        position=np.array(railing_position)
                        + np.array(
                            [
                                -(rail_b / 2 - rail_c),
                                0.0,
                                -(rabbet_depth + rail_t + rail_d + rail_fi / 2),
                            ]
                        ),
                    )
                )
                railing_objs.append(
                    Rebar_fcl(
                        diameter=rail_fi,
                        length=rail_d,
                        transformation=np.eye(3),
                        position=np.array(railing_position)
                        + np.array(
                            [
                                -(rail_b / 2 - rail_c),
                                -(rail_a / 2 - rail_c),
                                -(rabbet_depth + rail_t + rail_d / 2),
                            ]
                        ),
                    )
                )
                railing_objs.append(
                    Rebar_fcl(
                        diameter=rail_fi,
                        length=rail_d,
                        transformation=np.eye(3),
                        position=np.array(railing_position)
                        + np.array(
                            [
                                rail_b / 2 - rail_c,
                                rail_a / 2 - rail_c,
                                -(rabbet_depth + rail_t + rail_d / 2),
                            ]
                        ),
                    )
                )
                railing_objs.append(
                    Rebar_fcl(
                        diameter=rail_fi,
                        length=rail_a - 2 * rail_c,
                        transformation=transformation,
                        position=np.array(railing_position)
                        + np.array(
                            [
                                rail_b / 2 - rail_c,
                                0.0,
                                -(rabbet_depth + rail_t + rail_d + rail_fi / 2),
                            ]
                        ),
                    )
                )
                railing_objs.append(
                    Rebar_fcl(
                        diameter=rail_fi,
                        length=rail_d,
                        transformation=np.eye(3),
                        position=np.array(railing_position)
                        + np.array(
                            [
                                rail_b / 2 - rail_c,
                                -(rail_a / 2 - rail_c),
                                -(rabbet_depth + rail_t + rail_d / 2),
                            ]
                        ),
                    )
                )
            return railing_objs


class StairFCLModel:
    def __init__(self):
        self.geoms = []  # 几何体列表
        self.objs = []  # 对象列表

    def add_rebar(self, rebar, dia):
        objs = []
        for i in range(len(rebar) - 1):
            line = np.array([rebar[i], rebar[i + 1]])
            x = line[1][0] - line[0][0]
            y = line[1][1] - line[0][1]
            z = line[1][2] - line[0][2]
            length = np.sqrt(np.sum(np.square(np.array([x, y, z])))) - dia
            original_direction = np.array([[0, 0, 1]])
            final_direction = np.array([[x, y, z]])

            transformation = rotation_matrix_from_vectors(
                original_direction, final_direction
            )

            position = np.array(
                [
                    (line[1][0] + line[0][0]) / 2,
                    (line[1][1] + line[0][1]) / 2,
                    (line[1][2] + line[0][2]) / 2,
                ]
            )
            obj = Rebar_fcl(
                diameter=dia,
                length=length,
                transformation=transformation,
                position=position,
            )
            objs.append(obj)
        self.add_obj(objs)

    def add_obj(self, objs_new):
        for obj_new in objs_new:
            if obj_new.type == "Cylinder":
                geo_new = fcl.Cylinder(obj_new.radius, obj_new.length)  # 实例化fcl对象
                T = np.array(obj_new.position)  # 保存中心点T
                obj_new = fcl.CollisionObject(geo_new, fcl.Transform(T))  # 生成碰撞对象
                self.geoms.append(geo_new)
                self.objs.append(obj_new)
            elif obj_new.type == "Box":
                geo_new = fcl.Box(obj_new.x, obj_new.y, obj_new.z)
                R = obj_new.transformation
                T = obj_new.position
                obj_new = fcl.CollisionObject(geo_new, fcl.Transform(R, T))
                self.geoms.append(geo_new)
                self.objs.append(obj_new)
            elif obj_new.type == "Rebar":
                geo_new = fcl.Cylinder(obj_new.diameter / 2, obj_new.length)
                R = obj_new.transformation
                T = obj_new.position
                obj_new = fcl.CollisionObject(geo_new, fcl.Transform(R, T))
                self.geoms.append(geo_new)
                self.objs.append(obj_new)

    def new_rebar_fcl(self, rebar, dia):
        rebar_fcls = []
        for i in range(len(rebar) - 1):
            line = np.array([rebar[i], rebar[i + 1]])
            x = line[1][0] - line[0][0]
            y = line[1][1] - line[0][1]
            z = line[1][2] - line[0][2]
            length = np.sqrt(np.sum(np.square(np.array([x, y, z])))) - dia
            original_direction = np.array([[0, 0, 1]])
            final_direction = np.array([[x, y, z]])

            transformation = rotation_matrix_from_vectors(
                original_direction, final_direction
            )

            position = np.array(
                [
                    (line[1][0] + line[0][0]) / 2,
                    (line[1][1] + line[0][1]) / 2,
                    (line[1][2] + line[0][2]) / 2,
                ]
            )
            rebar_fcl = Rebar_fcl(
                diameter=dia,
                length=length,
                transformation=transformation,
                position=position,
            )
            rebar_fcls.append(rebar_fcl)
        return rebar_fcls

    def new_obj(self, objs_new):
        objs = []
        for obj_new in objs_new:
            if obj_new.type == "Cylinder":
                geo_new = fcl.Cylinder(obj_new.radius, obj_new.length)
                T = np.array(obj_new.position)
                obj_new = fcl.CollisionObject(geo_new, fcl.Transform(T))
                objs.append(obj_new)
            elif obj_new.type == "Rebar":
                geo_new = fcl.Cylinder(obj_new.diameter / 2, obj_new.length)
                R = obj_new.transformation
                T = obj_new.position
                obj_new = fcl.CollisionObject(geo_new, fcl.Transform(R, T))
                objs.append(obj_new)
            elif obj_new.type == "Box":
                geo_new = fcl.Box(obj_new.x, obj_new.y, obj_new.z)
                T = obj_new.position
                R = obj_new.transformation
                obj_new = fcl.CollisionObject(geo_new, fcl.Transform(R, T))
                objs.append(obj_new)
        return objs

    def collision_check_add(self, objs_new):
        """
        :param objs_new: [obj, obj]
        :return:
        """
        # 创建manager
        manager_orginal = (
            fcl.DynamicAABBTreeCollisionManager()
        )  # 实例化（轴对齐的包围盒型动态碰撞检测管理器）
        manager_new = fcl.DynamicAABBTreeCollisionManager()  # 实例化

        manager_orginal.registerObjects(self.objs)  # 注册
        manager_new.registerObjects(objs_new)  # 注册

        manager_orginal.setup()
        manager_new.setup()
        # 创建碰撞请求结构
        crequest = fcl.CollisionRequest(num_max_contacts=10000, enable_contact=True)
        cresult = fcl.CollisionResult()
        cdata = fcl.CollisionData(crequest, cresult)

        # 运行碰撞请求
        manager_orginal.collide(manager_new, cdata, fcl.defaultCollisionCallback)

        if len(cdata.result.contacts) != 0:
            for contact in cdata.result.contacts:
                # 提取contacts中的碰撞几何
                coll_geom_0 = contact.o1  # 碰撞是成对出现的所有是o1,o2
                coll_geom_1 = contact.o2
                # print(coll_geom_0, coll_geom_1, contact.pos)
            return True
        else:
            return False

    def collision_agent(self, agent: Agent):
        # 增加智能体并将新智能体执行碰撞检测

        objs_new = []
        geom = fcl.Sphere(math.ceil(agent.size / 2))
        obj = fcl.CollisionObject(geom, fcl.Transform(np.array(agent.position)))
        objs_new.append(obj)

        # 创建manager
        manager_orginal = fcl.DynamicAABBTreeCollisionManager()  # 实例化
        manager_new = fcl.DynamicAABBTreeCollisionManager()  # 实例化

        manager_orginal.registerObjects(self.objs)  # 注册
        manager_new.registerObjects(objs_new)  # 注册

        manager_orginal.setup()
        manager_new.setup()
        # 创建碰撞请求结构
        crequest = fcl.CollisionRequest(num_max_contacts=10000, enable_contact=True)
        cdata = fcl.CollisionData(crequest, fcl.CollisionResult())

        # 运行碰撞请求
        manager_orginal.collide(manager_new, cdata, fcl.defaultCollisionCallback)

        if len(cdata.result.contacts) != 0:
            for contact in cdata.result.contacts:
                # 提取contacts中的碰撞几何
                coll_geom_0 = contact.o1  # 碰撞是成对出现的所有是o1,o2
                coll_geom_1 = contact.o2
                # print(coll_geom_0, coll_geom_1, contact.pos)
            return True
        else:
            return False
