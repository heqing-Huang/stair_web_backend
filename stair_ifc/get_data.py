"""
# File       : data_initial.py
# Time       ：2022/9/09 11:27
# Author     ：CR_X
# version    ：python 3.8
# Description：
"""
from dc_rebar import Rebar, IndexedPolyCurve

from stair_structure.model import StructuralDesign, StructuralDesignResult
from stair_detailed.models import (
    DetailedDesign,
    DetailedDesignResult,
    RoundHeadParameter,
    AnchorParameter,
    RailParameter,
)

from typing import List

from stair_ifc.rebar_arc_point import rebar_arc, IfcRebarData


class StairData:
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
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度

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
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.pouring_way = (
            self.detailed_design.inserts_detailed.pouring_way
        )  # 楼梯的浇筑方式,影响预埋件的位置及数量

    def get_step_points(self) -> List[List]:
        """
        计算楼梯主体几何角点
        要求：1.顶端和底端挑耳除外；2.以楼梯立面图右下角为初始点(0,0,0)，逆时针方向前进。
        :return:
        """
        # 添加底端板三角点
        geometry_points = [
            [0.0, 0.0],
            [0.0, float(self.bottom_thickness)],
            [float(self.bottom_top_length), float(self.bottom_thickness)],
        ]
        for i in range(self.steps_number - 1):
            geometry_points.append(
                [
                    self.bottom_top_length + i * self.steps_b,
                    self.bottom_thickness + (i + 1) * self.steps_h,
                ]
            )
            geometry_points.append(
                [
                    self.bottom_top_length + (i + 1) * self.steps_b,
                    self.bottom_thickness + (i + 1) * self.steps_h,
                ]
            )

        geometry_points = geometry_points + [
            [float(self.bottom_top_length + self.ln), float(self.h_total)],
            [float(self.l_total), float(self.h_total)],
            [float(self.l_total), float(self.h_total - self.top_thickness)],
            [
                float(self.l_total - self.top_bottom_length),
                float(self.h_total - self.top_thickness),
            ],
            [float(self.bottom_bottom_length), 0.0],
            [0.0, 0.0],
        ]
        return geometry_points

    def get_top_ear(self) -> List[List]:
        """
        顶端挑耳，逆时针方向。
        :return:
        """
        top_ear_points = [
            [
                float(self.bottom_top_length + self.ln),
                float(self.h_total - self.top_thickness),
            ],
            [float(self.bottom_top_length + self.ln), float(self.h_total)],
            [float(self.l_total), float(self.h_total)],
            [float(self.l_total), float(self.h_total - self.top_thickness)],
            [
                float(self.bottom_top_length + self.ln),
                float(self.h_total - self.top_thickness),
            ],
        ]
        return top_ear_points

    def get_bottom_ear(self) -> List[List]:
        """
        顶端挑耳，逆时针方向。
        :return:
        """
        bottom_ear_points = [
            [0.0, 0.0],
            [0.0, float(self.bottom_thickness)],
            [float(self.bottom_bottom_length), float(self.bottom_thickness)],
            [float(self.bottom_bottom_length), 0.0],
            [0.0, 0.0],
        ]
        return bottom_ear_points

    def get_body_width(self) -> float:
        return float(self.width)

    def get_top_ear_width(self) -> float:
        return float(self.detailed_design.geometric_detailed.top_b)

    def get_bottom_ear_width(self) -> float:
        return float(self.detailed_design.geometric_detailed.bottom_b)

    def get_top_hole_position(self) -> List[List[float]]:
        """
        计算销键或连接孔洞的位置,以孔洞的下边缘的圆心为标志点
        :return:
        """
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

        top_hole_position = [
            [
                float(top_left_horizon_b2),
                float(self.l_total - top_left_long_a2),
                float(self.h_total - self.top_thickness),
            ],
            [
                float(self.width - top_right_horizon_b1),
                float(self.l_total - top_right_long_a1),
                float(self.h_total - self.top_thickness),
            ],
        ]  # 由底端到顶端，由左到向右
        return top_hole_position

    def get_bottom_hole_position(self) -> List[List[float]]:
        """
        计算销键或连接孔洞的位置,以孔洞的下边缘的圆心为标志点
        :return:
        """
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

        bottom_hole_position = [
            [float(bottom_left_horizon_b4), float(bottom_left_long_a4), 0.0],
            [
                float(self.width - bottom_right_horizon_b3),
                float(bottom_right_long_a3),
                0.0,
            ],
        ]

        return bottom_hole_position

    def get_top_hole_section(self) -> List[List[float]]:
        top_hole_type = self.detailed_design.construction_detailed.top_hole_type
        top_hole = self.detailed_design.construction_detailed.top_hole
        if top_hole_type.value == 0:  # 固定支座
            self.top_hole_section = [
                [0.0, 0.0],
                [top_hole.fix_hinge_d2 / 2.0, 0.0],
                [top_hole.fix_hinge_c2 / 2.0, float(self.top_thickness)],
                [0.0, float(self.top_thickness)],
                [0.0, 0.0],
            ]
        else:  # 滑动支座
            self.top_hole_section = [
                [0.0, 0.0],
                [top_hole.sliding_hinge_f1 / 2.0, 0.0],
                [
                    top_hole.sliding_hinge_d1 / 2.0,
                    float(self.top_thickness - top_hole.sliding_hinge_h1),
                ],
                [
                    top_hole.sliding_hinge_e1 / 2.0,
                    float(self.top_thickness - top_hole.sliding_hinge_h1),
                ],
                [top_hole.sliding_hinge_c1 / 2.0, float(self.top_thickness)],
                [0.0, float(self.top_thickness)],
                [0.0, 0.0],
            ]
        return self.top_hole_section

    def get_bottom_hole_section(self) -> List[List[float]]:
        bottom_hole_type = self.detailed_design.construction_detailed.bottom_hole_type
        bottom_hole = self.detailed_design.construction_detailed.bottom_hole
        if bottom_hole_type.value == 0:  # 固定支座
            self.bottom_hole_section = [
                [0.0, 0.0],
                [bottom_hole.fix_hinge_d2 / 2.0, 0.0],
                [bottom_hole.fix_hinge_c2 / 2.0, float(self.top_thickness)],
                [0.0, float(self.bottom_thickness)],
                [0.0, 0.0],
            ]
        else:  # 滑动支座
            self.bottom_hole_section = [
                [0.0, 0.0],
                [bottom_hole.sliding_hinge_f1 / 2.0, 0.0],
                [
                    bottom_hole.sliding_hinge_d1 / 2.0,
                    float(self.bottom_thickness - bottom_hole.sliding_hinge_h1),
                ],
                [
                    bottom_hole.sliding_hinge_e1 / 2.0,
                    float(self.bottom_thickness - bottom_hole.sliding_hinge_h1),
                ],
                [bottom_hole.sliding_hinge_c1 / 2.0, float(self.bottom_thickness)],
                [0.0, float(self.bottom_thickness)],
                [0.0, 0.0],
            ]
        return self.bottom_hole_section

    def get_step_slot_position(self) -> List[List[float]]:
        """
        计算防滑槽的位置信息
        :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]] 最右侧
        """
        step_slot_c1 = self.detailed_design.construction_detailed.step_slot_position.c1
        step_slot_c3 = self.detailed_design.construction_detailed.step_slot_position.c3
        positions = []
        init_position = [
            float(step_slot_c1),
            float(self.bottom_top_length + step_slot_c3),
            float(self.bottom_thickness),
        ]  # 初始位置
        for num in range(self.steps_number):
            position = [
                init_position[0],
                init_position[1] + float(num * self.steps_b),
                init_position[2] + float((num + 1) * self.steps_h),
            ]
            positions.append(position)

        return positions

    def get_slot_position(self) -> List[List[float]]:
        """
        计算防滑槽的位置信息
        :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]] 最右侧
        """
        step_slot_e = self.detailed_design.construction_detailed.step_slot.e
        step_slot_a = self.detailed_design.construction_detailed.step_slot.a
        step_slot_b = self.detailed_design.construction_detailed.step_slot.b
        step_slot_c = self.detailed_design.construction_detailed.step_slot.c

        positions = [
            [float(step_slot_e), 0.0, 0.0],
            [float(step_slot_e), float(step_slot_a + step_slot_b + step_slot_c), 0.0],
        ]
        return positions

    def get_slot_length(self):
        """
        计算防滑槽的位置信息 用于拉伸的主体长度
        :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]]
        """
        step_slot_e = self.detailed_design.construction_detailed.step_slot.e
        step_slot_c1 = self.detailed_design.construction_detailed.step_slot_position.c1
        step_slot_c2 = self.detailed_design.construction_detailed.step_slot_position.c2
        length = float(
            self.width - step_slot_c1 - step_slot_c2 - 2 * step_slot_e
        )  # 防滑槽的长度

        return length

    def get_slot_section(self) -> List[List[float]]:
        """
        计算防滑槽的配置信息 用于拉伸的主要参数
        :return: Dict
        """
        step_slot_a = self.detailed_design.construction_detailed.step_slot.a
        step_slot_b = self.detailed_design.construction_detailed.step_slot.b
        step_slot_d = self.detailed_design.construction_detailed.step_slot.d
        sections = [
            [0.0, 0.0],
            [float(step_slot_a + step_slot_b), 0.0],
            [float(step_slot_b), float(-step_slot_d)],
            [0.0, 0.0],
        ]
        return sections

    def get_slope_position(self):
        """
        计算斜坡的位置信息
        :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]] 最右侧
        """
        step_slot_c1 = self.detailed_design.construction_detailed.step_slot_position.c1
        step_slot_c2 = self.detailed_design.construction_detailed.step_slot_position.c2
        step_slot_a = self.detailed_design.construction_detailed.step_slot.a
        step_slot_b = self.detailed_design.construction_detailed.step_slot.b
        step_slot_c = self.detailed_design.construction_detailed.step_slot.c
        step_slot_e = self.detailed_design.construction_detailed.step_slot.e
        length = float(self.width - step_slot_c2 - step_slot_c1 - step_slot_e)  # 防滑槽的长度
        positions = [
            [[0.0, 0.0, 0.0], [float(length), 0.0, 0.0]],
            [
                [0.0, float(step_slot_a + step_slot_b + step_slot_c), 0.0],
                [float(length), float(step_slot_a + step_slot_b + step_slot_c), 0.0],
            ],
        ]
        return positions

    def get_slope_section(self):
        """
        计算防滑槽的配置信息  小坡面
        :return: Dict
        """
        step_slot_a = self.detailed_design.construction_detailed.step_slot.a
        step_slot_b = self.detailed_design.construction_detailed.step_slot.b
        step_slot_e = self.detailed_design.construction_detailed.step_slot.e
        sections_1 = [
            [
                [0.0, 0.0],
                [0.0, float(step_slot_e)],
                [float(step_slot_a + step_slot_b), float(step_slot_e)],
                [float(step_slot_a + step_slot_b), 0.0],
                [0.0, 0.0],
            ],
            [
                [0.0, 0.0],
                [0.0, float(step_slot_e)],
                [float(step_slot_a + step_slot_b), float(step_slot_e)],
                [float(step_slot_a + step_slot_b), 0.0],
                [0.0, 0.0],
            ],
        ]
        sections_2 = [
            [float(step_slot_b), float(step_slot_e)],
            [float(step_slot_b), 0.0],
        ]  # 截面2的位置

        return sections_1, sections_2

    def get_slope_length(self):
        """
        计算防滑槽的位置信息 用于拉伸的主体长度
        :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]]
        """
        step_slot_d = self.detailed_design.construction_detailed.step_slot.d
        length = float(step_slot_d)  # 防滑槽的长度

        return length

    def get_water_drip_curve(self):
        """
        滴水线槽坐标位置
        :return:
        """
        curves = []
        a1 = self.detailed_design.construction_detailed.water_drip_position.a1
        a2 = self.detailed_design.construction_detailed.water_drip_position.a2
        a3 = self.detailed_design.construction_detailed.water_drip_position.a3
        if self.water_drop_layout.value == 0:  # 只有上部
            curves.append(
                [
                    [float(self.width + self.bottom_b), float(a2), 0.0],
                    [float(self.width - a3), float(a2), 0.0],
                    [float(self.width - a3), float(self.bottom_bottom_length), 0.0],
                    [
                        float(self.width - a3),
                        float(self.l_total - self.top_bottom_length),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        float(self.width - a3),
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        float(self.width + self.top_b),
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                ]
            )
        elif self.water_drop_layout.value == 1:  # 只有下部
            curves.append(
                [
                    [0.0, float(a2), 0.0],
                    [float(a3), float(a2), 0.0],
                    [float(a3), float(self.bottom_bottom_length), 0.0],
                    [
                        float(a3),
                        float(self.l_total - self.top_bottom_length),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        float(a3),
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        0.0,
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                ]
            )
        else:
            curves.append(
                [
                    [float(self.width + self.bottom_b), float(a2), 0.0],
                    [float(self.width - a3), float(a2), 0.0],
                    [float(self.width - a3), float(self.bottom_bottom_length), 0.0],
                    [
                        float(self.width - a3),
                        float(self.l_total - self.top_bottom_length),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        float(self.width - a3),
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        float(self.width + self.top_b),
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                ]
            )
            curves.append(
                [
                    [0.0, float(a2), 0.0],
                    [float(a3), float(a2), 0.0],
                    [float(a3), float(self.bottom_bottom_length), 0.0],
                    [
                        float(a3),
                        float(self.l_total - self.top_bottom_length),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        float(a3),
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                    [
                        0.0,
                        float(self.l_total - a1),
                        float(self.h_total - self.top_thickness),
                    ],
                ]
            )
        return curves

    def get_water_drop_section(self):
        """
        滴水线槽配置信息
        :return:
        """
        section_shape = self.detailed_design.construction_detailed.water_drip_shape
        sections = []
        if section_shape.value == 0:  # TRAPEZOID = 0 SEMICIRCLE = 1
            water_drop_a = self.detailed_design.construction_detailed.water_drip.a
            water_drop_b = self.detailed_design.construction_detailed.water_drip.b
            water_drop_c = self.detailed_design.construction_detailed.water_drip.c
            if self.water_drop_layout.value == 0:  # 只有上部

                sections.append(
                    [
                        [0.0, 0.0],
                        [
                            -float(water_drop_c),
                            float((water_drop_b - water_drop_a) / 2),
                        ],
                        [
                            -float(water_drop_c),
                            float((water_drop_b + water_drop_a) / 2),
                        ],
                        [0.0, float(water_drop_b)],
                    ]
                )
                return sections, section_shape
            elif self.water_drop_layout.value == 1:  # 只有下部
                sections.append(
                    [
                        [0.0, 0.0],
                        [float(water_drop_c), float((water_drop_b - water_drop_a) / 2)],
                        [float(water_drop_c), float((water_drop_b + water_drop_a) / 2)],
                        [0.0, float(water_drop_b)],
                    ]
                )
                return sections, section_shape
            else:
                sections.append(
                    [
                        [0.0, 0.0],
                        [
                            -float(water_drop_c),
                            float((water_drop_b - water_drop_a) / 2),
                        ],
                        [
                            -float(water_drop_c),
                            float((water_drop_b + water_drop_a) / 2),
                        ],
                        [0.0, float(water_drop_b)],
                    ]
                )

                sections.append(
                    [
                        [0.0, 0.0],
                        [float(water_drop_c), float((water_drop_b - water_drop_a) / 2)],
                        [float(water_drop_c), float((water_drop_b + water_drop_a) / 2)],
                        [0.0, float(water_drop_b)],
                    ]
                )
                return sections, section_shape
        else:
            water_drop_a = self.detailed_design.construction_detailed.water_drip.a
            water_drop_b = self.detailed_design.construction_detailed.water_drip.b
            if self.water_drop_layout.value == 0:  # 只有上部
                sections.append(
                    [
                        [0.0, 0.0],
                        [-float(water_drop_b), float(water_drop_a / 2)],
                        [0.0, float(water_drop_a)],
                    ]
                )
                return sections, section_shape
            elif self.water_drop_layout.value == 1:  # 只有下部
                sections.append(
                    [
                        [0.0, 0.0],
                        [float(water_drop_b), float(water_drop_a / 2)],
                        [0.0, float(water_drop_a)],
                    ]
                )
                return sections, section_shape
            else:
                sections.append(
                    [
                        [0.0, 0.0],
                        [-float(water_drop_b), float(water_drop_a / 2)],
                        [0.0, float(water_drop_a)],
                    ]
                )
                sections.append(
                    [
                        [0.0, 0.0],
                        [float(water_drop_b), float(water_drop_a / 2)],
                        [0.0, float(water_drop_a)],
                    ]
                )
                return sections, section_shape

    def get_rail_embedded_parts_position(self) -> List[List[float]]:
        """
        计算栏杆预埋件的坐标点：栏杆上表面,栏杆布置侧为 上楼梯方向的左右侧
        :return:
        """
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
            position_y.append(float(self.bottom_top_length + i * self.steps_b - y_b))
        position_z = []
        for i in rail_number:
            position_z.append(float(self.bottom_thickness + i * self.steps_h))
        positions = []
        for i in range(len(position_x)):
            for j in range(len(rail_number)):
                positions.append([position_x[i], position_y[j], position_z[j]])
        return positions

    def get_lifting_embedded_parts_position(self) -> List[List[float]]:
        """
        计算吊装预埋件的坐标点：预埋件上部中心点
        :return:
        """
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
        positions = [
            [position_x[0], position_y[0], position_z[0]],
            [position_x[0], position_y[1], position_z[1]],
            [position_x[1], position_y[0], position_z[0]],
            [position_x[1], position_y[1], position_z[1]],
        ]

        return positions

    def get_demoulding_embedded_parts_position(self):
        """
        计算脱模预埋件的坐标点
        :return:
        """
        # 获取信息
        # 侧面脱模预埋件的定位
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
        demolding_t_y = self.l_total - demolding_a  # 顶端侧面脱模预埋件y坐标
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

        positions = [
            [float(self.width), demolding_t_y, demolding_t_z],
            [float(self.width), demolding_b_y, demolding_b_z],
        ]
        z_direction = [-1.0, 0.0, 0.0]
        x_direction = [0.0, -self.sin, self.cos]

        return positions, z_direction, x_direction


class RailingSingleData(object):
    """
    单个栏杆预埋件定位信息
    """

    def __init__(self, single_rail_data: RailParameter):
        self.single_rail_data = single_rail_data

        self.rail_a = float(self.single_rail_data.a)  # 栏杆预埋件的长
        self.rail_b = float(self.single_rail_data.b)  # 栏杆预埋件的长
        self.rail_c = float(self.single_rail_data.c)
        self.rail_d = float(self.single_rail_data.d)
        self.rail_fi = float(self.single_rail_data.fi)
        self.rail_t = float(self.single_rail_data.t)
        self.rabbet_depth = float(self.single_rail_data.depth)
        self.rabbet_add_d = float(self.single_rail_data.length)

    def get_rail_embedded_parts_rabbet(self):
        """
        计算栏杆预埋件的企口: 上截面比下截面一边宽5mm, 企口深度15mm
        :return:
        """
        section_1 = [
            [
                -float(self.rail_a / 2 + self.rabbet_add_d),
                float(self.rail_b / 2 + self.rabbet_add_d),
            ],
            [
                float(self.rail_a / 2 + self.rabbet_add_d),
                float(self.rail_b / 2 + self.rabbet_add_d),
            ],
            [
                float(self.rail_a / 2 + self.rabbet_add_d),
                -float(self.rail_b / 2 + self.rabbet_add_d),
            ],
            [
                -float(self.rail_a / 2 + self.rabbet_add_d),
                -float(self.rail_b / 2 + self.rabbet_add_d),
            ],
            [
                -float(self.rail_a / 2 + self.rabbet_add_d),
                float(self.rail_b / 2 + self.rabbet_add_d),
            ],
        ]
        section_2 = [
            [-float(self.rail_a / 2), float(self.rail_b / 2)],
            [float(self.rail_a / 2), float(self.rail_b / 2)],
            [float(self.rail_a / 2), -float(self.rail_b / 2)],
            [-float(self.rail_a / 2), -float(self.rail_b / 2)],
            [-float(self.rail_a / 2), float(self.rail_b / 2)],
        ]
        return section_1, section_2, self.rabbet_depth

    def get_single_rail_plate(self):
        """
        计算单个栏杆预埋件拆分成四个包围框角点的函数:焊板+C型钢筋(标志点分别为竖向钢筋连接点1,3、水平钢筋形状中心2)
        :return:
        """
        # 获取栏杆预埋件的数据
        section = [
            [-float(self.rail_a / 2), float(self.rail_b / 2)],
            [float(self.rail_a / 2), float(self.rail_b / 2)],
            [float(self.rail_a / 2), -float(self.rail_b / 2)],
            [-float(self.rail_a / 2), -float(self.rail_b / 2)],
            [-float(self.rail_a / 2), float(self.rail_b / 2)],
        ]
        return section, self.rail_t

    def get_single_rail_rebar(self) -> List[IfcRebarData]:
        """
        计算单个栏杆预埋件拆分成四个包围框角点的函数:焊板+C型钢筋(标志点分别为竖向钢筋连接点1,3、水平钢筋形状中心2)
        :return:
        """
        new_rebars = []
        # 获取栏杆预埋件的数据
        rebar_points_1 = [
            [
                float(self.rail_a / 2 - self.rail_c),
                float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth),
            ],
            [
                float(self.rail_a / 2 - self.rail_c),
                float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth + self.rail_d),
            ],
            [
                float(self.rail_a / 2 - self.rail_c),
                -float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth + self.rail_d),
            ],
            [
                float(self.rail_a / 2 - self.rail_c),
                -float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth),
            ],
        ]
        rebar_1 = Rebar(
            radius=float(self.rail_fi / 2), poly=IndexedPolyCurve(rebar_points_1)
        )
        new_rebars.append(rebar_arc(rebar=rebar_1))
        rebar_points_2 = [
            [
                -float(self.rail_a / 2 - self.rail_c),
                float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth),
            ],
            [
                -float(self.rail_a / 2 - self.rail_c),
                float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth + self.rail_d),
            ],
            [
                -float(self.rail_a / 2 - self.rail_c),
                -float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth + self.rail_d),
            ],
            [
                -float(self.rail_a / 2 - self.rail_c),
                -float(self.rail_b / 2 - self.rail_c),
                float(self.rail_t + self.rabbet_depth),
            ],
        ]
        rebar_2 = Rebar(
            radius=float(self.rail_fi / 2), poly=IndexedPolyCurve(rebar_points_2)
        )
        new_rebars.append(rebar_arc(rebar=rebar_2))
        return new_rebars


class RoundingHeadSingleData(object):
    """
    圆头吊钉吊装预埋件信息
    """

    def __init__(self, single_rounding_head_data: RoundHeadParameter):
        self.single_rounding_head_data = single_rounding_head_data
        self.rabbet_radius = float(self.single_rounding_head_data.radius)  # 企口的埋入半径
        self.top_diameter = float(self.single_rounding_head_data.top_diameter)  # 顶部直径
        self.top_height = float(self.single_rounding_head_data.top_height)  # 顶部高度
        self.top_adjacent_height = float(
            self.single_rounding_head_data.top_adjacent_height
        )  # 顶部连接高度
        self.middle_diameter = float(
            self.single_rounding_head_data.middle_diameter
        )  # 中间直径
        self.middle_height = float(self.single_rounding_head_data.middle_height)  # 中间高度
        self.bottom_adjacent_height = float(
            self.single_rounding_head_data.bottom_adjacent_height
        )  # 底部连接高度
        self.bottom_diameter = float(
            self.single_rounding_head_data.bottom_diameter
        )  # 底部半径
        self.bottom_height = float(self.single_rounding_head_data.bottom_height)  # 底部高度

    def get_rounding_head_rabbet(self):
        """
        计算圆头吊钉的企口: 半径为 self.rabbet_radius 的半球
        :return:
        """
        rabbet_section = [
            [0.0, 0.0],
            [self.rabbet_radius, 0.0],
            [
                round(2**0.5, 5) / 2 * self.rabbet_radius,
                round(2**0.5, 5) / 2 * self.rabbet_radius,
            ],
            [0.0, self.rabbet_radius],
        ]
        return rabbet_section

    def get_rounding_head_ring(self):
        """
        计算圆头吊钉的半横截面
        :return:
        """
        ring_section = [
            [(self.rabbet_radius - self.top_height) / 2, 0.0],
            [(self.rabbet_radius - self.top_height) / 2, self.top_diameter / 2],
            [(self.rabbet_radius + self.top_height) / 2, self.top_diameter / 2],
            [
                (self.rabbet_radius + self.top_height) / 2 + self.top_adjacent_height,
                self.middle_diameter / 2,
            ],
            [
                (self.rabbet_radius + self.top_height) / 2
                + self.top_adjacent_height
                + self.middle_height,
                self.middle_diameter / 2,
            ],
            [
                (self.rabbet_radius + self.top_height) / 2
                + self.top_adjacent_height
                + self.middle_height
                + self.bottom_adjacent_height,
                self.bottom_diameter / 2,
            ],
            [
                (self.rabbet_radius + self.top_height) / 2
                + self.top_adjacent_height
                + self.middle_height
                + self.bottom_adjacent_height
                + self.bottom_height,
                self.bottom_diameter / 2,
            ],
            [
                (self.rabbet_radius + self.top_height) / 2
                + self.top_adjacent_height
                + self.middle_height
                + self.bottom_adjacent_height
                + self.bottom_height,
                0.0,
            ],
            [(self.rabbet_radius - self.top_height) / 2, 0.0],
        ]
        return ring_section


class AnchorSingleData(object):
    """
    圆头吊钉吊装预埋件信息
    """

    def __init__(self, single_anchor_data: AnchorParameter):
        self.single_anchor_data = single_anchor_data
        self.rabbet_length = float(self.single_anchor_data.m_length)  # 企口的深度
        self.rabbet_diameter = float(self.single_anchor_data.m_diameter)  # 企口的直径
        self.o_diameter = float(self.single_anchor_data.o_diameter)  # 锚栓直径，最外侧
        self.e_diameter = float(self.single_anchor_data.e_diameter)  # 螺纹公称直径
        self.g = float(self.single_anchor_data.g)  # 螺纹嵌入深度
        self.b = float(self.single_anchor_data.b)  # 螺纹嵌入箭头长度
        self.length = float(self.single_anchor_data.length)  # 锚栓长度
        self.s_diameter = float(self.single_anchor_data.s_diameter)  # 卡槽直径，即小钢筋直径
        self.a = float(self.single_anchor_data.a)  # 卡槽边距
        self.l_p = float(self.single_anchor_data.l_p)  # 卡槽长度

    def get_anchor_rabbet(self):
        """
        计算圆头吊钉的企口: 半径为 self.rabbet_radius 的半球
        :return:
        """
        rabbet_section = [
            [0.0, 0.0],
            [self.rabbet_length, 0.0],
            [self.rabbet_length, self.rabbet_diameter / 2],
            [0.0, self.rabbet_diameter / 2],
            [0.0, 0.0],
        ]
        return rabbet_section

    def get_anchor(self):
        """
        计算锚栓的半横截面 参考坐标系是旋转成的锚栓局部坐标系
        :return:
        """
        ring_section = [
            [self.rabbet_length, self.e_diameter / 2],
            [self.rabbet_length, self.o_diameter / 2],
            [self.rabbet_length + self.length, self.o_diameter / 2],
            [self.rabbet_length + self.length, 0.0],
            [self.rabbet_length + self.g + self.b, 0.0],
            [self.rabbet_length + self.g, self.e_diameter / 2],
            [self.rabbet_length, self.e_diameter / 2],
        ]
        return ring_section

    def get_anchor_rebar(self):
        """
        计算锚栓中钢筋 参考坐标系是预埋件的局部坐标系
        :return:
        """
        rebar_points = [
            [
                float(self.l_p / 2),
                0.0,
                float(self.rabbet_length + self.length - self.a),
            ],
            [
                -float(self.l_p / 2),
                0.0,
                float(self.rabbet_length + self.length - self.a),
            ],
        ]
        rebar = Rebar(
            radius=float(self.s_diameter / 2), poly=IndexedPolyCurve(rebar_points)
        )
        new_rebar = rebar_arc(rebar=rebar)
        return new_rebar


# class StairGeometryData:
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult):
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#
#         self.steps_number = self.structure_design.geometric.steps_number
#         self.steps_h = self.structure_design_result.steps_h
#         self.steps_b = self.structure_design_result.steps_b
#         self.width = self.detailed_design.geometric_detailed.width
#
#         self.top_b = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
#         self.bottom_b = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
#         self.bottom_thickness = self.detailed_design.geometric_detailed.bottom_thickness
#         self.top_thickness = self.detailed_design.geometric_detailed.top_thickness
#         self.bottom_top_length = self.detailed_design.geometric_detailed.bottom_top_length
#         self.top_bottom_length = self.detailed_design_result.top_bottom_length
#         self.bottom_bottom_length = self.detailed_design_result.bottom_bottom_length
#         self.ln = self.structure_design.geometric.clear_span
#         self.height = self.structure_design.geometric.height
#         self.l_total = self.detailed_design_result.l_total  # 纵向总长
#         self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
#
#     def get_step_points(self) -> List[List]:
#         """
#         计算楼梯主体几何角点
#         要求：1.顶端和底端挑耳除外；2.以楼梯立面图右下角为初始点(0,0,0)，逆时针方向前进。
#         :return:
#         """
#         # 添加底端板三角点
#         geometry_points = [[0., 0.],
#                            [0., float(self.bottom_thickness)],
#                            [float(self.bottom_top_length), float(self.bottom_thickness)]]
#         for i in range(self.steps_number - 1):
#             geometry_points.append(
#                 [self.bottom_top_length + i * self.steps_b, self.bottom_thickness + (i + 1) * self.steps_h])
#             geometry_points.append(
#                 [self.bottom_top_length + (i + 1) * self.steps_b, self.bottom_thickness + (i + 1) * self.steps_h])
#
#         geometry_points = geometry_points + [
#             [float(self.bottom_top_length + self.ln), float(self.h_total)],
#             [float(self.l_total), float(self.h_total)],
#             [float(self.l_total), float(self.height)],
#             [float(self.l_total - self.top_bottom_length), float(self.height)],
#             [float(self.bottom_bottom_length), 0.],
#             [0., 0.]]
#         return geometry_points
#
#     def get_top_ear(self) -> List[List]:
#         """
#         顶端挑耳，逆时针方向。
#         :return:
#         """
#         top_ear_points = [
#             [float(self.bottom_top_length + self.ln),
#              float(self.height)],
#             [float(self.bottom_top_length + self.ln),
#              float(self.h_total)],
#             [float(self.l_total),
#              float(self.h_total)],
#             [float(self.l_total),
#              float(self.height)],
#             [float(self.bottom_top_length + self.ln),
#              float(self.height)], ]
#         return top_ear_points
#
#     def get_bottom_ear(self) -> List[List]:
#         """
#         顶端挑耳，逆时针方向。
#         :return:
#         """
#         bottom_ear_points = [
#             [0., 0.],
#             [0., float(self.bottom_thickness)],
#             [float(self.bottom_bottom_length), float(self.bottom_thickness)],
#             [float(self.bottom_bottom_length), 0.],
#             [0., 0.]]
#         return bottom_ear_points
#
#     def get_body_width(self) -> float:
#         return float(self.width)
#
#     def get_top_ear_width(self) -> float:
#         return float(self.detailed_design.geometric_detailed.top_b)
#
#     def get_bottom_ear_width(self) -> float:
#         return float(self.detailed_design.geometric_detailed.bottom_b)
#
#
# class StairHoleData(object):
#     """
#     预留孔洞定位信息
#     """
#
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult) -> None:
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#         self.width = self.detailed_design.geometric_detailed.width
#         self.bottom_thickness = self.detailed_design.geometric_detailed.bottom_thickness
#         self.top_thickness = self.detailed_design.geometric_detailed.top_thickness
#
#         # 顶端孔洞边距
#         self.top_right_long_a1 = self.detailed_design.construction_detailed.top_hole_position.a1  # 顶端右侧纵向边距
#         self.top_right_horizon_b1 = self.detailed_design.construction_detailed.top_hole_position.b1  # 顶端右侧横向边距
#         self.top_left_long_a2 = self.detailed_design.construction_detailed.top_hole_position.a2  # 顶端左侧纵向边距
#         self.top_left_horizon_b2 = self.detailed_design.construction_detailed.top_hole_position.b2  # 顶端左侧横向边距
#         # 底端孔洞边距
#         self.bottom_right_long_a3 = self.detailed_design.construction_detailed.bottom_hole_position.a3  # 底端右侧纵向边距
#         self.bottom_right_horizon_b3 = self.detailed_design.construction_detailed.bottom_hole_position.b3  # 底端右侧横向边距
#         self.bottom_left_long_a4 = self.detailed_design.construction_detailed.bottom_hole_position.a4  # 底端左侧纵向边距
#         self.bottom_left_horizon_b4 = self.detailed_design.construction_detailed.bottom_hole_position.b4  # 底端左侧横向边距
#         self.l_total = self.detailed_design_result.l_total  # 纵向总长
#         self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
#
#     def get_top_hole_position(self) -> List[List[float]]:
#         """
#         计算销键或连接孔洞的位置,以孔洞的下边缘的圆心为标志点
#         :return:
#         """
#         top_hole_position = [[float(self.top_left_horizon_b2),
#                               float(self.l_total - self.top_left_long_a2),
#                               float(self.h_total - self.top_thickness)],
#                              [float(self.width - self.top_right_horizon_b1),
#                               float(self.l_total - self.top_right_long_a1),
#                               float(self.h_total - self.top_thickness)]]  # 由底端到顶端，由左到向右
#         return top_hole_position
#
#     def get_bottom_hole_position(self) -> List[List[float]]:
#         """
#         计算销键或连接孔洞的位置,以孔洞的下边缘的圆心为标志点
#         :return:
#         """
#         bottom_hole_position = [[float(self.bottom_left_horizon_b4),
#                                  float(self.bottom_left_long_a4),
#                                  0.],
#                                 [float(self.width - self.bottom_right_horizon_b3),
#                                  float(self.bottom_right_long_a3),
#                                  0.]]
#
#         return bottom_hole_position
#
#     def get_top_hole_section(self) -> List[List[float]]:
#         top_hole_type = self.detailed_design.construction_detailed.top_hole_type
#         top_hole = self.detailed_design.construction_detailed.top_hole
#         if top_hole_type.value == 0:  # 固定支座
#             self.top_hole_section = [[0., 0.],
#                                      [top_hole.fix_hinge_d2 / 2., 0.],
#                                      [top_hole.fix_hinge_c2 / 2., float(self.top_thickness)],
#                                      [0., float(self.top_thickness)],
#                                      [0., 0.]]
#         else:  # 滑动支座
#             self.top_hole_section = [[0., 0.],
#                                      [top_hole.sliding_hinge_f1 / 2., 0.],
#                                      [top_hole.sliding_hinge_d1 / 2.,
#                                       float(self.top_thickness - top_hole.sliding_hinge_h1)],
#                                      [top_hole.sliding_hinge_e1 / 2.,
#                                       float(self.top_thickness - top_hole.sliding_hinge_h1)],
#                                      [top_hole.sliding_hinge_c1 / 2., float(self.top_thickness)],
#                                      [0., float(self.top_thickness)],
#                                      [0., 0.]]
#         return self.top_hole_section
#
#     def get_bottom_hole_section(self) -> List[List[float]]:
#         bottom_hole_type = self.detailed_design.construction_detailed.bottom_hole_type
#         bottom_hole = self.detailed_design.construction_detailed.bottom_hole
#         if bottom_hole_type.value == 0:  # 固定支座
#             self.bottom_hole_section = [[0., 0.],
#                                         [bottom_hole.fix_hinge_d2 / 2., 0.],
#                                         [bottom_hole.fix_hinge_c2 / 2., float(self.top_thickness)],
#                                         [0., float(self.bottom_thickness)],
#                                         [0., 0.]]
#         else:  # 滑动支座
#             self.bottom_hole_section = [[0., 0.],
#                                         [bottom_hole.sliding_hinge_f1 / 2., 0.],
#                                         [bottom_hole.sliding_hinge_d1 / 2.,
#                                          float(self.bottom_thickness - bottom_hole.sliding_hinge_h1)],
#                                         [bottom_hole.sliding_hinge_e1 / 2.,
#                                          float(self.bottom_thickness - bottom_hole.sliding_hinge_h1)],
#                                         [bottom_hole.sliding_hinge_c1 / 2., float(self.bottom_thickness)],
#                                         [0., float(self.bottom_thickness)],
#                                         [0., 0.]]
#         return self.bottom_hole_section
#
#
# class StepSlotData(object):
#     """
#     计算防滑槽的位置信息
#     """
#
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult) -> None:
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#
#         self.steps_number = self.structure_design.geometric.steps_number
#         self.steps_h = self.structure_design_result.steps_h
#         self.steps_b = self.structure_design_result.steps_b
#         self.width = self.detailed_design.geometric_detailed.width
#
#         self.bottom_top_length = self.detailed_design.geometric_detailed.bottom_top_length
#         self.bottom_thickness = self.detailed_design.geometric_detailed.bottom_thickness
#         self.step_slot_mode = self.detailed_design.construction_detailed.step_slot_design_mode
#
#     def get_step_slot_position(self) -> List[List[float]]:
#         """
#         计算防滑槽的位置信息
#         :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]] 最右侧
#         """
#         step_slot_c1 = self.detailed_design.construction_detailed.step_slot_position.c1
#         step_slot_c3 = self.detailed_design.construction_detailed.step_slot_position.c3
#         positions = []
#         init_position = [float(step_slot_c1),
#                          float(self.bottom_top_length + step_slot_c3),
#                          float(self.bottom_thickness)]  # 初始位置
#         for num in range(self.steps_number):
#             position = [init_position[0],
#                         init_position[1] + float(num * self.steps_b),
#                         init_position[2] + float((num + 1) * self.steps_h)]
#             positions.append(position)
#
#         return positions
#
#     def get_slot_position(self) -> List[List[float]]:
#         """
#         计算防滑槽的位置信息
#         :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]] 最右侧
#         """
#         step_slot_e = self.detailed_design.construction_detailed.step_slot.e
#         step_slot_a = self.detailed_design.construction_detailed.step_slot.a
#         step_slot_b = self.detailed_design.construction_detailed.step_slot.b
#         step_slot_c = self.detailed_design.construction_detailed.step_slot.c
#
#         positions = [[float(step_slot_e),
#                       0.,
#                       0.],
#                      [float(step_slot_e),
#                       float(step_slot_a + step_slot_b + step_slot_c),
#                       0.]]
#         return positions
#
#     def get_slot_length(self):
#         """
#         计算防滑槽的位置信息 用于拉伸的主体长度
#         :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]]
#         """
#         step_slot_e = self.detailed_design.construction_detailed.step_slot.e
#         step_slot_c1 = self.detailed_design.construction_detailed.step_slot_position.c1
#         step_slot_c2 = self.detailed_design.construction_detailed.step_slot_position.c2
#         length = float(self.width - step_slot_c1 - step_slot_c2 - 2 * step_slot_e)  # 防滑槽的长度
#
#         return length
#
#     def get_slot_section(self) -> List[List[float]]:
#         """
#         计算防滑槽的配置信息 用于拉伸的主要参数
#         :return: Dict
#         """
#         step_slot_a = self.detailed_design.construction_detailed.step_slot.a
#         step_slot_b = self.detailed_design.construction_detailed.step_slot.b
#         step_slot_d = self.detailed_design.construction_detailed.step_slot.d
#         sections = [[0., 0.],
#                     [float(step_slot_a + step_slot_b), 0.],
#                     [float(step_slot_b), float(-step_slot_d)],
#                     [0., 0.]]
#         return sections
#
#     def get_slope_position(self):
#         """
#         计算斜坡的位置信息
#         :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]] 最右侧
#         """
#         step_slot_c1 = self.detailed_design.construction_detailed.step_slot_position.c1
#         step_slot_c2 = self.detailed_design.construction_detailed.step_slot_position.c2
#         step_slot_a = self.detailed_design.construction_detailed.step_slot.a
#         step_slot_b = self.detailed_design.construction_detailed.step_slot.b
#         step_slot_c = self.detailed_design.construction_detailed.step_slot.c
#         step_slot_e = self.detailed_design.construction_detailed.step_slot.e
#         length = float(self.width - step_slot_c2 - step_slot_c1 - step_slot_e)  # 防滑槽的长度
#         positions = [[[0., 0., 0.],
#                       [float(length), 0., 0.]],
#                      [[0., float(step_slot_a + step_slot_b + step_slot_c), 0.],
#                       [float(length), float(step_slot_a + step_slot_b + step_slot_c), 0.]],
#                      ]
#         return positions
#
#     def get_slope_section(self):
#         """
#         计算防滑槽的配置信息  小坡面
#         :return: Dict
#         """
#         step_slot_a = self.detailed_design.construction_detailed.step_slot.a
#         step_slot_b = self.detailed_design.construction_detailed.step_slot.b
#         step_slot_e = self.detailed_design.construction_detailed.step_slot.e
#         sections_1 = [[[0., 0.],
#                        [0., float(step_slot_e)],
#                        [float(step_slot_a + step_slot_b), float(step_slot_e)],
#                        [float(step_slot_a + step_slot_b), 0.],
#                        [0., 0.]],
#                       [[0., 0.],
#                        [0., float(step_slot_e)],
#                        [float(step_slot_a + step_slot_b), float(step_slot_e)],
#                        [float(step_slot_a + step_slot_b), 0.],
#                        [0., 0.]]]
#         sections_2 = [[float(step_slot_b), float(step_slot_e)],
#                       [float(step_slot_b), 0.]]  # 截面2的位置
#
#         return sections_1, sections_2
#
#     def get_slope_length(self):
#         """
#         计算防滑槽的位置信息 用于拉伸的主体长度
#         :return:[[防滑槽组1],[防滑槽组2],[防滑槽组3],[防滑槽组4]]
#         """
#         step_slot_d = self.detailed_design.construction_detailed.step_slot.d
#         length = float(step_slot_d)  # 防滑槽的长度
#
#         return length
#
#
# class WaterDripData(object):
#     """
#     滴水线槽的位置信息
#     """
#
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult) -> None:
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#
#         self.water_drop_mode = self.detailed_design.construction_detailed.water_drip_design_mode
#         self.water_drop_layout = self.detailed_design.construction_detailed.water_drip_layout  # 上 0  下 1  都有 2
#
#         self.top_b = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
#         self.bottom_b = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
#         self.bottom_bottom_length = self.detailed_design_result.bottom_bottom_length
#         self.top_bottom_length = self.detailed_design_result.top_bottom_length
#         self.width = self.structure_design.geometric.width
#         self.l_total = self.detailed_design_result.l_total  # 纵向总长
#         self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
#         self.top_thickness = self.detailed_design.geometric_detailed.top_thickness
#
#     def get_water_drip_curve(self):
#         """
#         滴水线槽坐标位置
#         :return:
#         """
#         curves = []
#         a1 = self.detailed_design.construction_detailed.water_drip_position.a1
#         a2 = self.detailed_design.construction_detailed.water_drip_position.a2
#         a3 = self.detailed_design.construction_detailed.water_drip_position.a3
#         if self.water_drop_layout.value == 0:  # 只有上部
#             curves.append([[float(self.width + self.bottom_b), float(a2), 0.],
#                            [float(self.width - a3), float(a2), 0.],
#                            [float(self.width - a3), float(self.bottom_bottom_length), 0.],
#                            [float(self.width - a3), float(self.l_total - self.top_bottom_length),
#                             float(self.h_total - self.top_thickness)],
#                            [float(self.width - a3), float(self.l_total - a1),
#                             float(self.h_total - self.top_thickness)],
#                            [float(self.width + self.top_b), float(self.l_total - a1),
#                             float(self.h_total - self.top_thickness)],
#                            ])
#         elif self.water_drop_layout.value == 1:  # 只有下部
#             curves.append([[0., float(a2), 0.],
#                            [float(a3), float(a2), 0.],
#                            [float(a3), float(self.bottom_bottom_length), 0.],
#                            [float(a3), float(self.l_total - self.top_bottom_length),
#                             float(self.h_total - self.top_thickness)],
#                            [float(a3), float(self.l_total - a1), float(self.h_total - self.top_thickness)],
#                            [0., float(self.l_total - a1), float(self.h_total - self.top_thickness)],
#                            ])
#         else:
#             curves.append([[float(self.width + self.bottom_b), float(a2), 0.],
#                            [float(self.width - a3), float(a2), 0.],
#                            [float(self.width - a3), float(self.bottom_bottom_length), 0.],
#                            [float(self.width - a3), float(self.l_total - self.top_bottom_length),
#                             float(self.h_total - self.top_thickness)],
#                            [float(self.width - a3), float(self.l_total - a1),
#                             float(self.h_total - self.top_thickness)],
#                            [float(self.width + self.top_b), float(self.l_total - a1),
#                             float(self.h_total - self.top_thickness)],
#                            ])
#             curves.append([[0., float(a2), 0.],
#                            [float(a3), float(a2), 0.],
#                            [float(a3), float(self.bottom_bottom_length), 0.],
#                            [float(a3), float(self.l_total - self.top_bottom_length),
#                             float(self.h_total - self.top_thickness)],
#                            [float(a3), float(self.l_total - a1), float(self.h_total - self.top_thickness)],
#                            [0., float(self.l_total - a1), float(self.h_total - self.top_thickness)],
#                            ])
#         return curves
#
#     def get_water_drop_section(self):
#         """
#         滴水线槽配置信息
#         :return:
#         """
#         section_shape = self.detailed_design.construction_detailed.water_drip_shape
#         sections = []
#         if section_shape.value == 0:  # TRAPEZOID = 0 SEMICIRCLE = 1
#             water_drop_a = self.detailed_design.construction_detailed.water_drip.a
#             water_drop_b = self.detailed_design.construction_detailed.water_drip.b
#             water_drop_c = self.detailed_design.construction_detailed.water_drip.c
#             if self.water_drop_layout.value == 0:  # 只有上部
#
#                 sections.append([[0., 0.],
#                                  [-float(water_drop_c), float((water_drop_b - water_drop_a) / 2)],
#                                  [-float(water_drop_c), float((water_drop_b + water_drop_a) / 2)],
#                                  [0., float(water_drop_b)]])
#                 return sections, section_shape
#             elif self.water_drop_layout.value == 1:  # 只有下部
#                 sections.append([[0., 0.],
#                                  [float(water_drop_c), float((water_drop_b - water_drop_a) / 2)],
#                                  [float(water_drop_c), float((water_drop_b + water_drop_a) / 2)],
#                                  [0., float(water_drop_b)]])
#                 return sections, section_shape
#             else:
#                 sections.append([[0., 0.],
#                                  [-float(water_drop_c), float((water_drop_b - water_drop_a) / 2)],
#                                  [-float(water_drop_c), float((water_drop_b + water_drop_a) / 2)],
#                                  [0., float(water_drop_b)]])
#
#                 sections.append([[0., 0.],
#                                  [float(water_drop_c), float((water_drop_b - water_drop_a) / 2)],
#                                  [float(water_drop_c), float((water_drop_b + water_drop_a) / 2)],
#                                  [0., float(water_drop_b)]])
#                 return sections, section_shape
#         else:
#             water_drop_a = self.detailed_design.construction_detailed.water_drip.a
#             water_drop_b = self.detailed_design.construction_detailed.water_drip.b
#             if self.water_drop_layout.value == 0:  # 只有上部
#                 sections.append([[0., 0.],
#                                  [-float(water_drop_b), float(water_drop_a / 2)],
#                                  [0., float(water_drop_a)]])
#                 return sections, section_shape
#             elif self.water_drop_layout.value == 1:  # 只有下部
#                 sections.append([[0., 0.],
#                                  [float(water_drop_b), float(water_drop_a / 2)],
#                                  [0., float(water_drop_a)]])
#                 return sections, section_shape
#             else:
#                 sections.append([[0., 0.],
#                                  [-float(water_drop_b), float(water_drop_a / 2)],
#                                  [0., float(water_drop_a)]])
#                 sections.append([[0., 0.],
#                                  [float(water_drop_b), float(water_drop_a / 2)],
#                                  [0., float(water_drop_a)]])
#                 return sections, section_shape
#
#
# class RailingEmbeddedPartsData(object):
#     """
#     栏杆预埋件定位信息
#     """
#
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult) -> None:
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#
#         self.rail_design_mode = self.detailed_design.inserts_detailed.rail_design_mode
#         self.rail_layout = self.detailed_design.inserts_detailed.rail_layout
#         self.width = self.detailed_design.geometric_detailed.width
#         self.bottom_top_length = self.detailed_design.geometric_detailed.bottom_top_length
#         self.bottom_thickness = self.detailed_design.geometric_detailed.bottom_thickness
#         self.steps_h = self.structure_design_result.steps_h
#         self.steps_b = self.structure_design_result.steps_b
#
#     def get_rail_embedded_parts_position(self) -> List[List[float]]:
#         """
#         计算栏杆预埋件的坐标点：栏杆上表面,栏杆布置侧为 上楼梯方向的左右侧
#         :return:
#         """
#         rail_number = self.detailed_design.inserts_detailed.rail_number  # 栏杆所在的阶数
#
#         x_a = self.detailed_design.inserts_detailed.rail_position.a  # 横向边距
#         y_b = self.detailed_design.inserts_detailed.rail_position.b  # 纵向距离台阶
#         # 栏杆布置左右侧
#         position_x = []
#         if self.rail_layout.value == 0:  # ONLY_RIGHT = 0 ONLY_LEFT = 1   BOTH = 2
#             position_x.append(float(self.width - x_a))
#         elif self.rail_layout.value == 1:
#             position_x.append(float(x_a))
#         else:
#             position_x.append(float(x_a))
#             position_x.append(float(self.width - x_a))
#         position_y = []
#         for i in rail_number:
#             position_y.append(float(self.bottom_top_length + i * self.steps_b - y_b))
#         position_z = []
#         for i in rail_number:
#             position_z.append(float(self.bottom_thickness + i * self.steps_h))
#         positions = []
#         for i in range(len(position_x)):
#             for j in range(len(rail_number)):
#                 positions.append([position_x[i], position_y[j], position_z[j]])
#         return positions
#
#
# class LiftingEmbeddedPartsData(object):
#     """
#     吊装预埋件定位坐标
#     """
#
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult) -> None:
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#
#         self.steps_b = self.structure_design_result.steps_b
#         self.steps_h = self.structure_design_result.steps_h
#         self.lifting_type = self.detailed_design_result.lifting_type
#         self.bottom_top_length = self.detailed_design.geometric_detailed.bottom_top_length
#         self.bottom_thickness = self.detailed_design.geometric_detailed.bottom_thickness
#         self.width = self.detailed_design.geometric_detailed.width
#         self.lifting_position_a = self.detailed_design.inserts_detailed.lifting_position.a
#         self.lifting_position_b = self.detailed_design.inserts_detailed.lifting_position.b
#         self.lifting_position_c = float(self.detailed_design.inserts_detailed.lifting_position.c)  # 吊装预埋件左侧横向边距
#         self.lifting_position_d = float(self.detailed_design.inserts_detailed.lifting_position.d)  # 吊装预埋件右侧横向边距
#
#     def get_lifting_embedded_parts_position(self) -> List[List[float]]:
#         """
#         计算吊装预埋件的坐标点：预埋件上部中心点
#         :return:
#         """
#         edge_a = self.bottom_top_length + (self.lifting_position_a - 0.5) * self.steps_b  # 吊装预埋件顶端纵向边距
#         edge_b = self.bottom_top_length + (self.lifting_position_b - 0.5) * self.steps_b  # 吊装预埋件底端纵向边距
#         top_h = self.bottom_thickness + self.lifting_position_a * self.steps_h  # 下部吊装件坐标
#         bottom_h = self.bottom_thickness + self.lifting_position_b * self.steps_h  # 上部吊装件坐标
#
#         position_x = [self.lifting_position_c, float(self.width - self.lifting_position_d)]
#         position_y = [float(edge_b), float(edge_a)]
#         position_z = [float(bottom_h), float(top_h)]
#         positions = [[position_x[0], position_y[0], position_z[0]],
#                      [position_x[0], position_y[1], position_z[1]],
#                      [position_x[1], position_y[0], position_z[0]],
#                      [position_x[1], position_y[1], position_z[1]]]
#
#         return positions
#
#
# class DemouldingEmbeddedPartsData(object):
#     """
#     脱模预埋件位置
#     """
#
#     def __init__(self, structure_design: StructuralDesign, structure_design_result: StructuralDesignResult,
#                  detailed_design: DetailedDesign, detailed_design_result: DetailedDesignResult) -> None:
#         self.structure_design = structure_design
#         self.structure_design_result = structure_design_result
#         self.detailed_design = detailed_design
#         self.detailed_design_result = detailed_design_result
#         self.tan = self.detailed_design_result.tan
#         self.cos = self.detailed_design_result.cos
#         self.sin = self.detailed_design_result.sin
#         self.pouring_way = self.detailed_design.inserts_detailed.pouring_way  # 楼梯的浇筑方式,影响预埋件的位置及数量
#         self.demolding_type = self.detailed_design.inserts_detailed.demolding_type  # 脱模预埋件类型
#         self.demolding_parameter = self.detailed_design_result.demolding_parameter  # 脱模预埋件参数
#         self.demolding_a = float(self.detailed_design.inserts_detailed.demolding_position.a)  # 脱模埋件顶端纵向边距
#         self.demolding_b = float(self.detailed_design.inserts_detailed.demolding_position.b)  # 脱模埋件底端纵向边距
#         self.demolding_c = float(self.detailed_design.inserts_detailed.demolding_position.c)  # 脱模埋件左侧横向边距
#         self.demolding_d = float(self.detailed_design.inserts_detailed.demolding_position.d)  # 脱模埋件右侧横向边距
#         self.demolding_t = float(self.detailed_design.inserts_detailed.demolding_position.t)  # 脱模埋件厚度方向边距
#         self.h_total = float(self.detailed_design_result.h_total)  # 楼梯总高度
#         self.l_total = float(self.detailed_design_result.l_total)  # 纵向总长
#         self.width = float(self.detailed_design.geometric_detailed.width)
#         self.bottom_bottom_length = float(self.detailed_design_result.bottom_bottom_length)
#
#     def get_demoulding_embedded_parts_position(self):
#         """
#         计算脱模预埋件的坐标点
#         :return:
#         """
#         # 获取信息
#         # 侧面脱模预埋件的定位
#         # 顶端侧面脱模预埋件y坐标
#         demolding_t_y = self.l_total - self.demolding_a  # 顶端侧面脱模预埋件y坐标
#         # 顶端侧面脱模预埋件z坐标
#         demolding_t_z = (self.l_total - self.bottom_bottom_length - self.demolding_a) \
#                         * self.tan + self.demolding_t / self.cos
#         # 底端侧面脱模预埋件y坐标
#         demolding_b_y = self.demolding_b
#         # 底端侧面脱模预埋件z坐标
#         demolding_b_z = (self.demolding_b - self.bottom_bottom_length) * self.tan + self.demolding_t / self.cos
#
#         positions = [[self.width, demolding_t_y, demolding_t_z],
#                      [self.width, demolding_b_y, demolding_b_z]]
#         z_direction = [-1., 0., 0.]
#         x_direction = [0., -self.sin, self.cos]
#
#         return positions, z_direction, x_direction
