"""
Aim：此模块用于生成楼梯的几何数据
Date:2022/8/31
Author：zhangchao
"""
from stair_dxf.stair_design.datas import Point
from typing import List


class StairGeometry(object):
    """
    生成楼梯几何数据类
    """

    def __init__(self, slab_struct, detail_slab, struct_book, detail_book) -> None:
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.generate_basic_datas()  # 更新

    def generate_basic_datas(self):
        """
        产生基础数据
        :return:
        """
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
        self.h = self.slab_struct.geometric.height
        self.l_total = self.detail_book.l_total  # 纵向总长
        self.h_total = self.detail_book.h_total  # 楼梯总高度
        self.tan = self.detail_book.tan
        self.cos = self.detail_book.cos
        self.sin = self.detail_book.sin
        self.cover = self.slab_struct.construction.concrete_cover_thickness  # 保护层厚度

    def get_concrete_body_left(self) -> List[Point]:
        """
        计算楼梯主体左侧几何角点
        要求：1.顶端和底端挑耳除外；2.以楼梯立面图右下角为初始点(0,0,0)，逆时针方向前进。
        :return:
        """
        # 添加底端板三角点
        geometry_points = [
            Point(x=0, y=0, z=0),
            Point(x=0, y=0, z=self.h2),
            Point(x=0, y=self.lb_d, z=self.h2),
        ]
        # 添加踏步角点
        for i in range(self.n - 1):
            point_1 = Point(
                x=0, y=self.lb_d + i * self.tabu_b, z=self.h2 + (i + 1) * self.tabu_h
            )
            point_2 = Point(
                0,
                y=self.lb_d + (i + 1) * self.tabu_b,
                z=self.h2 + (i + 1) * self.tabu_h,
            )
            geometry_points.append(point_1)
            geometry_points.append(point_2)
        # 添加顶端板右上角点
        point_3 = Point(x=0, y=self.lb_d + self.ln, z=self.h2 + self.h)
        geometry_points.append(point_3)
        # 添加顶端板左上角点
        point_4 = Point(x=0, y=self.lb_d + self.ln + self.lt_d, z=self.h2 + self.h)
        geometry_points.append(point_4)
        # 添加顶端板左下角点
        point_5 = Point(
            x=0, y=self.lb_d + self.ln + self.lt_d, z=self.h2 + self.h - self.h1
        )
        geometry_points.append(point_5)
        # 添加顶端下边右角点
        point_6 = Point(
            x=0,
            y=self.lb_d + self.ln + self.lt_d - self.l1t,
            z=self.h2 + self.h - self.h1,
        )
        geometry_points.append(point_6)
        # 添加底端下边左角点
        point_7 = Point(x=0, y=self.l1b, z=0)
        geometry_points.append(point_7)
        return geometry_points

    def get_concrete_body_right(self) -> List[Point]:
        """
        计算楼梯主体右侧几何角点
        要求：1.顶端和底端挑耳除外；2.以楼梯立面图右下角为初始点(0,0,0)，逆时针方向前进。
        :return:
        """
        # 添加底端板三角点
        geometry_points = [
            Point(x=self.b0, y=0, z=0),
            Point(x=self.b0, y=0, z=self.h2),
            Point(x=self.b0, y=self.lb_d, z=self.h2),
        ]
        # 添加踏步角点
        for i in range(self.n - 1):
            point_1 = Point(
                x=self.b0,
                y=self.lb_d + i * self.tabu_b,
                z=self.h2 + (i + 1) * self.tabu_h,
            )
            point_2 = Point(
                x=self.b0,
                y=self.lb_d + (i + 1) * self.tabu_b,
                z=self.h2 + (i + 1) * self.tabu_h,
            )
            geometry_points.append(point_1)
            geometry_points.append(point_2)
        # 添加顶端板右上角点
        point_3 = Point(x=self.b0, y=self.lb_d + self.ln, z=self.h2 + self.h)
        geometry_points.append(point_3)
        # 添加顶端板左上角点
        point_4 = Point(
            x=self.b0, y=self.lb_d + self.ln + self.lt_d, z=self.h2 + self.h
        )
        geometry_points.append(point_4)
        # 添加顶端板左下角点
        point_5 = Point(
            x=self.b0, y=self.lb_d + self.ln + self.lt_d, z=self.h2 + self.h - self.h1
        )
        geometry_points.append(point_5)
        # 添加顶端下边右角点
        point_6 = Point(
            x=self.b0,
            y=self.lb_d + self.ln + self.lt_d - self.l1t,
            z=self.h2 + self.h - self.h1,
        )
        geometry_points.append(point_6)
        # 添加底端下边左角点
        point_7 = Point(x=self.b0, y=self.l1b, z=0)
        geometry_points.append(point_7)
        return geometry_points

    def get_bottom_ear_left(self) -> List[Point]:
        """
        计算底端左侧挑耳几何角点：以右下角为起点，逆时针方向
        :return:
        """
        bottom_ear = [
            Point(x=self.b0, y=0, z=0),
            Point(x=self.b0, y=0, z=self.h2),
            Point(x=self.b0, y=min(self.lb_d, self.l1b), z=self.h2),
            Point(x=self.b0, y=min(self.lb_d, self.l1b), z=0),
        ]
        return bottom_ear

    def get_bottom_ear_right(self) -> List[Point]:
        """
        计算底端挑耳右侧几何角点：以右下角为起点，逆时针方向
        :return:
        """
        bottom_ear = [
            Point(x=self.b0 + self.b2, y=0, z=0),
            Point(x=self.b0 + self.b2, y=0, z=self.h2),
            Point(x=self.b0 + self.b2, y=min(self.lb_d, self.l1b), z=self.h2),
            Point(x=self.b0 + self.b2, y=min(self.lb_d, self.l1b), z=0),
        ]
        return bottom_ear

    def get_top_ear_left(self) -> List[Point]:
        """
        计算顶端左侧挑耳的几何角点：以右下角为起点，逆时针方向
        :return:
        """
        top_ear = [
            Point(x=self.b0, y=self.lb_d + self.ln, z=self.h2 + self.h - self.h1),
            Point(x=self.b0, y=self.lb_d + self.ln, z=self.h2 + self.h),
            Point(x=self.b0, y=self.lb_d + self.ln + self.lt_d, z=self.h2 + self.h),
            Point(
                x=self.b0,
                y=self.lb_d + self.ln + self.lt_d,
                z=self.h2 + self.h - self.h1,
            ),
        ]
        return top_ear

    def get_top_ear_right(self) -> List[Point]:
        """
        计算顶端右侧挑耳的几何角点：以右下角为起点，逆时针方向
        :return:
        """
        top_ear = [
            Point(
                x=self.b0 + self.b1, y=self.lb_d + self.ln, z=self.h2 + self.h - self.h1
            ),
            Point(x=self.b0 + self.b1, y=self.lb_d + self.ln, z=self.h2 + self.h),
            Point(
                x=self.b0 + self.b1,
                y=self.lb_d + self.ln + self.lt_d,
                z=self.h2 + self.h,
            ),
            Point(
                x=self.b0 + self.b1,
                y=self.lb_d + self.ln + self.lt_d,
                z=self.h2 + self.h - self.h1,
            ),
        ]
        return top_ear
