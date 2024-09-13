import numpy as np
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakePrism,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSweep,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeWedge,
)  # 拉伸形成实体
import copy
import math

# 1.算法系列
from typing import List, Dict

from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_XYZ, gp_Vec, gp_Trsf, gp_Circ, gp_Ax1

from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_Transform,
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut

from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe

from stair_dxf.generate_drawing.occ_drawing.start_draw import OCCData
from OCC.Core.GProp import GProp_GProps  # 几何模型属性
from OCC.Core.BRepGProp import brepgprop_VolumeProperties  # solid几何模型属性

from stair_dxf.generate_drawing.occ_drawing.occ_expand_function import (
    make_ax3,
    rewrite_get_sorted_hlr_edges,
    discretize_edge,
    discretize_wire,
    compute_project_shape,
    compute_project_minimal_distance_pnt,
    compute_project_center_to_shape,
    my_BRepAlgoAPI_Fuse,
    my_BRepAlgoAPI_Cut,
    my_BRepAlgoAPI_Common,
    fuse_shape,
    rotation_solid,
    transform_solid_to_step_data,
)


class BuildStairSolidModel(object):
    """
    建立楼梯实体模型
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.occ_data = OCCData(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_data()  # 产生基本数据

    def generate_basic_data(self):
        """
        产生基本数据
        :return:
        """
        self.b0 = self.detail_slab.geometric_detailed.width
        self.b1 = self.detail_slab.geometric_detailed.top_b  # 下部挑耳
        self.b2 = self.detail_slab.geometric_detailed.bottom_b  # 下部挑耳
        self.bottom_node_type = (
            self.detail_slab.construction_detailed.bottom_hole_type.value
        )  # HoleType.FIXED_HINGE =0 , HoleType.SLIDING_HINGE=1
        self.top_node_type = self.detail_slab.construction_detailed.top_hole_type.value
        self.hoist_embedded_name = self.detail_book.lifting_parameter.name  # DJ-130
        self.hoist_embedded_type = (
            self.detail_slab.inserts_detailed.lifting_type.value
        )  # ROUNDING_HEAD = 0 #    ANCHOR = 1
        self.demold_embedded_name = self.detail_book.demolding_parameter.name  # 0 DJ
        # 滴水线槽后期逻辑判断，若无滴水线槽则无滴水线槽排布的value，逻辑判断应该是首先判断有无滴水线槽，若有才进行滴水线槽形状和排布位置的判断
        self.water_drip_design_mode = (
            self.detail_slab.construction_detailed.water_drip_design_mode.value
        )  # 1--manual,2--no
        self.step_slot_design_mode = (
            self.detail_slab.construction_detailed.step_slot_design_mode.value
        )  # 防滑槽设计模式 0--automatic,manual--1,no--2

    def build_stair_solid(self):
        """
        建立楼梯实体模型
        :return:
        """
        stair_vertex = self.occ_data.get_stair_left_vertex_data()  # 获取楼梯左侧角点数据
        total_length = self.occ_data.get_stair_all_width()  # 获取楼梯中部，顶部，底部宽度数据
        length = total_length[0]
        stair_datas_close = copy.deepcopy(stair_vertex)
        stair_datas_close.append(stair_vertex[0])  # 形成封闭图形
        stair_profile = BRepBuilderAPI_MakeWire()  # 创建空的轮廓
        for num in range(len(stair_datas_close) - 1):
            start_point = stair_datas_close[num]  # 起点
            end_point = stair_datas_close[num + 1]  # 终点
            edge = BRepBuilderAPI_MakeEdge(
                gp_Pnt(start_point.x, start_point.y, start_point.z),
                gp_Pnt(end_point.x, end_point.y, end_point.z),
            ).Edge()
            stair_profile.Add(edge)
        stair_face = BRepBuilderAPI_MakeFace(stair_profile.Wire()).Shape()  # 形成平面
        stretch_dir = gp_Vec(length, 0, 0)  # 拉伸方向及长度
        my_body = BRepPrimAPI_MakePrism(stair_face, stretch_dir)
        return my_body.Shape()

    def build_top_edge_ear(self):
        """
        建立顶部边缘挑耳模型
        :return:
        """
        top_ear_points = self.occ_data.get_stair_top_ear_left_data()  # 获取顶部挑耳左侧数据
        total_length = self.occ_data.get_stair_all_width()  # 获取楼梯中部、顶部、底部宽度数据
        length = total_length[1]
        edge_ear = copy.deepcopy(top_ear_points)
        edge_ear.append(top_ear_points[0])
        ear_wire = BRepBuilderAPI_MakeWire()
        # 建立主要实体
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        main_solid = BRepAlgoAPI_Fuse(main_solid, cut_shape).Shape()  # 合并实体
        for num in range(len(edge_ear) - 1):
            start_point = top_ear_points[num]
            end_point = top_ear_points[num - 1]
            edge = BRepBuilderAPI_MakeEdge(
                gp_Pnt(start_point.x, start_point.y, start_point.z),
                gp_Pnt(end_point.x, end_point.y, end_point.z),
            ).Edge()
            ear_wire.Add(edge)
        stretch_ear = gp_Vec(length, 0, 0)
        ear_face = BRepBuilderAPI_MakeFace(ear_wire.Wire())
        # 若顶端有挑耳则进行拉伸，若无挑耳则无需拉伸
        if self.b1 != 0:
            ear_solid = BRepPrimAPI_MakePrism(ear_face.Face(), stretch_ear).Shape()
            main_solid = BRepAlgoAPI_Fuse(main_solid, ear_solid).Shape()
        ear_solid = BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()

        return ear_solid

    def build_bottom_edge_ear(self):
        """
        建立底部边缘挑耳模型，需要考虑无底端挑耳的特殊情况
        :return:
        """
        bottom_ear_points = self.occ_data.get_stair_bottom_ear_left_data()  # 获取底部挑耳左侧数据
        total_length = self.occ_data.get_stair_all_width()  # 获取楼梯中部、顶部、底部宽度数据
        length = total_length[2]
        # 建立主要实体
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        main_solid = BRepAlgoAPI_Fuse(main_solid, cut_shape).Shape()  # 合并实体
        edge_ear = copy.deepcopy(bottom_ear_points)
        edge_ear.append(bottom_ear_points[0])
        ear_wire = BRepBuilderAPI_MakeWire()
        for num in range(len(edge_ear) - 1):
            start_point = bottom_ear_points[num]
            end_point = bottom_ear_points[num - 1]
            edge = BRepBuilderAPI_MakeEdge(
                gp_Pnt(start_point.x, start_point.y, start_point.z),
                gp_Pnt(end_point.x, end_point.y, end_point.z),
            ).Edge()
            ear_wire.Add(edge)
        stretch_ear = gp_Vec(length, 0, 0)
        ear_face = BRepBuilderAPI_MakeFace(ear_wire.Wire())
        # 若底端有挑耳则进行拉伸，若无挑耳则无需拉伸
        if self.b2 != 0:
            ear_solid = BRepPrimAPI_MakePrism(ear_face.Face(), stretch_ear).Shape()
            main_solid = BRepAlgoAPI_Fuse(main_solid, ear_solid).Shape()
        ear_solid = BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()
        return ear_solid

    @staticmethod
    def build_stretch_profile_solid(
        profiles: List[List[float]], direction: List[float]
    ):
        """
        建立拉伸实体
        :param profiles: 实体边缘轮廓点
        :param direction: 拉伸方向
        :return:
        """
        # 建立主要实体
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        main_solid = BRepAlgoAPI_Fuse(main_solid, cut_shape).Shape()  # 合并实体
        edge_profile = copy.deepcopy(profiles)
        edge_profile.append(profiles[0])  # 形成封闭图形
        profile_wire = BRepBuilderAPI_MakeWire()
        for num in range(len(edge_profile) - 1):
            start_point = edge_profile[num]
            end_point = edge_profile[num + 1]
            edge = BRepBuilderAPI_MakeEdge(
                gp_Pnt(start_point[0], start_point[1], start_point[2]),
                gp_Pnt(end_point[0], end_point[1], end_point[2]),
            ).Edge()
            profile_wire.Add(edge)
        stretch_dir = gp_Vec(direction[0], direction[1], direction[2])
        profile_face = BRepBuilderAPI_MakeFace(profile_wire.Wire())
        profile_solid = BRepPrimAPI_MakePrism(profile_face.Face(), stretch_dir).Shape()
        main_solid = BRepAlgoAPI_Fuse(main_solid, profile_solid).Shape()
        shape_solid = BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()
        return shape_solid

    @staticmethod
    def build_single_internal_corner(point: List[float], length: int, radius: float):
        """
        建立单个阴角:用四分之一圆柱体将长方体挖去部分
        :param point:坐标点
        :param length: 总的长度
        :param radius: 圆的半径
        :return:
        """
        point_center = copy.deepcopy(point)  # 生成圆心点
        point_center[1] -= radius
        point_center[2] += radius
        point_1 = copy.deepcopy(point)
        point_1[2] += radius
        point_2 = copy.deepcopy(point)
        point_2[1] -= radius
        point_corner = copy.deepcopy(point)  # 生成长方体的角点
        point_corner[1] -= radius
        # 生成立方体
        box = BRepPrimAPI_MakeBox(
            gp_Pnt(point_corner[0], point_corner[1], point_corner[2]),
            length,
            radius,
            radius,
        ).Shape()
        # 生成四分之一圆柱体
        edge_1 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_center[0], point_center[1], point_center[2]),
        ).Edge()
        edge_2 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
            gp_Pnt(point_center[0], point_center[1], point_center[2]),
        ).Edge()
        quarter_arc = GC_MakeArcOfCircle(
            gp_Circ(
                gp_Ax2(
                    gp_Pnt(point_center[0], point_center[1], point_center[2]),
                    gp_Dir(1, 0, 0),
                ),
                radius,
            ),
            math.pi,
            3 * math.pi / 2,
            True,
        )
        edge_3 = BRepBuilderAPI_MakeEdge(quarter_arc.Value()).Edge()
        wire = BRepBuilderAPI_MakeWire(edge_1, edge_2, edge_3).Wire()
        face = BRepBuilderAPI_MakeFace(wire).Face()
        quarter_cylinder = BRepPrimAPI_MakePrism(face, gp_Vec(length, 0, 0)).Shape()
        internal_corner = BRepAlgoAPI_Cut(box, quarter_cylinder)
        internal_corner.SimplifyResult()
        return internal_corner.Shape()

    def build_all_internal_corner(self):
        """
        建立所有踏步阴角圆弧
        :return:
        """
        internal_info = self.occ_data.get_stair_internal_data()
        loc = internal_info["location"]  # 坐标位置
        radius = internal_info["radius"]  # 阴角圆弧直径
        length = self.b0  # 拉伸宽度
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(loc)):
            current_point = loc[num]
            current_solid = self.build_single_internal_corner(
                current_point, length, radius
            )
            main_solid = my_BRepAlgoAPI_Fuse(main_solid, current_solid).Shape()
        main_solid = my_BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()
        return main_solid

    @staticmethod
    def build_single_hoist_embedded_sphere_rabbet(point: List[float], radius: float):
        """
        建立单个吊装预埋件企口---半球
        :param point: 企口标志点
        :param radius: 企口圆孔
        :return:
        """
        center = copy.deepcopy(point)  # 建立圆心
        my_sphere = BRepPrimAPI_MakeSphere(
            gp_Pnt(center[0], center[1], center[2]), radius, math.pi
        ).Shape()
        # 旋转
        transform = gp_Trsf()
        # 绕指定轴顺时针旋转一定角度
        transform.SetRotation(
            gp_Ax1(gp_Pnt(center[0], center[1], center[2]), gp_Dir(1, 0, 0)),
            -math.pi / 2,
        )
        rotate_sphere = BRepBuilderAPI_Transform(my_sphere, transform, False).Shape()
        return rotate_sphere

    @staticmethod
    def build_single_hoist_embedded_cone_rabbet(
        point: List[float], radius_b: float, radius_t: float, height: float
    ):
        """
        建立单个吊装预埋件企口---圆台
        :param point: 圆台底部圆心
        :param radius_b: 圆台底部直径
        :param radius_t: 圆台顶部直径
        :param height: 圆台高度
        :return:
        """
        center = copy.deepcopy(point)  # 建立圆心
        my_cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(center[0], center[1], center[2]), gp_Dir(0, 0, 1)),
            radius_b,
            radius_t,
            height,
        ).Shape()
        return my_cone

    def build_specific_hoist_embedded_part_rabbet(self, num: int):
        """
        建立特定吊装预埋件企口模型：用于单个构件投影或剖切等特殊操作
        :param num:指定吊装预埋件企口
        :return:
        """
        hoist_info = self.occ_data.get_hoist_embedded_part_datas()
        hoist_points = hoist_info["location"]  # 获取坐标点
        hoist_config = hoist_info["specification"]  # 吊装预埋件配置信息
        hoist_type = hoist_info["type"]  # 吊装预埋件类型  0--圆头吊钉，1---预埋锚栓
        if hoist_type == 0:  # 圆头吊钉
            radius = hoist_config["radius"]
            current_point = copy.deepcopy(hoist_points[num][0])
            current_point[2] += radius
            rabbet_model = self.build_single_hoist_embedded_sphere_rabbet(
                current_point, radius
            )
        else:  # 预埋锚栓
            m_length = hoist_config["m_length"]
            m_diam = hoist_config["m_diameter"]  # 顶部直径
            o_diam = hoist_config["o_diameter"]  # 锚杆直径
            current_point = copy.deepcopy(hoist_points[num][0])
            rabbet_model = self.build_single_hoist_embedded_cone_rabbet(
                current_point, o_diam / 2, m_diam / 2, m_length
            )
        return rabbet_model

    def build_all_hoist_embedded_rabbet(self):
        """
        建立所有吊装预埋件企口形状
        hoist_type:0--rounding_head,1---anchor
        :return:
        """
        hoist_info = self.occ_data.get_hoist_embedded_part_datas()
        hoist_points = hoist_info["location"]  # 获取坐标点
        hoist_config = hoist_info["specification"]  # 吊装预埋件配置信息
        hoist_type = hoist_info["type"]  # 吊装预埋件类型
        # 建立所有模型
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if hoist_type == 0:  # 圆头吊钉
            radius = hoist_config["radius"]
            for num in range(len(hoist_points)):
                current_point = copy.deepcopy(hoist_points[num][0])
                current_point[2] += radius
                rabbet_model = self.build_single_hoist_embedded_sphere_rabbet(
                    current_point, radius
                )
                main_solid = my_BRepAlgoAPI_Fuse(main_solid, rabbet_model).Shape()
        else:  # 预埋锚栓
            m_length = hoist_config["m_length"]
            m_diam = hoist_config["m_diameter"]  # 顶部直径
            o_diam = hoist_config["o_diameter"]  # 锚杆直径
            for num in range(len(hoist_points)):
                current_point = copy.deepcopy(hoist_points[num][0])
                rabbet_model = self.build_single_hoist_embedded_cone_rabbet(
                    current_point, o_diam / 2, m_diam / 2, m_length
                )
                main_solid = my_BRepAlgoAPI_Fuse(main_solid, rabbet_model).Shape()
        main_solid = my_BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()
        return main_solid

    @staticmethod
    def build_single_external_corner(point: List[float], length: float, side: float):
        """
        建立单个楼梯阳角三棱柱数据
        :param point: 参考点
        :param length: 拉伸长度
        :param side: 边长
        :return:
        """
        point_0 = copy.deepcopy(point)
        point_0[1] += side
        point_1 = copy.deepcopy(point)
        point_2 = copy.deepcopy(point)
        point_2[2] -= side
        edge_1 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_0[0], point_0[1], point_0[2]),
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
        ).Edge()
        edge_2 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
        ).Edge()
        edge_3 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_0[0], point_0[1], point_0[2]),
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
        ).Edge()
        wire = BRepBuilderAPI_MakeWire(edge_1, edge_2, edge_3).Wire()
        face = BRepBuilderAPI_MakeFace(wire).Face()
        triangle_shape = BRepPrimAPI_MakePrism(face, gp_Vec(length, 0, 0)).Shape()
        return triangle_shape

    def build_all_external_corner(self):
        """
        建立所有阳角数据
        :return:
        """
        external_info = self.occ_data.get_stair_external_data()
        loc = external_info["location"]  # 坐标位置
        side = external_info["side"]  # 边长
        length_0 = self.b0
        length_1 = self.b0 + self.b1
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(loc)):
            current_point = loc[num]
            if num != len(loc) - 1:
                current_solid = self.build_single_external_corner(
                    current_point, length_0, side
                )
            else:
                current_solid = self.build_single_external_corner(
                    current_point, length_1, side
                )
            main_solid = my_BRepAlgoAPI_Fuse(main_solid, current_solid).Shape()
        main_solid = my_BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()
        return main_solid

    @staticmethod
    def build_single_hole(hole_loc, height, diams):
        """
        建立单个孔洞数据---静态方法
        :return:
        """
        bottom_r = diams[0] / 2
        top_r = diams[1] / 2
        cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), bottom_r, top_r, height
        ).Shape()
        # my_cone = TopoDS_Shape(cone)  # 获取对象的拓扑信息，各个面的点和方向
        point = hole_loc
        move = gp_Vec(point[0], point[1], point[2])  # 平移点
        transform = gp_Trsf()  # 建立变换矩阵
        transform.SetTranslation(move)  # 对矩阵进行平移变换
        cone = BRepBuilderAPI_Transform(cone, transform, False).Shape()  # 不复制该圆柱
        return cone

    def build_specific_hole_shape(self, num: int):
        """
        建立指定孔洞形状：用于单个孔洞投影和剖切等测试
        :param num: 孔洞序号
        :return:
        """
        hole_datas = self.occ_data.get_hole_data()  # 获取孔洞数据
        hole_data = hole_datas[num]
        solid = self.build_single_hole(hole_data[0], hole_data[1], hole_data[2])
        return solid

    def build_all_hole_shape(self):
        """
        建立多个孔洞形状
        :return:
        """
        hole_datas = self.occ_data.get_hole_data()  # 获取孔洞数据
        main_solid = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for i in range(len(hole_datas)):
            hole_data = hole_datas[i]
            solid = self.build_single_hole(hole_data[0], hole_data[1], hole_data[2])
            main_solid = my_BRepAlgoAPI_Fuse(main_solid, solid).Shape()
        main_solid = my_BRepAlgoAPI_Cut(main_solid, cut_shape).Shape()
        return main_solid

    @staticmethod
    def build_single_step_slot_shape(point, shape):
        """
        建立单个防滑槽的OCC模型
        :param point:
        :param shape:
        :return:TopoDS_Shape
        """
        spacing = shape["spacing"]  # 组内防滑槽的中心间距
        step_d = shape["d"]
        step_e = shape["e"]
        length = shape["length"]  # 长度
        width = shape["width"]  # 宽度
        half_length = length / 2
        point_1 = [point[0], point[1] - width / 2, point[2]]  # 踏步面
        point_2 = [point[0], point[1] + width / 2, point[2]]  # 踏步面
        point_3 = [point[0], point[1], point[2] - step_d]  # 底部尖点
        point_4 = [point[0] + step_e, point[1], point[2] - step_d]  # 三棱锥顶点
        mirror_center = [
            point[0] + half_length,
            point[1],
            point[2] - step_d / 3,
        ]  # 镜像中心
        # 1.建立三棱柱
        edge_1 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
        ).Edge()
        edge_2 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_3[0], point_3[1], point_3[2]),
        ).Edge()
        edge_3 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
            gp_Pnt(point_3[0], point_3[1], point_3[2]),
        ).Edge()
        wire_1 = BRepBuilderAPI_MakeWire(edge_1, edge_2, edge_3).Wire()
        face_0 = BRepBuilderAPI_MakeFace(wire_1).Face()
        triangle_prism = BRepPrimAPI_MakePrism(
            face_0, gp_Vec(length, 0, 0)
        ).Shape()  # 建立三棱柱实体

        # 2.建立三棱锥---很难直接建立，智能通过三棱柱，然后用布尔运算操作切掉不需要的部分
        triangle_prism_0 = BRepPrimAPI_MakePrism(
            face_0, gp_Vec(step_e, 0, 0)
        ).Shape()  # 建立防滑槽边缘三棱柱实体
        edge_4_1 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_4[0], point_4[1], point_4[2]),
        ).Edge()
        edge_4_2 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_4[0], point_4[1], point_4[2]),
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
        ).Edge()
        vec_21 = np.array(np.array(point_1) - np.array(point_2)) / np.linalg.norm(
            np.array(point_1) - np.array(point_2)
        )
        vec_14 = np.array(np.array(point_4) - np.array(point_1)) / np.linalg.norm(
            np.array(point_4) - np.array(point_1)
        )

        normal_1_2_4 = np.cross(vec_21, vec_14) * length  # 四边形平面法向量
        point_1_0 = copy.deepcopy(point_4)
        point_2_0 = copy.deepcopy(point_4)
        point_1_0[1] = point_1[1]
        point_2_0[1] = point_2[1]
        edge_110 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_1_0[0], point_1_0[1], point_1_0[2]),
        ).Edge()
        edge_220 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
            gp_Pnt(point_2_0[0], point_2_0[1], point_2_0[2]),
        ).Edge()
        edge_1020 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1_0[0], point_1_0[1], point_1_0[2]),
            gp_Pnt(point_2_0[0], point_2_0[1], point_2_0[2]),
        ).Edge()
        wire_124 = BRepBuilderAPI_MakeWire()
        wire_124.Add(edge_1)
        wire_124.Add(edge_110)
        wire_124.Add(edge_220)
        wire_124.Add(edge_1020)
        face_124 = BRepBuilderAPI_MakeFace(wire_124.Wire()).Face()
        solid_124 = BRepPrimAPI_MakePrism(
            face_124, gp_Vec(normal_1_2_4[0], normal_1_2_4[1], normal_1_2_4[2])
        ).Shape()

        triangle_pyramid = my_BRepAlgoAPI_Cut(triangle_prism_0, solid_124).Shape()
        copy_triangle_pyramid = my_BRepAlgoAPI_Cut(triangle_prism_0, solid_124).Shape()
        # 3.镜像复制三棱锥
        transform_1 = gp_Trsf()
        transform_1.SetMirror(
            gp_Ax1(
                gp_Pnt(mirror_center[0], mirror_center[1], mirror_center[2]),
                gp_Dir(0, 1, 0),
            )
        )
        copy_triangle_pyramid = BRepBuilderAPI_Transform(
            copy_triangle_pyramid, transform_1, False
        ).Shape()
        # 再旋转180度，不知为何镜像后实体朝向相反了
        transform_2 = gp_Trsf()
        transform_2.SetRotation(
            gp_Ax1(
                gp_Pnt(mirror_center[0], mirror_center[1], mirror_center[2]),
                gp_Dir(1, 0, 0),
            ),
            math.pi,
        )
        copy_triangle_pyramid = BRepBuilderAPI_Transform(
            copy_triangle_pyramid, transform_2, False
        ).Shape()

        total_triangle_pyramid = my_BRepAlgoAPI_Fuse(
            triangle_pyramid, copy_triangle_pyramid
        ).Shape()

        # 4.从三棱柱中挖出三棱锥
        first_triangle_prism = my_BRepAlgoAPI_Cut(
            triangle_prism, total_triangle_pyramid
        ).Shape()
        second_triangle_prism = my_BRepAlgoAPI_Cut(
            triangle_prism, total_triangle_pyramid
        ).Shape()
        # # 5.复制被挖去的三棱锥并平移
        transform_3 = gp_Trsf()
        transform_3.SetTranslation(gp_Vec(gp_XYZ(0, spacing, 0)))
        second_triangle_prism = BRepBuilderAPI_Transform(
            second_triangle_prism, transform_3, False
        ).Shape()
        two_triangle_pyramid = my_BRepAlgoAPI_Fuse(
            first_triangle_prism, second_triangle_prism
        ).Shape()
        return two_triangle_pyramid

    def build_specific_step_slot_shape(self, num: int):
        """
        建立特定防滑槽形状模型：用于单个防滑槽投影或剖切
        :param num: 指定序号
        :return: TopoDS_Shape
        """
        step_slot_data = self.occ_data.get_step_slot_datas()  # 获取防滑槽数据
        shape_data = step_slot_data["config"]  # 形状信息
        step_loc = step_slot_data["location"]  # 坐标信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        design_mode = shape_data["design_mode"]  # 防滑槽设计模式
        if design_mode != 2:
            # 遍历每个台阶的防滑槽
            point = copy.deepcopy(step_loc[num])  # 当前台阶的防滑槽坐标点
            solid = self.build_single_step_slot_shape(point, shape_data)
            props = GProp_GProps()  # 获取几何属性查询函数
            brepgprop_VolumeProperties(solid, props)  # 计算出当前实体的几何属性
            cog = props.CentreOfMass()  # 计算当前实体的形心
            cog_x, cog_y, cog_z = cog.Coord()
            # print("形心位置为:", (cog_x, cog_y, cog_z))
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, solid).Shape()
        else:  # 若无防滑槽形状，则返回空的三维空间
            pass
        final_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()  # 移除多余形状的操作
        return final_shape

    def build_all_step_slot_shape(self):
        """
        建立所有防滑槽形状
        :return: TopoDS_Shape
        """
        step_slot_data = self.occ_data.get_step_slot_datas()  # 获取防滑槽数据
        shape_data = step_slot_data["config"]  # 形状信息
        step_loc = step_slot_data["location"]  # 坐标信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        design_mode = shape_data["design_mode"]  # 防滑槽设计模式
        if design_mode != 2:
            # 遍历每个台阶的防滑槽
            for num in range(len(step_loc)):
                point = copy.deepcopy(step_loc[num])  # 当前台阶的防滑槽坐标点
                solid = self.build_single_step_slot_shape(point, shape_data)
                props = GProp_GProps()  # 获取几何属性查询函数
                brepgprop_VolumeProperties(solid, props)  # 计算出当前实体的几何属性
                cog = props.CentreOfMass()  # 计算当前实体的形心
                cog_x, cog_y, cog_z = cog.Coord()
                # print("形心位置为:", (cog_x, cog_y, cog_z))
                init_shape = my_BRepAlgoAPI_Fuse(init_shape, solid).Shape()
        else:  # 若无防滑槽形状，则返回空的三维空间
            pass
        final_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()  # 移除多余形状的操作
        return final_shape

    @staticmethod
    def build_segment_pipe_sweep_shape(points, shape):
        """
        通过两段的轮廓绘制指定形状的图形
        :param points: 线段
        :param shape: 形状
        :return: TopoDS_Shape
        """
        water_a = shape["a"]
        water_b = shape["b"]
        water_c = shape["c"]
        point_1 = copy.deepcopy(points[0])  # 首点
        point_2 = copy.deepcopy(points[1])  # 末点
        mid_z = (point_1[2] + point_2[2]) / 2  # 区分顶部和底部滴水线槽
        direction = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )  # 前进方向向量
        if direction[0] != 0:  # 针对平行于x方向的滴水线槽进行角点操作,保证x方向和y方向线槽角点能够较好衔接
            add_x = direction[0] * water_b / 2
            if max(point_1[2], point_2[2]) < 10:  # 判断为底部线槽线段---该阈值无特殊含义，仅仅是为了区分顶部和底部线槽
                point_2[0] += add_x
            else:
                point_1[0] -= add_x
        edge_12 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
        ).Edge()
        wire = BRepBuilderAPI_MakeWire(edge_12).Wire()
        origin = copy.deepcopy(points[0])  # 圆心点

        base_x = np.array([1, 0, 0])  # x方向的基向量
        base_y = np.array([0, 1, 0])  # y方向的基向量
        shape_type = shape["design_shape"]  # 滴水线槽形状
        if shape_type == 0:  # 若为梯形截面
            wire_shape = BRepBuilderAPI_MakeWire()
            point_0 = copy.deepcopy(origin)  # 第一个点
            point_1 = copy.deepcopy(origin)
            point_2 = copy.deepcopy(origin)
            point_3 = copy.deepcopy(origin)
            if direction[0] != 0 and direction[1] == 0 and direction[2] == 0:  # x方向
                point_0[1] = point_0[1] + water_b / 2
                point_1 = [point_1[0], point_1[1] + water_a / 2, point_1[2] + water_c]
                point_2 = [point_2[0], point_2[1] - water_a / 2, point_2[2] + water_c]
                point_3[1] = point_0[1] - water_b
                edge_1 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                ).Edge()
                edge_2 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                ).Edge()
                edge_3 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                ).Edge()
                edge_4 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                ).Edge()
            elif direction[0] == 0 and direction[1] == 0 and direction[2] == 1:  # 沿着z方向
                point_0[0] = point_0[0] - water_b / 2
                point_1 = [point_1[0] - water_a / 2, point_1[1] - water_c, point_1[2]]
                point_2 = [point_2[0] + water_a / 2, point_2[1] - water_c, point_2[2]]
                point_3[0] = point_0[0] + water_b
                edge_1 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                ).Edge()
                edge_2 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                ).Edge()
                edge_3 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                ).Edge()
                edge_4 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                ).Edge()

            elif (
                direction[0] == 0 and direction[1] != 0 and direction[2] != 0
            ):  # 沿着y方向，y和z的值变化，x的值不变化
                normal_dir = np.cross(base_x, direction)  # 法向量
                point_0[0] = point_0[0] - water_b / 2
                add_length = normal_dir * water_c
                add_length = add_length.tolist()
                point_1 = [
                    point_1[0] - water_a / 2 + add_length[0],
                    point_1[1] + add_length[1],
                    point_1[2] + add_length[2],
                ]
                point_2 = [
                    point_2[0] + water_a / 2 + add_length[0],
                    point_2[1] + add_length[1],
                    point_2[2] + add_length[2],
                ]
                point_3[0] = point_0[0] + water_b
                edge_1 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                ).Edge()
                edge_2 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                ).Edge()
                edge_3 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                ).Edge()
                edge_4 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                ).Edge()
            else:  # y水平方向
                point_0[0] = point_0[0] - water_b / 2
                point_1 = [point_1[0] - water_a / 2, point_1[1], point_1[2] + water_c]
                point_2 = [point_2[0] + water_a / 2, point_2[1], point_2[2] + water_c]
                point_3[0] = point_0[0] + water_b
                edge_1 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                ).Edge()
                edge_2 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_1[0], point_1[1], point_1[2]),
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                ).Edge()
                edge_3 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_2[0], point_2[1], point_2[2]),
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                ).Edge()
                edge_4 = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(point_3[0], point_3[1], point_3[2]),
                    gp_Pnt(point_0[0], point_0[1], point_0[2]),
                ).Edge()
            wire_shape.Add(edge_1)
            wire_shape.Add(edge_2)
            wire_shape.Add(edge_3)
            wire_shape.Add(edge_4)
        else:  # 建立半圆环
            if direction[0] != 0 and direction[1] == 0 and direction[2] == 0:  # x方向
                point_1 = [origin[0], origin[1] - water_b / 2, origin[2]]
                point_2 = [origin[0], origin[1] + water_b / 2, origin[2]]
                point_3 = [origin[0], origin[1], origin[2] + water_b / 2]
            elif direction[0] == 0 and direction[1] == 0 and direction[2] == 1:  # 沿着z方向
                point_1 = [origin[0] - water_b / 2, origin[1], origin[2]]
                point_2 = [origin[0] + water_b / 2, origin[1], origin[2]]
                point_3 = [origin[0], origin[1] - water_b / 2, origin[2]]
            elif (
                direction[0] == 0 and direction[1] != 0 and direction[2] != 0
            ):  # 沿着y方向，y和z的值变化，x的值不变化
                normal_dir = np.cross(base_x, direction)  # 法向量
                add_length = normal_dir * water_b
                add_length = add_length.tolist()
                point_1 = [origin[0] - water_b / 2, origin[1], origin[2]]
                point_2 = [origin[0] + water_b / 2, origin[1], origin[2]]
                point_3 = [
                    origin[0] + add_length[0],
                    origin[1] + add_length[1],
                    origin[2] + add_length[2],
                ]
            else:  # y方向
                point_1 = [origin[0] - water_b / 2, origin[1], origin[2]]
                point_2 = [origin[0] + water_b / 2, origin[1], origin[2]]
                point_3 = [origin[0], origin[1], origin[2] + water_b / 2]
            half_circle = GC_MakeArcOfCircle(
                gp_Pnt(point_1[0], point_1[1], point_1[2]),
                gp_Pnt(point_3[0], point_3[1], point_3[2]),
                gp_Pnt(point_2[0], point_2[1], point_2[2]),
            )
            edge_1 = BRepBuilderAPI_MakeEdge(half_circle.Value()).Edge()
            edge_2 = BRepBuilderAPI_MakeEdge(
                gp_Pnt(point_1[0], point_1[1], point_1[2]),
                gp_Pnt(point_2[0], point_2[1], point_2[2]),
            ).Edge()
            wire_shape = BRepBuilderAPI_MakeWire(edge_1, edge_2)
        face = BRepBuilderAPI_MakeFace(wire_shape.Shape()).Face()  # 形成平面
        sweep_solid = BRepOffsetAPI_MakePipe(wire, face).Shape()
        return sweep_solid

    def build_single_water_drip_shape(self, points, shape):
        """
        建立单个防滑槽数据库
        :param points:单个防滑槽坐标点
        :param shape:防滑槽形状
        :return:TopoDS_Shape
        """
        # 获取滴水线槽形状数据
        origin = [points[0][0], points[0][1], points[0][2]]  # 标志点
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(points) - 1):
            point_s = copy.deepcopy(points[num])  # 起点
            point_e = copy.deepcopy(points[num + 1])  # 终点
            point_path = [point_s, point_e]  # 点的路径
            solid = self.build_segment_pipe_sweep_shape(point_path, shape)
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, solid).Shape()
        final_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()  # 移除多余形状的操作
        return final_shape

    def build_all_water_drip_shape(self):
        """
        建立滴水线槽数据集合
        :return: TopoDS_Shape
        """
        water_datas = self.occ_data.get_water_drip_datas()  # 获取滴水线槽数据
        water_locs = water_datas["location"]  # 防滑槽坐标点
        water_shape = water_datas["config"]  # 防滑槽型号数据
        design_mode = water_shape["design_mode"]  # 设计模式
        design_location = water_shape["design_location"]  # 设计部位
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if design_mode != 2:  # 含有滴水线槽数据
            if design_location == 2:  # 两侧布置
                for num in range(len(water_locs)):
                    current_point = copy.deepcopy(water_locs[num])
                    solid = self.build_single_water_drip_shape(
                        current_point, water_shape
                    )
                    init_shape = my_BRepAlgoAPI_Fuse(init_shape, solid).Shape()  # 合并两实体
            else:  # 单侧布置
                current_point = copy.deepcopy(water_locs)
                solid = self.build_single_water_drip_shape(current_point, water_shape)
                init_shape = my_BRepAlgoAPI_Fuse(init_shape, solid).Shape()  # 合并两实体
        else:
            pass
        final_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()  # 移除多余形状的操作
        return final_shape

    @staticmethod
    def build_polyline_and_polyarc_shape(points, radius_):
        """
        形成弯曲的钢筋系列：平直段，圆弧段，平直段
        :param points: 首点，初始起弯点，终止弯折点，终点
        :param radius_: 钢筋直径
        :return:
        """

        vec_s_m = np.array(np.array(points[1]) - np.array(points[0])).tolist()
        # 扫掠轨迹
        # 平直段
        edge_s_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        # 圆弧段
        arc = GC_MakeArcOfCircle(
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
            gp_Vec(vec_s_m[0], vec_s_m[1], vec_s_m[2]),
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
        )
        edge_arc = BRepBuilderAPI_MakeEdge(arc.Value()).Edge()
        # 平直段
        edge_m_e = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
        ).Edge()  # 拓扑顶点
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_s_m)  # 平直段
        wire_profile.Add(edge_arc)  # 圆弧段
        wire_profile.Add(edge_m_e)  # 平直段
        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_s_m[0], vec_s_m[1], vec_s_m[2]),
            ),
            radius_[1],
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()
        return sweep_shape

    @staticmethod
    def build_u_polyarc_shape(points, radius_):
        """
        针对u型钢筋形成OCC三维模型
        :param points: 首点，第一个起弯点，第一个结束弯折点，第二个起弯点，第二个结束弯折点，结束点
        :param radius_: 钢筋直径
        :return:
        """
        # 首点和第一个弯折点形成的向量
        vec_1 = np.array(np.array(points[1]) - np.array(points[0])).tolist()
        # 第一个弯折结束点和第二个弯折起始点形成的向量
        vec_2 = np.array(np.array(points[3]) - np.array(points[2])).tolist()
        # 扫掠轨迹
        # 第一个平直段
        edge_s_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        # 第一个圆弧段
        arc_1 = GC_MakeArcOfCircle(
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
            gp_Vec(vec_1[0], vec_1[1], vec_1[2]),
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
        )
        edge_arc_1 = BRepBuilderAPI_MakeEdge(arc_1.Value()).Edge()
        # 第二个平直段
        edge_m_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
        ).Edge()  # 拓扑顶点
        # 第二个圆弧段
        arc_2 = GC_MakeArcOfCircle(
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
            gp_Vec(vec_2[0], vec_2[1], vec_2[2]),
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
        )
        edge_arc_2 = BRepBuilderAPI_MakeEdge(arc_2.Value()).Edge()
        # 第三个平直段
        edge_m_e = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
            gp_Pnt(points[5][0], points[5][1], points[5][2]),
        ).Edge()  # 拓扑顶点
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_s_m)  # 平直段
        wire_profile.Add(edge_arc_1)  # 圆弧段
        wire_profile.Add(edge_m_m)  # 平直段
        wire_profile.Add(edge_arc_2)  # 圆弧段
        wire_profile.Add(edge_m_e)  # 平直段

        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_1[0], vec_1[1], vec_1[2]),
            ),
            radius_,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()

        return sweep_shape

    @staticmethod
    def build_five_sections_polyarc_shape(points, radius_):
        """
        针对吊点加强筋形成OCC三维模型
        :param points: 首点，第一个起弯点，第一个结束弯折点，第二个起弯点，第二个结束弯折点，第三个起弯点，第三个结束弯折点，结束点
        :param radius_: 钢筋直径
        :return:
        """
        # 首点和第一个弯折点形成的向量
        vec_1 = np.array(np.array(points[1]) - np.array(points[0])).tolist()
        vec_2 = np.array(np.array(points[3]) - np.array(points[2])).tolist()
        vec_3 = np.array(np.array(points[5]) - np.array(points[4])).tolist()
        # 扫掠轨迹
        # 第一个平直段
        edge_1 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        # 第一个圆弧段
        arc_1 = GC_MakeArcOfCircle(
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
            gp_Vec(vec_1[0], vec_1[1], vec_1[2]),
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
        )
        edge_arc_1 = BRepBuilderAPI_MakeEdge(arc_1.Value()).Edge()
        # 第二个平直段
        edge_2 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
        ).Edge()  # 拓扑顶点
        # 第二个圆弧段
        arc_2 = GC_MakeArcOfCircle(
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
            gp_Vec(vec_2[0], vec_2[1], vec_2[2]),
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
        )
        edge_arc_2 = BRepBuilderAPI_MakeEdge(arc_2.Value()).Edge()
        # 第三个平直段
        edge_3 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
            gp_Pnt(points[5][0], points[5][1], points[5][2]),
        ).Edge()  # 拓扑顶点
        # 第三个圆弧段
        arc_3 = GC_MakeArcOfCircle(
            gp_Pnt(points[5][0], points[5][1], points[5][2]),
            gp_Vec(vec_3[0], vec_3[1], vec_3[2]),
            gp_Pnt(points[6][0], points[6][1], points[6][2]),
        )
        edge_arc_3 = BRepBuilderAPI_MakeEdge(arc_3.Value()).Edge()
        # 第四个平直段
        edge_4 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[6][0], points[6][1], points[6][2]),
            gp_Pnt(points[7][0], points[7][1], points[7][2]),
        ).Edge()  # 拓扑顶点
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_1)  # 平直段
        wire_profile.Add(edge_arc_1)  # 圆弧段
        wire_profile.Add(edge_2)  # 平直段
        wire_profile.Add(edge_arc_2)  # 圆弧段
        wire_profile.Add(edge_3)  # 平直段
        wire_profile.Add(edge_arc_3)  # 圆弧段
        wire_profile.Add(edge_4)  # 平直段

        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_1[0], vec_1[1], vec_1[2]),
            ),
            radius_,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()

        return sweep_shape

    @staticmethod
    def build_z_polyarc_shape(points, radius_):
        """
        针对Z型钢筋形成OCC三维模型
        :param points: 首点，第一个起弯点，第一个结束弯折点，第二个起弯点，第二个结束弯折点，结束点
        :param radius_: 钢筋直径
        :return:
        """
        # 首点和第一个弯折点形成的向量
        vec_1 = np.array(np.array(points[1]) - np.array(points[0])).tolist()
        # 第一个弯折结束点和第二个弯折起始点形成的向量
        vec_2 = np.array(np.array(points[3]) - np.array(points[2])).tolist()
        # 扫掠轨迹
        # 第一个平直段
        edge_s_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        # 第一个圆弧段
        arc_1 = GC_MakeArcOfCircle(
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
            gp_Vec(vec_1[0], vec_1[1], vec_1[2]),
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
        )
        edge_arc_1 = BRepBuilderAPI_MakeEdge(arc_1.Value()).Edge()
        # 第二个平直段
        edge_m_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
        ).Edge()  # 拓扑顶点
        # 第二个圆弧段
        arc_2 = GC_MakeArcOfCircle(
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
            gp_Vec(vec_2[0], vec_2[1], vec_2[2]),
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
        )
        edge_arc_2 = BRepBuilderAPI_MakeEdge(arc_2.Value()).Edge()
        # 第三个平直段
        edge_m_e = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
            gp_Pnt(points[5][0], points[5][1], points[5][2]),
        ).Edge()  # 拓扑顶点
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_s_m)  # 平直段
        wire_profile.Add(edge_arc_1)  # 圆弧段
        wire_profile.Add(edge_m_m)  # 平直段
        wire_profile.Add(edge_arc_2)  # 圆弧段
        wire_profile.Add(edge_m_e)  # 平直段
        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_1[0], vec_1[1], vec_1[2]),
            ),
            radius_,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()
        return sweep_shape

    @staticmethod
    def build_L_polyarc_shape(points, radius_):
        """
        针对L型钢筋形成OCC三维模型
        :param points: 首点，第一个起弯点，第一个结束弯折点，结束点
        :param radius_: 钢筋直径
        :return:
        """
        # 首点和第一个弯折点形成的向量
        vec_1 = np.array(np.array(points[1]) - np.array(points[0])) / np.linalg.norm(
            np.array(points[1]) - np.array(points[0])
        )
        vec_1 = vec_1.tolist()
        # 扫掠轨迹
        # 第一个平直段
        edge_s_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        # 第一个圆弧段
        arc_1 = GC_MakeArcOfCircle(
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
            gp_Vec(vec_1[0], vec_1[1], vec_1[2]),
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
        )
        edge_arc_1 = BRepBuilderAPI_MakeEdge(arc_1.Value()).Edge()
        # 第二个平直段
        edge_m_e = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
        ).Edge()  # 拓扑顶点
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_s_m)  # 平直段
        wire_profile.Add(edge_arc_1)  # 圆弧段
        wire_profile.Add(edge_m_e)  # 平直段
        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_1[0], vec_1[1], vec_1[2]),
            ),
            radius_,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()
        return sweep_shape

    @staticmethod
    def build_line_shape(points, radius_):
        """
        针对直线型钢筋形成OCC三维模型
        :param points: 首点，结束点
        :param radius_: 钢筋直径
        :return:
        """
        # 首点和第一个弯折点形成的向量
        vec_1 = np.array(np.array(points[1]) - np.array(points[0])) / np.linalg.norm(
            np.array(points[1]) - np.array(points[0])
        )
        vec_1 = vec_1.tolist()
        # 扫掠轨迹
        # 第一个平直段
        edge_s_m = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_s_m)  # 平直段

        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_1[0], vec_1[1], vec_1[2]),
            ),
            radius_,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()
        return sweep_shape

    @staticmethod
    def build_close_rebar_polyarc_shape(points, radius_):
        """
        针对封闭钢筋形成OCC三维模型
        :param points: 第一个起弯点，第一个结束弯折点，第二个起弯点，第二个结束弯折点，第三个起弯点，第三个结束弯折点，第四个起弯点，第四个结束弯折点
        :param radius_: 钢筋直径
        :return:
        """
        # 第四个结束弯折点和第一个弯折点形成的向量
        vec_1 = np.array(np.array(points[1]) - np.array(points[0])).tolist()
        # 第一个弯折结束点和第二个弯折起始点形成的向量
        vec_2 = np.array(np.array(points[3]) - np.array(points[2])).tolist()
        # 第二个弯折结束点和第三个弯折起始点形成的向量
        vec_3 = np.array(np.array(points[5]) - np.array(points[4])).tolist()
        # 第三个弯折结束点和第四个弯折起始点形成的向量
        vec_4 = np.array(np.array(points[7]) - np.array(points[6])).tolist()
        # 扫掠轨迹
        # 第一个平直段
        edge_1 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
        ).Edge()  # 拓扑顶点
        # 第一个圆弧段
        arc_1 = GC_MakeArcOfCircle(
            gp_Pnt(points[1][0], points[1][1], points[1][2]),
            gp_Vec(vec_1[0], vec_1[1], vec_1[2]),
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
        )
        edge_arc_1 = BRepBuilderAPI_MakeEdge(arc_1.Value()).Edge()
        # 第二个平直段
        edge_2 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[2][0], points[2][1], points[2][2]),
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
        ).Edge()  # 拓扑顶点
        # 第二个圆弧段
        arc_2 = GC_MakeArcOfCircle(
            gp_Pnt(points[3][0], points[3][1], points[3][2]),
            gp_Vec(vec_2[0], vec_2[1], vec_2[2]),
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
        )
        edge_arc_2 = BRepBuilderAPI_MakeEdge(arc_2.Value()).Edge()
        # 第三个平直段
        edge_3 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[4][0], points[4][1], points[4][2]),
            gp_Pnt(points[5][0], points[5][1], points[5][2]),
        ).Edge()  # 拓扑顶点
        # 第三个圆弧段
        arc_3 = GC_MakeArcOfCircle(
            gp_Pnt(points[5][0], points[5][1], points[5][2]),
            gp_Vec(vec_3[0], vec_3[1], vec_3[2]),
            gp_Pnt(points[6][0], points[6][1], points[6][2]),
        )
        edge_arc_3 = BRepBuilderAPI_MakeEdge(arc_3.Value()).Edge()
        # 第四个平直段
        edge_4 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(points[6][0], points[6][1], points[6][2]),
            gp_Pnt(points[7][0], points[7][1], points[7][2]),
        ).Edge()  # 拓扑顶点
        # 第四个圆弧段
        arc_4 = GC_MakeArcOfCircle(
            gp_Pnt(points[7][0], points[7][1], points[7][2]),
            gp_Vec(vec_4[0], vec_4[1], vec_4[2]),
            gp_Pnt(points[0][0], points[0][1], points[0][2]),
        )
        edge_arc_4 = BRepBuilderAPI_MakeEdge(arc_4.Value()).Edge()
        wire_profile = BRepBuilderAPI_MakeWire()
        wire_profile.Add(edge_1)  # 平直段
        wire_profile.Add(edge_arc_1)  # 圆弧段
        wire_profile.Add(edge_2)  # 平直段
        wire_profile.Add(edge_arc_2)  # 圆弧段
        wire_profile.Add(edge_3)  # 平直段
        wire_profile.Add(edge_arc_3)  # 圆弧段
        wire_profile.Add(edge_4)  # 平直段
        wire_profile.Add(edge_arc_4)  # 圆弧段

        # 创建钢筋圆面
        circle = gp_Circ(
            gp_Ax2(
                gp_Pnt(points[1][0], points[1][1], points[1][2]),
                gp_Dir(vec_1[0], vec_1[1], vec_1[2]),
            ),
            radius_,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()  # 线
        wire = BRepBuilderAPI_MakeWire(edge).Wire()  # 多段线
        face = BRepBuilderAPI_MakeFace(wire).Face()  # 平面
        # 将圆面沿着轨迹进行扫描
        sweep_shape = BRepOffsetAPI_MakePipe(wire_profile.Wire(), face).Shape()

        return sweep_shape

    def build_specific_railing_embedded_shape(self, num: int):
        """
        建立特定栏杆预埋件形状模型：用于特定栏杆埋件投影或剖切图
        :param num: 埋件序号
        :return:
        """
        railing_datas = self.occ_data.get_railing_embedded_datas()  # 获取栏杆预埋件的数据信息
        location = railing_datas["location"]  # 栏杆预埋件坐标位置
        direction = railing_datas["layout"]["direction"]  # 预埋件的方向

        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        # 特定栏杆预埋件
        point_loc = copy.deepcopy(location[num])  # 栏杆预埋件的位置
        weld_loc = copy.deepcopy(
            railing_datas["layout"]["weld_box"]["location"]
        )  # 预埋件的方向
        weld_shape = copy.deepcopy(
            railing_datas["layout"]["weld_box"]["shape"]
        )  # 钢板的形状
        rebar_info = copy.deepcopy(railing_datas["layout"]["rebar"]["path"])  # 钢筋路径
        # 将焊接钢板局部坐标系向全局坐标系平移----左下角点为坐标原点
        weld_loc[0] += point_loc[0] - weld_shape[0] / 2
        weld_loc[1] += point_loc[1] - weld_shape[1] / 2
        weld_loc[2] += point_loc[2] - weld_shape[2]
        weld_box = BRepPrimAPI_MakeBox(
            gp_Pnt(weld_loc[0], weld_loc[1], weld_loc[2]),
            weld_shape[0],
            weld_shape[1],
            weld_shape[2],
        ).Shape()
        init_shape = BRepAlgoAPI_Fuse(init_shape, weld_box).Shape()
        # 遍历钢筋序列
        for k in range(len(rebar_info)):
            rebar_point_, diam_ = copy.deepcopy(rebar_info[k])  # 钢筋1
            # 将钢筋局部坐标向全局坐标系平移
            for i in range(len(rebar_point_)):
                rebar_point_[i][0] += point_loc[0]
                rebar_point_[i][1] += point_loc[1]
                rebar_point_[i][2] += point_loc[2]
            rebar_shape_ = self.build_polyline_and_polyarc_shape(rebar_point_, diam_)
            init_shape = BRepAlgoAPI_Fuse(init_shape, rebar_shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_railing_embedded_shape(self):
        """
        生成栏杆预埋件OCC图形
        :return:TopoDS_Shape
        """
        railing_datas = self.occ_data.get_railing_embedded_datas()  # 获取栏杆预埋件的数据信息
        location = railing_datas["location"]  # 栏杆预埋件坐标位置
        direction = railing_datas["layout"]["direction"]  # 预埋件的方向

        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(location)):  # 每个栏杆预埋件
            point_loc = copy.deepcopy(location[num])  # 栏杆预埋件的位置
            weld_loc = copy.deepcopy(
                railing_datas["layout"]["weld_box"]["location"]
            )  # 预埋件的方向
            weld_shape = copy.deepcopy(
                railing_datas["layout"]["weld_box"]["shape"]
            )  # 钢板的形状
            rebar_info = copy.deepcopy(railing_datas["layout"]["rebar"]["path"])  # 钢筋路径
            # 将焊接钢板局部坐标系向全局坐标系平移----左下角点为坐标原点
            weld_loc[0] += point_loc[0] - weld_shape[0] / 2
            weld_loc[1] += point_loc[1] - weld_shape[1] / 2
            weld_loc[2] += point_loc[2] - weld_shape[2]
            weld_box = BRepPrimAPI_MakeBox(
                gp_Pnt(weld_loc[0], weld_loc[1], weld_loc[2]),
                weld_shape[0],
                weld_shape[1],
                weld_shape[2],
            ).Shape()
            init_shape = BRepAlgoAPI_Fuse(init_shape, weld_box).Shape()
            # 遍历钢筋序列
            for k in range(len(rebar_info)):
                rebar_point_, diam_ = copy.deepcopy(rebar_info[k])  # 钢筋1
                # 将钢筋局部坐标向全局坐标系平移
                for i in range(len(rebar_point_)):
                    rebar_point_[i][0] += point_loc[0]
                    rebar_point_[i][1] += point_loc[1]
                    rebar_point_[i][2] += point_loc[2]
                rebar_shape_ = self.build_polyline_and_polyarc_shape(
                    rebar_point_, diam_
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, rebar_shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_rail_embedded_weld_shape(self):
        """
        建立栏杆预埋件焊板形状
        :return:
        """
        railing_datas = self.occ_data.get_railing_embedded_datas()  # 获取栏杆预埋件的数据信息
        location = railing_datas["location"]  # 栏杆预埋件坐标位置
        direction = railing_datas["layout"]["direction"]  # 预埋件的方向

        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(location)):  # 每个栏杆预埋件
            point_loc = copy.deepcopy(location[num])  # 栏杆预埋件的位置
            weld_loc = copy.deepcopy(
                railing_datas["layout"]["weld_box"]["location"]
            )  # 预埋件的方向
            weld_shape = copy.deepcopy(
                railing_datas["layout"]["weld_box"]["shape"]
            )  # 钢板的形状
            rebar_info = copy.deepcopy(railing_datas["layout"]["rebar"]["path"])  # 钢筋路径
            # 将焊接钢板局部坐标系向全局坐标系平移----左下角点为坐标原点
            weld_loc[0] += point_loc[0] - weld_shape[0] / 2
            weld_loc[1] += point_loc[1] - weld_shape[1] / 2
            weld_loc[2] += point_loc[2] - weld_shape[2]
            weld_box = BRepPrimAPI_MakeBox(
                gp_Pnt(weld_loc[0], weld_loc[1], weld_loc[2]),
                weld_shape[0],
                weld_shape[1],
                weld_shape[2],
            ).Shape()
            init_shape = BRepAlgoAPI_Fuse(init_shape, weld_box).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_rail_embedded_U_rebar_shape(self):
        """
        建立栏杆预埋件U型钢筋形状
        :return:
        """
        railing_datas = self.occ_data.get_railing_embedded_datas()  # 获取栏杆预埋件的数据信息
        location = railing_datas["location"]  # 栏杆预埋件坐标位置
        direction = railing_datas["layout"]["direction"]  # 预埋件的方向
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(location)):  # 每个栏杆预埋件
            point_loc = copy.deepcopy(location[num])  # 栏杆预埋件的位置
            rebar_info = copy.deepcopy(railing_datas["layout"]["rebar"]["path"])  # 钢筋路径
            # 遍历钢筋序列
            for k in range(len(rebar_info)):
                rebar_point_, diam_ = copy.deepcopy(rebar_info[k])  # 钢筋1
                # 将钢筋局部坐标向全局坐标系平移
                for i in range(len(rebar_point_)):
                    rebar_point_[i][0] += point_loc[0]
                    rebar_point_[i][1] += point_loc[1]
                    rebar_point_[i][2] += point_loc[2]
                rebar_shape_ = self.build_polyline_and_polyarc_shape(
                    rebar_point_, diam_
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, rebar_shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_mid_distribution_rebar_shape(self):
        """
        建立中部分布筋的OCC模型:当两根钢筋距离太近时，几何体作布尔运算容易出错,需要寻找原因,易重叠
        :return:
        """
        mid_rebars = self.occ_data.get_all_mid_distribute_rebar()  # 获取所有中部分布筋数据
        rebar_radius = mid_rebars["radius"]  # 中部分布筋的半径
        rebars_path = mid_rebars["path"]  # 中部分布筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_u_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_long_rebar_shape(self):
        """
        底部纵筋信息OCC模型
        :return:
        """
        bottom_rebar = self.occ_data.get_bottom_long_rebar_datas()
        rebar_radius = bottom_rebar["radius"]  # 底部纵筋的半径
        rebars_path = bottom_rebar["path"]  # 底部纵筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_L_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_long_rebar_shape(self):
        """
        顶部纵筋信息OCC模型
        :return:
        """
        top_rebar = self.occ_data.get_top_long_rebar_datas()  # 获取顶部纵筋数据
        rebar_radius = top_rebar["radius"]  # 底部纵筋的半径
        rebars_path = top_rebar["path"]  # 底部纵筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_z_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_edge_long_rebar_shape(self):
        """
        底部边缘纵筋信息OCC模型
        :return:
        """
        bottom_edge_rebar = (
            self.occ_data.get_bottom_edge_long_rebar_datas()
        )  # 获取底部边缘纵筋数据
        rebar_radius = bottom_edge_rebar["radius"]  # 底部边缘纵筋的半径
        rebars_path = bottom_edge_rebar["path"]  # 底部边缘纵筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_line_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_edge_long_rebar_shape(self):
        """
        顶部边缘纵筋信息OCC模型
        :return:
        """
        top_edge_rebar = self.occ_data.get_top_edge_long_rebar_datas()  # 获取顶部边缘纵筋数据
        rebar_radius = top_edge_rebar["radius"]  # 顶部边缘纵筋的半径
        rebars_path = top_edge_rebar["path"]  # 顶部边缘纵筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_line_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_edge_stir_shape(self):
        """
        底部边缘箍筋信息OCC模型
        :return:
        """
        bottom_edge_stir = self.occ_data.get_bottom_edge_stir_datas()  # 获取底部边缘箍筋数据
        rebar_radius = bottom_edge_stir["radius"]  # 底部边缘箍筋的半径
        rebars_path = bottom_edge_stir["path"]  # 底部边缘箍筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_close_rebar_polyarc_shape(
                rebar_path, rebar_radius
            )  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_edge_stir_shape(self):
        """
        顶部边缘箍筋信息OCC模型
        :return:
        """
        top_edge_stir = self.occ_data.get_top_edge_stir_datas()  # 获取顶部边缘箍筋数据
        rebar_radius = top_edge_stir["radius"]  # 顶部边缘箍筋的半径
        rebars_path = top_edge_stir["path"]  # 顶部边缘箍筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_close_rebar_polyarc_shape(
                rebar_path, rebar_radius
            )  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_specific_hole_rein_rebar_shape(self, num: int):
        """
        建立单个孔洞加强筋形状模型：用于孔洞加强筋投影或剖切等操作
        :param num: 序号
        :return:
        """
        hole_rein_rebar = self.occ_data.get_hole_rein_rebar_datas()  # 获取孔洞加强筋数据
        rebar_radius = hole_rein_rebar["radius"]  # 获取钢筋的半径
        rebars_path = hole_rein_rebar["path"]  # 获取钢筋的路径信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        # 单个孔洞加强筋模型
        rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
        shape_ = self.build_L_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
        init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_hole_rein_rebar_shape(self):
        """
        孔洞加强钢筋信息OCC模型
        :return:
        """
        hole_rein_rebar = self.occ_data.get_hole_rein_rebar_datas()  # 获取孔洞加强筋数据
        rebar_radius = hole_rein_rebar["radius"]  # 获取钢筋的半径
        rebars_path = hole_rein_rebar["path"]  # 获取钢筋的路径信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_L_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_hoist_rein_long_rebar_shape(self):
        """
        吊装加强纵筋信息OCC模型
        :return:
        """
        hoist_rein_long_rebar = (
            self.occ_data.get_hoist_rein_long_rebar_datas()
        )  # 获取吊装加强纵筋数据
        rebar_radius = hoist_rein_long_rebar["radius"]  # 获取钢筋的半径
        rebars_path = hoist_rein_long_rebar["path"]  # 获取钢筋的路径信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_five_sections_polyarc_shape(
                rebar_path, rebar_radius
            )  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_hoist_rein_point_rebar_shape(self):
        """
        孔洞加强钢筋信息OCC模型
        :return:
        """
        hoist_rein_point_rebar = (
            self.occ_data.get_hoist_rein_point_rebar_datas()
        )  # 获取吊装加强点筋数据
        rebar_radius = hoist_rein_point_rebar["radius"]  # 获取钢筋的半径
        rebars_path = hoist_rein_point_rebar["path"]  # 获取钢筋的路径信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_line_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_edge_rein_rebar_shape(self):
        """
        上部边缘加强筋信息OCC模型
        :return:
        """
        top_edge_rein_rebar = (
            self.occ_data.get_top_edge_rein_rebar_datas()
        )  # 获取顶部边缘加强筋数据
        rebar_radius = top_edge_rein_rebar["radius"]  # 获取钢筋的半径信息
        rebars_path = top_edge_rein_rebar["path"]  # 获取钢筋的路径信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_z_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_edge_rein_rebar_shape(self):
        """
        底部边缘加强筋信息OCC模型
        :return:
        """
        bottom_edge_rein_rebar = (
            self.occ_data.get_bottom_edge_rein_rebar_datas()
        )  # 获取底部边缘加强筋数据
        rebar_radius = bottom_edge_rein_rebar["radius"]  # 获取钢筋的直径
        rebars_path = bottom_edge_rein_rebar["path"]  # 获取钢筋的路径
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(rebars_path)):
            rebar_path = copy.deepcopy(rebars_path[num])  # 获取钢筋当前路径
            shape_ = self.build_L_polyarc_shape(rebar_path, rebar_radius)  # 建立U型形状
            init_shape = my_BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = my_BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    @staticmethod
    def build_rounding_heading_embedded_part_shape(
        point: List[float], shape: Dict, theta: float, rotation_axis: List[float]
    ):
        """
        圆头吊装预埋件数据
        :param point: 预埋件标志点
        :param shape: 预埋件型号数据
        :param theta: 预埋件旋转角，逆时针为正，默认为沿着z正方向
        :param rotation_axis: 预埋件旋转轴
        :return: 预埋件的三维图形
        """
        # 基本数据
        top_diam = shape["top_diameter"]
        top_height = shape["top_height"]
        top_adjacent_height = shape["top_adjacent_height"]
        middle_diam = shape["middle_diameter"]
        middle_height = shape["middle_height"]
        bottom_adjacent_height = shape["bottom_adjacent_height"]
        bottom_diam = shape["bottom_diameter"]
        bottom_height = shape["bottom_height"]
        # 各组成部分初始坐标点
        point_ = copy.deepcopy(point)  # 全局坐标系位置
        point_t_1 = [0, 0, -top_height]  # 顶圆柱的标志点
        point_t_2 = [0, 0, -top_height - top_adjacent_height]  # 顶圆台的标志点
        point_t_3 = [
            0,
            0,
            -top_height - top_adjacent_height - middle_height,
        ]  # 中部圆柱的标志点
        point_t_4 = [
            0,
            0,
            -top_height - top_adjacent_height - middle_height,
        ]  # 底部圆台的标志点
        point_t_5 = [
            0,
            0,
            -top_height
            - top_adjacent_height
            - middle_height
            - bottom_adjacent_height
            - bottom_height,
        ]  # 底部圆柱的标志点
        # 顶圆柱
        top_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_t_1[0], point_t_1[1], point_t_1[2]), gp_Dir(0, 0, 1)),
            top_diam / 2,
            top_height,
        ).Shape()
        # 顶圆台
        top_truncated_cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(point_t_2[0], point_t_2[1], point_t_2[2]), gp_Dir(0, 0, 1)),
            middle_diam / 2,
            top_diam / 2,
            top_adjacent_height,
        ).Shape()
        compound_shape = my_BRepAlgoAPI_Fuse(top_cylinder, top_truncated_cone).Shape()
        # 中圆柱
        middle_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_t_3[0], point_t_3[1], point_t_3[2]), gp_Dir(0, 0, 1)),
            middle_diam / 2,
            middle_height,
        ).Shape()
        compound_shape = my_BRepAlgoAPI_Fuse(compound_shape, middle_cylinder).Shape()
        # 底圆台
        bottom_truncated_cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(point_t_4[0], point_t_4[1], point_t_4[2]), gp_Dir(0, 0, -1)),
            middle_diam / 2,
            bottom_diam / 2,
            bottom_adjacent_height,
        ).Shape()
        compound_shape = my_BRepAlgoAPI_Fuse(
            compound_shape, bottom_truncated_cone
        ).Shape()
        # 底圆柱
        bottom_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_t_5[0], point_t_5[1], point_t_5[2]), gp_Dir(0, 0, 1)),
            bottom_diam / 2,
            bottom_height,
        ).Shape()
        compound_shape = my_BRepAlgoAPI_Fuse(compound_shape, bottom_cylinder).Shape()
        # 开始几何变换
        #
        transform_1 = gp_Trsf()
        transform_1.SetTranslation(gp_Vec(point_[0], point_[1], point_[2]))
        compound_shape = BRepBuilderAPI_Transform(
            compound_shape, transform_1, False
        ).Shape()
        transform_2 = gp_Trsf()
        transform_2.SetRotation(
            gp_Ax1(
                gp_Pnt(point_[0], point_[1], point_[2]),
                gp_Dir(rotation_axis[0], rotation_axis[1], rotation_axis[2]),
            ),
            theta,
        )
        compound_shape = BRepBuilderAPI_Transform(
            compound_shape, transform_2, False
        ).Shape()
        return compound_shape

    @staticmethod
    def build_basic_embedded_anchor_embedded_part_shape(
        point: List[float], shape: Dict, theta: float, rotation_axis: List[float]
    ):
        """
        单个预埋锚栓OCC模型:无顶部企口形状
        :param point: 预埋锚栓标志点
        :param shape: 预埋锚栓数据
        :param theta: 预埋锚栓旋转角，逆时针为正，初始状态竖直向上
        :param rotation_axis: 预埋锚栓旋转轴
        :return:
        """
        # 1.基本数据
        top_diam = shape["m_diameter"]  # 底端直径
        top_length = shape["m_length"]  # 顶端长度
        bottom_diam = shape["o_diameter"]  # 底端直径
        bottom_length = shape["length"]  # 底端长度
        rebar_diam = shape["s_diameter"]  # 锚固钢筋的直径
        rebar_length = shape["l_p"]  # 锚固钢筋的长度
        edge_length = shape["a"]  # 钢筋与底端距离
        e_diam = shape["e_diameter"]  # 嵌入直径
        e_length = shape["g"]  # 嵌入长度
        b_length = shape["b"]  # 圆锥的高度
        # 2.各组成部分初始坐标点
        point_1 = [0, 0, -top_length]  # 圆台坐标原点
        point_2 = [0, 0, -top_length - bottom_length]  # 圆柱坐标原点
        point_3 = [0, 0, -top_length - bottom_length + edge_length]  # 钢筋分两段，便于旋转
        point_4 = [0, 0, -top_length - e_length]  # 嵌入圆柱体，布尔运算去掉
        point_5 = [0, 0, -top_length - e_length - b_length]  # 圆锥体，布尔运算去掉
        # 3.开始建立实体
        # 3.1 顶部圆台
        # 3.2 底部圆柱
        compound_shape = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_2[0], point_2[1], point_2[2]), gp_Dir(0, 0, 1)),
            bottom_diam / 2,
            bottom_length,
        ).Shape()
        # 3.3 形成连接体
        # 3.4 左段钢筋
        rebar_left_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_3[0], point_3[1], point_3[2]), gp_Dir(1, 0, 0)),
            rebar_diam / 2,
            rebar_length / 2,
        ).Shape()
        # 3.5 右段钢筋
        rebar_right_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_3[0], point_3[1], point_3[2]), gp_Dir(-1, 0, 0)),
            rebar_diam / 2,
            rebar_length / 2,
        ).Shape()
        # 3.6 完整钢筋实体
        rebar_cylinder = BRepAlgoAPI_Fuse(
            rebar_left_cylinder, rebar_right_cylinder
        ).Shape()
        # 3.7 合并钢筋实体
        compound_shape = BRepAlgoAPI_Fuse(compound_shape, rebar_cylinder).Shape()  #
        # 3.8 嵌入圆柱体
        implant_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_4[0], point_4[1], point_4[2]), gp_Dir(0, 0, 1)),
            e_diam / 2,
            e_length / 2,
        ).Shape()
        implant_cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(point_5[0], point_5[1], point_5[2]), gp_Dir(0, 0, 1)),
            0,
            e_diam / 2,
            b_length,
        ).Shape()
        implant_solid = BRepAlgoAPI_Fuse(implant_cylinder, implant_cone).Shape()
        # # 3.9 去除嵌入圆柱体和圆锥体
        compound_shape = BRepAlgoAPI_Cut(compound_shape, implant_solid).Shape()
        # 4. 几何变换
        transform_1 = gp_Trsf()
        transform_1.SetTranslation(gp_Vec(point[0], point[1], point[2]))
        compound_shape = BRepBuilderAPI_Transform(
            compound_shape, transform_1, False
        ).Shape()  # 不复制该个体
        transform_2 = gp_Trsf()
        transform_2.SetRotation(
            gp_Ax1(
                gp_Pnt(point[0], point[1], point[2]),
                gp_Dir(rotation_axis[0], rotation_axis[1], rotation_axis[2]),
            ),
            theta,
        )
        compound_shape = BRepBuilderAPI_Transform(
            compound_shape, transform_2, False
        ).Shape()  # 不复制该个体
        return compound_shape

    @staticmethod
    def build_embedded_anchor_embedded_part_shape(
        point: List[float], shape: Dict, theta: float, rotation_axis: List[float]
    ):
        """
        单个预埋锚栓OCC模型
        :param point: 预埋锚栓标志点
        :param shape: 预埋锚栓数据
        :param theta: 预埋锚栓旋转角，逆时针为正，初始状态竖直向上
        :param rotation_axis: 预埋锚栓旋转轴
        :return:
        """
        # 1.基本数据
        top_diam = shape["m_diameter"]  # 底端直径
        top_length = shape["m_length"]  # 顶端长度
        bottom_diam = shape["o_diameter"]  # 底端直径
        bottom_length = shape["length"]  # 底端长度
        rebar_diam = shape["s_diameter"]  # 锚固钢筋的直径
        rebar_length = shape["l_p"]  # 锚固钢筋的长度
        edge_length = shape["a"]  # 钢筋与底端距离
        e_diam = shape["e_diameter"]  # 嵌入直径
        e_length = shape["g"]  # 嵌入长度
        b_length = shape["b"]  # 圆锥的高度
        # 2.各组成部分初始坐标点
        point_1 = [0, 0, -top_length]  # 圆台坐标原点
        point_2 = [0, 0, -top_length - bottom_length]  # 圆柱坐标原点
        point_3 = [0, 0, -top_length - bottom_length + edge_length]  # 钢筋分两段，便于旋转
        point_4 = [0, 0, -top_length - e_length]  # 嵌入圆柱体，布尔运算去掉
        point_5 = [0, 0, -top_length - e_length - b_length]  # 圆锥体，布尔运算去掉
        # 3.开始建立实体
        # 3.1 顶部圆台
        top_truncated_cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(point_1[0], point_1[1], point_1[2]), gp_Dir(0, 0, 1)),
            bottom_diam / 2,
            top_diam / 2,
            top_length,
        ).Shape()
        # 3.2 底部圆柱
        bottom_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_2[0], point_2[1], point_2[2]), gp_Dir(0, 0, 1)),
            bottom_diam / 2,
            bottom_length,
        ).Shape()
        # 3.3 形成连接体
        compound_shape = my_BRepAlgoAPI_Fuse(
            top_truncated_cone, bottom_cylinder
        ).Shape()
        # 3.4 左段钢筋
        rebar_left_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_3[0], point_3[1], point_3[2]), gp_Dir(1, 0, 0)),
            rebar_diam / 2,
            rebar_length / 2,
        ).Shape()
        # 3.5 右段钢筋
        rebar_right_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_3[0], point_3[1], point_3[2]), gp_Dir(-1, 0, 0)),
            rebar_diam / 2,
            rebar_length / 2,
        ).Shape()
        # 3.6 完整钢筋实体
        rebar_cylinder = BRepAlgoAPI_Fuse(
            rebar_left_cylinder, rebar_right_cylinder
        ).Shape()
        # 3.7 合并钢筋实体
        compound_shape = BRepAlgoAPI_Fuse(compound_shape, rebar_cylinder).Shape()  #
        # 3.8 嵌入圆柱体
        implant_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(point_4[0], point_4[1], point_4[2]), gp_Dir(0, 0, 1)),
            e_diam / 2,
            e_length / 2,
        ).Shape()
        implant_cone = BRepPrimAPI_MakeCone(
            gp_Ax2(gp_Pnt(point_5[0], point_5[1], point_5[2]), gp_Dir(0, 0, 1)),
            0,
            e_diam / 2,
            b_length,
        ).Shape()
        implant_solid = BRepAlgoAPI_Fuse(implant_cylinder, implant_cone).Shape()
        # # 3.9 去除嵌入圆柱体和圆锥体
        compound_shape = BRepAlgoAPI_Cut(compound_shape, implant_solid).Shape()
        # 4. 几何变换
        transform_1 = gp_Trsf()
        transform_1.SetTranslation(gp_Vec(point[0], point[1], point[2]))
        compound_shape = BRepBuilderAPI_Transform(
            compound_shape, transform_1, False
        ).Shape()  # 不复制该个体
        transform_2 = gp_Trsf()
        transform_2.SetRotation(
            gp_Ax1(
                gp_Pnt(point[0], point[1], point[2]),
                gp_Dir(rotation_axis[0], rotation_axis[1], rotation_axis[2]),
            ),
            theta,
        )
        compound_shape = BRepBuilderAPI_Transform(
            compound_shape, transform_2, False
        ).Shape()  # 不复制该个体
        return compound_shape

    def build_specific_hoist_embedded_part_shape(self, num: int):
        """
        建立特定吊装预埋件形状模型：用于单个吊装预埋件投影或剖切操作
        :param num: 吊装预埋件序号
        :return:
        """
        hoist_info = self.occ_data.get_hoist_embedded_part_datas()  # 获取吊装预埋件信息
        name = hoist_info["type"]  # 获取吊装预埋件的类型
        shape = hoist_info["specification"]  # 获取吊装预埋件的规格
        embedded_parts = hoist_info["location"]  # 获取吊装预埋件定位信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        # 建立单个吊装预埋件
        embedded_loc = copy.deepcopy(embedded_parts[num])  # 获取钢筋当前路径---坐标、旋转角，旋转轴
        if name == 0:
            shape_ = self.build_rounding_heading_embedded_part_shape(
                embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
            )
        else:
            shape_ = self.build_embedded_anchor_embedded_part_shape(
                embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
            )
        init_shape = BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_hoist_embedded_part_shape(self):
        """
        建立吊装预埋件OCC模型，hoist_info: 包含吊装预埋件类型，吊装预埋件的位置，型号，旋转轴，旋转角度--逆时针
        :return:TopoDS_Shape
        """
        hoist_info = self.occ_data.get_hoist_embedded_part_datas()  # 获取吊装预埋件信息
        name = hoist_info["type"]  # 获取吊装预埋件的类型
        shape = hoist_info["specification"]  # 获取吊装预埋件的规格
        embedded_parts = hoist_info["location"]  # 获取吊装预埋件定位信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(embedded_parts)):
            embedded_loc = copy.deepcopy(embedded_parts[num])  # 获取钢筋当前路径---坐标、旋转角，旋转轴
            if name == 0:
                shape_ = self.build_rounding_heading_embedded_part_shape(
                    embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
                )
            else:
                shape_ = self.build_embedded_anchor_embedded_part_shape(
                    embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
                )
            init_shape = BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_specific_basic_demold_embedded_part_shape(self, num: int):
        """
        建立特定特殊脱模预埋件形状模型：用于单个脱模预埋件投影或剖切操作
        :param num: 序号
        :return:
        """
        demold_info = self.occ_data.get_demold_embedded_part_datas()  # 获取吊装预埋件数据信息
        name = demold_info["type"]  # 获取脱模预埋件的类型
        shape = demold_info["specification"]  # 获取脱模预埋件的规格
        embedded_parts = demold_info["location"]  # 获取脱模预埋件定位信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        # 建立单个脱模预埋件
        embedded_loc = copy.deepcopy(embedded_parts[num])  # 获取钢筋当前路径---坐标、旋转角，旋转轴
        if name == 0:
            shape_ = self.build_rounding_heading_embedded_part_shape(
                embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
            )
        else:
            shape_ = self.build_basic_embedded_anchor_embedded_part_shape(
                embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
            )
        init_shape = BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_specific_demold_embedded_part_shape(self, num: int):
        """
        建立特定脱模预埋件形状模型：用于单个脱模预埋件投影或剖切操作
        :param num: 序号
        :return:
        """
        demold_info = self.occ_data.get_demold_embedded_part_datas()  # 获取吊装预埋件数据信息
        name = demold_info["type"]  # 获取脱模预埋件的类型
        shape = demold_info["specification"]  # 获取脱模预埋件的规格
        embedded_parts = demold_info["location"]  # 获取脱模预埋件定位信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        # 建立单个脱模预埋件
        embedded_loc = copy.deepcopy(embedded_parts[num])  # 获取钢筋当前路径---坐标、旋转角，旋转轴
        if name == 0:
            shape_ = self.build_rounding_heading_embedded_part_shape(
                embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
            )
        else:
            shape_ = self.build_embedded_anchor_embedded_part_shape(
                embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
            )
        init_shape = BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_demold_embedded_part_shape(self):
        """
        建立脱模预埋件OCC模型，demold_info: 包含脱模预埋件类型，脱模预埋件的位置，型号，旋转轴，旋转角度--逆时针
        :return:
        """
        demold_info = self.occ_data.get_demold_embedded_part_datas()  # 获取吊装预埋件数据信息
        name = demold_info["type"]  # 获取脱模预埋件的类型
        shape = demold_info["specification"]  # 获取脱模预埋件的规格
        embedded_parts = demold_info["location"]  # 获取脱模预埋件定位信息
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for num in range(len(embedded_parts)):
            embedded_loc = copy.deepcopy(embedded_parts[num])  # 获取钢筋当前路径---坐标、旋转角，旋转轴
            if name == 0:
                shape_ = self.build_rounding_heading_embedded_part_shape(
                    embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
                )
            else:
                shape_ = self.build_embedded_anchor_embedded_part_shape(
                    embedded_loc[0], shape, embedded_loc[1], embedded_loc[2]
                )
            init_shape = BRepAlgoAPI_Fuse(init_shape, shape_).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_left_beam_shape(self):
        """
        建立底端左侧平台梁OCC模型
        :return:
        """
        # 获取楼梯平台梁和平台板信息
        ladder_beam_and_slab_info = (
            self.occ_data.get_ladder_beam_and_slab_datas()
        )  # 获取楼梯平台梁和平台板数据
        bottom_beam_info = ladder_beam_and_slab_info["bottom_beam"]  # 底部平台梁信息
        bottom_beam_loc = bottom_beam_info["location"]  # 坐标点
        direction = bottom_beam_info["direction"]  # 方向
        bottom_beam_shape = self.build_stretch_profile_solid(bottom_beam_loc, direction)
        return bottom_beam_shape

    def build_bottom_left_slab_shape(self):
        """
        建立底端左侧平台板OCC模型
        :return:
        """
        # 获取楼梯平台梁和平台板信息
        ladder_beam_and_slab_info = (
            self.occ_data.get_ladder_beam_and_slab_datas()
        )  # 获取楼梯平台梁和平台板数据
        bottom_slab_info = ladder_beam_and_slab_info["bottom_slab"]  # 底部平台板信息
        bottom_slab_loc = bottom_slab_info["location"]  # 坐标点
        direction = bottom_slab_info["direction"]  # 方向
        bottom_slab_shape = self.build_stretch_profile_solid(bottom_slab_loc, direction)
        return bottom_slab_shape

    def build_top_right_beam_shape(self):
        """
        建立顶端右侧平台梁OCC模型
        :return:
        """
        # 获取楼梯平台梁和平台板信息
        ladder_beam_and_slab_info = (
            self.occ_data.get_ladder_beam_and_slab_datas()
        )  # 获取楼梯平台梁和平台板数据
        top_beam_info = ladder_beam_and_slab_info["top_beam"]  # 顶部平台梁信息
        top_beam_loc = top_beam_info["location"]  # 坐标点
        direction = top_beam_info["direction"]  # 方向
        top_beam_shape = self.build_stretch_profile_solid(top_beam_loc, direction)
        return top_beam_shape

    def build_top_right_slab_shape(self):
        """
        建立顶端右侧平台板OCC模型
        :return:
        """
        # 获取楼梯平台梁和平台板信息
        ladder_beam_and_slab_info = (
            self.occ_data.get_ladder_beam_and_slab_datas()
        )  # 获取楼梯平台梁和平台板数据
        top_slab_info = ladder_beam_and_slab_info["top_slab"]  # 底部平台板信息
        top_slab_loc = top_slab_info["location"]  # 坐标点
        direction = top_slab_info["direction"]  # 方向
        top_slab_shape = self.build_stretch_profile_solid(top_slab_loc, direction)
        return top_slab_shape

    def build_bottom_beam_slab_shape(self):
        """
        建立底部平台板和平台梁
        :return:
        """
        # 获取楼梯平台梁和平台板信息
        ladder_beam_and_slab_info = (
            self.occ_data.get_ladder_beam_and_slab_datas()
        )  # 获取楼梯平台梁和平台板数据
        bottom_beam_info = ladder_beam_and_slab_info["bottom_beam"]  # 底部平台梁信息
        bottom_beam_loc = bottom_beam_info["location"]  # 坐标点
        direction_b = bottom_beam_info["direction"]  # 方向
        bottom_slab_info = ladder_beam_and_slab_info["bottom_slab"]  # 底部平台板信息
        bottom_slab_loc = bottom_slab_info["location"]  # 坐标点
        direction_s = bottom_slab_info["direction"]  # 方向
        bottom_slab_shape = self.build_stretch_profile_solid(
            bottom_slab_loc, direction_s
        )
        bottom_beam_shape = self.build_stretch_profile_solid(
            bottom_beam_loc, direction_b
        )
        combine_model = my_BRepAlgoAPI_Fuse(
            bottom_slab_shape, bottom_beam_shape
        ).Shape()  # 合并平台板和平台梁模型
        return combine_model

    def build_top_beam_slab_shape(self):
        """
        建立顶部平台板和平台梁
        :return:
        """
        # 获取楼梯平台梁和平台板信息
        ladder_beam_and_slab_info = (
            self.occ_data.get_ladder_beam_and_slab_datas()
        )  # 获取楼梯平台梁和平台板数据
        top_beam_info = ladder_beam_and_slab_info["top_beam"]  # 顶部平台梁信息
        top_beam_loc = top_beam_info["location"]  # 坐标点
        direction_b = top_beam_info["direction"]  # 方向
        top_slab_info = ladder_beam_and_slab_info["top_slab"]  # 顶部平台板信息
        top_slab_loc = top_slab_info["location"]  # 坐标点
        direction_s = top_slab_info["direction"]  # 方向
        top_slab_shape = self.build_stretch_profile_solid(top_slab_loc, direction_s)
        top_beam_shape = self.build_stretch_profile_solid(top_beam_loc, direction_b)
        combine_model = my_BRepAlgoAPI_Fuse(
            top_slab_shape, top_beam_shape
        ).Shape()  # 合并平台板和平台梁模型
        return combine_model

    @staticmethod
    def build_single_nut_shape(
        nut_loc: List[float], rebar_diam: float, nut_thick: float, edge_l: float
    ):
        """
        建立单个OCC螺母模型
        :param nut_loc: 螺母自己欧标点
        :param rebar_diam: 锚固钢筋直径
        :param nut_thick: 螺母厚度
        :param edge_l: 螺母边长
        :return:
        """
        direction = [0, 0, nut_thick]  # 拉伸方向
        add_x = edge_l * math.cos(math.pi / 6)
        add_y = edge_l / 2 + edge_l * math.sin(math.pi / 6)
        point_1 = copy.deepcopy(nut_loc)
        point_1[0] += -add_x
        point_1[1] += edge_l / 2
        point_2 = copy.deepcopy(point_1)
        point_2[1] += -edge_l
        point_3 = copy.deepcopy(nut_loc)
        point_3[1] += -add_y
        point_4 = copy.deepcopy(nut_loc)
        point_4[0] += add_x
        point_4[1] += -edge_l / 2
        point_5 = copy.deepcopy(nut_loc)
        point_5[0] += add_x
        point_5[1] += edge_l / 2
        point_6 = copy.deepcopy(nut_loc)
        point_6[1] += add_y
        profile_point = [point_1, point_2, point_3, point_4, point_5, point_6]
        nut_profile_points = copy.deepcopy(profile_point)
        nut_profile_points.append(profile_point[0])  # 形成封闭图形
        profile_wire = BRepBuilderAPI_MakeWire()
        for num in range(len(nut_profile_points) - 1):
            start_point = nut_profile_points[num]
            end_point = nut_profile_points[num + 1]
            edge = BRepBuilderAPI_MakeEdge(
                gp_Pnt(start_point[0], start_point[1], start_point[2]),
                gp_Pnt(end_point[0], end_point[1], end_point[2]),
            ).Edge()
            profile_wire.Add(edge)
        stretch_dir = gp_Vec(direction[0], direction[1], direction[2])
        profile_face = BRepBuilderAPI_MakeFace(profile_wire.Wire())
        profile_solid = BRepPrimAPI_MakePrism(profile_face.Face(), stretch_dir).Shape()
        profile_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(
                gp_Pnt(nut_loc[0], nut_loc[1], nut_loc[2]),
                gp_Dir(direction[0], direction[1], direction[2]),
            ),
            rebar_diam / 2,
            nut_thick,
        ).Shape()
        cut_model = my_BRepAlgoAPI_Cut(profile_solid, profile_cylinder).Shape()
        return cut_model

    @staticmethod
    def build_single_shim_shape(
        shim_loc: List[float], rebar_diam: float, shim_thick: float, shim_l: float
    ):
        """
        单个垫片OCC模型
        :param shim_loc: 垫片坐标点
        :param rebar_diam: 锚固钢筋直径
        :param shim_thick: 垫片厚度
        :param shim_l: 垫片边长
        :return:
        """
        direction = [0, 0, shim_thick]  # 拉伸方向
        point_1 = copy.deepcopy(shim_loc)
        point_1[0] += -shim_l / 2
        point_1[1] += shim_l / 2
        point_2 = copy.deepcopy(point_1)
        point_2[1] += -shim_l
        point_3 = copy.deepcopy(point_2)
        point_3[0] += shim_l
        point_4 = copy.deepcopy(point_3)
        point_4[1] += shim_l
        profile_point = [point_1, point_2, point_3, point_4]
        shim_profile_points = copy.deepcopy(profile_point)
        shim_profile_points.append(profile_point[0])  # 形成封闭图形
        profile_wire = BRepBuilderAPI_MakeWire()
        for num in range(len(shim_profile_points) - 1):
            start_point = shim_profile_points[num]
            end_point = shim_profile_points[num + 1]
            edge = BRepBuilderAPI_MakeEdge(
                gp_Pnt(start_point[0], start_point[1], start_point[2]),
                gp_Pnt(end_point[0], end_point[1], end_point[2]),
            ).Edge()
            profile_wire.Add(edge)
        stretch_dir = gp_Vec(direction[0], direction[1], direction[2])
        profile_face = BRepBuilderAPI_MakeFace(profile_wire.Wire())
        profile_solid = BRepPrimAPI_MakePrism(profile_face.Face(), stretch_dir).Shape()
        profile_cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(
                gp_Pnt(shim_loc[0], shim_loc[1], shim_loc[2]),
                gp_Dir(direction[0], direction[1], direction[2]),
            ),
            rebar_diam / 2,
            shim_thick,
        ).Shape()
        cut_model = my_BRepAlgoAPI_Cut(profile_solid, profile_cylinder).Shape()
        return cut_model

    def build_single_connect_anchor_rebar_shape(
        self, point: List[List[float]], radius: float
    ):
        """
        建立连接锚固钢筋形状---无论顶部还是底部都适用
        :param point: 钢筋轮廓点
        :param radius: 钢筋的半径
        :return: TopoShape
        """
        return self.build_L_polyarc_shape(point, radius)

    def build_single_slide_hinge_shape(
        self,
        point_r: List[List[float]],
        point_n: List[float],
        point_s: List[float],
        rebar_diam: float,
        nut_thick: float,
        nut_width: float,
        shim_thick: float,
        shim_width: float,
    ):
        """
        建立单个滑动铰支座模型
        :param point_r: 钢筋坐标点
        :param point_n: 螺纹坐标点
        :param point_s: 垫片坐标点
        :param rebar_diam: 钢筋直径
        :param nut_thick: 螺母的厚度
        :param nut_width: 螺母的宽度
        :param shim_thick: 垫片的厚度
        :param shim_width: 垫片的宽度
        :return:
        """
        rebar_model = self.build_single_connect_anchor_rebar_shape(
            point_r, rebar_diam / 2
        )
        nut_model = self.build_single_nut_shape(
            point_n, rebar_diam, nut_thick, nut_width
        )
        shim_model = self.build_single_shim_shape(
            point_s, rebar_diam, shim_thick, shim_width
        )
        combine_model = BRepAlgoAPI_Fuse(rebar_model, nut_model).Shape()
        final_model = BRepAlgoAPI_Fuse(combine_model, shim_model).Shape()  # 合并模型
        return final_model

    def build_single_fix_hinge_shape(
        self, point_r: List[List[float]], rebar_diam: float
    ):
        """
        建立单个固定铰支座模型
        :param point_r: 锚固坐标点
        :param rebar_diam: 钢筋直径
        :return:
        """
        rebar_model = self.build_single_connect_anchor_rebar_shape(
            point_r, rebar_diam / 2
        )
        return rebar_model

    def build_bottom_connect_rebar_nut_shim_shape(self):
        """
        建立底部锚固钢筋、螺母、垫片OCC模型
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        anchor_rebar_info = embedded_total_info["anchor_rebar_location"]  # 获取锚固钢筋坐标位置信息
        bottom_rebar_loc = anchor_rebar_info["bottom_rebar"]  # 底部锚固钢筋坐标点
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        bottom_hole_loc = hole_loc_info["bottom_hole"]  # 获取底部孔洞位置
        nut_loc_info = embedded_total_info["nut"]  # 螺母坐标信息
        bottom_nut_loc = nut_loc_info["bottom_nut"]  # 底部螺母坐标点
        shim_loc_info = embedded_total_info["shim"]  # 垫片坐标信息
        bottom_shim_loc = shim_loc_info["bottom_shim"]  # 底部垫片坐标信息
        element_info = embedded_total_info["element_info"]  # 部件信息
        bottom_shape = element_info["bottom_shape"]
        bottom_rebar_diam = bottom_shape.rebar_anchor_diameter  # 锚固钢筋的直径
        nut_thick = bottom_shape.nut_thickness  # 螺母厚度
        nut_diam = bottom_shape.nut_diameter  # 锚固钢筋直径
        nut_width = bottom_shape.nut_width  # 螺母宽度
        shim_thick = bottom_shape.shim_thickness  # 垫片厚度
        shim_width = bottom_shape.shim_width  # 垫片宽度
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if self.bottom_node_type == 1:  # 底端为滑动铰支座
            for point_h in bottom_hole_loc:  # 遍历每个孔洞位置
                point_r = []  # 锚固钢筋坐标点
                for point_1 in bottom_rebar_loc:
                    curr_p = [0, 0, 0]
                    curr_p[0] = point_h[0] + point_1[0]
                    curr_p[1] = point_h[1] + point_1[1]
                    curr_p[2] = point_h[2] + point_1[2]
                    point_r.append(curr_p)
                # 螺纹坐标点
                point_n = [0, 0, 0]
                point_n[0] = point_h[0] + bottom_nut_loc[0]
                point_n[1] = point_h[1] + bottom_nut_loc[1]
                point_n[2] = point_h[2] + bottom_nut_loc[2]
                # 垫片坐标点
                point_s = [0, 0, 0]
                point_s[0] = point_h[0] + bottom_shim_loc[0]
                point_s[1] = point_h[1] + bottom_shim_loc[1]
                point_s[2] = point_h[2] + bottom_shim_loc[2]
                current_model = self.build_single_slide_hinge_shape(
                    point_r,
                    point_n,
                    point_s,
                    bottom_rebar_diam,
                    nut_thick,
                    nut_width,
                    shim_thick,
                    shim_width,
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        else:  # 底端为固定铰支座
            for point_h in bottom_hole_loc:  # 遍历每个孔洞位置
                point_r = []  # 锚固钢筋坐标点
                for point_1 in bottom_rebar_loc:
                    curr_p = [0, 0, 0]
                    curr_p[0] = point_h[0] + point_1[0]
                    curr_p[1] = point_h[1] + point_1[1]
                    curr_p[2] = point_h[2] + point_1[2]
                    point_r.append(curr_p)
                current_model = self.build_single_fix_hinge_shape(
                    point_r, bottom_rebar_diam
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_connect_rebar_shape(self):

        """
        建立底部连接锚固钢筋形状
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        anchor_rebar_info = embedded_total_info["anchor_rebar_location"]  # 获取锚固钢筋坐标位置信息
        bottom_rebar_loc = anchor_rebar_info["bottom_rebar"]  # 底部锚固钢筋坐标点
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        bottom_hole_loc = hole_loc_info["bottom_hole"]  # 获取底部孔洞位置
        element_info = embedded_total_info["element_info"]  # 部件信息
        bottom_shape = element_info["bottom_shape"]
        bottom_rebar_diam = bottom_shape.rebar_anchor_diameter  # 锚固钢筋的直径
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for point_h in bottom_hole_loc:  # 遍历每个孔洞位置
            point_r = []  # 锚固钢筋坐标点
            for point_1 in bottom_rebar_loc:
                curr_p = [0, 0, 0]
                curr_p[0] = point_h[0] + point_1[0]
                curr_p[1] = point_h[1] + point_1[1]
                curr_p[2] = point_h[2] + point_1[2]
                point_r.append(curr_p)
            current_model = self.build_single_connect_anchor_rebar_shape(
                point_r, bottom_rebar_diam / 2
            )
            init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_connect_rebar_shape(self):

        """
        建立顶部连接锚固钢筋形状
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        anchor_rebar_info = embedded_total_info["anchor_rebar_location"]  # 获取锚固钢筋坐标位置信息
        top_rebar_loc = anchor_rebar_info["top_rebar"]  # 顶部锚固钢筋坐标点
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        top_hole_loc = hole_loc_info["top_hole"]  # 获取顶部孔洞位置
        element_info = embedded_total_info["element_info"]  # 部件信息
        top_shape = element_info["top_shape"]
        top_rebar_diam = top_shape.rebar_anchor_diameter  # 锚固钢筋的直径
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for point_h in top_hole_loc:  # 遍历每个孔洞位置
            point_r = []  # 锚固钢筋坐标点
            for point_1 in top_rebar_loc:
                curr_p = [0, 0, 0]
                curr_p[0] = point_h[0] + point_1[0]
                curr_p[1] = point_h[1] + point_1[1]
                curr_p[2] = point_h[2] + point_1[2]
                point_r.append(curr_p)
            current_model = self.build_single_connect_anchor_rebar_shape(
                point_r, top_rebar_diam / 2
            )
            init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_connect_nut_shape(self):
        """
        建立底部螺母形状
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        bottom_hole_loc = hole_loc_info["bottom_hole"]  # 获取底部孔洞位置
        nut_loc_info = embedded_total_info["nut"]  # 螺母坐标信息
        bottom_nut_loc = nut_loc_info["bottom_nut"]  # 底部螺母坐标点
        element_info = embedded_total_info["element_info"]  # 部件信息
        bottom_shape = element_info["bottom_shape"]
        nut_thick = bottom_shape.nut_thickness  # 螺母厚度
        nut_diam = bottom_shape.nut_diameter  # 锚固钢筋直径
        nut_width = bottom_shape.nut_width  # 螺母宽度
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if self.bottom_node_type == 1:  # 底端为滑动铰支座
            for point_h in bottom_hole_loc:  # 遍历每个孔洞位置
                # 螺纹坐标点
                point_n = [0, 0, 0]
                point_n[0] = point_h[0] + bottom_nut_loc[0]
                point_n[1] = point_h[1] + bottom_nut_loc[1]
                point_n[2] = point_h[2] + bottom_nut_loc[2]
                current_model = self.build_single_nut_shape(
                    point_n, nut_diam, nut_thick, nut_width
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        else:  # 底端为固定铰支座
            pass
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_connect_nut_shape(self):
        """
        建立顶部螺母形状
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        top_hole_loc = hole_loc_info["top_hole"]  # 获取顶部孔洞位置
        nut_loc_info = embedded_total_info["nut"]  # 螺母坐标信息
        top_nut_loc = nut_loc_info["top_nut"]  # 顶部螺母坐标点
        element_info = embedded_total_info["element_info"]  # 部件信息
        top_shape = element_info["top_shape"]
        nut_thick = top_shape.nut_thickness  # 螺母厚度
        nut_diam = top_shape.nut_diameter  # 锚固钢筋直径
        nut_width = top_shape.nut_width  # 螺母宽度
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if self.top_node_type == 1:  # 顶端为滑动铰支座
            for point_h in top_hole_loc:  # 遍历每个孔洞位置
                # 螺纹坐标点
                point_n = [0, 0, 0]
                point_n[0] = point_h[0] + top_nut_loc[0]
                point_n[1] = point_h[1] + top_nut_loc[1]
                point_n[2] = point_h[2] + top_nut_loc[2]
                current_model = self.build_single_nut_shape(
                    point_n, nut_diam, nut_thick, nut_width
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        else:  # 顶端为固定铰支座
            pass
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_bottom_connect_shim_shape(self):
        """
        建立底部连接垫片形状
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        bottom_hole_loc = hole_loc_info["bottom_hole"]  # 获取底部孔洞位置
        shim_loc_info = embedded_total_info["shim"]  # 垫片坐标信息
        bottom_shim_loc = shim_loc_info["bottom_shim"]  # 底部垫片坐标信息
        element_info = embedded_total_info["element_info"]  # 部件信息
        bottom_shape = element_info["bottom_shape"]
        bottom_rebar_diam = bottom_shape.rebar_anchor_diameter  # 锚固钢筋的直径
        shim_thick = bottom_shape.shim_thickness  # 垫片厚度
        shim_width = bottom_shape.shim_width  # 垫片宽度
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if self.bottom_node_type == 1:  # 底端为滑动铰支座
            for point_h in bottom_hole_loc:  # 遍历每个孔洞位置
                # 垫片坐标点
                point_s = [0, 0, 0]
                point_s[0] = point_h[0] + bottom_shim_loc[0]
                point_s[1] = point_h[1] + bottom_shim_loc[1]
                point_s[2] = point_h[2] + bottom_shim_loc[2]
                current_model = self.build_single_shim_shape(
                    point_s, bottom_rebar_diam, shim_thick, shim_width
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_connect_shim_shape(self):
        """
        建立顶部连接垫片形状
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        top_hole_loc = hole_loc_info["top_hole"]  # 获取顶部孔洞位置
        shim_loc_info = embedded_total_info["shim"]  # 垫片坐标信息
        top_shim_loc = shim_loc_info["top_shim"]  # 顶部垫片坐标信息
        element_info = embedded_total_info["element_info"]  # 部件信息
        top_shape = element_info["top_shape"]
        top_rebar_diam = top_shape.rebar_anchor_diameter  # 锚固钢筋的直径
        shim_thick = top_shape.shim_thickness  # 垫片厚度
        shim_width = top_shape.shim_width  # 垫片宽度
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if self.top_node_type == 1:  # 顶端为滑动铰支座
            for point_h in top_hole_loc:  # 遍历每个孔洞位置
                # 垫片坐标点
                point_s = [0, 0, 0]
                point_s[0] = point_h[0] + top_shim_loc[0]
                point_s[1] = point_h[1] + top_shim_loc[1]
                point_s[2] = point_h[2] + top_shim_loc[2]
                current_model = self.build_single_shim_shape(
                    point_s, top_rebar_diam, shim_thick, shim_width
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    def build_top_connect_rebar_nut_shim_shape(self):
        """
        建立顶部锚固钢筋、螺母、垫片OCC模型
        :return:
        """
        # 获取基本信息
        embedded_total_info = (
            self.occ_data.get_connect_embedded_part_datas()
        )  # 获取连接预埋件部件数据
        anchor_rebar_info = embedded_total_info["anchor_rebar_location"]  # 获取锚固钢筋坐标位置信息
        top_rebar_loc = anchor_rebar_info["top_rebar"]  # 顶部锚固钢筋坐标点
        hole_loc_info = embedded_total_info["hole_location"]  # 获取孔洞位置信息
        top_hole_loc = hole_loc_info["top_hole"]  # 获取顶部孔洞位置
        nut_loc_info = embedded_total_info["nut"]  # 螺母坐标信息
        top_nut_loc = nut_loc_info["top_nut"]  # 顶部螺母坐标点
        shim_loc_info = embedded_total_info["shim"]  # 垫片坐标信息
        top_shim_loc = shim_loc_info["top_shim"]  # 底部垫片坐标信息
        element_info = embedded_total_info["element_info"]  # 部件信息
        top_shape = element_info["top_shape"]
        top_rebar_diam = top_shape.rebar_anchor_diameter  # 锚固钢筋的直径
        nut_thick = top_shape.nut_thickness  # 螺母厚度
        nut_diam = top_shape.nut_diameter  # 锚固钢筋直径
        nut_width = top_shape.nut_width  # 螺母宽度
        shim_thick = top_shape.shim_thickness  # 垫片厚度
        shim_width = top_shape.shim_width  # 垫片宽度
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        if self.top_node_type == 1:  # 底端为滑动铰支座
            for point_h in top_hole_loc:  # 遍历每个孔洞位置
                point_r = []  # 锚固钢筋坐标点
                for point_1 in top_rebar_loc:
                    curr_p = [0, 0, 0]
                    curr_p[0] = point_h[0] + point_1[0]
                    curr_p[1] = point_h[1] + point_1[1]
                    curr_p[2] = point_h[2] + point_1[2]
                    point_r.append(curr_p)
                # 螺纹坐标点
                point_n = [0, 0, 0]
                point_n[0] = point_h[0] + top_nut_loc[0]
                point_n[1] = point_h[1] + top_nut_loc[1]
                point_n[2] = point_h[2] + top_nut_loc[2]
                # 垫片坐标点
                point_s = [0, 0, 0]
                point_s[0] = point_h[0] + top_shim_loc[0]
                point_s[1] = point_h[1] + top_shim_loc[1]
                point_s[2] = point_h[2] + top_shim_loc[2]
                current_model = self.build_single_slide_hinge_shape(
                    point_r,
                    point_n,
                    point_s,
                    top_rebar_diam,
                    nut_thick,
                    nut_width,
                    shim_thick,
                    shim_width,
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        else:  # 底端为固定铰支座
            for point_h in top_hole_loc:  # 遍历每个孔洞位置
                point_r = []  # 锚固钢筋坐标点
                for point_1 in top_rebar_loc:
                    curr_p = [0, 0, 0]
                    curr_p[0] = point_h[0] + point_1[0]
                    curr_p[1] = point_h[1] + point_1[1]
                    curr_p[2] = point_h[2] + point_1[2]
                    point_r.append(curr_p)
                current_model = self.build_single_fix_hinge_shape(
                    point_r, top_rebar_diam
                )
                init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape

    @staticmethod
    def build_face_stretch_solid(points: List[List[float]], length: float):
        """
        将平面沿着一个方向拉伸
        :param points: 点集合
        :param length:
        :return:
        """
        point_1 = points[0]
        point_2 = points[1]
        point_3 = points[2]
        point_4 = points[3]
        vec_12 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )
        vec_23 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
            np.array(point_3) - np.array(point_2)
        )

        normal_ = np.cross(vec_12, vec_23) * length  # 四边形平面法向量
        edge_12 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
        ).Edge()
        edge_23 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_2[0], point_2[1], point_2[2]),
            gp_Pnt(point_3[0], point_3[1], point_3[2]),
        ).Edge()
        edge_34 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_3[0], point_3[1], point_3[2]),
            gp_Pnt(point_4[0], point_4[1], point_4[2]),
        ).Edge()
        edge_41 = BRepBuilderAPI_MakeEdge(
            gp_Pnt(point_4[0], point_4[1], point_4[2]),
            gp_Pnt(point_1[0], point_1[1], point_1[2]),
        ).Edge()
        wire_ = BRepBuilderAPI_MakeWire()
        wire_.Add(edge_12)
        wire_.Add(edge_23)
        wire_.Add(edge_34)
        wire_.Add(edge_41)
        face_ = BRepBuilderAPI_MakeFace(wire_.Wire()).Face()
        solid_ = BRepPrimAPI_MakePrism(
            face_, gp_Vec(normal_[0], normal_[1], normal_[2])
        ).Shape()
        return solid_

    def build_single_rail_rabbet_shape(
        self,
        point: List[float],
        rabbet_x: float,
        rabbet_y: float,
        rabbet_height: float,
        rabbet_extend_width: float,
    ):
        """
        建立单个企口形状信息
        :param point: 企口坐标点
        :param rabbet_x: 企口底部x方向长度
        :param rabbet_y: 企口底部y方向长度
        :param rabbet_height: 企口高度
        :param rabbet_extend_width: 企口两边延伸宽度
        :return:
        """
        origin = copy.deepcopy(point)  # 企口形状中心
        origin[2] += rabbet_height  # 企口高度
        dx = rabbet_x + 2 * rabbet_extend_width
        dy = rabbet_y + 2 * rabbet_extend_width
        dz = rabbet_height
        add_value = 2 * rabbet_extend_width
        # 开始创建点
        # 顶部四个点
        point_t_1 = copy.deepcopy(origin)
        point_t_1[0] += -dx / 2
        point_t_1[1] += dy / 2
        point_t_2 = copy.deepcopy(point_t_1)
        point_t_2[1] += -dy
        point_t_3 = copy.deepcopy(point_t_2)
        point_t_3[0] += dx
        point_t_4 = copy.deepcopy(point_t_3)
        point_t_4[1] += dy
        # 底部四个点
        point_b_1 = copy.deepcopy(point_t_1)
        point_b_1[0] += +rabbet_extend_width
        point_b_1[1] += -rabbet_extend_width
        point_b_1[2] += -rabbet_height
        point_b_2 = copy.deepcopy(point_t_2)
        point_b_2[0] += +rabbet_extend_width
        point_b_2[1] += +rabbet_extend_width
        point_b_2[2] += -rabbet_height
        point_b_3 = copy.deepcopy(point_t_3)
        point_b_3[0] += -rabbet_extend_width
        point_b_3[1] += +rabbet_extend_width
        point_b_3[2] += -rabbet_height
        point_b_4 = copy.deepcopy(point_t_4)
        point_b_4[0] += -rabbet_extend_width
        point_b_4[1] += -rabbet_extend_width
        point_b_4[2] += -rabbet_height
        # 多边形
        point_l = [
            [point_t_1[0], point_t_1[1] + add_value, point_t_1[2]],
            [point_b_1[0], point_b_1[1] + add_value, point_b_1[2]],
            [point_b_2[0], point_b_2[1] - add_value, point_b_2[2]],
            [point_t_2[0], point_t_2[1] - add_value, point_t_2[2]],
        ]
        point_t = [
            [point_t_1[0] - add_value, point_t_1[1], point_t_1[2]],
            [point_t_4[0] + add_value, point_t_4[1], point_t_4[2]],
            [point_b_4[0] + add_value, point_b_4[1], point_b_4[2]],
            [point_b_1[0] - add_value, point_b_1[1], point_b_1[2]],
        ]
        point_r = [
            [point_t_4[0], point_t_4[1] + add_value, point_t_4[2]],
            [point_t_3[0], point_t_3[1] - add_value, point_t_3[2]],
            [point_b_3[0], point_b_3[1] - add_value, point_b_3[2]],
            [point_b_4[0], point_b_4[1] + add_value, point_b_4[2]],
        ]
        point_b = [
            [point_t_3[0] + add_value, point_t_3[1], point_t_3[2]],
            [point_t_2[0] - add_value, point_t_2[1], point_t_2[2]],
            [point_b_2[0] - add_value, point_b_2[1], point_b_2[2]],
            [point_b_3[0] + add_value, point_b_3[1], point_b_3[2]],
        ]
        length = max(dx, dy)  # 获取最长的值
        # 建立实体
        box = BRepPrimAPI_MakeBox(
            gp_Ax2(
                gp_Pnt(origin[0] + dx / 2, origin[1] - dy / 2, origin[2]),
                gp_Dir(0, 0, -1),
            ),
            dx,
            dy,
            dz,
        ).Shape()
        solid_l = self.build_face_stretch_solid(point_l, length)
        solid_r = self.build_face_stretch_solid(point_r, length)
        solid_t = self.build_face_stretch_solid(point_t, length)
        solid_b = self.build_face_stretch_solid(point_b, length)
        # 去掉不需要的实体
        cut_shape_l = BRepAlgoAPI_Cut(box, solid_l).Shape()
        cut_shape_r = BRepAlgoAPI_Cut(cut_shape_l, solid_r).Shape()
        cut_shape_t = BRepAlgoAPI_Cut(cut_shape_r, solid_t).Shape()
        final_cut_shape = BRepAlgoAPI_Cut(cut_shape_t, solid_b).Shape()
        # # 合并实体
        # fuse_shape = BRepAlgoAPI_Fuse(solid_l,solid_r).Shape()
        # fuse_shape = BRepAlgoAPI_Fuse(fuse_shape,solid_t).Shape()
        # fuse_shape = BRepAlgoAPI_Fuse(fuse_shape,solid_b).Shape()
        # fuse_shape = BRepAlgoAPI_Fuse(fuse_shape,box).Shape()
        # display.DisplayShape(solid_l, update=True)
        # start_display()
        return final_cut_shape

    def build_total_rail_rabbet_shape(self):
        """
        建立所有企口形状模型
        :return:
        """
        rail_rabbet_info = self.occ_data.get_rail_embedded_rabbet_datas()  # 获取栏杆埋件企口信息
        rail_rabbet_loc = rail_rabbet_info["location"]
        rail_rabbet_shape = rail_rabbet_info["rabbet_shape"]  # 获取栏杆埋件企口形状
        rabbet_x = rail_rabbet_shape["rail_x"]
        rabbet_y = rail_rabbet_shape["rail_y"]
        rabbet_height = rail_rabbet_shape["height"]
        rabbet_extend_width = rail_rabbet_shape["extend_width"]
        # 建立OCC模型
        init_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        cut_shape = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -100), 1, 1, 1).Shape()
        for point in rail_rabbet_loc:  # 遍历每个栏杆预埋件位置
            current_model = self.build_single_rail_rabbet_shape(
                point, rabbet_x, rabbet_y, rabbet_height, rabbet_extend_width
            )
            init_shape = BRepAlgoAPI_Fuse(init_shape, current_model).Shape()
        init_shape = BRepAlgoAPI_Cut(init_shape, cut_shape).Shape()
        return init_shape


class BuildMergeAndCutModel(object):
    """
    建立楼梯合并和拆分模型
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.stair_solid = BuildStairSolidModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 建立楼梯实体模型类
        self.generate_basic_datas()

    def generate_basic_datas(self):
        """
        产生基础数据
        :return:
        """
        self.b0 = self.detail_slab.geometric_detailed.width
        self.lb_d = self.detail_slab.geometric_detailed.bottom_top_length
        self.lt_d = self.detail_slab.geometric_detailed.top_top_length
        self.ln = self.slab_struct.geometric.clear_span
        self.h = self.slab_struct.geometric.height
        self.h2 = self.detail_slab.geometric_detailed.bottom_thickness

    def get_stair_entity_complete_model(self):
        """
        建立楼梯实体完整模型：楼梯实体、顶端和底端挑耳、楼梯孔洞、楼梯防滑槽、楼梯滴水线槽实体
        :return: TopoDS_Shape
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        hole_model = self.stair_solid.build_all_hole_shape()  # 连接孔洞模型
        step_slot_model = self.stair_solid.build_all_step_slot_shape()  # 防滑槽模型
        water_drip_model = self.stair_solid.build_all_water_drip_shape()  # 滴水线槽模型
        update_model = my_BRepAlgoAPI_Fuse(simple_stair, top_ear).Shape()  # 合并顶部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(update_model, bottom_ear).Shape()  # 合并底部挑耳模型
        update_model = my_BRepAlgoAPI_Cut(update_model, hole_model).Shape()  # 挖去孔洞模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, step_slot_model
        ).Shape()  # 挖去防滑槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, water_drip_model
        ).Shape()  # 挖去滴水线槽模型
        return update_model

    def get_stair_solid_and_rail_rabbet_model(self):
        """
        获取楼梯实体和栏杆埋件企口模型
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        rail_rabbet = self.stair_solid.build_total_rail_rabbet_shape()  # 获取栏杆预埋件企口
        update_model = my_BRepAlgoAPI_Fuse(simple_stair, top_ear).Shape()  # 合并顶部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(update_model, bottom_ear).Shape()  # 合并底部挑耳模型
        update_model = my_BRepAlgoAPI_Cut(update_model, rail_rabbet).Shape()  # 合并底部挑耳模型
        return update_model

    def get_stair_entity_detailed_model(self):
        """
        建立楼梯实体精细模型：楼梯实体，顶端和底端挑耳、楼梯孔洞、楼梯防滑槽、楼梯滴水线槽实体、楼梯踏步阴角、楼梯踏步阳角
        :return: TopoDS_Shape
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        hole_model = self.stair_solid.build_all_hole_shape()  # 连接孔洞模型
        step_slot_model = self.stair_solid.build_all_step_slot_shape()  # 防滑槽模型
        water_drip_model = self.stair_solid.build_all_water_drip_shape()  # 滴水线槽模型
        internal_corner = self.stair_solid.build_all_internal_corner()  # 楼梯踏步阴角模型
        external_corner = self.stair_solid.build_all_external_corner()  # 楼梯踏步阳角模型
        update_model = my_BRepAlgoAPI_Fuse(simple_stair, top_ear).Shape()  # 合并顶部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(update_model, bottom_ear).Shape()  # 合并底部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(
            update_model, internal_corner
        ).Shape()  # 合并楼梯踏步阴角模型
        update_model = my_BRepAlgoAPI_Cut(update_model, hole_model).Shape()  # 挖去孔洞模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, step_slot_model
        ).Shape()  # 挖去防滑槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, water_drip_model
        ).Shape()  # 挖去滴水线槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, external_corner
        ).Shape()  # 挖去楼梯阳角模型

        return update_model

    def get_stair_solid_total_model(self):
        """
        建立楼梯实体精细模型：楼梯实体，顶端和底端挑耳、楼梯孔洞、楼梯防滑槽、楼梯滴水线槽实体、楼梯踏步阴角、楼梯踏步阳角、栏杆预埋件企口
        :return: TopoDS_Shape
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        hole_model = self.stair_solid.build_all_hole_shape()  # 连接孔洞模型
        step_slot_model = self.stair_solid.build_all_step_slot_shape()  # 防滑槽模型
        water_drip_model = self.stair_solid.build_all_water_drip_shape()  # 滴水线槽模型
        internal_corner = self.stair_solid.build_all_internal_corner()  # 楼梯踏步阴角模型
        external_corner = self.stair_solid.build_all_external_corner()  # 楼梯踏步阳角模型
        rail_rabbet_model = (
            self.stair_solid.build_total_rail_rabbet_shape()
        )  # 栏杆预埋件企口模型
        update_model = my_BRepAlgoAPI_Fuse(simple_stair, top_ear).Shape()  # 合并顶部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(update_model, bottom_ear).Shape()  # 合并底部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(
            update_model, internal_corner
        ).Shape()  # 合并楼梯踏步阴角模型
        update_model = my_BRepAlgoAPI_Cut(update_model, hole_model).Shape()  # 挖去孔洞模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, step_slot_model
        ).Shape()  # 挖去防滑槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, water_drip_model
        ).Shape()  # 挖去滴水线槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, external_corner
        ).Shape()  # 挖去楼梯阳角模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, rail_rabbet_model
        ).Shape()  # 挖去栏杆预埋件企口模型
        return update_model

    def get_stair_entity_construct_model(self):
        """
        建立楼梯实体构造模型：楼梯实体、顶端和底端挑耳、楼梯孔洞、楼梯防滑槽、楼梯滴水线槽实体、楼梯踏步阴角、楼梯踏步阳角、楼梯吊装件企口
        :return:
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        hole_model = self.stair_solid.build_all_hole_shape()  # 连接孔洞模型
        step_slot_model = self.stair_solid.build_all_step_slot_shape()  # 防滑槽模型
        water_drip_model = self.stair_solid.build_all_water_drip_shape()  # 滴水线槽模型
        internal_corner = self.stair_solid.build_all_internal_corner()  # 楼梯踏步阴角模型
        external_corner = self.stair_solid.build_all_external_corner()  # 楼梯踏步阳角模型
        rabbet_model = self.stair_solid.build_all_hoist_embedded_rabbet()  # 楼梯吊装企口模型
        update_model = my_BRepAlgoAPI_Fuse(simple_stair, top_ear).Shape()  # 合并顶部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(update_model, bottom_ear).Shape()  # 合并底部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(
            update_model, internal_corner
        ).Shape()  # 合并楼梯踏步阴角模型
        update_model = my_BRepAlgoAPI_Cut(update_model, hole_model).Shape()  # 挖去孔洞模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, step_slot_model
        ).Shape()  # 挖去防滑槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, water_drip_model
        ).Shape()  # 挖去滴水线槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, external_corner
        ).Shape()  # 挖去楼梯阳角模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, rabbet_model
        ).Shape()  # 挖去楼梯吊装企口模型
        return update_model

    def get_stair_entity_total_construct_model(self):
        """
        建立楼梯实体构造模型：楼梯实体、顶端和底端挑耳、楼梯孔洞、楼梯防滑槽、楼梯滴水线槽实体、楼梯踏步阴角、楼梯踏步阳角、楼梯吊装件企口
        栏杆预埋件，脱模预埋件
        :return:
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        hole_model = self.stair_solid.build_all_hole_shape()  # 连接孔洞模型
        step_slot_model = self.stair_solid.build_all_step_slot_shape()  # 防滑槽模型
        water_drip_model = self.stair_solid.build_all_water_drip_shape()  # 滴水线槽模型
        internal_corner = self.stair_solid.build_all_internal_corner()  # 楼梯踏步阴角模型
        external_corner = self.stair_solid.build_all_external_corner()  # 楼梯踏步阳角模型
        rabbet_model = self.stair_solid.build_all_hoist_embedded_rabbet()  # 楼梯吊装企口模型
        rail_rabbet_model = (
            self.stair_solid.build_total_rail_rabbet_shape()
        )  # 栏杆预埋件企口模型
        update_model = my_BRepAlgoAPI_Fuse(simple_stair, top_ear).Shape()  # 合并顶部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(update_model, bottom_ear).Shape()  # 合并底部挑耳模型
        update_model = my_BRepAlgoAPI_Fuse(
            update_model, internal_corner
        ).Shape()  # 合并楼梯踏步阴角模型
        update_model = my_BRepAlgoAPI_Cut(update_model, hole_model).Shape()  # 挖去孔洞模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, step_slot_model
        ).Shape()  # 挖去防滑槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, water_drip_model
        ).Shape()  # 挖去滴水线槽模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, external_corner
        ).Shape()  # 挖去楼梯阳角模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, rabbet_model
        ).Shape()  # 挖去楼梯吊装企口模型
        update_model = my_BRepAlgoAPI_Cut(
            update_model, rail_rabbet_model
        ).Shape()  # 挖去栏杆预埋件企口
        return update_model

    def get_stair_rebar_part_model(self):
        """
        建立楼梯钢筋网笼模型
        :return:
        """
        from OCC.Core import BOPAlgo  # 加速布尔运算

        BOPAlgo.BOPAlgo_Algo.SetParallelMode(True)
        bottom_long_rebar_model = (
            self.stair_solid.build_bottom_long_rebar_shape()
        )  # 底部纵筋形状模型
        top_long_rebar_model = self.stair_solid.build_top_long_rebar_shape()  # 顶部纵筋形状模型
        update_model_1 = BRepAlgoAPI_Fuse(
            bottom_long_rebar_model, top_long_rebar_model
        ).Shape()  # 合并顶部纵筋
        mid_distribute_rebar_model = (
            self.stair_solid.build_mid_distribution_rebar_shape()
        )  # 中部分布筋形状模型
        update_model_1 = BRepAlgoAPI_Fuse(
            update_model_1, mid_distribute_rebar_model
        ).Shape()  # 合并中部分布筋
        bottom_edge_long_rebar_model = (
            self.stair_solid.build_bottom_edge_long_rebar_shape()
        )  # 底部边缘纵筋模型
        update_model_1 = BRepAlgoAPI_Fuse(
            update_model_1, bottom_edge_long_rebar_model
        ).Shape()
        top_edge_long_rebar_model = (
            self.stair_solid.build_top_edge_long_rebar_shape()
        )  # 顶部边缘纵筋模型
        update_model_1 = BRepAlgoAPI_Fuse(
            update_model_1, top_edge_long_rebar_model
        ).Shape()
        # update_model_2
        bottom_edge_stir_model = (
            self.stair_solid.build_bottom_edge_stir_shape()
        )  # 底部边缘箍筋模型
        top_edge_stir_model = self.stair_solid.build_top_edge_stir_shape()  # 顶部边缘箍筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            bottom_edge_stir_model, top_edge_stir_model
        ).Shape()
        hole_rein_rebar_model = (
            self.stair_solid.build_hole_rein_rebar_shape()
        )  # 孔洞加强钢筋模型
        update_model_2 = BRepAlgoAPI_Fuse(update_model_2, hole_rein_rebar_model).Shape()
        hoist_rein_long_rebar_model = (
            self.stair_solid.build_hoist_rein_long_rebar_shape()
        )  # 吊装加强纵筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, hoist_rein_long_rebar_model
        ).Shape()
        hoist_rein_point_rebar_model = (
            self.stair_solid.build_hoist_rein_point_rebar_shape()
        )  # 吊装加强点筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, hoist_rein_point_rebar_model
        ).Shape()
        bottom_edge_rein_rebar_model = (
            self.stair_solid.build_bottom_edge_rein_rebar_shape()
        )  # 下部边缘加强筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, bottom_edge_rein_rebar_model
        ).Shape()
        top_edge_rein_rebar_model = (
            self.stair_solid.build_top_edge_rein_rebar_shape()
        )  # 上部边缘加强筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, top_edge_rein_rebar_model
        ).Shape()
        # 合并两个模型
        update_model = BRepAlgoAPI_Fuse(update_model_1, update_model_2).Shape()
        return update_model

    def get_stair_rebar_embedded_part_model(self):
        """
        获取楼梯钢筋和预埋件数据模型
        :return:
        """
        from OCC.Core import BOPAlgo  # 加速布尔运算

        BOPAlgo.BOPAlgo_Algo.SetParallelMode(True)
        bottom_long_rebar_model = (
            self.stair_solid.build_bottom_long_rebar_shape()
        )  # 底部纵筋形状模型
        top_long_rebar_model = self.stair_solid.build_top_long_rebar_shape()  # 顶部纵筋形状模型
        update_model_1 = BRepAlgoAPI_Fuse(
            bottom_long_rebar_model, top_long_rebar_model
        ).Shape()  # 合并顶部纵筋
        mid_distribute_rebar_model = (
            self.stair_solid.build_mid_distribution_rebar_shape()
        )  # 中部分布筋形状模型
        update_model_1 = BRepAlgoAPI_Fuse(
            update_model_1, mid_distribute_rebar_model
        ).Shape()  # 合并中部分布筋
        bottom_edge_long_rebar_model = (
            self.stair_solid.build_bottom_edge_long_rebar_shape()
        )  # 底部边缘纵筋模型
        update_model_1 = BRepAlgoAPI_Fuse(
            update_model_1, bottom_edge_long_rebar_model
        ).Shape()
        top_edge_long_rebar_model = (
            self.stair_solid.build_top_edge_long_rebar_shape()
        )  # 顶部边缘纵筋模型
        update_model_1 = BRepAlgoAPI_Fuse(
            update_model_1, top_edge_long_rebar_model
        ).Shape()
        # update_model_2
        bottom_edge_stir_model = (
            self.stair_solid.build_bottom_edge_stir_shape()
        )  # 底部边缘箍筋模型
        top_edge_stir_model = self.stair_solid.build_top_edge_stir_shape()  # 顶部边缘箍筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            bottom_edge_stir_model, top_edge_stir_model
        ).Shape()
        hole_rein_rebar_model = (
            self.stair_solid.build_hole_rein_rebar_shape()
        )  # 孔洞加强钢筋模型
        update_model_2 = BRepAlgoAPI_Fuse(update_model_2, hole_rein_rebar_model).Shape()
        hoist_rein_long_rebar_model = (
            self.stair_solid.build_hoist_rein_long_rebar_shape()
        )  # 吊装加强纵筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, hoist_rein_long_rebar_model
        ).Shape()
        hoist_rein_point_rebar_model = (
            self.stair_solid.build_hoist_rein_point_rebar_shape()
        )  # 吊装加强点筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, hoist_rein_point_rebar_model
        ).Shape()
        bottom_edge_rein_rebar_model = (
            self.stair_solid.build_bottom_edge_rein_rebar_shape()
        )  # 下部边缘加强筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, bottom_edge_rein_rebar_model
        ).Shape()
        top_edge_rein_rebar_model = (
            self.stair_solid.build_top_edge_rein_rebar_shape()
        )  # 上部边缘加强筋模型
        update_model_2 = BRepAlgoAPI_Fuse(
            update_model_2, top_edge_rein_rebar_model
        ).Shape()
        # 合并两个模型
        update_model = BRepAlgoAPI_Fuse(update_model_1, update_model_2).Shape()
        rail_model = self.stair_solid.build_railing_embedded_shape()
        hoist_model = self.stair_solid.build_hoist_embedded_part_shape()
        demold_model = self.stair_solid.build_demold_embedded_part_shape()
        update_model = BRepAlgoAPI_Fuse(update_model, rail_model).Shape()
        update_model = BRepAlgoAPI_Fuse(update_model, hoist_model).Shape()
        update_model = BRepAlgoAPI_Fuse(update_model, demold_model).Shape()
        return update_model

    def get_hoist_embedded_part_rabbet_model(self):
        """
        获取吊装预埋件企口模型
        :return:
        """
        rabbet_model = self.stair_solid.build_all_hoist_embedded_rabbet()
        return rabbet_model

    def get_embedded_part_model(self):
        """
        建立楼梯预埋件模型
        :return:
        """
        rail_embedded_model = self.stair_solid.build_railing_embedded_shape()  # 栏杆预埋件模型
        hoist_embedded_part_model = (
            self.stair_solid.build_hoist_embedded_part_shape()
        )  # 吊装预埋件模型
        update_model = BRepAlgoAPI_Fuse(
            rail_embedded_model, hoist_embedded_part_model
        ).Shape()
        demold_embedded_part_model = (
            self.stair_solid.build_demold_embedded_part_shape()
        )  # 脱模预埋件模型
        update_model = BRepAlgoAPI_Fuse(
            update_model, demold_embedded_part_model
        ).Shape()

        return update_model

    def get_hoist_embedded_part_model(self):
        """
        建立吊装预埋件模型
        :return:
        """
        hoist_embedded_part_model = (
            self.stair_solid.build_hoist_embedded_part_shape()
        )  # 吊装预埋件模型
        return hoist_embedded_part_model

    def get_rail_embedded_part_model(self):
        """
        建立栏杆预埋件模型
        :return:
        """
        rail_embedded_part_model = (
            self.stair_solid.build_railing_embedded_shape()
        )  # 栏杆预埋件模型
        return rail_embedded_part_model

    def get_demold_embedded_part_model(self):
        """
        建立脱模预埋件模型
        :return:
        """
        demold_embedded_part_model = (
            self.stair_solid.build_demold_embedded_part_shape()
        )  # 脱模预埋件模型
        return demold_embedded_part_model

    def get_stair_and_ear_model(self):
        """
        获取楼梯和挑耳模型数据
        :return:
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        stair_ear = fuse_shape(simple_stair, top_ear, bottom_ear)  # 合并楼梯主体和挑耳模型
        return stair_ear

    def get_stair_and_ear_corner_model(self):
        """
        获取楼梯主体、挑耳、踏步阴角、踏步阳角、防滑槽形成实体模型
        :return:
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        step_slot = self.stair_solid.build_all_step_slot_shape()  # 防滑槽
        internal_corner = self.stair_solid.build_all_internal_corner()  # 楼梯踏步阴角
        external_corner = self.stair_solid.build_all_external_corner()  # 楼梯踏步阳角
        stair_ear = fuse_shape(simple_stair, top_ear, bottom_ear)  # 合并楼梯主体和挑耳模型
        stair_model = my_BRepAlgoAPI_Cut(stair_ear, step_slot).Shape()  # 切掉防滑槽数据模型
        stair_solid = my_BRepAlgoAPI_Fuse(
            stair_model, internal_corner
        ).Shape()  # 合并楼梯阴角模型
        stair_solid = my_BRepAlgoAPI_Cut(
            stair_solid, external_corner
        ).Shape()  # 切掉楼梯阳角模型
        return stair_solid

    def get_stair_and_ear_corner_single_model(self):
        """
        获取楼梯主体、挑耳、踏步阴角、踏步阳角形成实体模型
        :return:
        """
        simple_stair = self.stair_solid.build_stair_solid()  # 混凝土实体
        top_ear = self.stair_solid.build_top_edge_ear()  # 顶部挑耳
        bottom_ear = self.stair_solid.build_bottom_edge_ear()  # 底部挑耳
        internal_corner = self.stair_solid.build_all_internal_corner()  # 楼梯踏步阴角
        external_corner = self.stair_solid.build_all_external_corner()  # 楼梯踏步阳角
        stair_ear = fuse_shape(simple_stair, top_ear, bottom_ear)  # 合并楼梯主体和挑耳模型
        stair_solid = my_BRepAlgoAPI_Fuse(
            stair_ear, internal_corner
        ).Shape()  # 合并楼梯阴角模型
        stair_solid = my_BRepAlgoAPI_Cut(
            stair_solid, external_corner
        ).Shape()  # 切掉楼梯阳角模型
        return stair_solid

    def get_stair_ear_symmetry_model(self):
        """
        获取实体沿着xoy平面镜像的实体模型
        :return:
        """
        solid_model = self.get_stair_and_ear_model()  # 获取楼梯和挑耳模型
        total_x = self.b0
        total_y = self.lb_d + self.lt_d + self.ln
        total_h = self.h2 + self.h
        mirror_center = [total_x / 2, total_y / 2, total_h / 2]
        origin = [0, 0, 0]  # 旋转中心
        rotate_axis = [0, 1, 0]
        angle = -math.pi
        transform_1 = gp_Trsf()
        transform_1.SetRotation(
            gp_Ax1(
                gp_Pnt(origin[0], origin[1], origin[2]),
                gp_Dir(rotate_axis[0], rotate_axis[1], rotate_axis[2]),
            ),
            angle,
        )
        # 镜像
        mirror_axis = [1, 0, 0]
        transform_2 = gp_Trsf()
        transform_2.SetMirror(
            gp_Ax1(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))
        )
        rotate_solid = BRepBuilderAPI_Transform(solid_model, transform_1, False).Shape()
        symmetry_solid = BRepBuilderAPI_Transform(
            rotate_solid, transform_2, False
        ).Shape()
        return solid_model

    def get_stair_all_hole_model(self):
        """
        得到楼梯连接孔洞模型数据
        :return:
        """
        return self.stair_solid.build_all_hole_shape()

    def get_single_hole_model(self):
        """
        获取单个孔洞模型
        :return:
        """
        num = 2
        hole_model = self.stair_solid.build_specific_hole_shape(num)  # 孔洞模型
        return hole_model

    def get_single_hole_rein_rebar_model(self):
        """
        获取单个孔洞加强筋哦行
        :return:
        """
        num = 0
        hole_rein_rebar_model = self.stair_solid.build_specific_hole_rein_rebar_shape(
            num
        )  # 获取单个孔洞加强筋模型
        return hole_rein_rebar_model

    def get_single_step_slot_model(self, num: int):
        """
        获取单个防滑槽模型
        num为指定的防滑槽
        :return:
        """
        step_slot_model = self.stair_solid.build_specific_step_slot_shape(
            num
        )  # 获取单个防滑槽模型
        return step_slot_model

    def get_single_hoist_embedded_part_model(self):
        """
        获取单个吊装预埋件模型
        :return:
        """
        num = 0
        hoist_embedded_part_model = (
            self.stair_solid.build_specific_hoist_embedded_part_shape(num)
        )  # 获取单个吊装预埋件模型
        return hoist_embedded_part_model

    def get_single_hoist_embedded_part_rabbet(self):
        """
        获取单个吊装预埋件企口模型
        :return:
        """
        num = 0
        hoist_embedded_part_model = (
            self.stair_solid.build_specific_hoist_embedded_part_rabbet(num)
        )  # 获取单个吊装预埋件企口模型
        return hoist_embedded_part_model

    def get_single_rail_embedded_part_model(self):
        """
        获取单个栏杆预埋件模型
        :return:
        """
        num = 0
        rail_embedded_part_model = (
            self.stair_solid.build_specific_railing_embedded_shape(num)
        )  # 获取单个栏杆预埋件模型
        return rail_embedded_part_model

    def get_single_basic_demold_embedded_part_model(self):
        """
        获取单个基本脱模预埋件，若为预埋锚栓，其顶部无企口
        :return:
        """
        num = 0
        demold_embedded_part_model = (
            self.stair_solid.build_specific_basic_demold_embedded_part_shape(num)
        )
        return demold_embedded_part_model

    def get_single_demold_embedded_part_model(self):
        """
        获得单个脱模预埋件模型
        :return:
        """
        num = 0
        demold_embedded_part_model = (
            self.stair_solid.build_specific_demold_embedded_part_shape(num)
        )  # 单个脱模预埋件
        return demold_embedded_part_model

    def get_rail_embedded_weld_shape(self):
        """
        获取栏杆预埋件焊板形状
        :return:
        """
        return self.stair_solid.build_rail_embedded_weld_shape()

    def get_rail_embedded_U_rebar_shape(self):
        """
        获取栏杆预埋U型钢筋形状
        :return:
        """
        return self.stair_solid.build_rail_embedded_U_rebar_shape()

    def get_stair_all_step_slot_model(self):
        """
        获取楼梯防滑槽模型数据
        :return:
        """
        return self.stair_solid.build_all_step_slot_shape()

    def get_stair_all_water_drip_model(self):
        """
        获取滴水线槽模型数据
        :return:
        """
        return self.stair_solid.build_all_water_drip_shape()

    def get_bottom_long_rebar_model(self):
        """
        获取底部纵筋模型
        :return:
        """
        return self.stair_solid.build_bottom_long_rebar_shape()

    def get_top_long_rebar_model(self):
        """
        获取顶部纵筋模型
        :return:
        """
        return self.stair_solid.build_top_long_rebar_shape()

    def get_mid_distribute_rebar_model(self):
        """
        获取中部分布筋模型
        :return:
        """
        return self.stair_solid.build_mid_distribution_rebar_shape()

    def get_bottom_edge_long_rebar_model(self):
        """
        获取底部边缘纵筋模型
        :return:
        """
        return self.stair_solid.build_bottom_edge_long_rebar_shape()

    def get_top_edge_long_rebar_model(self):
        """
        获取顶部边缘加强筋模型
        :return:
        """
        return self.stair_solid.build_top_edge_long_rebar_shape()

    def get_bottom_edge_stir_model(self):
        """
        获取底部边缘箍筋模型
        :return:
        """
        return self.stair_solid.build_bottom_edge_stir_shape()

    def get_top_edge_stir_model(self):
        """
        获取顶部边缘箍筋模型
        :return:
        """
        return self.stair_solid.build_top_edge_stir_shape()

    def get_hole_rein_rebar_model(self):
        """
        获取孔洞加强筋模型
        :return:
        """
        return self.stair_solid.build_hole_rein_rebar_shape()

    def get_hoist_rein_long_rebar_model(self):
        """
        获取吊装加强纵筋模型
        :return:
        """
        return self.stair_solid.build_hoist_rein_long_rebar_shape()

    def get_hoist_rein_point_rebar_model(self):
        """
        获取吊点加强点筋模型
        :return:
        """
        return self.stair_solid.build_hoist_rein_point_rebar_shape()

    def get_bottom_edge_rein_rebar_model(self):
        """
        获取底部边缘加强筋模型
        :return:
        """
        return self.stair_solid.build_bottom_edge_rein_rebar_shape()

    def get_top_edge_rein_rebar_model(self):
        """
        获取顶部边缘加强筋模型
        :return:
        """
        return self.stair_solid.build_top_edge_rein_rebar_shape()

    def get_bottom_left_beam_model(self):
        """
        获取底部左侧平台梁模型
        :return:
        """
        return self.stair_solid.build_bottom_left_beam_shape()

    def get_bottom_left_slab_model(self):
        """
        获取底部左侧平台板模型
        :return:
        """
        return self.stair_solid.build_bottom_left_slab_shape()

    def get_top_right_beam_model(self):
        """
        获取顶部右侧平台梁模型
        :return:
        """
        return self.stair_solid.build_top_right_beam_shape()

    def get_top_right_slab_model(self):
        """
        获取顶部右侧平台板模型
        :return:
        """
        return self.stair_solid.build_top_right_slab_shape()

    def get_bottom_left_beam_slab_model(self):
        """
        获取底部左侧平台板和平台梁模型
        :return:
        """
        return self.stair_solid.build_bottom_beam_slab_shape()

    def get_top_right_beam_slab_model(self):
        """
        获取顶部右侧平台板和平台梁模型
        :return:
        """
        return self.stair_solid.build_top_beam_slab_shape()

    def get_top_connect_rebar_nut_shim_model(self):
        """
        获取顶部连接件系列模型
        :return:
        """
        return self.stair_solid.build_top_connect_rebar_nut_shim_shape()

    def get_bottom_connect_rebar_nut_shim_model(self):
        """
        获取底部连接件系列模型
        :return:
        """
        return self.stair_solid.build_bottom_connect_rebar_nut_shim_shape()

    def get_bottom_connect_rebar_model(self):
        """
        获取底部连接件锚固钢筋哦行
        :return:
        """
        return self.stair_solid.build_bottom_connect_rebar_shape()

    def get_top_connect_rebar_model(self):
        """
        获取顶部连接锚固钢筋模型
        :return:
        """
        return self.stair_solid.build_top_connect_rebar_shape()

    def get_bottom_connect_nut_model(self):
        """
        获取底部连接垫片形状
        :return:
        """
        return self.stair_solid.build_bottom_connect_nut_shape()

    def get_top_connect_nut_model(self):
        """
        获取顶部连接垫片形状
        :return:
        """
        return self.stair_solid.build_top_connect_nut_shape()

    def get_bottom_connect_shim_model(self):
        """
        获取底部连接垫片形状
        :return:
        """
        return self.stair_solid.build_bottom_connect_shim_shape()

    def get_top_connect_shim_model(self):
        """
        获取顶部连接垫片形状
        :return:
        """
        return self.stair_solid.build_top_connect_shim_shape()

    def get_total_rail_embedded_rabbet_model(self):
        """
        获取所有栏杆预埋件企口模型
        :return:
        """
        return self.stair_solid.build_total_rail_rabbet_shape()
