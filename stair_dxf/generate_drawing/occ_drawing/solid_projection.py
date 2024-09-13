"""

    实体或者构件的平面投影图

"""
import copy

import numpy as np

from stair_dxf.generate_drawing.occ_drawing.start_draw import OCCData

from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Pln
from OCC.Core.TopoDS import (
    TopoDS_Shape,
    TopoDS_Vertex,
    TopoDS_Wire,
)  # 获取实体的拓扑形状,拓扑顶点，拓扑多线段
from stair_dxf.generate_drawing.occ_drawing.occ_expand_function import (
    compute_project_shape,
    rotation_solid,
    move_solid,
    get_U_rein_rebar_vertex,
)
from stair_dxf.generate_drawing.occ_drawing.occ_solid import (
    BuildMergeAndCutModel,
    BuildStairSolidModel,
)
from typing import List, Optional, Tuple, Dict
from stair_dxf.stair_design.datas import (
    HoleReinRebar,
    RebarData,
    RoundHeadHangingNailInformation,
    EmbeddedAnchorInformation,
    HoistRebar,
    LadderBeamAndSlabInformation,
    NodeInformation,
    ConnectionHoleInformation,
    ConnectElementShapeInfo,
    LadderDoubleJointInformation,
    RebarConfig,
)  # 从深化设计数据库种获取数据
from stair_dxf.stair_design.countrebar import (
    HoleReinforceRebar,
    HoistingEmbeddedPartsLoc,
    HoleLocation,
    BottomLongitudinalRebar,
    TopLongitudinalRebar,
    MidDistributionRebar,
    BottomEdgeLongitudinalRebar,
    TopEdgeLongitudinalRebar,
    BottomEdgeStirrup,
    TopEdgeStirrup,
    HoistingReinforceLongitudinalRebar,
    HoistingReinforcePointRebar,
    TopEdgeReinforceRebar,
    BottomEdgeReinforceRebar,
    StepSlotLoc,
    RailingEmbeddedPart,
)
from OCC.Extend.TopologyUtils import TopologyExplorer, discretize_edge
from stair_dxf.generate_drawing.dxf_drawing_generate.basic_method import (
    get_vector_of_two_points,
)
import math

# 导入自建计算几何库
from stair_dxf.generate_drawing.Geometry.GeomAlgo import (
    pointInPolygon,
    intersectLineLine,
    pointOnRay,
    segmentInPolygonPoint,
    pointInPolygon_xz,
    pointInPolygon_xy,
)
from stair_dxf.generate_drawing.Geometry.GeomBase import Point3D
from stair_dxf.generate_drawing.Geometry.GeomLines import Polyline, Ray, Line, Segment

# 报错日志
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


# 图形精度


class StairTopViewData(object):
    """
    楼梯俯视图数据获取：需要将各个部件的数据分别绘制
    楼梯实体投影，防滑槽投影，空洞投影，预埋件示意图投影，空洞加强筋投影，虚线。
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.drawing_precision = 0.001
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def get_stair_solid_projection(self):
        """
        得到主体投影图
        :return:
        """
        entity_model = self.composite_model.get_stair_and_ear_model()  # 获取楼梯实体模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)
        return points

    def get_step_slot_projection(self):
        """
        获取防滑槽投影数据
        :return:
        """
        entity_model = self.composite_model.get_stair_all_step_slot_model()  # 获取楼梯防滑槽模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)
        return points

    def get_hole_projection(self):
        """
        获取孔洞投影数据
        :return:
        """
        entity_model = self.composite_model.get_stair_all_hole_model()  # 获取楼梯孔洞模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)

        return points

    def get_hoist_embedded_projection(self):
        """
        获取吊装预埋件投影---实体直接投影是一个面，并非轮廓图形，此处绘制的是一个示意图
        :return:
        """
        hoist_embedded_datas = HoistingEmbeddedPartsLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        hoist_loc = hoist_embedded_datas.get_hoist_embedded_part_loc()
        hoist_config = hoist_embedded_datas.get_hoist_embedded_part_info()
        hoist_shape = hoist_config["specification"]
        if hoist_config["type"] == 0:
            top_r = hoist_shape["top_diameter"] / 2
            bottom_r = hoist_shape["bottom_diameter"] / 2
        else:
            top_r = hoist_shape["m_diameter"] / 2  # 顶部直径
            bottom_r = hoist_shape["o_diameter"] / 2  # 底部直径
        hoist_radius = [top_r, bottom_r]  # 吊装件直径集合
        hoist_fact_loc = []
        for num in range(len(hoist_loc)):
            current_point = hoist_loc[num]
            hoist_fact_loc.append([current_point.x, current_point.y, 0])  # 投影后z值为0
        return hoist_fact_loc, hoist_radius

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面切割实体，获得切割后的图形
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_hole_rein_rebar_section(self):
        """
        获取孔洞加强钢筋数据：模型的八段分别操作
        :return:[[(),()],[(),()]]
        """
        # 先筛选顶部四个孔洞加强筋
        rebar_data = RebarData()
        hole_rebar = HoleReinRebar()
        hole_reinforce_model = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        hole_rebar_loc = hole_reinforce_model.get_rebar_model()
        top_loc = hole_rebar_loc[0][0]  # 底部点
        bottom_loc = hole_rebar_loc[-1][0]  # 顶部点
        hole_rein_rebar_model = (
            self.composite_model.get_hole_rein_rebar_model()
        )  # 获取楼梯孔洞加强筋模型
        from OCC.Extend.TopologyUtils import TopologyExplorer, discretize_edge

        rebar_locs = [
            [top_loc.x, top_loc.y, top_loc.z],
            [bottom_loc.x, bottom_loc.y, bottom_loc.z],
        ]
        points = []
        for num in range(len(rebar_locs)):
            current_point = rebar_locs[num]
            plane = gp_Pln(
                gp_Pnt(current_point[0], current_point[1], current_point[2]),
                gp_Dir(0, 0, 1),
            )  # 剖切平面
            compound_ = self.get_solid_cut_drawing(
                plane, hole_rein_rebar_model
            ).Shape()  # TopoDS_compound
            edge = []
            if compound_:
                edge += list(TopologyExplorer(compound_).edges())
            points_ = []
            for edge_ in edge:
                points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
                points_.append(points_3d)
            points.append(points_)
        return points


class StairBottomViewData(object):
    """
    楼梯仰视图数据获取：需要将各个部件的数据分别绘制
    楼梯实体投影，实体投影，孔洞投影，孔洞加强筋投影，虚线。
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.drawing_precision = 0.001
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def get_stair_solid_projection(self):
        """
        得到主体仰视投影图
        :return:
        """
        entity_model = self.composite_model.get_stair_and_ear_model()  # 获取楼梯实体模型
        point_0 = [0, 0, 0]
        normal = [0, 0, -1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)
        # draw_multiple_line(points)
        return points

    def get_hole_projection(self):
        """
        获取孔洞投影数据
        :return:
        """
        entity_model = self.composite_model.get_stair_all_hole_model()  # 获取楼梯孔洞模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)

        return points

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面切割实体，获得切割后的图形
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_hole_rein_rebar_section(self):
        """
        获取孔洞加强钢筋数据：模型的八段分别操作
        :return:[[(),()],[(),()]]
        """
        # 先筛选顶部四个孔洞加强筋
        rebar_data = RebarData()
        hole_rebar = HoleReinRebar()
        hole_reinforce_model = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        hole_rebar_loc = hole_reinforce_model.get_rebar_model()
        top_loc = hole_rebar_loc[0][0]  # 底部点
        bottom_loc = hole_rebar_loc[-1][0]  # 顶部点
        hole_rein_rebar_model = (
            self.composite_model.get_hole_rein_rebar_model()
        )  # 获取楼梯孔洞加强筋模型

        rebar_locs = [
            [top_loc.x, top_loc.y, top_loc.z],
            [bottom_loc.x, bottom_loc.y, bottom_loc.z],
        ]
        points = []
        for num in range(len(rebar_locs)):
            current_point = rebar_locs[num]
            plane = gp_Pln(
                gp_Pnt(current_point[0], current_point[1], current_point[2]),
                gp_Dir(0, 0, 1),
            )  # 剖切平面
            compound_ = self.get_solid_cut_drawing(
                plane, hole_rein_rebar_model
            ).Shape()  # TopoDS_compound
            edge = []
            if compound_:
                edge += list(TopologyExplorer(compound_).edges())
            points_ = []
            for edge_ in edge:
                points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
                points_.append(points_3d)
            points.append(points_)
        return points


class StairLeftViewData(object):
    """
    楼梯左侧视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.drawing_precision = 0.001
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.hole_rein_rebar = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.hoist_loc = HoistingEmbeddedPartsLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def get_stair_entity_left_view(self):
        """
        获取楼梯左侧视图
        :return:
        """
        entity_model = (
            self.composite_model.get_stair_entity_complete_model()
        )  # 获取楼梯实体模型
        base = [0, 0, 0]  # 旋转基点
        rotation_axis = [0, 1, 0]
        angle = math.pi / 2
        rotation_stair_entity = rotation_solid(entity_model, base, rotation_axis, angle)
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(
            rotation_stair_entity, origin, project_dir
        )
        # draw_multiple_line(points)
        return points

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_left_cut_drawing(self):
        """
        获取楼梯左侧面剖切图
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        loc_x = self.b0 / 2
        loc_y = (self.lb_d + self.ln + self.lt_d) / 2
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_left_cut_drawing(self):
        """
        获取孔洞左侧剖切图
        :return:
        """
        hole_model = self.composite_model.get_stair_all_hole_model()  # 获取所有孔洞模型
        hole_loc = self.hole_info.get_hole_loc()
        first_hole = hole_loc[0]
        current_point = [first_hole.x, first_hole.y, first_hole.z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hole_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_rein_rebar_projection(self):
        """
        获取孔洞加强筋侧视图
        :return:
        """
        rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2
        limit_y = (self.ln + self.lb_d + self.lt_d) / 2
        left_loc = []  # 左侧孔洞位置
        for num in range(len(rein_rebar_loc)):
            current_rebar = rein_rebar_loc[num]  # 当前钢筋
            current_point = current_rebar[0]  # 当前点
            if current_point.x < limit_x:
                left_loc.append(current_rebar)
        # 获取每根钢筋的角点
        total_vertex = []
        for num in range(len(left_loc)):
            current_rebar = left_loc[num]
            rebar_loc = get_U_rein_rebar_vertex(current_rebar, diam, limit_y)
            total_vertex.append(rebar_loc)
        return total_vertex

    def get_hoist_embedded_cut_shape(self):
        """
        剖切吊装预埋件
        :return:
        """
        hoist_loc = self.hoist_loc.get_hoist_embedded_part_loc()  # 获取吊装预埋件坐标位置
        limit_x = self.b0 / 2
        left_loc = []
        for num in range(len(hoist_loc)):
            current_point = hoist_loc[num]  # 当前吊装预埋件位置
            if current_point.x < limit_x:
                left_loc.append([current_point.x, current_point.y, current_point.z])
        # 剖切平面
        hoist_model = self.composite_model.get_hoist_embedded_part_model()
        origin = left_loc[0]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hoist_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_hoist_embedded_projection_data(self):
        """
        获取吊装预埋件投影数据
        :return:
        """
        hoist_model = self.composite_model.get_hoist_embedded_part_model()  # 获取吊装预埋件
        point_0 = [0, 0, 0]
        normal = [1, 0, 0]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(hoist_model, origin, project_dir)
        # draw_multiple_line(points)
        return points


class StairRightViewData(object):
    """
    楼梯右侧视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.drawing_precision = 0.001
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )

        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def get_stair_entity_right_view(self):
        """
        获取楼梯右侧视图
        :return:
        """
        entity_model = self.composite_model.get_stair_entity_complete_model()
        base = [0, 0, 0]  # 旋转基点
        rotation_axis = [0, 1, 0]
        angle = -math.pi / 2
        rotation_stair_entity = rotation_solid(entity_model, base, rotation_axis, angle)
        point_0 = [0, 0, 0]  # 投影参考点
        normal = [0, 0, 1]  # 投影方向
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(
            rotation_stair_entity, origin, project_dir
        )
        return points


class StairSectionOneViewData(object):
    """
    楼梯剖切1-1视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_section_view(self):
        """
        楼梯剖面1-1视图--顶部
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        loc_y = self.lb_d + self.ln + self.lt_d - self.cover
        top_loc = []
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]  # 当前吊装预埋件位置
            if current_point.y > limit_y:
                top_loc.append([current_point.x, current_point.y, current_point.z])
        # 剖切平面
        stair_model = self.composite_model.get_stair_and_ear_model()
        origin = top_loc[0]
        origin[1] = loc_y
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, stair_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_view_data(self):
        """
        获取楼梯孔洞数据
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        top_loc = []
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]  # 当前吊装预埋件位置
            if current_point.y > limit_y:
                top_loc.append([current_point.x, current_point.y, current_point.z])
        # 剖切平面
        hole_model = self.composite_model.get_stair_all_hole_model()
        origin = top_loc[0]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hole_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_


class StairSectionTwoViewData(object):
    """
    楼梯剖切2-2视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_section_view(self):
        """
        楼梯剖面2-2视图--底部
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        bottom_loc = []
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]  # 当前吊装预埋件位置
            if current_point.y < limit_y:
                bottom_loc.append([current_point.x, current_point.y, current_point.z])
        # 剖切平面
        stair_model = self.composite_model.get_stair_and_ear_model()
        origin = bottom_loc[0]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, stair_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_view_data(self):
        """
        获取楼梯孔洞数据
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        bottom_loc = []
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]  # 当前吊装预埋件位置
            if current_point.y < limit_y:
                bottom_loc.append([current_point.x, current_point.y, current_point.z])
        # 剖切平面
        hole_model = self.composite_model.get_stair_all_hole_model()
        origin = bottom_loc[0]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hole_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_


class StairReinforceViewData(object):
    """
    楼梯配筋图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_construction_datas()  # 产生所有构造数据
        self.generate_rebar_basic_datas()  # 产生所有钢筋数据
        self.generate_embedded_basic_datas()  # 产生所有埋件数据
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def generate_construction_datas(self):
        """
        产生构造数据信息
        :return:
        """
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

    def generate_rebar_basic_datas(self):
        """
        产生钢筋基础数据
        :return:
        """
        self.hole_rein_rebar = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.bottom_rebar = BottomLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 下部纵筋数据
        self.top_rebar = TopLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部纵筋数
        self.mid_distribute_rebar = MidDistributionRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 中部分布筋数据
        self.top_edge_long_rebar = TopEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘纵筋
        self.bottom_edge_long_rebar = BottomEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 下部边缘纵筋
        self.top_edge_stir = TopEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘箍筋
        self.bottom_edge_stir = BottomEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 下部边缘箍筋
        self.bottom_edge_rein_rebar = BottomEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 下部边缘加强筋
        self.top_edge_rein_rebar = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘加强筋
        self.hoist_rein_long_rebar = HoistingReinforceLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 吊装加强纵筋
        self.hoist_rein_point_rebar = HoistingReinforcePointRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 吊装加强点筋

    def generate_embedded_basic_datas(self):
        """
        产生预埋件基础数据
        :return:
        """
        self.hoist_loc = HoistingEmbeddedPartsLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_profile_cut_drawing(self):
        """
        获取楼梯侧面轮廓剖切图
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        loc_x = self.b0 / 2
        loc_y = (self.lb_d + self.ln + self.lt_d) / 2
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_profile_cut_drawing(self):
        """
        获取孔洞轮廓剖切图
        :return:
        """
        hole_model = self.composite_model.get_stair_all_hole_model()  # 获取所有孔洞模型
        hole_loc = self.hole_info.get_hole_loc()
        first_hole = hole_loc[0]
        current_point = [first_hole.x, first_hole.y, first_hole.z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hole_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_rein_rebar_projection(self):
        """
        获取孔洞加强筋侧视图
        :return:
        """
        rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2
        limit_y = (self.ln + self.lb_d + self.lt_d) / 2
        left_loc = []  # 左侧孔洞位置
        for num in range(len(rein_rebar_loc)):
            current_rebar = rein_rebar_loc[num]  # 当前钢筋
            current_point = current_rebar[0]  # 当前点
            if current_point.x < limit_x:
                left_loc.append(current_rebar)
        # 获取每根钢筋的角点
        total_vertex = []
        for num in range(len(left_loc)):
            current_rebar = left_loc[num]
            rebar_loc = get_U_rein_rebar_vertex(current_rebar, diam, limit_y)
            total_vertex.append(rebar_loc)
        return total_vertex

    def get_hoist_embedded_cut_shape(self):
        """
        剖切吊装预埋件
        :return:
        """
        hoist_loc = self.hoist_loc.get_hoist_embedded_part_loc()  # 获取吊装预埋件坐标位置
        limit_x = self.b0 / 2
        left_loc = []
        for num in range(len(hoist_loc)):
            current_point = hoist_loc[num]  # 当前吊装预埋件位置
            if current_point.x < limit_x:
                left_loc.append([current_point.x, current_point.y, current_point.z])
        # 剖切平面
        hoist_model = self.composite_model.get_hoist_embedded_part_model()
        origin = left_loc[0]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hoist_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_bottom_long_rebar_cut_shape(self):
        """
        获取底部纵筋剖切形状数据
        :return:
        """
        bottom_rebar_loc = self.bottom_rebar.get_rebar_model()  # 获取钢筋数据
        current_point = bottom_rebar_loc[0][0]  # 获取当前点
        # 剖切平面
        bottom_long_rebar_model = self.composite_model.get_bottom_long_rebar_model()
        origin = [current_point.x, current_point.y, current_point.z]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_top_long_rebar_cut_shape(self):
        """
        获取顶部纵筋剖切形状数据
        :return:
        """
        top_rebar_loc = self.top_rebar.get_rebar_model()  # 获取钢筋坐标数据
        current_point = top_rebar_loc[0][0]  # 获取钢筋上一点
        # 剖切平面
        top_long_rebar_model = (
            self.composite_model.get_top_long_rebar_model()
        )  # 获取顶部纵筋模型
        origin = [current_point.x, current_point.y, current_point.z]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_mid_distribute_rebar_cut_shape(self):
        """
        获取中部分布筋剖切形状数据
        :return:
        """
        mid_distribute_rebar_loc = (
            self.mid_distribute_rebar.get_double_rebar_model()
        )  # 获取中部分布筋位置数据
        single_rebar_loc = mid_distribute_rebar_loc[0]
        point_2 = single_rebar_loc[1]  # 第二个点
        point_3 = single_rebar_loc[2]  # 第三个点
        origin = [
            (point_2.x + point_3.x) / 2,
            (point_2.y + point_3.y) / 2,
            (point_2.z + point_3.z) / 2,
        ]
        # 剖切平面
        mid_double_rebar_model = (
            self.composite_model.get_mid_distribute_rebar_model()
        )  # 获取顶部纵筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, mid_double_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_top_edge_long_rebar_cut_shape(self):
        """
        获得顶部边缘纵筋
        :return:
        """
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar.get_rebar_model()
        )  # 获取顶部边缘纵筋坐标
        current_point = top_edge_long_rebar_loc[0]
        point_s = current_point[0]
        point_e = current_point[1]
        origin = [
            (point_s.x + point_e.x) / 2,
            (point_s.y + point_e.y) / 2,
            (point_s.z + point_e.z) / 2,
        ]
        # 剖切平面
        top_edge_long_rebar_model = (
            self.composite_model.get_top_edge_long_rebar_model()
        )  # 获取顶部纵筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_edge_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_bottom_edge_long_rebar_cut_shape(self):
        """
        获得底部边缘纵筋
        :return:
        """
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar.get_rebar_model()
        )  # 获取底部边缘纵筋坐标
        current_point = bottom_edge_long_rebar_loc[0]
        point_s = current_point[0]
        point_e = current_point[1]
        origin = [
            (point_s.x + point_e.x) / 2,
            (point_s.y + point_e.y) / 2,
            (point_s.z + point_e.z) / 2,
        ]
        # 剖切平面
        bottom_edge_long_rebar_model = (
            self.composite_model.get_bottom_edge_long_rebar_model()
        )  # 获取底部纵筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_edge_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_top_edge_stir_cut_shape(self):
        """
        获取顶部边缘箍筋
        :return:
        """
        top_edge_stir_loc = self.top_edge_stir.get_rebar_model()  # 获取顶部边缘箍筋坐标
        current_rebar = top_edge_stir_loc[0]
        point_s = current_rebar[0]
        origin = [point_s.x, point_s.y, point_s.z]
        # 剖切平面顶
        top_edge_stir_model = self.composite_model.get_top_edge_stir_model()  # 获取底部纵筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_edge_stir_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_bottom_edge_stir_cut_shape(self):
        """
        获取底部边缘箍筋
        :return:
        """
        bottom_edge_stir_loc = self.bottom_edge_stir.get_rebar_model()  # 获取底部边缘箍筋坐标
        current_rebar = bottom_edge_stir_loc[0]
        point_s = current_rebar[0]
        origin = [point_s.x, point_s.y, point_s.z]
        # 剖切平面顶
        bottom_edge_stir_model = (
            self.composite_model.get_bottom_edge_stir_model()
        )  # 获取底部边缘箍筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_edge_stir_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_hoist_rein_long_rebar_cut_shape(self):
        """
        获取吊装加强纵向筋
        :return:
        """
        hoist_rein_long_rebar_loc = (
            self.hoist_rein_long_rebar.get_rebar_model()
        )  # 获取吊装加强纵筋
        current_rebar = hoist_rein_long_rebar_loc[0]
        point_s = current_rebar[0]
        origin = [point_s.x, point_s.y, point_s.z]
        # 剖切平面顶
        hoist_rein_long_rebar_model = (
            self.composite_model.get_hoist_rein_long_rebar_model()
        )  # 获取吊装加强纵向钢筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hoist_rein_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_hoist_rein_point_rebar_cut_shape(self):
        """
        获取吊装加强点筋
        :return:
        """
        hoist_rein_point_rebar_loc = (
            self.hoist_rein_point_rebar.get_rebar_model()
        )  # 获取吊装加强点筋
        current_rebar = hoist_rein_point_rebar_loc[0]
        point_s = current_rebar[0]
        point_e = current_rebar[1]
        origin = [
            (point_s.x + point_e.x) / 2,
            (point_s.y + point_e.y) / 2,
            (point_s.z + point_e.z) / 2,
        ]
        # 剖切平面顶
        hoist_rein_point_rebar_model = (
            self.composite_model.get_hoist_rein_point_rebar_model()
        )  # 获取吊装加强点筋模型
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(1, 0, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hoist_rein_point_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_hoist_embedded_projection_data(self):
        """
        获取吊装预埋件投影数据
        :return:
        """
        hoist_model = self.composite_model.get_hoist_embedded_part_model()  # 获取吊装预埋件
        point_0 = [0, 0, 0]
        normal = [1, 0, 0]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(hoist_model, origin, project_dir)
        # draw_multiple_line(points)
        return points


class StairRebarSectionAToAViewData(object):
    """
    楼梯钢筋a-a剖面图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_construction_datas()  # 产生所有构造数据
        self.generate_rebar_basic_datas()  # 产生所有钢筋数据
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def generate_construction_datas(self):
        """
        产生构造数据信息
        :return:
        """
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

    def generate_rebar_basic_datas(self):
        """
        产生钢筋基础数据
        :return:
        """
        self.hole_rein_rebar = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_rebar = TopLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部纵筋数
        self.top_edge_long_rebar = TopEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘纵筋
        self.top_edge_stir = TopEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘箍筋
        self.top_edge_rein_rebar = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘加强筋
        self.bottom_edge_rein_rebar = BottomEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 底端边缘加强筋

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_profile_cut_drawing(self):
        """
        获取楼梯顶端板轮廓剖切图
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_model()
        loc_x = self.b0 / 2
        loc_y = self.lb_d + self.ln + 4 * self.lt_d / 5
        loc_z = self.h + self.h2 - self.h1 / 2
        current_point = [loc_x, loc_y, loc_z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_profile_cut_drawing(self):
        """
        获取孔洞轮廓剖切图
        :return:
        """
        hole_model = self.composite_model.get_stair_all_hole_model()  # 获取所有孔洞模型
        hole_loc = self.hole_info.get_hole_loc()
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2  # 楼梯顶部孔洞和底部孔洞限制y
        cut_point = []
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]  # 当前点
            if current_point.y >= limit_y:
                cut_point.append(current_point)
        first_hole = cut_point[0]
        current_point = [first_hole.x, first_hole.y, first_hole.z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hole_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_rein_rebar_projection(self):
        """
        获取孔洞加强筋剖切图
        :return:List[List[List[float]]]
        """
        rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        rebar_diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2
        limit_y = (self.ln + self.lb_d + self.lt_d) / 2
        top_rebar = []  # 顶部钢筋
        top_rebar_point = []  # 顶部钢筋标志点
        for num in range(len(rein_rebar_loc)):
            current_rebar = rein_rebar_loc[num]  # 当前钢筋
            current_point = current_rebar[2]  # 获取点
            if current_point.y > limit_y:
                top_rebar.append(current_rebar)
                top_rebar_point.append(current_point)
        current_rebar = rein_rebar_loc[0]
        rebar_point_s = current_rebar[0]
        rebar_point_e = current_rebar[-1]
        rebar_length = abs(rebar_point_e.x - rebar_point_s.x) + rebar_diam  # 当前钢筋的长度
        profile_point = []  # 孔洞加强筋轮廓点
        for num in range(len(top_rebar_point)):
            current_point = top_rebar_point[num]
            point_1 = [
                current_point.x - rebar_length / 2,
                current_point.y,
                current_point.z + rebar_diam / 2,
            ]
            point_2 = [point_1[0], point_1[1], point_1[2] - rebar_diam]
            point_3 = [point_2[0] + rebar_length, point_2[1], point_2[2]]
            point_4 = [point_3[0], point_3[1], point_3[2] + rebar_diam]
            profile_point.append([point_1, point_2])
            profile_point.append([point_2, point_3])
            profile_point.append([point_3, point_4])
            profile_point.append([point_4, point_1])
        return profile_point

    def get_stair_top_edge_long_rebar_cut_shape(self):
        """
        获取楼梯顶部边缘纵筋剖切图
        :return:List[List[List[float]]]
        """
        top_edge_long_rebar_model = (
            self.composite_model.get_top_edge_long_rebar_model()
        )  # 获取所有顶部边缘纵筋模型
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar.get_rebar_model()
        )  # 获取顶部边缘纵筋坐标点
        current_rebar_loc = top_edge_long_rebar_loc[0]  # 获取首根钢筋
        point_1 = current_rebar_loc[0]  # 钢筋首点
        point_2 = current_rebar_loc[1]  # 钢筋末点
        current_point = [
            (point_1.x + point_2.x) / 2,
            (point_1.y + point_2.y) / 2,
            (point_1.z + point_2.z) / 2,
        ]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_edge_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_bottom_edge_rein_rebar_cut_shape(self):
        """
        获取底部边缘加强筋剖切图:由于该形状特殊，通过剖切或投影无法实现钢筋的形状生成或者是通过斜面剖切，然后再将斜面旋转到平面上
        要点：底部边缘纵筋只需最大的z坐标
        :return:
        """
        bottom_edge_rein_rebar_loc = (
            self.bottom_edge_rein_rebar.get_rebar_model()
        )  # 获取底部边缘加强筋的坐标点
        bottom_edge_rein_rebar_diam = (
            self.bottom_edge_rein_rebar.get_rebar_diameter()
        )  # 获取底部边缘加强筋
        top_edge_stir_loc = self.top_edge_stir.get_rebar_model()  # 获取上部边缘箍筋的坐标点
        top_edge_stir_diam = self.top_edge_stir.get_rebar_diameter()  # 获取上部边缘箍筋的直径
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar.get_rebar_model()
        )  # 获取上部边缘纵筋的坐标点
        top_edge_long_rebar_diam = (
            self.top_edge_long_rebar.get_rebar_diameter()
        )  # 获取上部边缘纵筋的直径
        loc_y = 0
        min_z = self.h2 + self.h  # 初始化z的坐标值
        min_x = self.b0
        max_x = 0
        # 遍历顶部边缘纵筋，获取最大的z坐标值
        for num in range(len(top_edge_long_rebar_loc)):
            current_rebar = top_edge_long_rebar_loc[num]
            current_point = current_rebar[0]  # 获取首点
            if current_point.z < min_z:
                min_z = current_point.z
        # 遍历底部边缘加强筋，获取最大和最小的x坐标值
        for num in range(len(bottom_edge_rein_rebar_loc)):
            current_rebar = bottom_edge_rein_rebar_loc[num]
            for point in current_rebar:
                if point.x > max_x:
                    max_x = point.x
                if point.x < min_x:
                    min_x = point.x
        loc_z = min_z + (top_edge_long_rebar_diam + bottom_edge_rein_rebar_diam) / 2
        loc_x_l = (
            min_x  # + (top_edge_stir_diam + bottom_edge_rein_rebar_diam) / 2  # 左侧点位置
        )
        loc_x_r = (
            max_x  # - (top_edge_stir_diam + bottom_edge_rein_rebar_diam) / 2  # 右侧点的位置
        )
        result = {}
        result["location"] = [[loc_x_l, loc_y, loc_z], [loc_x_r, loc_y, loc_z]]  # 标注坐标点
        result["diameter"] = bottom_edge_rein_rebar_diam  # 钢筋直径
        return result

    def get_top_edge_rein_rebar_cut_shape(self):
        """
        获取顶部边缘加强筋剖切图:由于该形状特殊，通过剖切或投影无法实现钢筋的形状生成或者是通过斜面剖切，然后再将斜面旋转到平面上
        要点：顶部边缘纵筋只需最大的z坐标
        :return:
        """
        top_edge_rein_rebar_loc = (
            self.top_edge_rein_rebar.get_rebar_model()
        )  # 获取顶部边缘加强筋的坐标点
        top_edge_rein_rebar_diam = (
            self.top_edge_rein_rebar.get_rebar_diameter()
        )  # 获取顶部边缘加强筋
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar.get_rebar_model()
        )  # 获取上部边缘纵筋的坐标点
        top_edge_long_rebar_diam = (
            self.top_edge_long_rebar.get_rebar_diameter()
        )  # 获取上部边缘纵筋的直径
        loc_y = 0
        max_z = 0
        min_x = self.b0
        max_x = 0
        # 遍历顶部边缘纵筋，获取最大的z坐标值
        for num in range(len(top_edge_long_rebar_loc)):
            current_rebar = top_edge_long_rebar_loc[num]
            current_point = current_rebar[0]  # 获取首点
            if current_point.z > max_z:
                max_z = current_point.z
        # 遍历顶部边缘箍筋，获取最大和最小的x坐标值
        for num in range(len(top_edge_rein_rebar_loc)):
            current_rebar = top_edge_rein_rebar_loc[num]
            for point in current_rebar:
                if point.x > max_x:
                    max_x = point.x
                if point.x < min_x:
                    min_x = point.x
        loc_z = max_z - (top_edge_long_rebar_diam + top_edge_rein_rebar_diam) / 2
        loc_x_l = min_x  #  左侧点位置
        loc_x_r = max_x  # 右侧点的位置
        result = {}
        result["location"] = [[loc_x_l, loc_y, loc_z], [loc_x_r, loc_y, loc_z]]  # 标注坐标点
        result["diameter"] = top_edge_rein_rebar_diam  # 钢筋直径
        return result

    def get_top_edge_stir_cut_shape(self):
        """
        获取顶部边缘箍筋剖切形状
        :return:
        """
        top_edge_stir_loc = self.top_edge_stir.get_rebar_model()  # 获取顶部边缘箍筋的坐标点
        top_edge_stir_diam = self.top_edge_stir.get_rebar_diameter()  # 获取顶部边缘箍筋的直径
        min_z = self.h2 + self.h  # 初始化最小z值
        max_z = 0  # 初始化最大z值
        loc_y = 0
        range_x = []  # 顶部边缘箍筋x的位置
        # 获取顶部边缘箍筋的最大和最小z坐标值，以及所有钢筋的x坐标
        for num in range(len(top_edge_stir_loc)):
            current_rebar = top_edge_stir_loc[num]
            current_point = current_rebar[0]  # 当前钢筋的首点
            range_x.append(current_point.x)
            for point in current_rebar:
                if point.z > max_z:
                    max_z = point.z
                if point.z < min_z:
                    min_z = point.z
        line_loc = []
        # 形成直线段
        for num in range(len(range_x)):
            loc_x_l = range_x[num] - top_edge_stir_diam / 2
            loc_x_r = range_x[num] + top_edge_stir_diam / 2
            point_1 = [loc_x_l, loc_y, max_z]
            point_2 = [loc_x_l, loc_y, min_z]
            point_3 = [loc_x_r, loc_y, min_z]
            point_4 = [loc_x_r, loc_y, max_z]
            line_loc.append([point_1, point_2])
            line_loc.append([point_3, point_4])
        arc_loc = []  # 圆心，半径，起始角度，终止角度
        for num in range(len(range_x)):
            loc_x = range_x[num]
            theta_1 = 0
            theta_2 = 180
            theta_3 = 180
            theta_4 = 360
            center_1 = [loc_x, loc_y, max_z]
            center_2 = [loc_x, loc_y, min_z]
            arc_loc.append([center_1, top_edge_stir_diam / 2, theta_1, theta_2])
            arc_loc.append([center_2, top_edge_stir_diam / 2, theta_3, theta_4])
        result = {}
        result["line"] = line_loc  # 直线位置
        result["arc"] = arc_loc  # 圆弧位置
        return result


class StairRebarSectionBToBViewData(object):
    """
    楼梯钢筋b-b剖面图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_rebar_basic_datas()  # 产生所有钢筋数据
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.t = self.slab_struct.geometric.thickness
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

    def generate_rebar_basic_datas(self):
        """
        产生钢筋基础数据
        :return:
        """
        self.bottom_long_rebar = BottomLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 底部纵筋数据
        self.top_long_rebar = TopLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 顶部纵筋数据
        self.mid_distribute_rebar = MidDistributionRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_edge_rein_rebar = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘加强筋
        self.bottom_edge_rein_rebar = BottomEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 底端边缘加强筋

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_view_rotation_angle(self):
        """
        获取视图旋转角度:顺时针方向旋转
        :return:
        """
        cos_theta = self.cos
        angle = math.acos(cos_theta)
        return -angle

    def get_stair_solid_profile_cut_drawing(self):
        """
        获取楼梯中部板轮廓剖切图
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_model()
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = self.b0 / 2
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        xie_scope = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = xie_scope.tolist()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_solid_profile_character_point(self):
        """
        获取楼梯实体轮廓特征点
        :return:
        """
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = 0
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        vector_1 = np.array([-1, 0, 0])
        vector_2 = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = np.cross(vector_1, vector_2)
        special_point = np.array(current_point) + direction * self.t
        return special_point.tolist()

    def get_stair_solid_bottom_long_rebar_profile_cut_drawing(self):
        """
        获取实体底部纵筋轮廓剖切图
        :return:
        """
        bottom_long_rebar_model = (
            self.composite_model.get_bottom_long_rebar_model()
        )  # 底部纵筋模型
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = self.b0 / 2
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        xie_scope = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = xie_scope.tolist()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_long_rebar_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_solid_top_long_rebar_profile_cut_drawing(self):
        """
        获取实体顶部纵筋轮廓剖切图
        :return:
        """
        top_long_rebar_model = self.composite_model.get_top_long_rebar_model()  # 顶部纵筋模型
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = self.b0 / 2
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        xie_scope = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = xie_scope.tolist()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_long_rebar_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_solid_top_edge_rein_rebar_profile_cut_drawing(self):
        """
        获取实体顶部边缘加强筋轮廓剖切图
        :return:
        """
        top_edge_rein_rebar_model = (
            self.composite_model.get_top_edge_rein_rebar_model()
        )  # 顶部边缘加强筋模型
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = self.b0 / 2
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        xie_scope = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = xie_scope.tolist()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_edge_rein_rebar_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_solid_bottom_edge_rein_rebar_profile_cut_drawing(self):
        """
        获取实体底部边缘加强筋轮廓剖切图
        :return:
        """
        bottom_edge_rein_rebar_model = (
            self.composite_model.get_bottom_edge_rein_rebar_model()
        )  # 顶部边缘加强筋模型
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = self.b0 / 2
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        xie_scope = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = xie_scope.tolist()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_edge_rein_rebar_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_solid_mid_distribute_rebar_profile_cut_shape(self):
        """
        获取中部分布筋剖切轮廓图:先获得全局坐标系下的轮廓图
        :return:
        """
        mid_distribute_rebar_model = (
            self.composite_model.get_mid_distribute_rebar_model()
        )  # 获取中部分布筋模型
        mid_distribute_rebar_loc = (
            self.mid_distribute_rebar.get_double_rebar_model()
        )  # 获取中部分布筋坐标点
        current_num = int(len(mid_distribute_rebar_loc) / 2)  # 中部分布筋中间位置
        current_rebar = mid_distribute_rebar_loc[current_num]  # 当前钢筋
        next_rebar = mid_distribute_rebar_loc[current_num + 1]  # 下一根钢筋
        current_point = current_rebar[0]  # 获取当前钢筋的坐标点
        next_point = next_rebar[0]  # 获取下根钢筋的坐标点
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        xie_scope = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = xie_scope.tolist()
        # 第一个切平面
        plane_1 = gp_Pln(
            gp_Pnt(current_point.x, current_point.y, current_point.z),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        # 第二个切平面
        plane_2 = gp_Pln(
            gp_Pnt(next_point.x, next_point.y, next_point.z),
            gp_Dir(direction[0], direction[1], direction[2]),
        )  # 剖切平面
        compound_1 = self.get_solid_cut_drawing(
            plane_1, mid_distribute_rebar_model
        ).Shape()  # TopoDS_compound
        compound_2 = self.get_solid_cut_drawing(
            plane_2, mid_distribute_rebar_model
        ).Shape()  # TopoDS_compound
        # 获取所有钢筋切平面数据
        points_total = []
        points_1 = []
        # 获取第一个切平面坐标点
        edge = []
        if compound_1:
            edge += list(TopologyExplorer(compound_1).edges())
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_1.append(points_3d)
        # 获取第二个切平面坐标点
        points_2 = []
        edge = []
        if compound_2:
            edge += list(TopologyExplorer(compound_2).edges())
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_2.append(points_3d)
        # draw_multiple_line(points_)
        points_total.append(points_1)
        points_total.append(points_2)
        return points_total

    def get_stair_solid_mid_distribute_rebar_feature_point(self):
        """
        获取楼梯实体中部分布筋特征点：将全局坐标系转化为局部坐标系下的基点，用于与B-B配筋剖切图相匹配
        :return:
        """
        mid_distribute_rebar_loc = (
            self.mid_distribute_rebar.get_double_rebar_model()
        )  # 获取中部分布筋坐标点
        current_num = int(len(mid_distribute_rebar_loc) / 2)  # 中部分布筋中间位置
        current_rebar = mid_distribute_rebar_loc[current_num]  # 当前钢筋
        next_rebar = mid_distribute_rebar_loc[current_num + 1]  # 下一根钢筋
        limit_x = self.b0 / 2  # x方向限制
        min_z = self.h2 + self.h  # 获取钢筋最大z坐标值
        current_point = [0, 0, 0]  # 当前点
        # 筛选出符合条件的点
        for num in range(len(current_rebar)):
            point = current_rebar[num]
            if point.x < limit_x and point.z < min_z:
                min_z = point.z
                current_point[1] = point.y
                current_point[2] = point.z
        z_1 = (current_point[1] - self.l1b) * self.tan
        delta_z_1 = current_point[2] - z_1
        length_1 = delta_z_1 * self.cos  # 第一根钢筋距离混凝土实体下边缘沿厚度方向的距离。
        # 下一点的操作
        next_point = [0, 0, 0]  # 下一点
        min_z = self.h2 + self.h
        # 筛选出符合条件的点
        for num in range(len(next_rebar)):
            point = next_rebar[num]
            if point.x < limit_x and point.z < min_z:
                min_z = point.z
                next_point[1] = point.y
                next_point[2] = point.z
        z_2 = (next_point[1] - self.l1b) * self.tan
        delta_z_2 = next_point[2] - z_2
        length_2 = delta_z_2 * self.cos  # 第二根钢筋距离混凝土实体下边缘沿厚度方向的距离。
        # 求解斜向向量
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        vector_1 = np.array([-1, 0, 0])
        vector_2 = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        direction = np.cross(vector_1, vector_2)  # 方向
        current_special_m = (
            np.array([current_point[0], current_point[1], current_point[2]])
            + direction * length_1
        )
        current_special_p = current_special_m.tolist()
        next_special_m = (
            np.array([next_point[0], next_point[1], next_point[2]])
            + direction * length_2
        )
        next_special_p = next_special_m.tolist()
        special_points = [current_special_p, next_special_p]
        return special_points

    def get_stair_solid_mid_distribute_rebar_match_point(self):
        """
        获取楼梯中部分布筋匹配位置点：全局坐标系与局部坐标系之间的变换
        :return: List[List[List[List[float]]]]
        """
        mid_rebar_locs = self.get_stair_solid_mid_distribute_rebar_profile_cut_shape()
        current_rebar_loc = mid_rebar_locs[0]
        next_rebar_loc = mid_rebar_locs[1]
        base_point = self.get_stair_solid_profile_character_point()  # 基点
        special_points = (
            self.get_stair_solid_mid_distribute_rebar_feature_point()
        )  # 变换前的基点
        special_1 = special_points[0]  # 变换前基点1
        special_2 = special_points[1]  # 变换前基点2
        # 转换当前钢筋
        for i in range(len(current_rebar_loc)):
            for j in range(len(current_rebar_loc[i])):
                current_point = list(current_rebar_loc[i][j])  # 当前点
                transform_point = (
                    np.array(current_point) - np.array(special_1) + np.array(base_point)
                )
                current_rebar_loc[i][j] = transform_point.tolist()
        # 转换下一钢筋
        for i in range(len(next_rebar_loc)):
            for j in range(len(next_rebar_loc[i])):
                current_point = list(next_rebar_loc[i][j])  # 当前点
                transform_point = (
                    np.array(current_point) - np.array(special_2) + np.array(base_point)
                )
                next_rebar_loc[i][j] = transform_point.tolist()
        return mid_rebar_locs


class StairRebarSectionCToCViewData(object):
    """
    楼梯钢筋c-c剖面图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_rebar_basic_datas()  # 产生所有钢筋数据
        self.generate_construction_datas()  # 产生所有构造数据
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def generate_construction_datas(self):
        """
        产生构造数据信息
        :return:
        """
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

    def generate_rebar_basic_datas(self):
        """
        产生钢筋基础数据
        :return:
        """
        self.hole_rein_rebar = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.bottom_long_rebar = BottomLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部纵筋数
        self.bottom_edge_long_rebar = BottomEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘纵筋
        self.bottom_edge_stir = BottomEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘箍筋
        self.top_edge_rein_rebar = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘加强筋
        self.bottom_edge_rein_rebar = BottomEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 底端边缘加强筋

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_profile_cut_drawing(self):
        """
        获取楼梯底端板轮廓剖切图
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        loc_x = self.b0 / 2
        loc_y = self.lb_d / 2
        loc_z = self.h2 / 2
        current_point = [loc_x, loc_y, loc_z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_profile_cut_drawing(self):
        """
        获取底部孔洞轮廓剖切图
        :return:
        """
        hole_model = self.composite_model.get_stair_all_hole_model()  # 获取所有孔洞模型
        hole_loc = self.hole_info.get_hole_loc()
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2  # 楼梯顶部孔洞和底部孔洞限制y
        cut_point = []
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]  # 当前点
            if current_point.y <= limit_y:
                cut_point.append(current_point)
        first_hole = cut_point[0]
        current_point = [first_hole.x, first_hole.y, first_hole.z]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, hole_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_stair_hole_rein_rebar_projection(self):
        """
        获取底部孔洞加强筋剖切图
        :return:List[List[List[float]]]
        """
        rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        rebar_diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2
        limit_y = self.lb_d
        bottom_rebar = []  # 底部钢筋
        bottom_rebar_point = []  # 底部钢筋标志点
        # 获取底部加强筋坐标点
        for num in range(len(rein_rebar_loc)):
            current_rebar = rein_rebar_loc[num]  # 当前钢筋
            current_point = current_rebar[2]  # 获取点
            if current_point.y < limit_y:
                bottom_rebar.append(current_rebar)
                bottom_rebar_point.append(current_point)
        current_rebar = rein_rebar_loc[0]
        rebar_point_s = current_rebar[0]
        rebar_point_e = current_rebar[-1]
        rebar_length = abs(rebar_point_e.x - rebar_point_s.x) + rebar_diam  # 当前钢筋的长度
        profile_point = []  # 孔洞加强筋轮廓点
        # 获取底部加强筋轮廓点
        for num in range(len(bottom_rebar_point)):
            current_point = bottom_rebar_point[num]
            point_1 = [
                current_point.x - rebar_length / 2,
                current_point.y,
                current_point.z + rebar_diam / 2,
            ]
            point_2 = [point_1[0], point_1[1], point_1[2] - rebar_diam]
            point_3 = [point_2[0] + rebar_length, point_2[1], point_2[2]]
            point_4 = [point_3[0], point_3[1], point_3[2] + rebar_diam]
            profile_point.append([point_1, point_2])
            profile_point.append([point_2, point_3])
            profile_point.append([point_3, point_4])
            profile_point.append([point_4, point_1])
        return profile_point

    def get_stair_bottom_edge_long_rebar_cut_shape(self):
        """
        获取楼梯底部边缘纵筋剖切图
        :return:List[List[List[float]]]
        """
        bottom_edge_long_rebar_model = (
            self.composite_model.get_bottom_edge_long_rebar_model()
        )  # 获取所有底部边缘纵筋模型
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar.get_rebar_model()
        )  # 获取底部边缘纵筋坐标点
        current_rebar_loc = bottom_edge_long_rebar_loc[0]  # 获取首根钢筋
        point_1 = current_rebar_loc[0]  # 钢筋首点
        point_2 = current_rebar_loc[1]  # 钢筋末点
        current_point = [
            (point_1.x + point_2.x) / 2,
            (point_1.y + point_2.y) / 2,
            (point_1.z + point_2.z) / 2,
        ]
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_edge_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_bottom_long_rebar_cut_shape(self):
        """
        获取底部纵筋剖切图:
        :return:
        """
        bottom_long_rebar_model = (
            self.composite_model.get_bottom_long_rebar_model()
        )  # 获取底部纵筋模型
        bottom_long_rebar_loc = self.bottom_long_rebar.get_rebar_model()  # 获取底部纵筋的坐标点
        limit_z = self.h2
        plane_point = []  # 切平面上的点
        current_rebar_loc = bottom_long_rebar_loc[0]
        for num in range(len(current_rebar_loc)):
            current_point = current_rebar_loc[num]
            if current_point.z < limit_z:
                plane_point.append(current_point)
        point_1 = plane_point[0]
        point_2 = plane_point[1]
        origin = [
            (point_1.x + point_2.x) / 2,
            (point_1.y + point_2.y) / 2,
            (point_1.z + point_2.z) / 2,
        ]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_long_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_bottom_edge_rein_rebar_cut_shape(self):
        """
        获取底部边缘加强筋剖切图:由于该形状特殊，通过剖切或投影无法实现钢筋的形状生成或者是通过斜面剖切，然后再将斜面旋转到平面上
        要点：由于该钢筋在底部有平直段，故直接用平面切割而成
        :return:
        """
        bottom_edge_rein_rebar_model = (
            self.composite_model.get_bottom_edge_rein_rebar_model()
        )  # 获取底部边缘加强筋模型
        bottom_edge_rein_rebar_loc = (
            self.bottom_edge_rein_rebar.get_rebar_model()
        )  # 获取底部边缘加强筋的坐标点
        limit_z = self.h2
        plane_point = []  # 切平面上的点
        current_rebar_loc = bottom_edge_rein_rebar_loc[0]
        for num in range(len(current_rebar_loc)):
            current_point = current_rebar_loc[num]
            if current_point.z < limit_z:
                plane_point.append(current_point)
        point_1 = plane_point[0]
        point_2 = plane_point[1]
        origin = [
            (point_1.x + point_2.x) / 2,
            (point_1.y + point_2.y) / 2,
            (point_1.z + point_2.z) / 2,
        ]
        plane = gp_Pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(0, 1, 0))  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_edge_rein_rebar_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        # draw_multiple_line(points_)
        return points_

    def get_top_edge_rein_rebar_cut_shape(self):
        """
        获取顶部边缘加强筋剖切图:由于该形状特殊，通过剖切或投影无法实现钢筋的形状生成或者是通过斜面剖切，然后再将斜面旋转到平面上
        要点：顶部边缘纵筋只需最大的z坐标，顶部边缘加强筋只需最大和最小x坐标
        :return:
        """
        top_edge_rein_rebar_loc = (
            self.top_edge_rein_rebar.get_rebar_model()
        )  # 获取顶部边缘加强筋的坐标点
        top_edge_rein_rebar_diam = (
            self.top_edge_rein_rebar.get_rebar_diameter()
        )  # 获取顶部边缘加强筋
        bottom_edge_stir_loc = self.bottom_edge_stir.get_rebar_model()  # 获取上部边缘箍筋的坐标点
        bottom_edge_stir_diam = (
            self.bottom_edge_stir.get_rebar_diameter()
        )  # 获取上部边缘箍筋的直径
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar.get_rebar_model()
        )  # 获取下部边缘纵筋的坐标点
        bottom_edge_long_rebar_diam = (
            self.bottom_edge_long_rebar.get_rebar_diameter()
        )  # 获取下部边缘纵筋的直径
        loc_y = 0
        max_z = 0
        min_x = self.b0 + self.b2
        max_x = 0
        # 遍历底部边缘纵筋，获取最大的z坐标值
        for num in range(len(bottom_edge_long_rebar_loc)):
            current_rebar = bottom_edge_long_rebar_loc[num]
            current_point = current_rebar[0]  # 获取首点
            if current_point.z > max_z:
                max_z = current_point.z
        # 遍历底部边缘箍筋，获取最大和最小的x坐标值
        for num in range(len(top_edge_rein_rebar_loc)):
            current_rebar = top_edge_rein_rebar_loc[num]
            for point in current_rebar:
                if point.x > max_x:
                    max_x = point.x
                if point.x < min_x:
                    min_x = point.x
        loc_z = max_z - (bottom_edge_long_rebar_diam + top_edge_rein_rebar_diam) / 2
        loc_x_l = (
            min_x  # + (bottom_edge_stir_diam + top_edge_rein_rebar_diam) / 2  # 左侧点位置
        )
        loc_x_r = (
            max_x  # - (bottom_edge_stir_diam + top_edge_rein_rebar_diam) / 2  # 右侧点的位置
        )
        result = {}
        result["location"] = [[loc_x_l, loc_y, loc_z], [loc_x_r, loc_y, loc_z]]  # 标注坐标点
        result["diameter"] = top_edge_rein_rebar_diam  # 钢筋直径
        return result

    def get_bottom_edge_stir_cut_shape(self):
        """
        获取底部边缘箍筋剖切形状
        :return:
        """
        bottom_edge_stir_loc = self.bottom_edge_stir.get_rebar_model()  # 获取底部边缘箍筋的坐标点
        bottom_edge_stir_diam = (
            self.bottom_edge_stir.get_rebar_diameter()
        )  # 获取底部边缘箍筋的直径
        min_z = self.h2  # 初始化最小z值
        max_z = 0  # 初始化最大z值
        loc_y = 0
        range_x = []  # 底部边缘箍筋x的位置
        # 获取底部边缘箍筋的最大和最小z坐标值，以及所有钢筋的x坐标
        for num in range(len(bottom_edge_stir_loc)):
            current_rebar = bottom_edge_stir_loc[num]
            current_point = current_rebar[0]  # 当前钢筋的首点
            range_x.append(current_point.x)
            for point in current_rebar:
                if point.z > max_z:
                    max_z = point.z
                if point.z < min_z:
                    min_z = point.z
        line_loc = []
        # 形成直线段
        for num in range(len(range_x)):
            loc_x_l = range_x[num] - bottom_edge_stir_diam / 2
            loc_x_r = range_x[num] + bottom_edge_stir_diam / 2
            point_1 = [loc_x_l, loc_y, max_z]
            point_2 = [loc_x_l, loc_y, min_z]
            point_3 = [loc_x_r, loc_y, min_z]
            point_4 = [loc_x_r, loc_y, max_z]
            line_loc.append([point_1, point_2])
            line_loc.append([point_3, point_4])
        arc_loc = []  # 圆心，半径，起始角度，终止角度
        # 形成圆弧段
        for num in range(len(range_x)):
            loc_x = range_x[num]
            theta_1 = 0
            theta_2 = 180
            theta_3 = 180
            theta_4 = 360
            center_1 = [loc_x, loc_y, max_z]
            center_2 = [loc_x, loc_y, min_z]
            arc_loc.append([center_1, bottom_edge_stir_diam / 2, theta_1, theta_2])
            arc_loc.append([center_2, bottom_edge_stir_diam / 2, theta_3, theta_4])
        result = {}
        result["line"] = line_loc  # 直线位置
        result["arc"] = arc_loc  # 圆弧位置
        return result


class StairProjectionDrawingData(object):
    """
    建立楼梯投影图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def get_stair_entity_top_view(self):
        """
        获取楼梯实体俯视图数据
        :return:
        """
        entity_model = (
            self.composite_model.get_stair_entity_complete_model()
        )  # 获取楼梯实体模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)
        return points

    def get_stair_entity_left_view(self):
        """
        获取楼梯左侧视图
        :return:
        """
        entity_model = (
            self.composite_model.get_stair_entity_complete_model()
        )  # 获取楼梯实体模型
        base = [0, 0, 0]  # 旋转基点
        rotation_axis = [0, 1, 0]
        angle = math.pi / 2
        rotation_stair_entity = rotation_solid(entity_model, base, rotation_axis, angle)
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(
            rotation_stair_entity, origin, project_dir
        )
        return rotation_stair_entity

    def get_stair_entity_right_view(self):
        """
        获取楼梯右侧视图
        :return:
        """
        entity_model = self.composite_model.get_stair_entity_complete_model()
        base = [0, 0, 0]  # 旋转基点
        rotation_axis = [0, 1, 0]
        angle = -math.pi / 2
        rotation_stair_entity = rotation_solid(entity_model, base, rotation_axis, angle)
        point_0 = [0, 0, 0]  # 投影参考点
        normal = [0, 0, 1]  # 投影方向
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(
            rotation_stair_entity, origin, project_dir
        )

    def get_stair_entity_bottom_view(self):
        """
        获取楼梯俯视图
        :return:
        """
        entity_model = (
            self.composite_model.get_stair_entity_complete_model()
        )  # 获取楼梯实体模型
        base = [0, 0, 0]  # 旋转基点
        rotation_axis = [0, 1, 0]
        angle = math.pi
        rotation_stair_entity = rotation_solid(entity_model, base, rotation_axis, angle)
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(
            rotation_stair_entity, origin, project_dir
        )
        return rotation_stair_entity

    def get_hole_rein_rebar_top_view(self):
        """
        获取孔洞加强筋的俯视图
        :return:
        """
        hole_rebar_model = (
            self.composite_model.get_hole_rein_rebar_model()
        )  # 获取孔洞加强钢筋模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(hole_rebar_model, origin, project_dir)
        return hole_rebar_model

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面切割实体，获得切割后的图形
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_bottom_long_rebar_profile_drawing(self):
        """
        获取底部纵向钢筋的轮廓图
        :return:
        """
        plane = gp_Pln(gp_Pnt(0, 100, 0), gp_Dir(0, 1, 0))  # 剖切平面
        bottom_long_rebar_model = self.composite_model.get_bottom_long_rebar_model()
        wire_model = self.get_solid_cut_drawing(plane, bottom_long_rebar_model).Shape()
        return wire_model

    # def get_bottom_long_rebar_axis_profile_drawing(self):
    #     """
    #     获取底部纵筋轴向轮廓图
    #     :return:
    #     """
    #     rebar_locs = [[40,20,25],[214.16666666666666,20,25],[388.3333333333333,20,25],[562.5,20,25],
    #                   [736.6666666666666,20,25],[910.8333333333333,20,25],[1085,20,25]]
    #     from OCC.Core.TopoDS import TopoDS_Compound,TopoDS_Builder
    #     compound_shape = TopoDS_Compound()  # 构造一个复合体
    #     aBuilder = TopoDS_Builder()  # 建立一个TopoDS_Builder()
    #     aBuilder.MakeCompound(compound_shape)
    #     bottom_long_rebar_model = self.composite_model.get_bottom_long_rebar_model()
    #     for num in range(len(rebar_locs)):
    #         point_loc = copy.deepcopy(rebar_locs[num])
    #         plane = gp_Pln(gp_Pnt(point_loc[0], point_loc[1], point_loc[2]), gp_Dir(1, 0, 0))  # 剖切平面
    #         compound_ = self.get_solid_cut_drawing(plane, bottom_long_rebar_model).Shape()  # TopoDS_compound
    #         aBuilder.Add(compound_shape,compound_)
    #     return compound_shape

    # def get_single_bottom_long_rebar_axis_profile_drawing(self):
    #     """
    #     获取底部纵筋轴向轮廓坐标点
    #     :return:
    #     """
    #     from OCC.Core.TopExp import TopExp_Explorer  # 遍历工具类
    #     from OCC.Core.TopAbs import TopAbs_COMPOUND,TopAbs_WIRE,TopAbs_EDGE,TopAbs_FACE,TopAbs_SHELL,TopAbs_SOLID
    #     from OCC.Core.TopoDS import topods_Compound,topods_Edge,topods_Wire,topods_Face,topods_Shell,topods_Vertex,topods_Solid,\
    #         TopoDS_Compound,TopoDS_Solid  # 边界表示法核心类
    #     from OCC.Extend.TopologyUtils import TopologyExplorer,discretize_edge
    #     from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeWire,BRepBuilderAPI_MakeEdge
    #     rebar_locs = [[40,20,25],[214.16666666666666,20,25],[388.3333333333333,20,25],[562.5,20,25],
    #                   [736.6666666666666,20,25],[910.8333333333333,20,25],[1085,20,25]]
    #     point_loc = copy.deepcopy(rebar_locs[0])
    #     plane = gp_Pln(gp_Pnt(point_loc[0], point_loc[1], point_loc[2]), gp_Dir(1, 0, 0))  # 剖切平面
    #     bottom_long_rebar_model = self.composite_model.get_bottom_long_rebar_model()
    #     compound_ = self.get_solid_cut_drawing(plane, bottom_long_rebar_model).Shape()  # TopoDS_compound
    #     edge = []
    #     if compound_:
    #         edge += list(TopologyExplorer(compound_).edges())
    #     make_wire = BRepBuilderAPI_MakeWire()
    #     points = []
    #     for edge_ in edge:
    #         points_3d = discretize_edge(edge_, 0.000001)  # 转换成点
    #         points.append(points_3d)
    #         for i in range(len(points_3d) - 1):
    #             point_start = points_3d[i]
    #             start_pnt = gp_Pnt(*point_start)
    #             point_end = points_3d[i + 1]
    #             end_pnt = gp_Pnt(*point_end)
    #             make_edg_api = BRepBuilderAPI_MakeEdge(start_pnt, end_pnt)
    #             edg = make_edg_api.Edge()
    #             make_wire.Add(edg)
    #     return make_wire.Shape(),points


class StairStepSlotLeftViewData(object):
    """
    楼梯防滑槽左视图绘制数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        loc_x = self.b0 / 2
        loc_y = (self.lb_d + self.ln + self.lt_d) / 2
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_stair_solid_left_transverse_cut_plane_point(self):
        """
        获取楼梯左侧横向剖切平面点
        :return:
        """
        loc_x = self.b0 / 2
        loc_y = self.lb_d + self.tabu_b * (2 + 3 / 4)
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_stair_solid_right_transverse_cut_plane_point(self):
        """
        获取楼梯右侧横向剖切平面点
        :return:
        """
        loc_x = self.b0 / 2
        loc_y = self.lb_d + self.tabu_b * 3 / 4
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_stair_solid_long_cut_drawing(self):
        """
        获取楼梯纵向剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        points = []
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        points.append(points_)
        # draw_multiple_line(points_)
        return points_

    def get_stair_solid_left_transverse_cut_drawing(self):
        """
        获取楼梯实体左侧横向剖切数据：形成左侧剖切轮廓
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        current_point = self.get_stair_solid_left_transverse_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_left_transverse_boundary_data(self):
        """
        获取楼梯实体左侧横向边缘数据：形成左侧横向边界轮廓数据
        :return:[[point_1,point_2]]
        """
        left_bound_data = self.get_stair_solid_left_transverse_cut_drawing()
        key_point = self.get_stair_solid_left_transverse_cut_plane_point()
        loc_x = key_point[0]
        profile_loc = []  # 轮廓点
        for segment in left_bound_data:
            point_s = segment[0]  # 起点
            point_e = segment[1]  # 终点
            direction = get_vector_of_two_points(point_s, point_e)  # 获取两点形成的向量
            if direction[0] == 0 and direction[1] == 0 and direction[2] != 0:
                point_s = list(point_s)
                point_e = list(point_e)
                point_s[0] = loc_x  # 改变x坐标
                point_e[0] = loc_x  # 改变x坐标
                profile_loc.append([tuple(point_s), tuple(point_e)])
        final_edge = [profile_loc[0]]
        return final_edge

    def get_stair_solid_right_transverse_cut_drawing(self):
        """
        获取楼梯实体右侧横向剖切数据:形成右侧剖切轮廓
        :return:
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        current_point = self.get_stair_solid_right_transverse_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_right_transverse_boundary_data(self):
        """
        获取楼梯实体右侧横向边缘数据：形成右侧剖切轮廓边界数据
        :return:[[point_1,point_2]]
        """
        right_bound_data = self.get_stair_solid_right_transverse_cut_drawing()
        key_point = self.get_stair_solid_left_transverse_cut_plane_point()
        loc_x = key_point[0]  # x坐标
        profile_loc = []  # 轮廓点
        for segment in right_bound_data:
            point_s = segment[0]  # 起点
            point_e = segment[1]  # 终点
            direction = get_vector_of_two_points(point_s, point_e)  # 获取两点形成的向量
            if direction[0] == 0 and direction[1] == 0 and direction[2] != 0:
                point_s = list(point_s)
                point_e = list(point_e)
                point_s[0] = loc_x  # 改变x坐标
                point_e[0] = loc_x  # 改变x坐标
                profile_loc.append([tuple(point_s), tuple(point_e)])
        final_edge = [profile_loc[0]]
        return final_edge

    def get_step_slot_left_view_bounding_box(self):
        """
        得到防滑槽左侧图的包围框图形：获取纵向轮廓线指定包围框内部的图形数据
        :return:
        """
        left_boundary = self.get_stair_solid_left_transverse_boundary_data()  # 获取左侧边界点
        right_boundary = (
            self.get_stair_solid_right_transverse_boundary_data()
        )  # 获取右侧边界点
        left_top_p = []  # 左侧顶部点
        left_bottom_p = []  # 左侧底部点
        right_bottom_p = []  # 右侧底部点
        right_top_p = []  # 右侧顶部点
        left_seg = left_boundary[0]
        right_seg = right_boundary[0]
        max_z = (
            max(left_seg[0][2], left_seg[1][2], right_seg[0][2], right_seg[1][2]) + 10
        )  # 最大的z坐标值
        min_z = (
            min(left_seg[0][2], left_seg[1][2], right_seg[0][2], right_seg[1][2]) - 10
        )  # 最小的z坐标值
        # max_z = self.detail_slab.h2+self.slab_struct.h
        # min_z = 0
        # 遍历左侧边界点
        for segment in left_boundary:
            point_1 = list(copy.deepcopy(segment[0]))
            point_2 = list(copy.deepcopy(segment[1]))
            if point_1[2] <= point_2[2]:
                left_top_p = [point_2[0], point_2[1], point_2[2]]
                left_bottom_p = [point_1[0], point_1[1], point_1[2]]
            else:
                left_top_p = [point_1[0], point_1[1], point_1[2]]
                left_bottom_p = [point_2[0], point_2[1], point_2[2]]
        # 遍历右侧边界点
        for segment in right_boundary:
            point_1 = list(copy.deepcopy(segment[0]))
            point_2 = list(copy.deepcopy(segment[1]))
            if point_1[2] <= point_2[2]:
                right_top_p = [point_2[0], point_2[1], point_2[2]]
                right_bottom_p = [point_1[0], point_1[1], point_1[2]]
            else:
                right_top_p = [point_1[0], point_1[1], point_1[2]]
                right_bottom_p = [point_2[0], point_2[1], point_2[2]]
        left_top_p[2] = max_z
        right_top_p[2] = max_z
        left_bottom_p[2] = min_z
        right_bottom_p[2] = min_z
        return [
            tuple(left_top_p),
            tuple(left_bottom_p),
            tuple(right_bottom_p),
            tuple(right_top_p),
            tuple(left_top_p),
        ]

    def choose_inner_point_of_bounding_box(self):
        """
        选择包围框内部的点集合
        """
        bounding_box_loc = self.get_step_slot_left_view_bounding_box()
        cut_profile_point = self.get_stair_solid_long_cut_drawing()
        # print("初始点的个数：",len(cut_profile_point))
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    def get_stair_step_slot_detail_view_edge_value(self):
        """
        获取楼梯防滑槽部分详图y,z坐标的最大值和最小值
        :return:[min_y,max_y,min_z,max_z]
        """
        max_y = 0
        min_y = self.lb_d + self.ln + self.lt_d
        max_z = 0
        min_z = self.h2 + self.h
        profile_points = self.choose_inner_point_of_bounding_box()
        for segment in profile_points:
            for num in range(len(segment)):
                current_p = segment[num]
                if current_p[1] < min_y:
                    min_y = current_p[1]
                if current_p[1] > max_y:
                    max_y = current_p[1]
                if current_p[2] < min_z:
                    min_z = current_p[2]
                if current_p[2] > max_z:
                    max_z = current_p[2]
        edge_value = [min_y, max_y, min_z, max_z]
        return edge_value

    def get_stair_step_slot_top_arc_dimension_point_data(self):
        """
        得到楼梯防滑槽顶部圆弧标注点数据
        :return:radius,center_loc----圆的半径和圆心坐标点
        """
        boundary_loc = self.get_stair_step_slot_detail_view_edge_value()
        limit_y = (boundary_loc[0] + boundary_loc[1]) / 2  # y的边界限值
        profile_points = self.choose_inner_point_of_bounding_box()
        radius = 0  # 圆弧半径初始化
        center_loc = [0, 0, 0]
        for segment in profile_points:
            current_point = segment[0]
            num = len(segment)
            if num > 3 and current_point[1] > limit_y:  # 线段的点数量以及线段上点的位置大于y的中间值
                center = [0, 0, 0]
                for i in range(num):
                    point_loc = segment[i]  # 当前点的位置
                    center[0] += point_loc[0]
                    center[1] += point_loc[1]
                    center[2] += point_loc[2]
                center_loc[0] = center[0] / num
                center_loc[1] = center[1] / num
                center_loc[2] = center[2] / num
                radius = round(
                    np.linalg.norm(np.array(center_loc) - np.array(list(current_point)))
                )
        return radius, center_loc

    def get_stair_step_slot_top_dimension_point_data(self):
        """
        获取楼梯踏步顶部标注点数据
        :return:
        """
        edge_value = self.get_stair_step_slot_detail_view_edge_value()
        profile_points = self.choose_inner_point_of_bounding_box()
        limit_y = (edge_value[0] + edge_value[1]) / 2
        left_boundary_info = self.get_stair_solid_left_transverse_boundary_data()
        left_loc = left_boundary_info[0]
        limit_z = (left_loc[0][2] + left_loc[1][2]) / 2  # z的坐标限值
        # 特殊线段---顶部竖向线段
        top_segment = []
        for segment in profile_points:
            point_0 = segment[0]
            point_1 = segment[1]
            if (
                point_0[1] > limit_y
                and abs(point_1[1] - point_0[1]) == 0
                and abs(point_1[2] - point_0[2]) != 0
            ):
                top_segment.append(segment[0])
                top_segment.append(segment[1])
        special_locs = []
        # 遍历筛选符合要求的点集
        for segment in profile_points:
            point_0 = segment[0]  # 线段上的一点
            if point_0[1] >= limit_y and point_0[2] > limit_z:
                point_1 = segment[0]
                point_2 = segment[1]
                if point_1[1] < point_2[1]:
                    special_locs.append(segment)
                else:
                    special_locs.append([point_2, point_1])
        # 对坐标点集合排序:线段内部点y值是由小到大排列，线段外部y值是由大到小排列。----冒泡法排序
        for m in range(len(special_locs) - 1):
            for n in range(m + 1, len(special_locs)):
                segment_1 = special_locs[m]  # 线段上的首点
                segment_2 = special_locs[n]  # 线段上的首点
                if segment_1[1][1] < segment_2[1][1]:  # 若不满足条件，则进行交换变化
                    curr_p = segment_1
                    segment_1 = segment_2
                    segment_2 = curr_p
                    special_locs[m] = segment_1
                    special_locs[n] = segment_2
        # print("筛选出的点集为：",special_locs)
        # 左侧标注点
        segmet_1 = [list(special_locs[1][0]), list(special_locs[1][1])]
        point_l_1 = copy.deepcopy(segmet_1[1])
        point_l_2 = copy.deepcopy(segmet_1[1])
        point_l_2[2] = segmet_1[0][2]
        # 底部标注点
        segmet_2 = [list(special_locs[2][0]), list(special_locs[2][1])]
        point_b_2 = copy.deepcopy(segmet_1[0])
        point_b_1 = copy.deepcopy(segmet_1[0])
        point_b_1[1] = segmet_1[1][1]
        point_b_3 = copy.deepcopy(segmet_1[0])
        point_b_3[1] = segmet_2[0][1]
        # 顶部标注点
        loc_y_1 = top_segment[0][1]  # 获取y坐标值
        segmet_3 = [list(special_locs[4][0]), list(special_locs[4][1])]
        loc_y_2 = segmet_3[0][1]
        loc_z = special_locs[0][0][2]
        point_t_1 = copy.deepcopy(point_b_2)
        point_t_1[1] = loc_y_1
        point_t_1[2] = loc_z
        point_t_2 = copy.deepcopy(point_t_1)
        point_t_2[1] = loc_y_2
        point_t_3 = copy.deepcopy(point_b_2)
        point_t_3[2] = loc_z
        # 待标注点的位置
        dimension_loc = [
            [point_l_1, point_l_2],
            [point_b_1, point_b_2],
            [point_b_2, point_b_3],
            [point_t_1, point_t_2],
            [point_t_2, point_t_3],
        ]
        # print("待标注点的位置为：",dimension_loc)
        return dimension_loc

    def get_stair_step_slot_dimension_point_data(self):
        """
        得到防滑槽标注点数据
        :return:
        """
        edge_value = self.get_stair_step_slot_detail_view_edge_value()
        profile_points = self.choose_inner_point_of_bounding_box()
        limit_y = (edge_value[0] + edge_value[1]) / 2
        # 寻找待删除的列表
        delete_point = [0, 0, 0]
        for seg in profile_points:
            if len(seg) == 2 and (
                seg[0][2] == edge_value[2] or seg[1][2] == edge_value[2]
            ):
                point_1 = list(seg[0])
                point_2 = list(seg[1])
                if point_1[1] > point_2[1]:
                    delete_point[0] = point_1[0]
                    delete_point[1] = point_1[1]
                    delete_point[2] = point_1[2]
                else:
                    delete_point[0] = point_2[0]
                    delete_point[1] = point_2[1]
                    delete_point[2] = point_2[2]
        limit_z = delete_point[2]
        # print("待删除的点为：",delete_point)
        arc_seg = []
        for seg in profile_points:
            if len(seg) > 2 and seg[0][1] > limit_y:
                arc_seg.append(seg)
        # if len[arc_seg] == 0:
        #     logger.debug("未找到该段圆弧，无法进行后续计算！")
        # 待标注点位置限值
        max_arc_y = 0
        for point in arc_seg[0]:
            curr_p = list(point)
            if max_arc_y < curr_p[1]:
                max_arc_y = curr_p[1]
        # 待标注点
        dimension_point = []
        for seg in profile_points:
            for num in range(len(seg)):
                current_p = seg[num]
                value = np.linalg.norm(
                    np.array(delete_point) - np.array(list(current_p))
                )
                if current_p[1] > limit_y and current_p[2] > limit_z and len(seg) <= 2:
                    dimension_point.append(current_p)
        # print("待标注点为:",dimension_point)
        # 去除重复点
        index_sign = [0 for _ in range(len(dimension_point))]
        for m in range(len(dimension_point)):
            for n in range(len(dimension_point)):
                if m != n:
                    p_1 = list(dimension_point[m])
                    p_2 = list(dimension_point[n])
                    value = np.linalg.norm(np.array(p_1) - np.array(p_2))
                    if value <= 0.05:  # 由于小数保留精度的原因
                        index_sign[m] += 1
        new_dimension_point = []
        for num in range(len(index_sign)):
            key = index_sign[num]
            if key == 0:
                new_dimension_point.append(dimension_point[num])
        # print("去掉重复元素后的标注点为：",new_dimension_point)
        # 冒泡法排序
        for m in range(len(new_dimension_point) - 1):
            for n in range(1, len(new_dimension_point)):
                p_1 = list(new_dimension_point[m])
                p_2 = list(new_dimension_point[n])
                if p_1[1] < p_2[1]:
                    p_0 = copy.deepcopy(p_1)
                    p_1 = p_2
                    p_2 = p_0
                new_dimension_point[m] = tuple(p_1)
                new_dimension_point[n] = tuple(p_2)
        # print("排序后的点:",new_dimension_point)  # TODO点数对应不上
        return new_dimension_point


class StairStepSlotTopViewData(object):
    """
    楼梯防滑槽俯视图绘制数据:图形数据和标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.drawing_precision = 0.001  # 判断精度
        self.step_slot_config = StepSlotLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 防滑槽配置信息
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        loc_x = self.b0 / 2
        loc_y = (self.lb_d + self.ln + self.lt_d) / 2
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_stair_solid_projection(self):
        """
        得到主体投影图
        :return:
        """
        entity_model = self.composite_model.get_stair_and_ear_model()  # 获取楼梯实体模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)
        return points

    def get_step_slot_projection(self):
        """
        获取防滑槽投影数据
        :return:
        """
        entity_model = self.composite_model.get_single_step_slot_model(
            1
        )  # 获取楼梯防滑槽模型，第二个防滑槽模型
        point_0 = [0, 0, 0]
        normal = [0, 0, 1]
        origin = gp_Pnt(point_0[0], point_0[1], point_0[2])  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(entity_model, origin, project_dir)
        # draw_multiple_line(points)
        return points

    def get_stair_solid_left_transverse_cut_plane_point(self):
        """
        获取楼梯实体左侧横向剖切平面点
        """
        loc_x = self.b0 / 2
        loc_y = self.lb_d + self.tabu_b * 2
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_stair_solid_right_transverse_cut_plane_point(self):
        """
        获取楼梯实体右侧横向剖切平面点
        """
        loc_x = self.b0 / 2
        loc_y = self.lb_d + self.tabu_b * 1
        loc_z = (self.h + self.h2) / 2
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_step_slot_top_view_bounding_box(self):
        """
        防滑槽俯视图的包围框图形：获取楼梯俯视图和防滑槽俯视图的图形数据
        """
        left_key_point = self.get_stair_solid_left_transverse_cut_plane_point()
        right_key_point = self.get_stair_solid_right_transverse_cut_plane_point()
        # 轮廓边界点
        # 左侧边界点
        left_top_point = copy.deepcopy(left_key_point)
        left_top_point[2] = 0
        left_top_point[1] += 1
        left_bottom_point = copy.deepcopy(left_key_point)
        left_bottom_point[2] = 0
        left_bottom_point[1] += 1
        left_bottom_point[0] = -1
        # 右侧边界点
        right_top_point = copy.deepcopy(right_key_point)
        right_top_point[2] = 0
        right_top_point[1] += -1
        right_bottom_point = copy.deepcopy(right_key_point)
        right_bottom_point[2] = 0
        right_bottom_point[1] += -1
        right_bottom_point[0] = -1
        return [
            tuple(left_top_point),
            tuple(left_bottom_point),
            tuple(right_bottom_point),
            tuple(right_top_point),
            tuple(left_top_point),
        ]

    def choose_stair_projection_inner_point_of_bounding_box(self):
        """
        在楼梯实体俯视投影图和防滑槽俯视图中选择包围框内部数据进行显示。
        :return:[[point_1,point_2],[point_3,point_4]]
        """
        bounding_box_loc = self.get_step_slot_top_view_bounding_box()  # 获取包围框内点集合
        stair_project_point = self.get_stair_solid_projection()  # 获取楼梯投影轮廓数据点集合
        # 形成计算几何算法的封闭框
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        # 开始获取楼梯轮廓包围框内的点
        stair_polyline = []
        for seg in stair_project_point:
            current_seg = []
            judge_seg = [list(seg[0]), list(seg[1])]
            point_1 = Point3D()
            point_1.x = seg[0][0]
            point_1.y = seg[0][1]
            point_1.z = seg[0][2]
            point_2 = Point3D()
            point_2.x = seg[1][0]
            point_2.y = seg[1][1]
            point_2.z = seg[1][2]
            intersection_points = segmentInPolygonPoint(
                judge_seg, bounding_box_polyline
            )
            result_1 = pointInPolygon(point_1, bounding_box_polyline)
            result_2 = pointInPolygon(point_2, bounding_box_polyline)
            if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内
                current_seg.append(seg[0])
                current_seg.append(seg[1])
            elif result_1 != 0 and result_2 == 0:  # 第一点在包围框内，第二点在包围框外
                current_seg.append(seg[0])
                if len(intersection_points) == 1:
                    point_ = intersection_points[0]
                    current_seg.append((point_.x, point_.y, point_.z))
                else:
                    logger.debug("无交点或有多个交点，无法构成单根线段!")
            elif result_1 == 0 and result_2 != 0:  # 第一点在包围框外，第二点在包围框内
                current_seg.append(seg[1])
                if len(intersection_points) == 1:
                    point_ = intersection_points[0]
                    current_seg.append((point_.x, point_.y, point_.z))
                else:
                    logger.debug("无交点或有多个交点，无法构成单根线段")
            elif (
                result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
            ):  # 两点都不在包围框内，线段与包围框是相离或相交，若是相交则取出两交点，若相离则不保留该线段上的点。
                # 两点形成直线与包围框相交
                point_1 = intersection_points[0]
                point_2 = intersection_points[1]
                current_seg.append((point_1.x, point_1.y, point_1.z))
                current_seg.append((point_2.x, point_2.y, point_2.z))
            else:
                pass
            if len(current_seg) > 0:
                point_1 = current_seg[0]
                point_2 = current_seg[1]
                distance = np.linalg.norm(
                    np.array(list(point_2)) - np.array(list(point_1))
                )
                if distance > 1:  # 前述设置的阈值
                    stair_polyline.append(current_seg)
        return stair_polyline

    def choose_step_slot_projection_inner_point_of_bounding_box(self):
        """
        在楼梯防滑槽俯视图中选择包围框内部数据进行显示。
        :return:[[point_1,point_2],[point_3,point_4]]
        """
        bounding_box_loc = self.get_step_slot_top_view_bounding_box()  # 获取包围框内点集合
        step_slot_project_point = self.get_step_slot_projection()  # 获取楼梯防滑槽投影数据点集合
        # 形成计算几何算法的封闭框
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        # 开始获取楼梯轮廓包围框内的点
        step_slot_polyline = []
        for seg in step_slot_project_point:
            current_seg = []
            judge_seg = [list(seg[0]), list(seg[1])]
            point_1 = Point3D()
            point_1.x = seg[0][0]
            point_1.y = seg[0][1]
            point_1.z = seg[0][2]
            point_2 = Point3D()
            point_2.x = seg[1][0]
            point_2.y = seg[1][1]
            point_2.z = seg[1][2]
            intersection_points = segmentInPolygonPoint(
                judge_seg, bounding_box_polyline
            )
            result_1 = pointInPolygon(point_1, bounding_box_polyline)
            result_2 = pointInPolygon(point_2, bounding_box_polyline)
            if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内
                current_seg.append(seg[0])
                current_seg.append(seg[1])
            elif result_1 != 0 and result_2 == 0:  # 第一点在包围框内，第二点在包围框外
                current_seg.append(seg[0])
                if len(intersection_points) == 1:
                    point_ = intersection_points[0]
                    current_seg.append((point_.x, point_.y, point_.z))
                else:
                    logger.debug("无交点或有多个交点，无法构成单根线段!")
            elif result_1 == 0 and result_2 != 0:  # 第一点在包围框外，第二点在包围框内
                current_seg.append(seg[1])
                if len(intersection_points) == 1:
                    point_ = intersection_points[0]
                    current_seg.append((point_.x, point_.y, point_.z))
                else:
                    logger.debug("无交点或有多个交点，无法构成单根线段")
            elif (
                result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
            ):  # 两点都不在包围框内，线段与包围框是相离或相交，若是相交则取出两交点，若相离则不保留该线段上的点。
                # 两点形成直线与包围框相交
                point_1 = intersection_points[0]
                point_2 = intersection_points[1]
                current_seg.append((point_1.x, point_1.y, point_1.z))
                current_seg.append((point_2.x, point_2.y, point_2.z))
            else:
                pass
            if len(current_seg) > 0:
                step_slot_polyline.append(current_seg)
        return step_slot_polyline

    def get_stair_step_slot_special_line_shape_data(self):
        """
        获取防滑槽特殊形状数据
        :return:
        """
        # 获取防滑槽坐标点
        step_slot_data = self.choose_step_slot_projection_inner_point_of_bounding_box()
        # 获取防滑槽配置信息
        step_slot_info = self.step_slot_config.get_step_slot_configuration()
        step_slot_e = step_slot_info["e"]
        # 防滑槽纵向线段
        longitudinal_segment = []
        # 防滑槽横向线段
        transverse_segment = []
        # 遍历筛选出的横向和纵向线段
        for seg in step_slot_data:
            point_1 = list(seg[0])
            point_2 = list(seg[1])
            direction = np.array(np.array(point_2) - np.array(point_1))
            if direction[0] == 0 and direction[1] != 0:
                longitudinal_segment.append(seg)
            else:
                transverse_segment.append(seg)
        max_x = 0  # 最大的x值
        seg_0 = transverse_segment[0]
        for point in seg_0:
            if point[0] > max_x:
                max_x = point[0]
        seg_long_l = []  # 左侧纵向线段
        seg_long_r = []  # 右侧纵向线段
        seg_long_1 = longitudinal_segment[0]
        seg_long_2 = longitudinal_segment[1]
        if seg_long_1[0][1] > seg_long_2[0][1]:
            seg_long_l.append(seg_long_1[0])
            seg_long_l.append(seg_long_1[1])
            seg_long_r.append(seg_long_2[0])
            seg_long_r.append(seg_long_2[1])
        else:
            seg_long_l.append(seg_long_2[0])
            seg_long_l.append(seg_long_2[1])
            seg_long_r.append(seg_long_1[0])
            seg_long_r.append(seg_long_1[1])
        # 开始获取左侧特殊线上的点
        point_l_1 = seg_long_l[0]
        point_l_2 = seg_long_l[1]
        point_l_3 = [0, 0, 0]
        point_l_3[0] = point_l_1[0] + step_slot_e
        point_l_3[1] = (point_l_1[1] + point_l_2[1]) / 2
        point_l_3[2] = (point_l_1[2] + point_l_2[2]) / 2
        point_l_4 = copy.deepcopy(point_l_3)
        point_l_4[0] = max_x
        # 开始获取右侧特殊线上的点
        point_r_1 = seg_long_r[0]
        point_r_2 = seg_long_r[1]
        point_r_3 = [0, 0, 0]
        point_r_3[0] = point_r_1[0] + step_slot_e
        point_r_3[1] = (point_r_1[1] + point_r_2[1]) / 2
        point_r_3[2] = (point_r_1[2] + point_r_2[2]) / 2
        point_r_4 = copy.deepcopy(point_r_3)
        point_r_4[0] = max_x
        draw_line = [
            [point_l_1, point_l_3],
            [point_l_2, point_l_3],
            [point_l_3, point_l_4],
            [point_r_1, point_r_3],
            [point_r_2, point_r_3],
            [point_r_3, point_r_4],
        ]
        return draw_line

    def get_stair_step_slot_dimension_point_data(self):
        """
        开始获取楼梯防滑槽标注位置点数据
        :return:
        """
        # 获取防滑槽坐标点
        step_slot_data = self.choose_step_slot_projection_inner_point_of_bounding_box()
        # 获取单个踏步坐标点
        stair_data = self.choose_stair_projection_inner_point_of_bounding_box()
        # 获取防滑槽配置信息
        step_slot_info = self.step_slot_config.get_step_slot_configuration()
        step_slot_e = step_slot_info["e"]
        # 防滑槽纵向线段
        longitudinal_segment = []
        # 防滑槽横向线段
        transverse_segment = []
        # 遍历筛选出的横向和纵向线段
        for seg in step_slot_data:
            point_1 = list(seg[0])
            point_2 = list(seg[1])
            direction = np.array(np.array(point_2) - np.array(point_1))
            if direction[0] == 0 and direction[1] != 0:
                longitudinal_segment.append(seg)
            else:
                transverse_segment.append(seg)
        # 筛选出左右纵向线段
        seg_long_l = []  # 左侧纵向线段
        seg_long_r = []  # 右侧纵向线段
        seg_long_1 = longitudinal_segment[0]
        seg_long_2 = longitudinal_segment[1]
        if seg_long_1[0][1] > seg_long_2[0][1]:
            seg_long_l.append(seg_long_1[0])
            seg_long_l.append(seg_long_1[1])
            seg_long_r.append(seg_long_2[0])
            seg_long_r.append(seg_long_2[1])
        else:
            seg_long_l.append(seg_long_2[0])
            seg_long_l.append(seg_long_2[1])
            seg_long_r.append(seg_long_1[0])
            seg_long_r.append(seg_long_1[1])
        # 获取楼梯踏步剖切图形右侧下部角点
        bottom_right_point = [0, 0, 0]
        for seg in stair_data:
            point_1 = seg[0]
            point_2 = seg[1]
            direction = np.array(np.array(point_2) - np.array(point_1))
            if direction[0] == 0 and direction[1] != 0:
                if point_1[1] < point_2[1]:
                    bottom_right_point[0] = point_1[0]
                    bottom_right_point[1] = point_1[1]
                    bottom_right_point[2] = point_1[2]
                else:
                    bottom_right_point[0] = point_2[0]
                    bottom_right_point[1] = point_2[1]
                    bottom_right_point[2] = point_2[2]
        # 获取左侧防滑槽标志点
        left_point_1 = [0, 0, 0]  #
        point_l_1 = seg_long_l[0]
        point_l_2 = seg_long_l[1]
        if point_l_1[1] > point_l_2[1]:
            left_point_1[0] = point_l_1[0]
            left_point_1[1] = point_l_1[1]
            left_point_1[2] = point_l_1[2]
        else:
            left_point_1[0] = point_l_2[0]
            left_point_1[1] = point_l_2[1]
            left_point_1[2] = point_l_2[2]
        left_point_2 = copy.deepcopy(left_point_1)
        left_point_2[0] += step_slot_e
        left_point_0 = copy.deepcopy(bottom_right_point)
        left_point_0[1] = left_point_1[1]
        # 获取横向线段的中间点
        top_point_set = []
        for seg in transverse_segment:
            point_1 = seg[0]
            point_2 = seg[1]
            mid_x = (point_1[0] + point_2[0]) / 2
            mid_y = (point_1[1] + point_2[1]) / 2
            mid_z = (point_1[2] + point_2[2]) / 2
            top_point_set.append([mid_x, mid_y, mid_z])
        # 对获取的横向线段中点进行由大到小排序---冒泡法排序
        for m in range(len(top_point_set) - 1):
            for n in range(m + 1, len(top_point_set)):
                point_1 = top_point_set[m]
                point_2 = top_point_set[n]
                if point_1[1] < point_2[1]:
                    mid_point = copy.deepcopy(point_1)
                    point_1 = point_2
                    point_2 = mid_point
                    top_point_set[m] = point_1
                    top_point_set[n] = point_2
        bottom_right_point[0] = top_point_set[0][0]
        top_point_set.append(bottom_right_point)
        dimension_point = [
            [top_point_set[4], top_point_set[3]],
            [top_point_set[3], top_point_set[2]],
            [top_point_set[2], top_point_set[1]],
            [top_point_set[1], top_point_set[0]],
            [left_point_2, left_point_1],
            [left_point_1, left_point_0],
        ]
        # print("待标注点的坐标为：",dimension_point)
        return dimension_point


class StairBottomInstallNodeViewData(object):
    """
    楼梯底部安装节点视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 获取连接孔洞信息
        self.beam_slab_info = LadderBeamAndSlabInformation()  # 平台梁和平台板信息
        self.connect_element_info = ConnectElementShapeInfo()  # 连接单元信息
        self.top_node_info = self.detail_slab.construction_detailed.top_joint  # 顶部节点信息
        self.bottom_node_info = (
            self.detail_slab.construction_detailed.bottom_joint
        )  # 底部节点信息
        self.bottom_node_type = (
            self.detail_slab.construction_detailed.bottom_hole_type
        )  #
        self.top_node_type = self.detail_slab.construction_detailed.top_hole_type  #
        self.connect_hole_info = self.hole_info.get_hole_config()  # 连接孔洞尺寸信息
        self.drawing_precision = 0.001  # 判断精度
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置坐标
        limit_x = self.b0 / 2  # 连接孔洞限值
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2  # 连接孔洞限值
        current_point = [0, 0, 0]
        for point in hole_loc:
            if point.x > limit_x and point.y < limit_y:
                current_point[0] = point.x
                current_point[1] = point.y
                current_point[2] = point.z
        return current_point

    def get_stair_bottom_beam_and_slab_long_cut_drawing(self):
        """
        获取楼梯底端平台梁和板的纵向剖切图数据
        :return:
        """
        bottom_beam_slab_model = (
            self.composite_model.get_bottom_left_beam_slab_model()
        )  # 获取OCC模型
        current_point = self.get_stair_solid_long_cut_plane_point()  # 获取关键点
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, bottom_beam_slab_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_long_cut_drawing(self):
        """
        获取楼梯纵向剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_bottom_anchor_rebar_cut_drawing(self):
        """
        获取楼梯底端锚固钢筋剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_bottom_connect_rebar_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_bottom_nut_cut_drawing(self):
        """
        获取楼梯底端螺母剖切数据：形成纵向剖切轮廓
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_bottom_connect_nut_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_bottom_shim_cut_drawing(self):
        """
        获取楼梯底端垫片剖切数据：形成纵向剖切轮廓
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_bottom_connect_shim_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_bottom_connect_embedded_cut_drawing(self):
        """
        获取楼梯底端连接件纵向剖切轮廓数据
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_bottom_connect_rebar_nut_shim_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_bottom_connect_nut_projection_drawing(self):
        """
        获取楼梯底部连接螺母投影数据图
        :return:
        """
        bottom_nut_model = self.composite_model.get_bottom_connect_nut_model()
        point_0 = self.get_stair_solid_long_cut_plane_point()  # 基点很重要---会影响投影后的点绝对坐标
        normal = [1, 0, 0]
        origin = gp_Pnt(0, 0, 0)  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(bottom_nut_model, origin, project_dir)
        # draw_multiple_line(points)
        return points

    def get_stair_bottom_connect_shim_projection_drawing(self):
        """
        获取楼梯底部连接螺母垫片投影数据图
        :return:
        point_0 = self.get_stair_solid_long_cut_plane_point()  # 基点很重要---会影响投影后的点绝对坐标
        """
        bottom_shim_model = self.composite_model.get_bottom_connect_shim_model()
        normal = [1, 0, 0]
        origin = gp_Pnt(0, 0, 0)  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(bottom_shim_model, origin, project_dir)
        # draw_multiple_line(points)
        return points

    def get_stair_bottom_hole_cut_drawing(self):
        """
        获取楼梯底端孔洞剖切形状
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_all_hole_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    @staticmethod
    def choose_inner_point_of_bounding_box(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    def get_stair_bottom_shim_corner_point(self):
        """
        获取楼梯底部垫片角点坐标
        :return: List[Tuple[float]]
        """
        shim_profile_points = (
            self.get_stair_bottom_connect_shim_projection_drawing()
        )  # 底部垫片数据
        # 开始获取垫片轮廓点
        max_x = 0
        max_y = 0
        min_x = self.b0
        min_y = self.lb_d
        for segment in shim_profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (abs(segment[num][0]), abs(segment[num][1]), abs(segment[num][2])),
                    (
                        abs(segment[num + 1][0]),
                        abs(segment[num + 1][1]),
                        abs(segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                # 获取填充图形内部轮廓点
                for point in seg:
                    if max_x <= point[0]:
                        max_x = point[0]
                    if max_y <= point[1]:
                        max_y = point[1]
                    if min_x >= point[0]:
                        min_x = point[0]
                    if min_y >= point[1]:
                        min_y = point[1]
        bounding_box_loc = [
            (min_x, min_y, 0),
            (max_x, min_y, 0),
            (max_x, max_y, 0),
            (min_x, max_y, 0),
        ]
        return bounding_box_loc

    def get_bottom_beam_slab_bounding_profile_points(self):
        """
        获取底部平台梁和平台板包围框坐标点
        :return:
        """
        cut_point = self.get_stair_solid_long_cut_plane_point()  # 获取剖切点
        value_x = cut_point[0]  # 获取x坐标值
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets  # 获取平台板和平台梁信息
        bottom_beam_slab_info = beam_and_slab_info["bottom"]  # 底部平台板和平台梁信息
        add_value = 10  # 包围框增加值
        # 获取梁的基本信息
        beam_width = bottom_beam_slab_info.beam_width  # 梁宽度
        beam_area_width = bottom_beam_slab_info.beam_area_width  # 梁的部分宽度
        beam_height = bottom_beam_slab_info.beam_height  # 梁高度
        # 获取板的基本信息
        slab_thick = bottom_beam_slab_info.slab_thickness  # 板的厚度
        slab_length = bottom_beam_slab_info.slab_length  # 板的长度
        # 设置包围框点
        point_1 = [0, 0, 0]
        point_1[0] = value_x
        point_1[1] = beam_area_width + add_value
        point_1[2] = self.h2 + add_value
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -(beam_height + 2 * add_value)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(beam_width + slab_length / 3 + 2 * add_value)
        point_4 = copy.deepcopy(point_3)
        point_4[2] += beam_height + 2 * add_value
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]  # 图形数据包围框
        cut_profile_points = (
            self.get_stair_bottom_beam_and_slab_long_cut_drawing()
        )  # 获取底部平台板和平台梁的轮廓数据
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_bottom_local_shape_bounding_profile_points(self):
        """
        获取楼梯底部局部形状包围轮廓点
        :return:
        """
        cut_point = self.get_stair_solid_long_cut_plane_point()  # 获取剖切点
        value_x = cut_point[0]  # 获取x坐标值
        add_value = 10  # 包围框增加值
        # 获取楼梯基本数据点
        h2 = self.h2  # 获取底端板厚度
        lb = self.lb_d  # 获取底端上边长数据
        tabu_h = self.tabu_h  # 踏步高度
        tabu_w = self.tabu_b  # 踏步宽度
        # 获取包围框坐标点
        point_1 = [0, 0, 0]
        point_1[0] = value_x
        point_1[1] = -add_value
        point_1[2] = -add_value
        point_2 = copy.deepcopy(point_1)
        point_2[2] += h2 + 2 * add_value + tabu_h
        point_3 = copy.deepcopy(point_2)
        point_3[1] += lb + 2 * tabu_w / 3 + 2 * add_value
        point_4 = copy.deepcopy(point_3)
        point_4[2] = -add_value
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]  # 包围框
        cut_profile_points = self.get_stair_solid_long_cut_drawing()  # 获取楼梯实体轮廓点集
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_bottom_right_hole_bounding_profile_points(self):
        """
        获取楼梯底部右侧孔洞包围框轮廓数据点
        :return:
        """
        cut_point = self.get_stair_solid_long_cut_plane_point()  # 获取剖切点
        value_x = cut_point[0]  # 获取x坐标值
        add_value = 10  # 包围框增加值
        # 获取楼梯基本数据点
        h2 = self.h2  # 获取底端板厚度
        lb = self.lb_d  # 获取底端上边长数据
        tabu_h = self.tabu_h  # 踏步高度
        tabu_w = self.tabu_b  # 踏步宽度
        # 获取包围框坐标点
        point_1 = [0, 0, 0]
        point_1[0] = value_x
        point_1[1] = -add_value
        point_1[2] = -add_value
        point_2 = copy.deepcopy(point_1)
        point_2[2] += h2 + 2 * add_value + tabu_h
        point_3 = copy.deepcopy(point_2)
        point_3[1] += lb + 2 * tabu_w / 3 + 2 * add_value
        point_4 = copy.deepcopy(point_3)
        point_4[2] = -add_value
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]  # 包围框
        cut_profile_points = self.get_stair_bottom_hole_cut_drawing()  # 获取楼梯实体轮廓点集
        # 获取包围框内部的点
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_solid_beam_slab_left_boundary_data(self) -> List[List[float]]:
        """
        获取楼梯实体平台梁和板左侧边界坐标点
        :return:
        """
        beam_slab_profile_points = self.get_bottom_beam_slab_bounding_profile_points()
        min_y = 0  # 最小的y坐标值
        for seg in beam_slab_profile_points:
            for point in seg:
                if min_y >= point[1]:
                    min_y = point[1]
        draw_points = []
        # 获取x剖切平台板最左侧两点
        for seg in beam_slab_profile_points:
            for point in seg:
                if abs(point[1] - min_y) < self.drawing_precision:
                    draw_points.append(point)
        # 排序两点
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        if len(draw_points) == 2:
            if draw_points[0][2] > draw_points[1][2]:
                top_point[0] = draw_points[0][0]
                top_point[1] = draw_points[0][1]
                top_point[2] = draw_points[0][2]
                bottom_point[0] = draw_points[1][0]
                bottom_point[1] = draw_points[1][1]
                bottom_point[2] = draw_points[1][2]
            else:
                top_point[0] = draw_points[1][0]
                top_point[1] = draw_points[1][1]
                top_point[2] = draw_points[1][2]
                bottom_point[0] = draw_points[0][0]
                bottom_point[1] = draw_points[0][1]
                bottom_point[2] = draw_points[0][2]
        else:
            print("筛选出点的数量有误!")
        final_points = [top_point, bottom_point]
        return final_points

    def get_stair_solid_local_bottom_right_boundary_points(self):
        """
        获取楼梯实体局部底部右侧边缘点：绘制折断线
        :return:List[List[float]]
        """
        stair_profile_points = (
            self.get_stair_bottom_local_shape_bounding_profile_points()
        )
        max_y = 0  # 最大的y坐标值
        for seg in stair_profile_points:
            for point in seg:
                if max_y <= point[1]:
                    max_y = point[1]
        draw_points = []
        # 获取y剖切楼梯底部最右侧两点
        for seg in stair_profile_points:
            for point in seg:
                if abs(point[1] - max_y) < self.drawing_precision:
                    draw_points.append(point)
        # 排序两点
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        if len(draw_points) == 2:
            if draw_points[0][2] > draw_points[1][2]:
                top_point[0] = draw_points[0][0]
                top_point[1] = draw_points[0][1]
                top_point[2] = draw_points[0][2]
                bottom_point[0] = draw_points[1][0]
                bottom_point[1] = draw_points[1][1]
                bottom_point[2] = draw_points[1][2]
            else:
                top_point[0] = draw_points[1][0]
                top_point[1] = draw_points[1][1]
                top_point[2] = draw_points[1][2]
                bottom_point[0] = draw_points[0][0]
                bottom_point[1] = draw_points[0][1]
                bottom_point[2] = draw_points[0][2]
        else:
            print("筛选出点的数量有误!")
        final_points = [top_point, bottom_point]
        return final_points

    def get_stair_left_boundary_breakline(self):
        """
        获取楼梯左侧边界折断线
        :return:
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        left_loc = self.get_stair_solid_beam_slab_left_boundary_data()
        limit_z = (left_loc[0][2] + left_loc[1][2]) / 2
        node_data = self.bottom_node_info
        node_b = node_data.b
        top_loc = [0, 0, 0]
        bottom_loc = [0, 0, 0]
        for point in left_loc:
            if point[2] > limit_z:
                top_loc[0] = point[0]
                top_loc[1] = point[1]
                top_loc[2] = point[2] + node_b
            else:
                bottom_loc[0] = point[0]
                bottom_loc[1] = point[1]
                bottom_loc[2] = point[2]
        # 开始设置关键点
        point_1 = copy.deepcopy(top_loc)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_loc)
        point_3 = copy.deepcopy(top_loc)
        point_3[2] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_loc)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_right_boundary_breakline(self):
        """
        获取楼梯右侧边界折断线
        :return:
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        right_loc = self.get_stair_solid_local_bottom_right_boundary_points()
        limit_z = (right_loc[0][2] + right_loc[1][2]) / 2
        top_loc = [0, 0, 0]
        bottom_loc = [0, 0, 0]
        for point in right_loc:
            if point[2] > limit_z:
                top_loc[0] = point[0]
                top_loc[1] = point[1]
                top_loc[2] = point[2]
            else:
                bottom_loc[0] = point[0]
                bottom_loc[1] = point[1]
                bottom_loc[2] = point[2]
        # 开始设置关键点
        point_1 = copy.deepcopy(top_loc)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_loc)
        point_3 = copy.deepcopy(top_loc)
        point_3[2] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_loc)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_bottom_beam_slab_elevation_line_points(self):
        """
        获取标高线绘制坐标点
        :return:List[List[float]]
        """
        left_boundary_points = (
            self.get_stair_solid_beam_slab_left_boundary_data()
        )  # 获取左侧边界点
        top_point = left_boundary_points[0]
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        top_point[2] += node_b
        right_point = copy.deepcopy(top_point)
        right_point[1] = -node_a
        final_points = [top_point, right_point]
        return final_points

    def get_stair_solid_elevation_special_point(self):
        """
        开始获取楼梯实体标高特殊点坐标
        :return: List[List[List[float]],List[List[float],str]]
        """
        special_length = 36  # 三角形的高
        elevation_line_points = (
            self.get_stair_bottom_beam_slab_elevation_line_points()
        )  #
        left_point = elevation_line_points[0]
        point_0 = [left_point[0], 5 * left_point[1] / 6, left_point[2]]
        point_1 = copy.deepcopy(point_0)
        point_1[1] += -special_length / 2
        point_1[2] += special_length
        point_2 = copy.deepcopy(point_1)
        point_2[1] += special_length
        point_3 = copy.deepcopy(point_2)
        point_3[1] += 3 * special_length
        draw_line = [
            [point_0, point_1],
            [point_1, point_2],
            [point_2, point_3],
            [point_0, point_2],
        ]
        text_loc = copy.deepcopy(point_0)
        text_loc[2] += special_length
        text = "完成面标高"
        text_info = [text_loc, text]
        final_points = [draw_line, text_info]
        return final_points

    def get_stair_bottom_glue_hatch_points(self):
        """
        获取楼梯底部打胶填充点
        :return:
        """
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        # 填充图形轮廓点
        point_0 = [0, 0, 0]
        point_0[2] += self.h2
        point_1 = copy.deepcopy(point_0)
        point_1[1] += -node_a
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_b
        point_3 = copy.deepcopy(point_2)
        point_3[1] += node_a
        profile_points = [point_0, point_1, point_2, point_3]
        # 引出线点
        outline_length_1 = 150
        outline_length_2 = 150
        theta = math.pi / 3
        point_s_0 = copy.deepcopy(point_0)
        point_s_0[1] += -node_a / 2
        point_s_0[2] += -node_b / 5
        point_s_1 = copy.deepcopy(point_s_0)
        point_s_1[1] += -outline_length_1 * math.cos(theta)
        point_s_1[2] += outline_length_1 * math.sin(theta)
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_2
        outline_points = [[point_s_0, point_s_1], [point_s_1, point_s_2]]
        text_loc = copy.deepcopy(point_s_2)
        text = "打胶" + str(node_a) + "X" + str(node_b)
        outline_text = [text_loc, text]
        # 所有信息合集
        total_info = {}
        total_info["hatch_profile"] = profile_points
        total_info["outline"] = outline_points
        total_info["outline_text"] = outline_text
        return total_info

    def get_stair_bottom_pe_rod_hatch_points(self):
        """
        获取楼梯底部PE棒填充点
        :return:
        """
        edge = 4  # 边缘长度
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        diameter = node_a - edge
        origin = [0, 0, 0]
        origin[1] = -node_a / 2
        origin[2] = self.h2 - node_b - diameter / 2
        # 引出线点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 3
        point_1 = copy.deepcopy(origin)
        point_2 = copy.deepcopy(point_1)
        point_2[1] += -outline_length_1 * math.cos(theta)
        point_2[2] += outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -outline_length_2
        text_loc = copy.deepcopy(point_3)
        text = "PE棒"
        hatch_line = [origin, diameter]
        outline_points = [[point_1, point_2], [point_2, point_3]]
        outline_text = [text_loc, text]
        # 所有信息集合
        total_info = {}
        total_info["hatch_profile"] = hatch_line
        total_info["outline"] = outline_points
        total_info["outline_text"] = outline_text
        return total_info

    def get_stair_bottom_mid_hatch_pattern_points(self):
        """
        获取楼梯底部中间填充图案点集合
        :return:
        """
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        node_c = node_data.c
        # 获取填充轮廓点
        point_1 = [0, 0, 0]
        point_1[2] += self.h2 - node_b
        point_2 = copy.deepcopy(point_1)
        point_2[1] += -node_a
        point_3 = copy.deepcopy(point_2)
        point_3[2] += -(node_c + self.h2 - node_b)
        point_4 = copy.deepcopy(point_3)
        point_4[1] += node_a
        # 引出线点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 6
        point_s_1 = [0, 0, 0]
        point_s_1[1] = -node_a / 2
        point_s_1[2] = (point_2[2] + point_3[2]) / 2
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_1 * math.cos(theta)
        point_s_2[2] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        # 引出线文本注释
        text_loc = copy.deepcopy(point_s_3)
        text = "聚苯填充"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_text = [text_loc, text]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_bottom_slide_hatch_one_points(self):
        """
        获取楼梯底部滑动铰支座填充1坐标点
        """
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_c = node_data.c
        beam_area_width = (
            self.beam_slab_info.bottom_plate.beam_area_width
        )  # 获取底部平台两部分宽度
        fact_width = beam_area_width - node_a
        # 填充点
        point_1 = [0, 0, 0]
        point_2 = copy.deepcopy(point_1)
        point_2[2] = -node_c / 2
        point_3 = copy.deepcopy(point_2)
        point_3[1] = fact_width
        point_4 = copy.deepcopy(point_3)
        point_4[2] = 0
        # 引出线的点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_s_1 = [0, 3 * fact_width / 4, -node_c / 4]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += -outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        point_s_4 = [0, fact_width / 2, 0]
        point_s_5 = copy.deepcopy(point_s_4)
        point_s_5[1] += -outline_length_1 * math.cos(theta)
        point_s_5[2] += -outline_length_1 * math.sin(theta)
        point_s_6 = copy.deepcopy(point_s_5)
        point_s_6[1] += -outline_length_2
        # 引出线说明
        text_loc_1 = copy.deepcopy(point_s_3)
        text_1 = "M15水泥砂浆"
        text_loc_2 = copy.deepcopy(point_s_6)
        text_2 = "调平钢垫片"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points_1 = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_points_2 = [[point_s_4, point_s_5], [point_s_5, point_s_6]]
        outline_text_1 = [text_loc_1, text_1]
        outline_text_2 = [text_loc_2, text_2]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline_1"] = outline_points_1
        hatch_info["outline_text_1"] = outline_text_1
        hatch_info["outline_2"] = outline_points_2
        hatch_info["outline_text_2"] = outline_text_2
        return hatch_info

    def get_stair_bottom_slide_hatch_two_points(self):
        """
        获取楼梯底部滑动铰支座填充2坐标点
        """
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_c = node_data.c
        beam_area_width = (
            self.beam_slab_info.bottom_plate.beam_area_width
        )  # 获取底部平台两部分宽度
        fact_width = beam_area_width - node_a
        # 填充点
        point_1 = [0, 0, -node_c / 2]
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_c / 2
        point_3 = copy.deepcopy(point_2)
        point_3[1] += fact_width
        point_4 = copy.deepcopy(point_3)
        point_4[2] += node_c / 2
        # 引出线的点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_s_1 = [0, 3 * fact_width / 5, -3 * node_c / 4]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += -outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        # 引出线说明
        text_loc = copy.deepcopy(point_s_3)
        text = "油毡一层"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_text = [text_loc, text]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_bottom_fix_hatch_points(self):
        """
        获取楼梯底部固定铰支座填充坐标点
        """
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_c = node_data.c
        beam_area_width = (
            self.beam_slab_info.bottom_plate.beam_area_width
        )  # 获取底部平台两部分宽度
        fact_width = beam_area_width - node_a
        # 填充点
        point_1 = [0, 0, 0]
        point_2 = copy.deepcopy(point_1)
        point_2[2] = -node_c
        point_3 = copy.deepcopy(point_2)
        point_3[1] = fact_width
        point_4 = copy.deepcopy(point_3)
        point_4[2] = 0
        # 引出线的点
        # 第一跟引出线
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_s_1 = [0, 3 * fact_width / 4, -node_c / 2]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += -outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        # 第二根引出线
        point_s_4 = [0, fact_width / 2, 0]
        point_s_5 = copy.deepcopy(point_s_4)
        point_s_5[1] += -outline_length_1 * math.cos(theta)
        point_s_5[2] += -outline_length_1 * math.sin(theta)
        point_s_6 = copy.deepcopy(point_s_5)
        point_s_6[1] += -outline_length_2
        # 引出线说明
        text_loc_1 = copy.deepcopy(point_s_3)
        text_1 = "M15水泥砂浆"
        text_loc_2 = copy.deepcopy(point_s_6)
        text_2 = "调平垫片"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points_1 = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_points_2 = [[point_s_4, point_s_5], [point_s_5, point_s_6]]
        outline_text_1 = [text_loc_1, text_1]
        outline_text_2 = [text_loc_2, text_2]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline_1"] = outline_points_1
        hatch_info["outline_2"] = outline_points_2
        hatch_info["outline_text_1"] = outline_text_1
        hatch_info["outline_text_2"] = outline_text_2
        return hatch_info

    def get_stair_bottom_slide_hatch_points(self):
        """
        获取楼梯底部滑动铰支座填轮廓点
        :return:
        """
        bottom_shim_loc = self.get_stair_bottom_shim_corner_point()  # 获取底部垫片轮廓点
        max_y = bottom_shim_loc[-1][1]
        min_y = bottom_shim_loc[0][1]
        max_z = bottom_shim_loc[1][0]
        min_z = bottom_shim_loc[0][0]
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        hole_c1 = self.connect_hole_info["bottom_top_dia"]
        hole_h1 = self.connect_hole_info["bottom_h"]
        hole_e1 = self.connect_hole_info["bottom_top_dia_1"]
        # 计算关键点
        center_point = [0, hole_loc[1], self.h2]
        point_1 = copy.deepcopy(center_point)
        point_1[1] += -hole_c1 / 2
        point_2 = copy.deepcopy(center_point)
        point_2[1] += -hole_e1 / 2
        point_2[2] += -hole_h1
        point_3 = [0, min_y, min_z]
        point_4 = [0, min_y, max_z]
        point_5 = [0, max_y, max_z]
        point_6 = [0, max_y, min_z]
        point_7 = copy.deepcopy(center_point)
        point_7[0] = 0
        point_7[1] += hole_e1 / 2
        point_7[2] += -hole_h1
        point_8 = copy.deepcopy(center_point)
        point_8[1] += hole_c1 / 2
        bounding_box_loc = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        # 引出线点和说明
        outline_length_1 = 120
        outline_length_2 = 300
        point_s_1 = copy.deepcopy(center_point)
        point_s_1[2] += -hole_h1 / 2
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[2] += outline_length_1
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        # 文本位置和内容
        text_loc = copy.deepcopy(point_s_2)
        text = "砂浆封堵(平整、密实、光滑)"
        outline_text = [text_loc, text]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = bounding_box_loc
        hatch_info["outline"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_construction_method_dimension_points(self):
        """
        获取楼梯构造做法尺寸标注点
        :return:
        """
        # 获取标高上的点
        elevation_points = self.get_stair_bottom_beam_slab_elevation_line_points()
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        node_c = node_data.c
        point_0 = [0, 0, self.h2]
        point_1 = [0, -node_a, self.h2]
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_b
        point_3 = copy.deepcopy(point_2)
        point_3[2] = 0
        point_4 = copy.deepcopy(point_3)
        point_4[2] += -node_c
        point_5 = copy.deepcopy(elevation_points[0])
        point_6 = copy.deepcopy(point_5)
        point_6[2] += -node_b
        dimension_points = [
            [point_0, point_1],
            [point_1, point_2],
            [point_2, point_3],
            [point_3, point_4],
            [point_5, point_6],
        ]
        return dimension_points

    def get_stair_bottom_anchor_rebar_outline_points(self):
        """
        获取楼梯底端锚固钢筋引出线点
        :return:
        """
        bottom_rebar_diameter = (
            self.connect_element_info.bottom_connect_element.rebar_anchor_diameter
        )
        anchor_points = (
            self.get_stair_solid_bottom_anchor_rebar_cut_drawing()
        )  # 获取楼梯底端锚固钢筋剖切数据
        special_point = [0, 0, 0]
        for seg in anchor_points:
            if len(seg) > 2:
                mid_num = int(len(seg) / 2)  # 圆弧中中部点
                current_p = seg[mid_num]
                special_point[0] = current_p[0]
                special_point[1] = current_p[1]
                special_point[2] = current_p[2]
        # 引出线绘制
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 6
        point_1 = copy.deepcopy(special_point)
        point_2 = copy.deepcopy(point_1)
        point_2[1] += outline_length_1 * math.cos(theta)
        point_2[2] += -outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += outline_length_2
        text_loc = [
            (point_2[0] + point_3[0]) / 2,
            (point_2[1] + point_3[1]) / 2,
            (point_2[2] + point_3[2]) / 2,
        ]
        text = "Φ" + str(bottom_rebar_diameter)
        outline_points = [[point_1, point_2], [point_2, point_3]]
        text_points = [text_loc, text]
        outline_info = {}
        outline_info["outline"] = outline_points
        outline_info["outline_text"] = text_points
        return outline_info

    def get_stair_bottom_beam_outline_points(self):
        """
        获取楼梯底部梯梁引出线点集合
        :return:
        """
        beam_slab_points = self.get_bottom_beam_slab_bounding_profile_points()
        # 选出最小的z值
        min_z = self.h2
        for seg in beam_slab_points:
            for point in seg:
                if min_z > point[2]:
                    min_z = point[2]
        key_point = [0, 0, 0]
        for seg in beam_slab_points:
            if seg[0][2] == min_z and seg[1][2] == min_z:
                key_point[0] = (seg[0][0] + seg[1][0]) / 2
                key_point[1] = (seg[0][1] + seg[1][1]) / 3
                key_point[2] = (seg[0][2] + seg[1][2]) / 2
        # 获取引出线点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_1 = copy.deepcopy(key_point)
        point_2 = copy.deepcopy(point_1)
        point_2[1] += outline_length_1 * math.cos(theta)
        point_2[2] += -outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += outline_length_2
        outline_points = [[point_1, point_2], [point_2, point_3]]
        text_loc = [
            (point_2[0] + point_3[0]) / 2,
            (point_2[1] + point_3[1]) / 2,
            (point_2[2] + point_3[2]) / 2,
        ]
        text = "梯梁"
        outline_text = [text_loc, text]
        # 合集
        outline_info = {}
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info

    def get_stair_bottom_slide_nut_outline_points(self):
        """
        获取楼梯底部防滑槽螺母引出线点的注释
        :return:
        """
        nut_points = self.get_stair_bottom_connect_nut_projection_drawing()
        max_y = 0
        for seg in nut_points:
            for point in seg:
                if max_y < abs(point[1]):
                    max_y = abs(point[1])
        point_1 = [0, 0, 0]
        for seg in nut_points:  # 投影后的点比较特殊
            if abs(seg[0][1]) == max_y and abs(seg[1][1]) == max_y:
                point_1[0] = abs(seg[0][1] + seg[1][1]) / 2
                point_1[1] = abs(seg[0][0] + seg[1][0]) / 2
                point_1[2] = abs(seg[0][2] + seg[1][2]) / 2
        outline_length_1 = 120
        point_2 = copy.deepcopy(point_1)
        point_2[0] += outline_length_1
        text_loc = copy.deepcopy(point_2)
        text = "螺母"
        outline_points = [[point_1, point_2]]
        outline_text = [text_loc, text]
        outline_info = {}
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info

    def get_stair_bottom_fix_node_top_hatch_pattern_points(self):
        """
        获取楼梯底部固定节点顶部填充图案点
        :return:
        """
        hole_info = self.hole_info.get_hole_config()
        hole_c2 = hole_info["bottom_top_dia"]
        hole_d2 = hole_info["bottom_bottom_dia"]
        h2 = self.h2
        bottom_connect_info = self.connect_element_info.bottom_connect_element
        edge_t = bottom_connect_info.edge_t  # 顶部边距
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        height = edge_t / 2
        special_edge = (hole_c2 - hole_d2) * (h2 - height) / h2 / 2
        curr_dia = hole_d2 + 2 * special_edge
        # 开始确定轮廓点
        point_0 = [0, hole_loc[1], h2]
        point_1 = [0, 0, 0]
        point_1[2] = h2
        point_1[1] = hole_loc[1] - hole_c2 / 2
        point_2 = copy.deepcopy(point_1)
        point_2[1] += hole_c2
        point_3 = copy.deepcopy(point_0)
        point_3[2] += -height
        point_3[1] += curr_dia / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -curr_dia
        # 引出线
        outline_length_1 = 60
        outline_length_2 = 120
        point_s_1 = [0, hole_loc[1], h2 - height / 2]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[2] += outline_length_1
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        text_loc = copy.deepcopy(point_s_3)
        text = "砂浆封堵(平整,密实,光滑)"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_text = [text_loc, text]
        # 信息汇总
        hatch_info = {}
        hatch_info["hatch_points"] = hatch_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_bottom_fix_node_mid_hatch_pattern_points(self):
        """
        获取楼梯底端固定节点中部填充图案点集合
        :return:
        """
        top_hatch_info = self.get_stair_bottom_fix_node_top_hatch_pattern_points()
        top_hatch_points = top_hatch_info["hatch_points"]
        point_1 = copy.deepcopy(top_hatch_points[-2])
        point_2 = copy.deepcopy(top_hatch_points[-1])
        hole_info = self.hole_info.get_hole_config()  # 孔洞信息
        hole_c2 = hole_info["bottom_top_dia"]
        hole_d2 = hole_info["bottom_bottom_dia"]
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        # 获取锚固钢筋轮廓数据
        anchor_profile_points = self.get_stair_solid_bottom_anchor_rebar_cut_drawing()
        max_z = 0
        for seg in anchor_profile_points:
            for point in seg:
                if max_z < point[2]:
                    max_z = point[2]
        rebar_top_point = []
        for seg in anchor_profile_points:
            if seg[0][2] == max_z and seg[1][2] == max_z:
                if seg[0][1] < seg[1][1]:
                    rebar_top_point.append(list(seg[0]))
                    rebar_top_point.append(list(seg[1]))
                else:
                    rebar_top_point.append(list(seg[1]))
                    rebar_top_point.append(list(seg[0]))
        point_3 = [0, hole_loc[1] - hole_d2 / 2, 0]
        point_4 = copy.deepcopy(rebar_top_point[0])
        point_4[0] = 0
        point_4[2] = 0
        point_5 = copy.deepcopy(rebar_top_point[0])
        point_5[0] = 0
        point_6 = copy.deepcopy(rebar_top_point[1])
        point_6[0] = 0
        point_7 = copy.deepcopy(point_6)
        point_7[2] = 0
        point_8 = [0, hole_loc[1] + hole_d2 / 2, 0]
        mid_hatch_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        # 引出线标注
        outline_length_1 = 120
        point_s_1 = [
            (point_3[0] + point_4[0]) / 2,
            (point_3[1] + point_4[1]) / 2,
            (point_4[2] + point_5[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1
        text_loc = copy.deepcopy(point_s_1)
        text = "C40级CGM灌浆料灌实"
        outline_points = [[point_s_1, point_s_2]]
        outline_text = [text_loc, text]
        # 信息汇总
        outline_info = {}
        outline_info["hatch_points"] = mid_hatch_points
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info


class StairTopInstallNodeViewData(object):
    """
    楼梯顶部安装节点视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 获取连接孔洞信息
        self.beam_slab_info = LadderBeamAndSlabInformation()  # 平台梁和平台板信息
        self.connect_element_info = ConnectElementShapeInfo()  # 连接单元信息
        self.top_node_info = self.detail_slab.construction_detailed.top_joint  # 顶部节点信息
        self.bottom_node_info = (
            self.detail_slab.construction_detailed.bottom_joint
        )  # 底部节点信息
        self.bottom_node_type = (
            self.detail_slab.construction_detailed.bottom_hole_type
        )  #
        self.top_node_type = self.detail_slab.construction_detailed.top_hole_type  #
        self.connect_hole_info = self.hole_info.get_hole_config()  # 连接孔洞尺寸信息
        self.drawing_precision = 0.001  # 判断精度
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置坐标
        limit_x = self.b0 / 2  # 连接孔洞限值
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2  # 连接孔洞限值
        current_point = [0, 0, 0]
        for point in hole_loc:
            if point.x > limit_x and point.y > limit_y:
                current_point[0] = point.x
                current_point[1] = point.y
                current_point[2] = point.z
        return current_point

    def get_stair_top_beam_and_slab_long_cut_drawing(self):
        """
        获取楼梯顶端平台梁和板的纵向剖切图数据
        :return:
        """
        top_beam_slab_model = (
            self.composite_model.get_top_right_beam_slab_model()
        )  # 获取OCC模型
        current_point = self.get_stair_solid_long_cut_plane_point()  # 获取关键点
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, top_beam_slab_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_long_cut_drawing(self):
        """
        获取楼梯纵向剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_top_anchor_rebar_cut_drawing(self):
        """
        获取楼梯顶端锚固钢筋剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_top_connect_rebar_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_top_nut_cut_drawing(self):
        """
        获取楼梯顶端螺母剖切数据：形成纵向剖切轮廓
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_top_connect_nut_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_top_shim_cut_drawing(self):
        """
        获取楼梯顶端垫片剖切数据：形成纵向剖切轮廓
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_top_connect_shim_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_top_connect_embedded_cut_drawing(self):
        """
        获取楼梯顶端连接件纵向剖切轮廓数据
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_top_connect_rebar_nut_shim_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_top_connect_nut_projection_drawing(self):
        """
        获取楼梯顶部连接螺母投影数据图
        :return:
        """
        top_nut_model = self.composite_model.get_top_connect_nut_model()
        point_0 = self.get_stair_solid_long_cut_plane_point()  # 基点很重要---会影响投影后的点绝对坐标
        normal = [1, 0, 0]
        origin = gp_Pnt(0, 0, 0)  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(top_nut_model, origin, project_dir)
        # draw_multiple_line(points)
        return points

    def get_stair_top_connect_shim_projection_drawing(self):
        """
        获取楼梯顶部连接螺母垫片投影数据图
        :return:
        point_0 = self.get_stair_solid_long_cut_plane_point()  # 基点很重要---会影响投影后的点绝对坐标
        """
        top_shim_model = self.composite_model.get_top_connect_shim_model()
        normal = [1, 0, 0]
        origin = gp_Pnt(0, 0, 0)  # 参考原点
        project_dir = gp_Dir(normal[0], normal[1], normal[2])  # 投影方向
        project, points = compute_project_shape(top_shim_model, origin, project_dir)
        # draw_multiple_line(points)
        return points

    def get_stair_top_hole_cut_drawing(self):
        """
        获取楼梯顶端孔洞剖切形状
        :return: List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_all_hole_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    @staticmethod
    def choose_inner_point_of_bounding_box(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    def get_stair_top_shim_corner_point(self):
        """
        获取楼梯顶部垫片角点坐标
        :return: List[Tuple[float]]
        """
        shim_profile_points = (
            self.get_stair_top_connect_shim_projection_drawing()
        )  # 顶部垫片数据
        # 开始获取垫片轮廓点
        max_x = 0
        max_y = 0
        min_x = self.b0
        min_y = self.lb_d + self.lt_d + self.ln
        for segment in shim_profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (abs(segment[num][0]), abs(segment[num][1]), abs(segment[num][2])),
                    (
                        abs(segment[num + 1][0]),
                        abs(segment[num + 1][1]),
                        abs(segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                # 获取填充图形内部轮廓点
                for point in seg:
                    if max_x <= point[0]:
                        max_x = point[0]
                    if max_y <= point[1]:
                        max_y = point[1]
                    if min_x >= point[0]:
                        min_x = point[0]
                    if min_y >= point[1]:
                        min_y = point[1]
        bounding_box_loc = [
            (min_x, min_y, 0),
            (max_x, min_y, 0),
            (max_x, max_y, 0),
            (min_x, max_y, 0),
        ]
        return bounding_box_loc

    def get_top_beam_slab_bounding_profile_points(self):
        """
        获取顶部平台梁和平台板包围框坐标点
        :return:
        """
        node_data = self.top_node_info
        cut_point = self.get_stair_solid_long_cut_plane_point()  # 获取剖切点
        value_x = cut_point[0]  # 获取x坐标值
        beam_and_slab_info = self.beam_slab_info.ladder_info_sets  # 获取平台板和平台梁信息
        top_beam_slab_info = beam_and_slab_info["top"]  # 顶部平台板和平台梁信息
        add_value = 10  # 包围框增加值
        # 获取梁的基本信息
        beam_width = top_beam_slab_info.beam_width  # 梁宽度
        beam_area_width = top_beam_slab_info.beam_area_width  # 梁的部分宽度
        beam_height = top_beam_slab_info.beam_height  # 梁高度
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        # 获取板的基本信息
        slab_thick = top_beam_slab_info.slab_thickness  # 板的厚度
        slab_length = top_beam_slab_info.slab_length  # 板的长度
        # 设置包围框点
        point_1 = [0, total_l, total_h]
        point_1[0] = value_x
        point_1[1] += -(beam_area_width + add_value)
        point_1[2] += add_value
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -(beam_height + node_data.b + 2 * add_value)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += beam_width + slab_length / 3 + 2 * add_value
        point_4 = copy.deepcopy(point_3)
        point_4[2] += beam_height + node_data.b + 2 * add_value
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]  # 图形数据包围框
        cut_profile_points = (
            self.get_stair_top_beam_and_slab_long_cut_drawing()
        )  # 获取底部平台板和平台梁的轮廓数据
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_top_local_shape_bounding_profile_points(self):
        """
        获取楼梯顶部局部形状包围轮廓点:顶部设置的包围框需要特别注意，容易导致左侧y值不相等
        :return:
        """
        cut_point = self.get_stair_solid_long_cut_plane_point()  # 获取剖切点
        value_x = cut_point[0]  # 获取x坐标值
        add_value = 10  # 包围框增加值
        # 获取楼梯基本数据点
        h2 = self.h2  # 获取底端板厚度
        h1 = self.h1  # 获取顶端板的厚度
        lt = self.lt_d  # 获取顶端上边长数据
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        tabu_h = self.tabu_h  # 踏步高度
        tabu_w = self.tabu_b  # 踏步宽度
        # 获取包围框坐标点
        point_1 = [0, total_l, total_h]
        point_1[0] = value_x
        point_1[1] += add_value
        point_1[2] += add_value
        point_2 = copy.deepcopy(point_1)
        point_2[2] = 0
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(lt + 2 * tabu_w / 3 + 2 * add_value)
        point_4 = copy.deepcopy(point_3)
        point_4[2] = total_h + add_value
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]  # 包围框
        cut_profile_points = self.get_stair_solid_long_cut_drawing()  # 获取楼梯实体轮廓点集
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_top_right_hole_bounding_profile_points(self):
        """
        获取楼梯顶部右侧孔洞包围框轮廓数据点
        :return:
        """
        cut_point = self.get_stair_solid_long_cut_plane_point()  # 获取剖切点
        value_x = cut_point[0]  # 获取x坐标值
        add_value = 10  # 包围框增加值
        # 获取楼梯基本数据点
        h1 = self.h1  # 获取顶端板厚度
        lt = self.lt_d  # 获取顶端上边长数据
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        tabu_h = self.tabu_h  # 踏步高度
        tabu_w = self.tabu_b  # 踏步宽度
        # 获取包围框坐标点
        point_1 = [0, total_l, total_h]
        point_1[0] = value_x
        point_1[1] += add_value
        point_1[2] += add_value
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -(h1 + 2 * add_value + tabu_h)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(lt + 2 * tabu_w / 3 + 2 * add_value)
        point_4 = copy.deepcopy(point_3)
        point_4[2] = total_h + add_value
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]  # 包围框
        cut_profile_points = self.get_stair_top_hole_cut_drawing()  # 获取楼梯实体轮廓点集
        # 获取包围框内部的点
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_solid_beam_slab_right_boundary_data(self) -> List[List[float]]:
        """
        获取楼梯实体平台梁和板右侧边界坐标点
        :return:
        """
        beam_slab_profile_points = self.get_top_beam_slab_bounding_profile_points()
        max_y = 0  # 最大的y坐标值
        for seg in beam_slab_profile_points:
            for point in seg:
                if max_y <= point[1]:
                    max_y = point[1]
        draw_points = []
        # 获取x剖切平台板最右侧两点
        for seg in beam_slab_profile_points:
            for point in seg:
                if abs(point[1] - max_y) < self.drawing_precision:
                    draw_points.append(point)
        # 排序两点
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        if len(draw_points) == 2:
            if draw_points[0][2] > draw_points[1][2]:
                top_point[0] = draw_points[0][0]
                top_point[1] = draw_points[0][1]
                top_point[2] = draw_points[0][2]
                bottom_point[0] = draw_points[1][0]
                bottom_point[1] = draw_points[1][1]
                bottom_point[2] = draw_points[1][2]
            else:
                top_point[0] = draw_points[1][0]
                top_point[1] = draw_points[1][1]
                top_point[2] = draw_points[1][2]
                bottom_point[0] = draw_points[0][0]
                bottom_point[1] = draw_points[0][1]
                bottom_point[2] = draw_points[0][2]
        else:
            print("筛选出点的数量有误!")
        final_points = [top_point, bottom_point]
        return final_points

    def get_stair_solid_local_top_left_boundary_points(self):
        """
        获取楼梯实体局部顶部左侧边缘点：绘制折断线
        :return:List[List[float]]
        """
        stair_profile_points = self.get_stair_top_local_shape_bounding_profile_points()
        min_y = self.lb_d + self.ln + self.lt_d  # 最小的y坐标值
        for seg in stair_profile_points:
            for point in seg:
                if min_y >= point[1]:
                    min_y = point[1]
        draw_points = []
        # 获取y剖切楼梯底部最右侧两点
        for seg in stair_profile_points:
            for point in seg:
                if abs(point[1] - min_y) < self.drawing_precision:
                    draw_points.append(point)
        # 排序两点
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        if len(draw_points) == 2:
            if draw_points[0][2] > draw_points[1][2]:
                top_point[0] = draw_points[0][0]
                top_point[1] = draw_points[0][1]
                top_point[2] = draw_points[0][2]
                bottom_point[0] = draw_points[1][0]
                bottom_point[1] = draw_points[1][1]
                bottom_point[2] = draw_points[1][2]
            else:
                top_point[0] = draw_points[1][0]
                top_point[1] = draw_points[1][1]
                top_point[2] = draw_points[1][2]
                bottom_point[0] = draw_points[0][0]
                bottom_point[1] = draw_points[0][1]
                bottom_point[2] = draw_points[0][2]
        else:
            print("筛选出点的数量有误!")
        final_points = [top_point, bottom_point]
        return final_points

    def get_stair_right_boundary_breakline(self):
        """
        获取楼梯左侧边界折断线
        :return:
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        left_loc = self.get_stair_solid_beam_slab_right_boundary_data()
        limit_z = (left_loc[0][2] + left_loc[1][2]) / 2

        node_data = self.bottom_node_info
        node_b = node_data.b
        top_loc = [0, 0, 0]
        bottom_loc = [0, 0, 0]
        for point in left_loc:
            if point[2] > limit_z:
                top_loc[0] = point[0]
                top_loc[1] = point[1]
                top_loc[2] = point[2] + node_b
            else:
                bottom_loc[0] = point[0]
                bottom_loc[1] = point[1]
                bottom_loc[2] = point[2]
        # 开始设置关键点
        point_1 = copy.deepcopy(top_loc)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_loc)
        point_3 = copy.deepcopy(top_loc)
        point_3[2] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_loc)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_left_boundary_breakline(self):
        """
        获取楼梯左侧边界折断线
        :return:
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        right_loc = self.get_stair_solid_local_top_left_boundary_points()
        limit_z = (right_loc[0][2] + right_loc[1][2]) / 2
        top_loc = [0, 0, 0]
        bottom_loc = [0, 0, 0]
        for point in right_loc:
            if point[2] > limit_z:
                top_loc[0] = point[0]
                top_loc[1] = point[1]
                top_loc[2] = point[2]
            else:
                bottom_loc[0] = point[0]
                bottom_loc[1] = point[1]
                bottom_loc[2] = point[2]
        # 开始设置关键点
        point_1 = copy.deepcopy(top_loc)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_loc)
        point_3 = copy.deepcopy(top_loc)
        point_3[2] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_loc)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_top_beam_slab_elevation_line_points(self):
        """
        获取标高线绘制坐标点
        :return:List[List[float]]
        """
        right_boundary_points = (
            self.get_stair_solid_beam_slab_right_boundary_data()
        )  # 获取右侧边界点
        top_point = right_boundary_points[0]
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        total_l = self.lb_d + self.ln + self.lt_d
        top_point[2] += node_b
        left_point = copy.deepcopy(top_point)
        left_point[1] = total_l + node_a
        final_points = [top_point, left_point]
        return final_points

    def get_stair_solid_elevation_special_point(self):
        """
        开始获取楼梯实体标高特殊点坐标
        :return: List[List[List[float]],List[List[float],str]]
        """
        special_length = 36  # 三角形的高
        elevation_line_points = self.get_stair_top_beam_slab_elevation_line_points()  #
        right_point = elevation_line_points[0]
        left_point = elevation_line_points[1]
        add_y = abs(right_point[1] - left_point[1])
        point_0 = [left_point[0], left_point[1] + 2 * add_y / 3, left_point[2]]
        point_1 = copy.deepcopy(point_0)
        point_1[1] += -special_length / 2
        point_1[2] += special_length
        point_2 = copy.deepcopy(point_1)
        point_2[1] += special_length
        point_3 = copy.deepcopy(point_2)
        point_3[1] += 3 * special_length
        draw_line = [
            [point_0, point_1],
            [point_1, point_2],
            [point_2, point_3],
            [point_0, point_2],
        ]
        text_loc = copy.deepcopy(point_0)
        text_loc[2] += special_length
        text = "完成面标高"
        text_info = [text_loc, text]
        final_points = [draw_line, text_info]
        return final_points

    def get_stair_top_glue_hatch_points(self):
        """
        获取楼梯顶部节点打胶填充点
        :return:
        """
        node_data = self.top_node_info
        node_a = node_data.a
        node_b = node_data.b
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        # 填充图形轮廓点
        point_0 = [0, total_l, total_h]
        point_1 = copy.deepcopy(point_0)
        point_1[1] += node_a
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_b
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -node_a
        profile_points = [point_0, point_1, point_2, point_3]
        # 引出线点
        outline_length_1 = 150
        outline_length_2 = 150
        theta = math.pi / 3
        point_s_0 = copy.deepcopy(point_0)
        point_s_0[1] += node_a / 2
        point_s_0[2] += -node_b / 5
        point_s_1 = copy.deepcopy(point_s_0)
        point_s_1[1] += outline_length_1 * math.cos(theta)
        point_s_1[2] += outline_length_1 * math.sin(theta)
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_2
        outline_points = [[point_s_0, point_s_1], [point_s_1, point_s_2]]
        text_loc = copy.deepcopy(point_s_2)
        text = "打胶" + str(node_a) + "X" + str(node_b)
        outline_text = [text_loc, text]
        # 所有信息合集
        total_info = {}
        total_info["hatch_profile"] = profile_points
        total_info["outline"] = outline_points
        total_info["outline_text"] = outline_text
        return total_info

    def get_stair_top_pe_rod_hatch_points(self):
        """
        获取楼梯顶部PE棒填充点
        :return:
        """
        edge = 4  # 边缘长度

        node_data = self.top_node_info
        node_a = node_data.a
        node_b = node_data.b
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        diameter = node_a - edge
        origin = [0, total_l, total_h]
        origin[1] += node_a / 2
        origin[2] += -(node_b + diameter / 2)
        # 引出线点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 3
        point_1 = copy.deepcopy(origin)
        point_2 = copy.deepcopy(point_1)
        point_2[1] += outline_length_1 * math.cos(theta)
        point_2[2] += outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += outline_length_2
        text_loc = copy.deepcopy(point_3)
        text = "PE棒"
        hatch_line = [origin, diameter]
        outline_points = [[point_1, point_2], [point_2, point_3]]
        outline_text = [text_loc, text]
        # 所有信息集合
        total_info = {}
        total_info["hatch_profile"] = hatch_line
        total_info["outline"] = outline_points
        total_info["outline_text"] = outline_text
        return total_info

    def get_stair_top_mid_hatch_pattern_points(self):
        """
        获取楼梯顶部中间填充图案点集合
        :return:
        """
        node_data = self.top_node_info
        node_a = node_data.a
        node_b = node_data.b
        node_c = node_data.c
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        # 获取填充轮廓点
        point_1 = [0, total_l, total_h]
        point_1[2] += -node_b
        point_2 = copy.deepcopy(point_1)
        point_2[1] += node_a
        point_3 = copy.deepcopy(point_2)
        point_3[2] += -(node_c + self.h1 - node_b)
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -node_a
        # 引出线点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 6
        point_s_1 = [0, 0, 0]
        point_s_1[1] = total_l + node_a / 2
        point_s_1[2] = (point_2[2] + point_3[2]) / 2
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        # 引出线文本注释
        text_loc = copy.deepcopy(point_s_3)
        text = "聚苯填充"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_text = [text_loc, text]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_top_slide_hatch_one_points(self):
        """
        获取楼梯顶部滑动铰支座填充1坐标点
        """
        node_data = self.top_node_info
        node_a = node_data.a
        node_c = node_data.c
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        beam_area_width = (
            self.beam_slab_info.bottom_plate.beam_area_width
        )  # 获取底部平台两部分宽度
        fact_width = beam_area_width - node_a
        # 填充点
        point_1 = [0, total_l, total_h - self.h1]
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_c / 2
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -fact_width
        point_4 = copy.deepcopy(point_3)
        point_4[2] = total_h - self.h1
        # 引出线的点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_s_1 = [0, total_l - fact_width / 4, total_h - self.h1 - node_c / 4]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += -outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        point_s_4 = [0, total_l - fact_width / 2, total_h - self.h1]
        point_s_5 = copy.deepcopy(point_s_4)
        point_s_5[1] += -outline_length_1 * math.cos(theta)
        point_s_5[2] += -outline_length_1 * math.sin(theta)
        point_s_6 = copy.deepcopy(point_s_5)
        point_s_6[1] += -outline_length_2
        # 引出线说明
        text_loc_1 = copy.deepcopy(point_s_3)
        text_1 = "M15水泥砂浆"
        text_loc_2 = copy.deepcopy(point_s_6)
        text_2 = "调平钢垫片"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points_1 = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_points_2 = [[point_s_4, point_s_5], [point_s_5, point_s_6]]
        outline_text_1 = [text_loc_1, text_1]
        outline_text_2 = [text_loc_2, text_2]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline_1"] = outline_points_1
        hatch_info["outline_text_1"] = outline_text_1
        hatch_info["outline_2"] = outline_points_2
        hatch_info["outline_text_2"] = outline_text_2
        return hatch_info

    def get_stair_top_slide_hatch_two_points(self):
        """
        获取楼梯顶部滑动铰支座填充2坐标点
        """
        node_data = self.top_node_info
        node_a = node_data.a
        node_c = node_data.c
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        beam_area_width = self.beam_slab_info.top_plate.beam_area_width  # 获取顶部平台两部分宽度
        fact_width = beam_area_width - node_a
        # 填充点
        point_1 = [0, total_l, total_h - self.h1 - node_c / 2]
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_c / 2
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -fact_width
        point_4 = copy.deepcopy(point_3)
        point_4[2] += node_c / 2
        # 引出线的点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_s_1 = [
            0,
            total_l - 2 * fact_width / 5,
            total_h - self.h1 - 3 * node_c / 4,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += -outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        # 引出线说明
        text_loc = copy.deepcopy(point_s_3)
        text = "油毡一层"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_text = [text_loc, text]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_top_fix_hatch_points(self):
        """
        获取楼梯顶部固定铰支座填充坐标点
        """
        node_data = self.top_node_info
        node_a = node_data.a
        node_c = node_data.c
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        beam_area_width = self.beam_slab_info.top_plate.beam_area_width  # 获取顶部平台两部分宽度
        fact_width = beam_area_width - node_a
        # 填充点
        point_1 = [0, total_l, total_h - self.h1]
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -node_c
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -fact_width
        point_4 = copy.deepcopy(point_3)
        point_4[2] += node_c
        # 引出线的点
        # 第一跟引出线
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_s_1 = [0, total_l - fact_width / 4, total_h - self.h1 - node_c / 2]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[2] += -outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        # 第二根引出线
        point_s_4 = [0, total_l - fact_width / 2, total_h - self.h1]
        point_s_5 = copy.deepcopy(point_s_4)
        point_s_5[1] += -outline_length_1 * math.cos(theta)
        point_s_5[2] += -outline_length_1 * math.sin(theta)
        point_s_6 = copy.deepcopy(point_s_5)
        point_s_6[1] += -outline_length_2
        # 引出线说明
        text_loc_1 = copy.deepcopy(point_s_3)
        text_1 = "M15水泥砂浆"
        text_loc_2 = copy.deepcopy(point_s_6)
        text_2 = "调平垫片"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points_1 = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_points_2 = [[point_s_4, point_s_5], [point_s_5, point_s_6]]
        outline_text_1 = [text_loc_1, text_1]
        outline_text_2 = [text_loc_2, text_2]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = hatch_points
        hatch_info["outline_1"] = outline_points_1
        hatch_info["outline_2"] = outline_points_2
        hatch_info["outline_text_1"] = outline_text_1
        hatch_info["outline_text_2"] = outline_text_2
        return hatch_info

    def get_stair_top_slide_hatch_points(self):
        """
        获取楼梯顶部滑动铰支座填轮廓点
        :return:
        """
        top_shim_loc = self.get_stair_top_shim_corner_point()  # 获取顶部垫片轮廓点
        max_y = top_shim_loc[-1][1]
        min_y = top_shim_loc[0][1]
        max_z = top_shim_loc[1][0]
        min_z = top_shim_loc[0][0]
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        hole_c1 = self.connect_hole_info["top_top_dia"]
        hole_h1 = self.connect_hole_info["top_h"]
        hole_e1 = self.connect_hole_info["top_top_dia_1"]
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        # 计算关键点
        center_point = [0, hole_loc[1], total_h]
        point_1 = copy.deepcopy(center_point)
        point_1[1] += -hole_c1 / 2
        point_2 = copy.deepcopy(center_point)
        point_2[1] += -hole_e1 / 2
        point_2[2] += -hole_h1
        point_3 = [0, min_y, min_z]
        point_4 = [0, min_y, max_z]
        point_5 = [0, max_y, max_z]
        point_6 = [0, max_y, min_z]
        point_7 = copy.deepcopy(center_point)
        point_7[0] = 0
        point_7[1] += hole_e1 / 2
        point_7[2] += -hole_h1
        point_8 = copy.deepcopy(center_point)
        point_8[1] += hole_c1 / 2
        bounding_box_loc = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        # 引出线点和说明
        outline_length_1 = 120
        outline_length_2 = 300
        point_s_1 = copy.deepcopy(center_point)
        point_s_1[2] += -hole_h1 / 2
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[2] += outline_length_1
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        # 文本位置和内容
        text_loc = copy.deepcopy(point_s_2)
        text = "砂浆封堵(平整、密实、光滑)"
        outline_text = [text_loc, text]
        # 所有填充信息
        hatch_info = {}
        hatch_info["hatch_profile"] = bounding_box_loc
        hatch_info["outline"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_construction_method_dimension_points(self):
        """
        获取楼梯构造做法尺寸标注点
        :return:
        """
        # 获取标高上的点
        elevation_points = self.get_stair_top_beam_slab_elevation_line_points()
        node_data = self.bottom_node_info
        node_a = node_data.a
        node_b = node_data.b
        node_c = node_data.c
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        point_0 = [0, total_l + node_a, total_h]
        point_1 = [0, total_l, total_h]
        point_2 = copy.deepcopy(point_0)
        point_2[2] += -node_b
        point_3 = copy.deepcopy(point_2)
        point_3[2] = total_h - self.h1
        point_4 = copy.deepcopy(point_3)
        point_4[2] += -node_c
        point_5 = copy.deepcopy(elevation_points[0])
        point_6 = copy.deepcopy(point_5)
        point_6[2] += -node_b
        dimension_points = [
            [point_0, point_1],
            [point_2, point_0],
            [point_3, point_2],
            [point_4, point_3],
            [point_6, point_5],
        ]
        return dimension_points

    def get_stair_top_anchor_rebar_outline_points(self):
        """
        获取楼梯顶端锚固钢筋引出线点
        :return:
        """
        top_rebar_diameter = (
            self.connect_element_info.top_connect_element.rebar_anchor_diameter
        )
        anchor_points = (
            self.get_stair_solid_top_anchor_rebar_cut_drawing()
        )  # 获取楼梯顶端锚固钢筋剖切数据
        special_point = [0, 0, 0]
        for seg in anchor_points:
            if len(seg) > 2:
                mid_num = int(len(seg) / 2)  # 圆弧中中部点
                current_p = seg[mid_num]
                special_point[0] = current_p[0]
                special_point[1] = current_p[1]
                special_point[2] = current_p[2]
        # 引出线绘制
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 6
        point_1 = copy.deepcopy(special_point)
        point_2 = copy.deepcopy(point_1)
        point_2[1] += outline_length_1 * math.cos(theta)
        point_2[2] += -outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += outline_length_2
        text_loc = [
            (point_2[0] + point_3[0]) / 2,
            (point_2[1] + point_3[1]) / 2,
            (point_2[2] + point_3[2]) / 2,
        ]
        text = "Φ" + str(top_rebar_diameter)
        outline_points = [[point_1, point_2], [point_2, point_3]]
        text_points = [text_loc, text]
        outline_info = {}
        outline_info["outline"] = outline_points
        outline_info["outline_text"] = text_points
        return outline_info

    def get_stair_top_beam_outline_points(self):
        """
        获取楼梯顶部梯梁引出线点集合
        :return:
        """
        beam_slab_points = self.get_top_beam_slab_bounding_profile_points()
        # 选出最小的z值
        min_z = self.h2 + self.h
        for seg in beam_slab_points:
            for point in seg:
                if min_z > point[2]:
                    min_z = point[2]
        key_point = [0, 0, 0]
        for seg in beam_slab_points:
            if seg[0][2] == min_z and seg[1][2] == min_z:
                key_point[0] = (seg[0][0] + seg[1][0]) / 2
                key_point[1] = (seg[0][1] + seg[1][1]) / 2
                key_point[2] = (seg[0][2] + seg[1][2]) / 2
        # 获取引出线点
        outline_length_1 = 120
        outline_length_2 = 120
        theta = math.pi / 4
        point_1 = copy.deepcopy(key_point)
        point_2 = copy.deepcopy(point_1)
        point_2[1] += outline_length_1 * math.cos(theta)
        point_2[2] += -outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += outline_length_2
        outline_points = [[point_1, point_2], [point_2, point_3]]
        text_loc = [
            (point_2[0] + point_3[0]) / 2,
            (point_2[1] + point_3[1]) / 2,
            (point_2[2] + point_3[2]) / 2,
        ]
        text = "梯梁"
        outline_text = [text_loc, text]
        # 合集
        outline_info = {}
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info

    def get_stair_top_slide_nut_outline_points(self):
        """
        获取楼梯顶部防滑槽螺母引出线点的注释
        :return:
        """
        nut_points = self.get_stair_top_connect_nut_projection_drawing()
        min_y = self.lb_d + self.ln + self.lt_d
        for seg in nut_points:
            for point in seg:
                if min_y > abs(point[1]):
                    min_y = abs(point[1])
        point_1 = [0, 0, 0]
        for seg in nut_points:  # 投影后的点比较特殊
            if abs(seg[0][1]) == min_y and abs(seg[1][1]) == min_y:
                point_1[0] = abs(seg[0][1] + seg[1][1]) / 2
                point_1[1] = abs(seg[0][0] + seg[1][0]) / 2
                point_1[2] = abs(seg[0][2] + seg[1][2]) / 2
        outline_length_1 = 120
        point_2 = copy.deepcopy(point_1)
        point_2[0] -= outline_length_1
        text_loc = copy.deepcopy(point_2)
        text = "螺母"
        outline_points = [[point_1, point_2]]
        outline_text = [text_loc, text]
        outline_info = {}
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info

    def get_stair_top_fix_node_top_hatch_pattern_points(self):
        """
        获取楼梯顶部固定节点顶部填充图案点
        :return:
        """
        hole_info = self.hole_info.get_hole_config()
        hole_c2 = hole_info["top_top_dia"]
        hole_d2 = hole_info["top_bottom_dia"]
        h1 = self.h1
        total_h = self.h2 + self.h
        top_connect_info = self.connect_element_info.top_connect_element
        edge_t = top_connect_info.edge_t  # 顶部边距
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        height = edge_t / 2
        special_edge = (hole_c2 - hole_d2) * (h1 - height) / h1 / 2
        curr_dia = hole_d2 + 2 * special_edge
        # 开始确定轮廓点
        point_0 = [0, hole_loc[1], total_h]
        point_1 = copy.deepcopy(point_0)
        point_1[1] += -hole_c2 / 2
        point_2 = copy.deepcopy(point_1)
        point_2[1] += hole_c2
        point_3 = copy.deepcopy(point_0)
        point_3[2] += -height
        point_3[1] += curr_dia / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -curr_dia
        # 引出线
        outline_length_1 = 60
        outline_length_2 = 120
        point_s_1 = [0, hole_loc[1], total_h - height / 2]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[2] += outline_length_1
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        text_loc = copy.deepcopy(point_s_3)
        text = "砂浆封堵(平整,密实,光滑)"
        hatch_points = [point_1, point_2, point_3, point_4]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        outline_text = [text_loc, text]
        # 信息汇总
        hatch_info = {}
        hatch_info["hatch_points"] = hatch_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_top_fix_node_mid_hatch_pattern_points(self):
        """
        获取楼梯顶端固定节点中部填充图案点集合
        :return:
        """
        top_hatch_info = self.get_stair_top_fix_node_top_hatch_pattern_points()
        top_hatch_points = top_hatch_info["hatch_points"]
        point_1 = copy.deepcopy(top_hatch_points[-2])
        point_2 = copy.deepcopy(top_hatch_points[-1])
        hole_info = self.hole_info.get_hole_config()  # 孔洞信息
        hole_c2 = hole_info["top_top_dia"]
        hole_d2 = hole_info["top_bottom_dia"]
        total_h = self.h1 + self.h
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        # 获取锚固钢筋轮廓数据
        anchor_profile_points = self.get_stair_solid_top_anchor_rebar_cut_drawing()
        max_z = 0
        for seg in anchor_profile_points:
            for point in seg:
                if max_z < point[2]:
                    max_z = point[2]
        rebar_top_point = []
        for seg in anchor_profile_points:
            if seg[0][2] == max_z and seg[1][2] == max_z:
                if seg[0][1] < seg[1][1]:
                    rebar_top_point.append(list(seg[0]))
                    rebar_top_point.append(list(seg[1]))
                else:
                    rebar_top_point.append(list(seg[1]))
                    rebar_top_point.append(list(seg[0]))
        point_3 = [0, hole_loc[1] - hole_d2 / 2, total_h - self.h1]
        point_4 = copy.deepcopy(rebar_top_point[0])
        point_4[0] = 0
        point_4[2] = total_h - self.h1
        point_5 = copy.deepcopy(rebar_top_point[0])
        point_5[0] = 0
        point_6 = copy.deepcopy(rebar_top_point[1])
        point_6[0] = 0
        point_7 = copy.deepcopy(point_6)
        point_7[2] = total_h - self.h1
        point_8 = [0, hole_loc[1] + hole_d2 / 2, total_h - self.h1]
        mid_hatch_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        # 引出线标注
        outline_length_1 = 120
        point_s_1 = [
            (point_3[0] + point_4[0]) / 2,
            (point_3[1] + point_4[1]) / 2,
            (point_4[2] + point_5[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] -= outline_length_1
        text_loc = copy.deepcopy(point_s_2)
        text = "C40级CGM灌浆料灌实"
        outline_points = [[point_s_1, point_s_2]]
        outline_text = [text_loc, text]
        # 信息汇总
        outline_info = {}
        outline_info["hatch_points"] = mid_hatch_points
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info


class StairDoubleSideWallJointLongViewData(object):
    """
    楼梯两侧墙缝节点视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.drawing_precision = 0.001
        self.rebar_for_bim = rebar_for_bim
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.ladder_double_side_joint_info = LadderDoubleJointInformation()
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        current_point = [self.b0 / 2, 0, 0]
        return current_point

    def get_stair_solid_long_cut_drawing(self):
        """
        获取楼梯纵向剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_and_ear_corner_single_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    @staticmethod
    def choose_inner_point_of_bounding_box(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    def get_stair_joint_bounding_box(self):
        """
        获取楼梯节点包围框信息
        :return:
        """
        tabu_b = self.tabu_b
        left_y = self.lb_d + tabu_b / 3
        right_y = self.lb_d + tabu_b * (2 + 2 / 3)
        max_z = self.h2 + self.h
        min_z = 0
        cut_point = self.get_stair_solid_long_cut_plane_point()
        value_x = cut_point[0]
        # 形成包围框上数据点
        point_1 = [value_x, left_y, max_z]
        point_2 = [value_x, left_y, min_z]
        point_3 = [value_x, right_y, min_z]
        point_4 = [value_x, right_y, max_z]
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_bounding_profile_points(self):
        """
        获取楼梯包围框轮廓数据点
        :return:
        """
        bounding_box_loc = self.get_stair_joint_bounding_box()  # 包围框
        cut_profile_points = self.get_stair_solid_long_cut_drawing()  # 获取楼梯实体轮廓点集
        # 获取包围框内部的点
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_left_boundary_point(self):
        """
        获取楼梯左侧边界点
        :return:
        """
        # 获取包围框数据
        min_y = self.lb_d + self.ln
        bounding_profile_points = self.get_stair_bounding_profile_points()
        for seg in bounding_profile_points:
            for point in seg:
                if min_y > point[1]:
                    min_y = point[1]
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        left_boundary_points = []
        for seg in bounding_profile_points:
            for point in seg:
                if abs(point[1] - min_y) < self.drawing_precision:
                    left_boundary_points.append(point)
        if len(left_boundary_points) == 2:
            _point_1 = left_boundary_points[0]
            _point_2 = left_boundary_points[1]
            if _point_1[2] > _point_2[2]:
                top_point[0] = _point_1[0]
                top_point[1] = _point_1[1]
                top_point[2] = _point_1[2]
                bottom_point[0] = _point_2[0]
                bottom_point[1] = _point_2[1]
                bottom_point[2] = _point_2[2]
            else:
                top_point[0] = _point_2[0]
                top_point[1] = _point_2[1]
                top_point[2] = _point_2[2]
                bottom_point[0] = _point_1[0]
                bottom_point[1] = _point_1[1]
                bottom_point[2] = _point_1[2]
        else:
            print("点的数量有误！")
        return [top_point, bottom_point]

    def get_stair_right_boundary_points(self):
        """
        获取楼梯右侧边界点
        :return:
        """
        # 获取包围框数据
        max_y = 0
        bounding_profile_points = self.get_stair_bounding_profile_points()
        for seg in bounding_profile_points:
            for point in seg:
                if max_y < point[1]:
                    max_y = point[1]
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        right_boundary_points = []
        for seg in bounding_profile_points:
            for point in seg:
                if abs(point[1] - max_y) < self.drawing_precision:
                    right_boundary_points.append(point)
        if len(right_boundary_points) == 2:
            _point_1 = right_boundary_points[0]
            _point_2 = right_boundary_points[1]
            if _point_1[2] > _point_2[2]:
                top_point[0] = _point_1[0]
                top_point[1] = _point_1[1]
                top_point[2] = _point_1[2]
                bottom_point[0] = _point_2[0]
                bottom_point[1] = _point_2[1]
                bottom_point[2] = _point_2[2]
            else:
                top_point[0] = _point_2[0]
                top_point[1] = _point_2[1]
                top_point[2] = _point_2[2]
                bottom_point[0] = _point_1[0]
                bottom_point[1] = _point_1[1]
                bottom_point[2] = _point_1[2]
        else:
            print("点的数量有误！")
        return [top_point, bottom_point]

    def get_stair_bottom_xie_boundary_points(self):
        """
        获取楼梯底部斜线轮廓
        :return:
        """
        left_boundary_points = self.get_stair_left_boundary_point()  # 左侧边界点
        right_boundary_points = self.get_stair_right_boundary_points()  # 右侧边界点
        bounding_profile_points = self.get_stair_bounding_profile_points()  # 包围框内轮廓点
        min_y = left_boundary_points[0][1]
        max_y = right_boundary_points[0][1]
        left_point = [0, 0, 0]  # 左侧点，z值偏小
        right_point = [0, 0, 0]  # 右侧点，z值偏大
        bottom_xie_points = []
        for seg in bounding_profile_points:
            if len(seg) == 2:
                _point_1 = seg[0]
                _point_2 = seg[1]
                if (
                    abs(_point_1[1] - min_y) < self.drawing_precision
                    and abs(_point_2[1] - max_y) < self.drawing_precision
                ) or (
                    abs(_point_2[1] - min_y)
                    and abs(_point_1[1] - max_y) < self.drawing_precision
                ):
                    bottom_xie_points.append(_point_1)
                    bottom_xie_points.append(_point_2)
        if bottom_xie_points[0][2] > bottom_xie_points[1][2]:
            right_point[0] = bottom_xie_points[0][0]
            right_point[1] = bottom_xie_points[0][1]
            right_point[2] = bottom_xie_points[0][2]
            left_point[0] = bottom_xie_points[1][0]
            left_point[1] = bottom_xie_points[1][1]
            left_point[2] = bottom_xie_points[1][2]
        else:
            right_point[0] = bottom_xie_points[1][0]
            right_point[1] = bottom_xie_points[1][1]
            right_point[2] = bottom_xie_points[1][2]
            left_point[0] = bottom_xie_points[0][0]
            left_point[1] = bottom_xie_points[0][1]
            left_point[2] = bottom_xie_points[0][2]
        return [right_point, left_point]

    def get_stair_top_boundary_points(self):
        """
        获取楼梯顶部边界点
        :return:
        """
        bounding_profile_points = self.get_stair_bounding_profile_points()  # 包围框内轮廓点
        bottom_xie_points = self.get_stair_bottom_xie_boundary_points()  # 获取楼梯底部斜向线上点
        limit_z = (bottom_xie_points[0][2] + bottom_xie_points[1][2]) / 2
        right_point = bottom_xie_points[0]
        top_boundary_points = []
        # 获取顶部边界线段
        for seg in bounding_profile_points:
            num = len(seg)  # 点的数量
            iters = 0
            for point in seg:
                if (
                    point[2] < limit_z
                    or abs(point[2] - right_point[2]) < self.drawing_precision
                ):
                    break
                iters += 1
            if iters == num:
                top_boundary_points.append(seg)
        return top_boundary_points

    def get_stair_left_breakline_points(self):
        """
        获取楼梯左侧折断线点集合
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        left_boundary_points = self.get_stair_left_boundary_point()
        top_point = left_boundary_points[0]
        bottom_point = left_boundary_points[1]
        # 开始设置关键点
        limit_z = (top_point[2] + bottom_point[2]) / 2
        point_1 = copy.deepcopy(top_point)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_point)
        point_3 = copy.deepcopy(top_point)
        point_3[2] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_point)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_right_breakline_points(self):
        """
        获取楼梯右侧折断线点集合
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        right_boundary_points = self.get_stair_right_boundary_points()
        top_point = right_boundary_points[0]
        bottom_point = right_boundary_points[1]
        # 开始设置关键点
        limit_z = (top_point[2] + bottom_point[2]) / 2
        point_1 = copy.deepcopy(top_point)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_point)
        point_3 = copy.deepcopy(top_point)
        point_3[2] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_point)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_top_edge_hatch_profile_points(self):
        """
        获取楼梯顶部边缘填充点
        :return:
        """
        bounding_profile_points = self.get_stair_top_boundary_points()  # 获取楼梯包围轮廓点
        left_boundary_points = self.get_stair_left_boundary_point()  # 获取楼梯左侧边界点
        left_top_point = copy.deepcopy(left_boundary_points[0])  # 获取左侧边界顶部点
        right_boundary_points = self.get_stair_right_boundary_points()  # 获取楼梯右侧边界点
        right_top_point = right_boundary_points[0]  # 获取楼梯右侧顶部点
        top_edge_thick = self.ladder_double_side_joint_info.top_edge_thick  # 顶部边界厚度
        top_mid_thick = self.ladder_double_side_joint_info.top_mid_thick  # 顶部中间厚度
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        add_y = abs(left_top_point[1] - self.lb_d)
        # 填充边缘点
        point_1 = copy.deepcopy(left_top_point)
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -top_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[1] += tabu_b - add_y + top_edge_thick
        point_4 = copy.deepcopy(point_3)
        point_4[2] += tabu_h
        point_5 = copy.deepcopy(point_4)
        point_5[1] += tabu_b
        point_6 = copy.deepcopy(point_5)
        point_6[2] += tabu_h
        point_7 = copy.deepcopy(point_6)
        point_7[1] = right_top_point[1]
        point_8 = copy.deepcopy(right_top_point)
        bounding_profile_points.append([tuple(point_1), tuple(point_2)])
        bounding_profile_points.append([tuple(point_2), tuple(point_3)])
        bounding_profile_points.append([tuple(point_3), tuple(point_4)])
        bounding_profile_points.append([tuple(point_4), tuple(point_5)])
        bounding_profile_points.append([tuple(point_5), tuple(point_6)])
        bounding_profile_points.append([tuple(point_6), tuple(point_7)])
        bounding_profile_points.append([tuple(point_7), tuple(point_8)])
        # 引出线
        outline_length_1 = 50
        outline_length_2 = 150
        theta = math.pi / 4
        point_s_1 = [
            (point_4[0] + point_5[0]) / 2,
            (point_4[1] + point_5[1]) / 2,
            (point_4[2] + point_5[2] + top_edge_thick) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_1 * math.cos(theta)
        point_s_2[2] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "耐候性密封胶"
        outline_text = [text_loc, text]
        # 信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = bounding_profile_points  # 线段填充
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_top_mid_hatch_profile_points(self):
        """
        获取楼梯顶部中间填充点
        :return:
        """
        left_boundary_points = self.get_stair_left_boundary_point()  # 获取楼梯左侧边界点
        left_top_point = copy.deepcopy(left_boundary_points[0])  # 获取左侧边界顶部点
        right_boundary_points = self.get_stair_right_boundary_points()  # 获取楼梯右侧边界点
        right_top_point = right_boundary_points[1]  # 获取楼梯右侧顶部点
        top_edge_thick = self.ladder_double_side_joint_info.top_edge_thick  # 顶部边界厚度
        top_mid_thick = self.ladder_double_side_joint_info.top_mid_thick  # 顶部中间厚度
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        add_y = abs(left_top_point[1] - self.lb_d)
        # 填充边缘点
        point_1 = copy.deepcopy(left_top_point)
        point_1[2] += -top_edge_thick
        point_2 = copy.deepcopy(point_1)
        point_2[1] += tabu_b - add_y + top_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[2] += tabu_h
        point_4 = copy.deepcopy(point_3)
        point_4[1] += tabu_b
        point_5 = copy.deepcopy(point_4)
        point_5[2] += tabu_h
        point_6 = copy.deepcopy(point_5)
        point_6[1] = right_top_point[1]
        point_7 = copy.deepcopy(point_6)
        point_7[2] += -top_mid_thick
        point_8 = copy.deepcopy(point_5)
        point_8[1] += top_mid_thick
        point_8[2] += -top_mid_thick
        point_9 = copy.deepcopy(point_8)
        point_9[2] += -tabu_h
        point_10 = copy.deepcopy(point_3)
        point_10[1] += top_mid_thick
        point_10[2] += -top_mid_thick
        point_11 = copy.deepcopy(point_10)
        point_11[2] += -tabu_h
        point_12 = copy.deepcopy(point_1)
        point_12[2] += -top_mid_thick
        profile_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
            point_9,
            point_10,
            point_11,
            point_12,
        ]
        # 引出线
        outline_length_1 = 100
        outline_length_2 = 150
        theta = math.pi / 4
        point_s_1 = [
            (point_10[0] + point_11[0]) / 2,
            (point_10[1] + point_11[1] - top_mid_thick) / 2,
            (point_10[2] + point_11[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_1 * math.cos(theta)
        point_s_2[2] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "PE棒Φ" + str(top_mid_thick)
        outline_text = [text_loc, text]
        # 信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_mid_hatch_profile_points(self):
        """
        获取楼梯中部填充轮廓点
        :return:
        """
        top_mid_info = self.get_stair_top_mid_hatch_profile_points()  # 获取顶端中部轮廓点
        top_mid_points = top_mid_info["profile_points"]
        bottom_mid_thick = self.ladder_double_side_joint_info.bottom_mid_thick
        bottom_edge_thick = self.ladder_double_side_joint_info.bottom_edge_thick
        bottom_boundary_points = (
            self.get_stair_bottom_xie_boundary_points()
        )  # 获取楼梯底部斜向点
        # 轮廓点集
        point_1 = copy.deepcopy(top_mid_points[-1])
        point_2 = copy.deepcopy(top_mid_points[-2])
        point_3 = copy.deepcopy(top_mid_points[-3])
        point_4 = copy.deepcopy(top_mid_points[-4])
        point_5 = copy.deepcopy(top_mid_points[-5])
        point_6 = copy.deepcopy(top_mid_points[-6])
        right_breakline_points = self.get_stair_right_breakline_points()
        left_breakline_points = self.get_stair_left_breakline_points()
        point_7 = copy.deepcopy(right_breakline_points[2])
        point_8 = copy.deepcopy(right_breakline_points[3])
        point_9 = copy.deepcopy(right_breakline_points[4])
        point_10 = copy.deepcopy(right_breakline_points[5])
        point_11 = copy.deepcopy(bottom_boundary_points[0])
        point_11[2] += bottom_edge_thick + bottom_mid_thick
        point_12 = copy.deepcopy(bottom_boundary_points[1])
        point_12[2] += bottom_edge_thick + bottom_mid_thick
        point_13 = copy.deepcopy(left_breakline_points[-3])
        point_14 = copy.deepcopy(left_breakline_points[-4])
        point_15 = copy.deepcopy(left_breakline_points[-5])
        point_16 = copy.deepcopy(left_breakline_points[-6])
        profile_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
            point_9,
            point_10,
            point_11,
            point_12,
            point_13,
            point_14,
            point_15,
            point_16,
        ]
        # 引出线
        outline_length_1 = 200
        outline_length_2 = 100
        theta = math.pi / 4
        point_s_1 = [
            (point_4[0] + point_5[0]) / 2,
            (point_4[1] + point_5[1] + bottom_mid_thick + bottom_edge_thick) / 2,
            (point_4[2] + point_5[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_1 * math.cos(theta)
        point_s_2[2] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "防火岩棉"
        outline_text = [text_loc, text]
        # 信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_bottom_mid_hatch_profile_points(self):
        """
        获取楼梯底部中间填充轮廓点
        :return:
        """
        bottom_mid_thick = self.ladder_double_side_joint_info.bottom_mid_thick
        bottom_edge_thick = self.ladder_double_side_joint_info.bottom_edge_thick
        bottom_boundary_points = (
            self.get_stair_bottom_xie_boundary_points()
        )  # 获取楼梯底部斜向点
        # 开始设置点
        point_1 = copy.deepcopy(bottom_boundary_points[0])
        point_1[2] += bottom_edge_thick + bottom_mid_thick
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -bottom_mid_thick
        point_3 = copy.deepcopy(bottom_boundary_points[1])
        point_3[2] += bottom_edge_thick
        point_4 = copy.deepcopy(point_3)
        point_4[2] += bottom_mid_thick
        profile_points = [point_1, point_2, point_3, point_4]
        return profile_points

    def get_stair_bottom_edge_hatch_profile_points(self):
        """
        获取楼梯底部边缘填充轮廓点
        :return:
        """
        bottom_edge_thick = self.ladder_double_side_joint_info.bottom_edge_thick
        bottom_boundary_points = (
            self.get_stair_bottom_xie_boundary_points()
        )  # 获取楼梯底部斜向点
        # 开始设置点
        point_1 = copy.deepcopy(bottom_boundary_points[0])
        point_1[2] += bottom_edge_thick
        point_2 = copy.deepcopy(point_1)
        point_2[2] += -bottom_edge_thick
        point_3 = copy.deepcopy(bottom_boundary_points[1])
        point_4 = copy.deepcopy(point_3)
        point_4[2] += bottom_edge_thick
        profile_points = [point_1, point_2, point_3, point_4]
        return profile_points


class StairDoubleSideWallJointTransverseViewData(object):
    """
    楼梯两侧墙缝节点横向视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.ladder_double_side_joint_info = LadderDoubleJointInformation()
        self.drawing_precision = 0.001
        self.hatch_offset_value = 5  # spline偏移值
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    def get_stair_basic_data(self):
        """
        获取楼梯基本参数
        :return:
        """
        tabu_b = self.tabu_b
        tabu_length = self.b0 / 3
        wall_thick = 2 * tabu_b / 3
        wall_length = self.b0 / 2
        # 信息汇总
        info = {}
        info["tabu_width"] = tabu_b
        info["tabu_length"] = tabu_length
        info["wall_thick"] = wall_thick
        info["wall_length"] = wall_length
        return info

    def get_stair_wall_profile_points(self):
        """
        获取楼梯墙轮廓点
        :return:
        """
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        # 点
        point_1 = [0, 0, 0]
        point_2 = [0, wall_length, 0]
        point_3 = [wall_thick, 0, 0]
        point_4 = [wall_thick, wall_length, 0]
        profile_points = [[point_2, point_1], [point_4, point_3]]
        # 引出线
        outline_length_1 = 200
        outline_length_2 = 100
        theta = math.pi / 4
        point_s_1 = [
            (point_2[0] + point_4[0]) / 2,
            (point_2[1] + point_4[1] - wall_edge_thick) / 2,
            (point_2[2] + point_4[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += -outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "楼梯两侧墙或梁"
        outline_text = [text_loc, text]
        # 信息汇总
        shape_info = {}
        shape_info["profile_points"] = profile_points
        shape_info["outline_points"] = outline_points
        shape_info["outline_text"] = outline_text
        return shape_info

    def get_stair_tabu_profile_points(self):
        """
        获取楼梯踏步轮廓点
        :return:
        """
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        tabu_l = basic_info["tabu_length"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        # 关键点
        point_1 = [wall_thick + wall_edge_thick, wall_length / 2 + tabu_b / 2, 0]
        point_2 = copy.deepcopy(point_1)
        point_2[0] += tabu_l
        point_3 = copy.deepcopy(point_1)
        point_3[1] += -tabu_b
        point_4 = copy.deepcopy(point_3)
        point_4[0] += tabu_l
        profile_points = [[point_1, point_2], [point_3, point_4]]
        return profile_points

    def get_stair_bottom_breakline_points(self):
        """
        获取楼梯底部折断线点
        :return:
        """
        # 折断线基本数据
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        # 获取基本数据
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        tabu_l = basic_info["tabu_length"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        left_point = [0, 0, 0]
        right_point = [wall_thick + wall_edge_thick + tabu_l, 0, 0]
        # 开始设置关键点
        limit_x = (left_point[0] + right_point[0]) / 2
        point_1 = copy.deepcopy(left_point)
        point_1[0] += -extend_l
        point_2 = copy.deepcopy(left_point)
        point_3 = copy.deepcopy(left_point)
        point_3[0] = limit_x - l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[0] += offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[0] += l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[0] += l_space
        point_7 = copy.deepcopy(right_point)
        point_8 = copy.deepcopy(point_7)
        point_8[0] += extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_top_breakline_points(self):
        """
        获取楼梯顶部折断线点
        :return:
        """
        # 折断线基本数据
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        # 获取基本数据
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_l = basic_info["tabu_length"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        left_point = [0, wall_length, 0]
        right_point = [wall_thick + wall_edge_thick + tabu_l, wall_length, 0]
        # 开始设置关键点
        limit_x = (left_point[0] + right_point[0]) / 2
        point_1 = copy.deepcopy(left_point)
        point_1[0] += -extend_l
        point_2 = copy.deepcopy(left_point)
        point_3 = copy.deepcopy(left_point)
        point_3[0] = limit_x - l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += -offset * math.cos(theta)
        point_4[0] += offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[1] += offset * math.cos(theta)
        point_5[0] += l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[0] += l_space
        point_7 = copy.deepcopy(right_point)
        point_8 = copy.deepcopy(point_7)
        point_8[0] += extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_right_breakline_points(self):
        """
        获取楼梯右侧折断线点
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_l = basic_info["tabu_length"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        top_loc = [wall_thick + wall_edge_thick + tabu_l, wall_length, 0]
        bottom_loc = [wall_thick + wall_edge_thick + tabu_l, 0, 0]
        limit_z = (top_loc[1] + bottom_loc[1]) / 2
        # 开始设置关键点
        point_1 = copy.deepcopy(top_loc)
        point_1[1] += extend_l
        point_2 = copy.deepcopy(top_loc)
        point_3 = copy.deepcopy(top_loc)
        point_3[1] = limit_z + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -offset * math.cos(theta)
        point_4[1] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[0] += offset * math.cos(theta)
        point_5[1] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[1] += -l_space
        point_7 = copy.deepcopy(bottom_loc)
        point_8 = copy.deepcopy(point_7)
        point_8[1] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_dimension_points(self):
        """
        获取楼梯标注点
        :return:
        """
        # 获取基本信息
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        point_1 = [wall_thick, wall_length / 2 - tabu_b / 2, 0]
        point_2 = copy.deepcopy(point_1)
        point_2[0] += wall_edge_thick
        dimension_loc = [[point_1, point_2]]
        return dimension_loc

    def get_stair_tabu_hatch_profile_points(self):
        """
        获取楼梯踏步填充轮廓点
        :return:
        """
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        edge_points = self.get_stair_tabu_profile_points()  # 获取楼梯边缘点
        point_1 = copy.deepcopy(edge_points[0][0])
        point_2 = copy.deepcopy(edge_points[0][1])
        right_breakline_points = self.get_stair_right_breakline_points()
        point_3 = copy.deepcopy(right_breakline_points[2])
        point_4 = copy.deepcopy(right_breakline_points[3])
        point_5 = copy.deepcopy(right_breakline_points[4])
        point_6 = copy.deepcopy(right_breakline_points[5])
        point_7 = copy.deepcopy(edge_points[1][1])
        point_8 = copy.deepcopy(edge_points[1][0])
        hatch_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        # 引出线点
        outline_length_1 = 100
        outline_length_2 = 100
        theta = math.pi / 3
        point_s_1 = [
            (point_1[0] + point_2[0]) / 2,
            (point_1[1] + point_2[1] - wall_edge_thick) / 2,
            (point_1[2] + point_2[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "预制楼梯"
        outline_text = [text_loc, text]
        # 填充信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = hatch_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_joint_top_edge_hatch_points(self):
        """`
        获取楼梯顶部填充点:轮廓点为多端线，三点的为spline线
        :return:
        """
        # 获取基本信息
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        top_edge_thick = self.ladder_double_side_joint_info.top_edge_thick
        top_mid_thick = self.ladder_double_side_joint_info.top_mid_thick
        # 获取点
        point_0 = [wall_thick, wall_length / 2 + tabu_b / 2, 0]
        point_1 = copy.deepcopy(point_0)
        point_1[0] += wall_edge_thick / 2
        point_1[1] += -self.hatch_offset_value
        point_2 = copy.deepcopy(point_0)
        point_2[0] += wall_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(top_edge_thick + self.hatch_offset_value)
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -wall_edge_thick / 2
        point_4[1] += self.hatch_offset_value
        point_5 = copy.deepcopy(point_3)
        point_5[0] += -wall_edge_thick
        profile_points = [
            [point_0, point_1, point_2],
            [point_2, point_3],
            [point_3, point_4, point_5],
            [point_5, point_0],
        ]
        # 引出线
        outline_length_1 = 300
        outline_length_2 = 100
        theta = math.pi / 6
        point_s_1 = copy.deepcopy(point_1)
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += -outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "耐候性密封胶"
        outline_text = [text_loc, text]
        # 填充信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_joint_top_mid_hatch_points(self):
        """
        开始楼梯节点顶部中间填充点
        :return:
        """
        # 获取基本信息
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        top_edge_thick = self.ladder_double_side_joint_info.top_edge_thick
        top_mid_thick = self.ladder_double_side_joint_info.top_mid_thick
        # 获取点
        point_0 = [
            wall_thick,
            wall_length / 2 + tabu_b / 2 - top_edge_thick - self.hatch_offset_value,
            0,
        ]
        point_1 = copy.deepcopy(point_0)
        point_1[0] += wall_edge_thick / 2
        point_1[1] += self.hatch_offset_value
        point_2 = copy.deepcopy(point_0)
        point_2[0] += wall_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(top_mid_thick - 2 * self.hatch_offset_value)
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -wall_edge_thick / 2
        point_4[1] += -self.hatch_offset_value
        point_5 = copy.deepcopy(point_3)
        point_5[0] += -wall_edge_thick
        profile_points = [
            [point_0, point_1, point_2],
            [point_2, point_3],
            [point_3, point_4, point_5],
            [point_5, point_0],
        ]
        # 引出线
        outline_length_1 = 300
        outline_length_2 = 100
        theta = math.pi / 7
        point_s_1 = [
            (point_1[0] + point_4[0]) / 2,
            (point_1[1] + point_4[1]) / 2,
            (point_1[2] + point_4[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += -outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "PE棒Φ25"
        outline_text = [text_loc, text]
        # 填充信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_mid_joint_hatch_points(self):
        """
        获取楼梯中部节点填充点
        :return:
        """
        # 获取基本信息
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        top_edge_thick = self.ladder_double_side_joint_info.top_edge_thick
        top_mid_thick = self.ladder_double_side_joint_info.top_mid_thick
        bottom_edge_thick = self.ladder_double_side_joint_info.bottom_edge_thick
        bottom_mid_thick = self.ladder_double_side_joint_info.bottom_mid_thick
        extra_b = (
            tabu_b
            - (top_edge_thick + top_mid_thick + bottom_mid_thick + bottom_edge_thick)
            + 2 * self.hatch_offset_value
        )
        # 获取点
        point_0 = [
            wall_thick,
            wall_length / 2
            + tabu_b / 2
            - top_edge_thick
            - top_mid_thick
            + self.hatch_offset_value,
            0,
        ]
        point_1 = copy.deepcopy(point_0)
        point_1[0] += wall_edge_thick / 2
        point_1[1] += -self.hatch_offset_value
        point_2 = copy.deepcopy(point_0)
        point_2[0] += wall_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -extra_b
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -wall_edge_thick / 2
        point_4[1] += self.hatch_offset_value
        point_5 = copy.deepcopy(point_3)
        point_5[0] += -wall_edge_thick
        profile_points = [
            [point_0, point_1, point_2],
            [point_2, point_3],
            [point_3, point_4, point_5],
            [point_5, point_0],
        ]
        # 引出线
        outline_length_1 = 300
        outline_length_2 = 100
        theta = math.pi / 7
        point_s_1 = [
            (point_1[0] + point_4[0]) / 2,
            (point_1[1] + point_4[1]) / 2,
            (point_1[2] + point_4[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += -outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "防火岩棉"
        outline_text = [text_loc, text]
        # 填充信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_joint_bottom_mid_hatch_points(self):
        """
        获取楼梯节点底部中间填充点
        :return:
        """
        # 获取基本信息
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        bottom_edge_thick = self.ladder_double_side_joint_info.bottom_edge_thick
        bottom_mid_thick = self.ladder_double_side_joint_info.bottom_mid_thick
        # 获取点
        point_0 = [
            wall_thick,
            wall_length / 2
            - tabu_b / 2
            + bottom_edge_thick
            + bottom_mid_thick
            - self.hatch_offset_value,
            0,
        ]
        point_1 = copy.deepcopy(point_0)
        point_1[0] += wall_edge_thick / 2
        point_1[1] += self.hatch_offset_value
        point_2 = copy.deepcopy(point_0)
        point_2[0] += wall_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(bottom_mid_thick - 2 * self.hatch_offset_value)
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -wall_edge_thick / 2
        point_4[1] += -self.hatch_offset_value
        point_5 = copy.deepcopy(point_3)
        point_5[0] += -wall_edge_thick
        profile_points = [
            [point_0, point_1, point_2],
            [point_2, point_3],
            [point_3, point_4, point_5],
            [point_5, point_0],
        ]
        # 引出线
        outline_length_1 = 300
        outline_length_2 = 100
        theta = math.pi / 6
        point_s_1 = [
            (point_1[0] + point_4[0]) / 2,
            (point_1[1] + point_4[1]) / 2,
            (point_1[2] + point_4[2]) / 2,
        ]
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += -outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "PE棒Φ25"
        outline_text = [text_loc, text]
        # 填充信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_joint_bottom_edge_hatch_points(self):
        """
        获取楼梯底部边缘填充点
        :return:
        """
        # 获取基本信息
        basic_info = self.get_stair_basic_data()  # 获取楼梯基本数据信息
        wall_thick = basic_info["wall_thick"]
        wall_length = basic_info["wall_length"]
        tabu_b = basic_info["tabu_width"]
        wall_edge_thick = self.ladder_double_side_joint_info.wall_edge_thick
        bottom_edge_thick = self.ladder_double_side_joint_info.bottom_edge_thick
        bottom_mid_thick = self.ladder_double_side_joint_info.bottom_mid_thick
        # 获取点
        point_0 = [
            wall_thick,
            wall_length / 2 - tabu_b / 2 + bottom_edge_thick + self.hatch_offset_value,
            0,
        ]
        point_1 = copy.deepcopy(point_0)
        point_1[0] += wall_edge_thick / 2
        point_1[1] += -self.hatch_offset_value
        point_2 = copy.deepcopy(point_0)
        point_2[0] += wall_edge_thick
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -(bottom_edge_thick + self.hatch_offset_value)
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -wall_edge_thick / 2
        point_4[1] += self.hatch_offset_value
        point_5 = copy.deepcopy(point_3)
        point_5[0] += -wall_edge_thick
        profile_points = [
            [point_0, point_1, point_2],
            [point_2, point_3],
            [point_3, point_4, point_5],
            [point_5, point_0],
        ]
        # 引出线
        outline_length_1 = 300
        outline_length_2 = 100
        theta = math.pi / 7
        point_s_1 = copy.deepcopy(point_1)
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[0] += -outline_length_1 * math.cos(theta)
        point_s_2[1] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[0] += -outline_length_2
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        text_loc = copy.deepcopy(point_s_3)
        text = "耐候性密封胶"
        outline_text = [text_loc, text]
        # 填充信息汇总
        hatch_info = {}
        hatch_info["profile_points"] = profile_points
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info


class StairBottomHoleReinRebarViewData(object):
    """
    获取楼梯底部孔洞加强筋视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.hole_loc_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.hole_rein_rebar_model = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.drawing_precision = 0.001
        self.hatch_offset_value = 5  # spline偏移值
        self.theta = math.pi / 6  # 折断线倾斜角度
        self.rebar_config = RebarConfig()
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        hole_loc = self.hole_loc_info.get_hole_loc()
        limit_x = self.b0 / 2
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2
        current_point = [0, 0, 0]
        for point in hole_loc:
            if point.x < limit_x and point.y < limit_y:
                current_point[0] = point.x
                current_point[1] = point.y
                current_point[2] = point.z
        return current_point

    def get_stair_solid_long_cut_drawing(self):
        """
        获取楼梯纵向剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_and_ear_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_hole_cut_drawing(self):
        """
        获取楼梯孔洞剖切数据：形成孔洞剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_all_hole_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    @staticmethod
    def choose_inner_point_of_bounding_box(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    def get_stair_hole_rein_rebar_cut_point(self):
        """
        获取楼梯孔洞加强筋剖切点
        :return:
        """
        hole_rebar_loc = self.hole_rein_rebar_model.get_rebar_model()
        limit_x = self.b0 / 2
        limit_z = self.h2 / 2
        current_point = [0, 0, 0]
        for rebar in hole_rebar_loc:
            key_point = rebar[0]
            if key_point.x < limit_x and key_point.z < limit_z:
                current_point[0] = key_point.x
                current_point[1] = key_point.y
                current_point[2] = key_point.z
        return current_point

    def get_stair_hole_rein_rebar_profile_points(self):
        """
        获取孔洞加强筋剖切轮廓点
        :return:
        """
        current_point = self.get_stair_hole_rein_rebar_cut_point()  # 剖切点
        solid_model = self.composite_model.get_hole_rein_rebar_model()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 0, 1),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_hole_rein_rebar_bounding_box_loc(self):
        """
        获取楼梯孔洞加强筋包围框坐标点
        :return:
        """
        current_point = self.get_stair_hole_rein_rebar_cut_point()
        point_1 = [0, 0, current_point[2]]
        point_2 = [self.b0 / 2, 0, current_point[2]]
        point_3 = [self.b0 / 2, self.lb_d, current_point[2]]
        point_4 = [0, self.lb_d, current_point[2]]
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_hole_rein_rebar_bounding_points(self):
        """
        获取楼梯孔洞加强筋包围框内的点
        :return:
        """
        bounding_box_loc = self.get_stair_hole_rein_rebar_bounding_box_loc()
        cut_profile_points = self.get_stair_hole_rein_rebar_profile_points()
        bounding_box_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_box_points

    def get_stair_bottom_left_hole_bounding_points(self):
        """
        获取楼梯底部左侧孔洞包围框
        :return:
        """
        hole_loc = self.get_stair_solid_long_cut_plane_point()  # 获取楼梯孔洞剖切点
        point_1 = [hole_loc[0], 0, -self.h2]
        point_2 = [hole_loc[0], 0, self.h2 + self.h]
        point_3 = [hole_loc[0], self.lb_d, self.h2 + self.h]
        point_4 = [hole_loc[0], self.lb_d, -self.h2]
        bounding_box_loc = [point_1, point_2, point_3, point_4]
        return bounding_box_loc

    def get_stair_bottom_left_hole_bounding_profile_points(self):
        """
        获取楼梯底端左侧孔洞包围框内轮廓点
        :return:
        """
        bounding_box_loc = self.get_stair_bottom_left_hole_bounding_points()  # 包围框
        hole_profile_points = self.get_stair_solid_hole_cut_drawing()  # 开始获取孔洞轮廓点
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, hole_profile_points
        )
        return bounding_profile_points

    def get_stair_bottom_local_profile_bounding_box(self):
        """
        获取楼梯底端局部轮廓包围框
        :return:
        """
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        tabu_h = self.tabu_h  # 踏步高度
        add_z = 20
        add_y = 20
        height = self.h2 + 2 * tabu_h / 3 + add_z
        loc_y = height * math.tan(self.theta) + self.lb_d
        point_1 = [hole_loc[0], -add_y, -add_z]
        point_2 = [hole_loc[0], loc_y, -add_z]
        point_3 = [
            hole_loc[0],
            self.lb_d - add_y * math.sin(self.theta),
            height - add_z + add_y * math.cos(self.theta),
        ]
        point_4 = [hole_loc[0], -add_y, height - add_z + add_y * math.cos(self.theta)]
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_bottom_local_bounding_profile_points(self):
        """
        获取楼梯底端局部包围轮廓点
        :return:
        """
        bounding_box_loc = self.get_stair_bottom_local_profile_bounding_box()
        cut_profile_points = self.get_stair_solid_long_cut_drawing()
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_bottom_left_boundary_points(self):
        """
        获取楼梯底端左侧折断线点
        :return:
        """
        profile_points = (
            self.get_stair_bottom_local_bounding_profile_points()
        )  # 获取纵向切割图形底部包围框内的点
        max_y = 0
        max_z = 0
        for seg in profile_points:
            for point in seg:
                if max_y < point[1]:
                    max_y = point[1]
                if max_z < point[2]:
                    max_z = point[2]
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        for seg in profile_points:
            for point in seg:
                if abs(max_y - point[1]) < self.drawing_precision:
                    bottom_point[0] = point[0]
                    bottom_point[1] = point[1]
                    bottom_point[2] = point[2]
                if abs(max_z - point[2]) < self.drawing_precision:
                    top_point[0] = point[0]
                    top_point[1] = point[1]
                    top_point[2] = point[2]
        draw_points = [top_point, bottom_point]
        return draw_points

    def get_stair_bottom_left_breakline_points(self):
        """
        获取楼梯底部左侧折断线点
        :return:
        """
        # 折断线特殊形状
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        special_points = self.get_stair_bottom_left_boundary_points()  # 获取楼梯底部边界特殊点
        top_point = special_points[0]  # 顶部点
        bottom_point = special_points[1]  # 底部点
        direction = np.array(
            np.array(bottom_point) - np.array(top_point)
        ) / np.linalg.norm(np.array(bottom_point) - np.array(top_point))
        base_dir = np.cross(direction, np.array([-1, 0, 0]))
        base_length_1 = offset * math.sin(theta)
        base_length_2 = offset * math.cos(theta)
        # 开始计算关键点
        point_2 = copy.deepcopy(top_point)
        point_1 = copy.deepcopy(point_2)
        _point_1 = np.array(point_1) + (-1) * direction * extend_l
        point_1 = _point_1.tolist()
        point_mid = [
            (top_point[0] + bottom_point[0]) / 2,
            (top_point[1] + bottom_point[1]) / 2,
            (top_point[2] + bottom_point[2]) / 2,
        ]
        point_mid_t = np.array(point_mid) + (-1) * direction * l_space / 2
        point_3 = point_mid_t.tolist()
        _point_4 = (
            np.array(point_3) + base_length_1 * direction + base_length_2 * base_dir
        )
        point_4 = _point_4.tolist()
        point_mid_b = np.array(point_mid) + direction * l_space / 2
        point_6 = point_mid_b.tolist()
        _point_5 = (
            np.array(point_6)
            + (-1) * direction * base_length_1
            + (-1) * base_dir * base_length_2
        )
        point_5 = _point_5.tolist()
        point_7 = copy.deepcopy(bottom_point)
        _point_8 = np.array(point_7) + direction * extend_l
        point_8 = _point_8.tolist()
        draw_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_points

    def get_stair_hole_rein_rebar_left_projection(self):
        """
        获取孔洞加强筋侧视图:矩形形状
        :return:List[List[List[List[float],List[float]]]]
        """
        rein_rebar_loc = self.hole_rein_rebar_model.get_rebar_model()  # 获取孔洞加强筋数据
        diam = self.hole_rein_rebar_model.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2
        limit_y = (self.ln + self.lb_d + self.lt_d) / 2
        left_loc = []  # 左侧孔洞位置
        for num in range(len(rein_rebar_loc)):
            current_rebar = rein_rebar_loc[num]  # 当前钢筋
            current_point = current_rebar[0]  # 当前点
            if current_point.x < limit_x and current_point.y < limit_y:
                left_loc.append(current_rebar)
        # 获取每根钢筋的角点
        total_vertex = []
        for num in range(len(left_loc)):
            current_rebar = left_loc[num]
            rebar_loc = get_U_rein_rebar_vertex(current_rebar, diam, limit_y)
            total_vertex.append(rebar_loc)
        return total_vertex

    def get_stair_slide_hole_edge_segment_points(self):
        """
        获取楼梯滑动支座孔洞边缘线段点：顶部、中部左侧、中部右侧、底部
        :return:
        """
        hole_profile_loc = self.get_stair_bottom_left_hole_bounding_profile_points()
        top_seg = []  # 顶端线段
        for seg in hole_profile_loc:
            if (
                abs(seg[0][2] - self.h2) < self.drawing_precision
                and abs(seg[1][2] - self.h2) < self.drawing_precision
            ):
                top_seg.append(list(seg[0]))
                top_seg.append(list(seg[1]))
        # 筛选出中间点
        mid_seg = []  # 中部线段
        for seg in hole_profile_loc:
            if (seg[0][2] > self.h2 / 2 and seg[0][2] < (self.h2 - 2)) and (
                seg[1][2] > self.h2 / 2 and seg[1][2] < (self.h2 - 2)
            ):
                if seg[0][1] > seg[1][1]:
                    mid_seg.append([seg[0], seg[1]])
                else:
                    mid_seg.append([seg[0], seg[1]])
        if mid_seg[0][0][1] < mid_seg[1][0][1]:
            _seg = mid_seg[0]
            mid_seg[0] = mid_seg[1]
            mid_seg[1] = _seg
        # 中部左侧线段和中部右侧线段
        mid_left_seg = mid_seg[0]
        mid_right_seg = mid_seg[1]
        mid_left_point_1 = [0, 0, 0]  # 中部左侧线段左侧点
        mid_left_point_2 = [0, 0, 0]  # 中部左侧线段右侧点
        mid_right_point_1 = [0, 0, 0]  # 中部右侧线段左侧点
        mid_right_point_2 = [0, 0, 0]  # 中部右侧线段右侧点
        # 筛选中部左侧线段
        if mid_left_seg[0][1] > mid_left_seg[1][1]:
            mid_left_point_1[0] = mid_left_seg[0][0]
            mid_left_point_1[1] = mid_left_seg[0][1]
            mid_left_point_1[2] = mid_left_seg[0][2]
            mid_left_point_2[0] = mid_left_seg[1][0]
            mid_left_point_2[1] = mid_left_seg[1][1]
            mid_left_point_2[2] = mid_left_seg[1][2]
        else:
            mid_left_point_1[0] = mid_left_seg[1][0]
            mid_left_point_1[1] = mid_left_seg[1][1]
            mid_left_point_1[2] = mid_left_seg[1][2]
            mid_left_point_2[0] = mid_left_seg[0][0]
            mid_left_point_2[1] = mid_left_seg[0][1]
            mid_left_point_2[2] = mid_left_seg[0][2]
        # 筛选中部右侧线段
        if mid_right_seg[0][1] > mid_right_seg[1][1]:
            mid_right_point_1[0] = mid_right_seg[0][0]
            mid_right_point_1[1] = mid_right_seg[0][1]
            mid_right_point_1[2] = mid_right_seg[0][2]
            mid_right_point_2[0] = mid_right_seg[1][0]
            mid_right_point_2[1] = mid_right_seg[1][1]
            mid_right_point_2[2] = mid_right_seg[1][2]
        else:
            mid_right_point_1[0] = mid_right_seg[1][0]
            mid_right_point_1[1] = mid_right_seg[1][1]
            mid_right_point_1[2] = mid_right_seg[1][2]
            mid_right_point_2[0] = mid_right_seg[0][0]
            mid_right_point_2[1] = mid_right_seg[0][1]
            mid_right_point_2[2] = mid_right_seg[0][2]
        # 筛选底部点
        bottom_left_point = [0, 0, 0]
        bottom_right_point = [0, 0, 0]
        for seg in hole_profile_loc:
            if seg[0][2] < self.h2 / 2 and seg[1][2] < self.h2 / 2:
                if seg[0][1] > seg[1][1]:
                    bottom_left_point[0] = seg[0][0]
                    bottom_left_point[1] = seg[0][1]
                    bottom_left_point[2] = seg[0][2]
                    bottom_right_point[0] = seg[1][0]
                    bottom_right_point[1] = seg[1][1]
                    bottom_right_point[2] = seg[1][2]
                else:
                    bottom_left_point[0] = seg[1][0]
                    bottom_left_point[1] = seg[1][1]
                    bottom_left_point[2] = seg[1][2]
                    bottom_right_point[0] = seg[0][0]
                    bottom_right_point[1] = seg[0][1]
                    bottom_right_point[2] = seg[0][2]
        mid_left_seg = [mid_left_point_1, mid_left_point_2]
        mid_right_seg = [mid_right_point_1, mid_right_point_2]
        bottom_seg = [bottom_left_point, bottom_right_point]
        seg_sets = [top_seg, mid_left_seg, mid_right_seg, bottom_seg]
        return seg_sets

    def get_stair_slide_hole_dimension_points(self):
        """
        获取楼梯滑动铰孔洞标注店
        :return:
        """
        # 楼梯底部左侧孔洞包围框轮廓点
        hole_seg = self.get_stair_slide_hole_edge_segment_points()
        # 计算顶部标注坐标点
        top_seg = hole_seg[0]  # 顶端线段
        top_left_point = [0, 0, 0]
        top_right_point = [0, 0, 0]
        # 判断两点的大小
        if top_seg[0][1] > top_seg[1][1]:
            top_left_point[0] = top_seg[0][0]
            top_left_point[1] = top_seg[0][1]
            top_left_point[2] = top_seg[0][2]
            top_right_point[0] = top_seg[1][0]
            top_right_point[1] = top_seg[1][1]
            top_right_point[2] = top_seg[1][2]
        else:
            top_left_point[0] = top_seg[1][0]
            top_left_point[1] = top_seg[1][1]
            top_left_point[2] = top_seg[1][2]
            top_right_point[0] = top_seg[0][0]
            top_right_point[1] = top_seg[0][1]
            top_right_point[2] = top_seg[0][2]
        # 特殊尺寸标注
        top_special_dimension = [top_left_point, top_right_point]
        top_special_text = "Ф" + str(
            int(round(abs(top_right_point[1] - top_left_point[1])))
        )
        # 筛选出中间点
        # 中部左侧线段和中部右侧线段
        mid_left_seg = hole_seg[1]
        mid_right_seg = hole_seg[2]
        mid_left_point_1 = [0, 0, 0]  # 中部左侧线段左侧点
        mid_left_point_2 = [0, 0, 0]  # 中部左侧线段右侧点
        mid_right_point_1 = [0, 0, 0]  # 中部右侧线段左侧点
        mid_right_point_2 = [0, 0, 0]  # 中部右侧线段右侧点
        # 筛选中部左侧线段
        if mid_left_seg[0][1] > mid_left_seg[1][1]:
            mid_left_point_1[0] = mid_left_seg[0][0]
            mid_left_point_1[1] = mid_left_seg[0][1]
            mid_left_point_1[2] = mid_left_seg[0][2]
            mid_left_point_2[0] = mid_left_seg[1][0]
            mid_left_point_2[1] = mid_left_seg[1][1]
            mid_left_point_2[2] = mid_left_seg[1][2]
        else:
            mid_left_point_1[0] = mid_left_seg[1][0]
            mid_left_point_1[1] = mid_left_seg[1][1]
            mid_left_point_1[2] = mid_left_seg[1][2]
            mid_left_point_2[0] = mid_left_seg[0][0]
            mid_left_point_2[1] = mid_left_seg[0][1]
            mid_left_point_2[2] = mid_left_seg[0][2]
        # 筛选中部右侧线段
        if mid_right_seg[0][1] > mid_right_seg[1][1]:
            mid_right_point_1[0] = mid_right_seg[0][0]
            mid_right_point_1[1] = mid_right_seg[0][1]
            mid_right_point_1[2] = mid_right_seg[0][2]
            mid_right_point_2[0] = mid_right_seg[1][0]
            mid_right_point_2[1] = mid_right_seg[1][1]
            mid_right_point_2[2] = mid_right_seg[1][2]
        else:
            mid_right_point_1[0] = mid_right_seg[1][0]
            mid_right_point_1[1] = mid_right_seg[1][1]
            mid_right_point_1[2] = mid_right_seg[1][2]
            mid_right_point_2[0] = mid_right_seg[0][0]
            mid_right_point_2[1] = mid_right_seg[0][1]
            mid_right_point_2[2] = mid_right_seg[0][2]
        # 顶部角点尺寸标注
        top_left_corner_point = copy.deepcopy(top_right_point)
        top_left_corner_point[1] = 0
        mid_dimension_loc = [
            [mid_left_point_2, mid_left_point_1],
            [mid_right_point_2, mid_right_point_1],
            [top_right_point, top_left_corner_point],
        ]
        mid_special_dimension_loc = [mid_right_point_1, mid_left_point_2]
        mid_special_dimension_text = "Ф" + str(
            int(round(abs(mid_left_point_1[1] - mid_right_point_2[1])))
        )
        # 筛选出底部点
        bottom_seg = hole_seg[3]
        bottom_left_point = [0, 0, 0]
        bottom_right_point = [0, 0, 0]
        if bottom_seg[0][1] > bottom_seg[1][1]:
            bottom_left_point[0] = bottom_seg[0][0]
            bottom_left_point[1] = bottom_seg[0][1]
            bottom_left_point[2] = bottom_seg[0][2]
            bottom_right_point[0] = bottom_seg[1][0]
            bottom_right_point[1] = bottom_seg[1][1]
            bottom_right_point[2] = bottom_seg[1][2]
        else:
            bottom_left_point[0] = bottom_seg[1][0]
            bottom_left_point[1] = bottom_seg[1][1]
            bottom_left_point[2] = bottom_seg[1][2]
            bottom_right_point[0] = bottom_seg[0][0]
            bottom_right_point[1] = bottom_seg[0][1]
            bottom_right_point[2] = bottom_seg[0][2]
        bottom_special_dimension_loc = [bottom_right_point, bottom_left_point]
        bottom_special_dimension_text = "Ф" + str(
            int(round(abs(bottom_left_point[1] - bottom_right_point[1])))
        )
        special_info = [
            [
                top_special_dimension,
                mid_special_dimension_loc,
                bottom_special_dimension_loc,
            ],
            [
                top_special_text,
                mid_special_dimension_text,
                bottom_special_dimension_text,
            ],
        ]
        # 标注信息
        dimension_info = {}
        dimension_info["special_dimension"] = special_info
        dimension_info["basic_dimension"] = mid_dimension_loc
        return dimension_info

    def get_stair_fix_hole_edge_segment_points(self):
        """
        开始楼梯固定支座边缘线段点：顶部线段、底部线段
        :return:
        """
        hole_profile_loc = self.get_stair_bottom_left_hole_bounding_profile_points()
        limit_z = self.h2 / 2
        top_seg = []
        bottom_seg = []
        for seg in hole_profile_loc:
            if abs(seg[0][2] - seg[1][2]) < self.drawing_precision:
                if seg[0][2] > limit_z:
                    top_seg.append(seg[0])
                    top_seg.append(seg[1])
                else:
                    bottom_seg.append(seg[0])
                    bottom_seg.append(seg[1])
        segment_sets = [top_seg, bottom_seg]
        return segment_sets

    def get_stair_fix_hole_dimension_points(self):
        """
        获取楼梯固定节点孔洞标注点
        :return:
        """
        # 楼梯底部左侧孔洞包围框轮廓点
        hole_seg = self.get_stair_fix_hole_edge_segment_points()
        top_seg = hole_seg[0]
        bottom_seg = hole_seg[1]
        # 获取顶部关键点
        top_left_point = [0, 0, 0]
        top_right_point = [0, 0, 0]
        if top_seg[0][1] > top_seg[1][1]:
            top_left_point[0] = top_seg[0][0]
            top_left_point[1] = top_seg[0][1]
            top_left_point[2] = top_seg[0][2]
            top_right_point[0] = top_seg[1][0]
            top_right_point[1] = top_seg[1][1]
            top_right_point[2] = top_seg[1][2]
        else:
            top_left_point[0] = top_seg[1][0]
            top_left_point[1] = top_seg[1][1]
            top_left_point[2] = top_seg[1][2]
            top_right_point[0] = top_seg[0][0]
            top_right_point[1] = top_seg[0][1]
            top_right_point[2] = top_seg[0][2]
        # 获取底部关键点
        bottom_left_point = [0, 0, 0]
        bottom_right_point = [0, 0, 0]
        if bottom_seg[0][1] > bottom_seg[1][1]:
            bottom_left_point[0] = bottom_seg[0][0]
            bottom_left_point[1] = bottom_seg[0][1]
            bottom_left_point[2] = bottom_seg[0][2]
            bottom_right_point[0] = bottom_seg[1][0]
            bottom_right_point[1] = bottom_seg[1][1]
            bottom_right_point[2] = bottom_seg[1][2]
        else:
            bottom_left_point[0] = bottom_seg[1][0]
            bottom_left_point[1] = bottom_seg[1][1]
            bottom_left_point[2] = bottom_seg[1][2]
            bottom_right_point[0] = bottom_seg[0][0]
            bottom_right_point[1] = bottom_seg[0][1]
            bottom_right_point[2] = bottom_seg[0][2]
        top_dimension_text = "Ф" + str(
            int(round(abs(top_right_point[1] - top_left_point[1])))
        )
        bottom_dimension_text = "Ф" + str(
            int(round(abs(bottom_right_point[1] - bottom_left_point[1])))
        )
        special_dimension = [
            [
                [top_left_point, top_right_point],
                [bottom_right_point, bottom_left_point],
            ],
            [top_dimension_text, bottom_dimension_text],
        ]
        return special_dimension

    def get_stair_profile_dimension_points(self):
        """
        获取楼梯轮廓标注点
        :return:
        """
        # 获取孔洞坐标点
        bottom_hole_loc = self.get_stair_solid_long_cut_plane_point()  # 获取楼梯实体纵向切割点
        rein_rebar_loc = self.get_stair_hole_rein_rebar_left_projection()  # 获取楼梯加强钢筋轮廓点
        bottom_profile_points = (
            self.get_stair_bottom_local_bounding_profile_points()
        )  # 获取楼梯底部轮廓点
        limit_z = self.h2 / 2
        # 顶部加强筋和底部加强筋
        top_rein_rebar = []
        bottom_rein_rebar = []
        for rebar in rein_rebar_loc:
            if rebar[0][0][2] > limit_z:
                for seg in rebar:
                    top_rein_rebar.append(seg)
            else:
                for seg in rebar:
                    bottom_rein_rebar.append(seg)
        # 孔洞加强筋的z值
        top_z = 0
        bottom_z = 0
        max_y = 0  # 底部加强钢筋的最大y坐标值
        for seg in top_rein_rebar:
            if abs(seg[0][1] - seg[1][1]) < self.drawing_precision:
                top_z = (seg[0][2] + seg[1][2]) / 2
        for seg in bottom_rein_rebar:
            if abs(seg[0][1] - seg[1][1]) < self.drawing_precision:
                bottom_z = (seg[0][2] + seg[1][2]) / 2
        for seg in bottom_rein_rebar:
            for point in seg:
                if point[1] > max_y:
                    max_y = point[1]
        min_y = self.lb_d
        for seg in bottom_profile_points:
            for point in seg:
                if point[1] < min_y:
                    min_y = point[1]
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        for seg in bottom_profile_points:
            if (
                abs(seg[0][1] - min_y) < self.drawing_precision
                and abs(seg[1][1] - min_y) < self.drawing_precision
            ):
                if seg[0][2] > seg[1][2]:
                    top_point[0] = seg[0][0]
                    top_point[1] = seg[0][1]
                    top_point[2] = seg[0][2]
                    bottom_point[0] = seg[1][0]
                    bottom_point[1] = seg[1][1]
                    bottom_point[2] = seg[1][2]
                else:
                    top_point[0] = seg[1][0]
                    top_point[1] = seg[1][1]
                    top_point[2] = seg[1][2]
                    bottom_point[0] = seg[0][0]
                    bottom_point[1] = seg[0][1]
                    bottom_point[2] = seg[0][2]
        # 第一层标注点
        # 从右侧底到顶
        point_1 = copy.deepcopy(list(bottom_point))
        point_2 = copy.deepcopy(point_1)
        point_2[2] = bottom_z
        point_3 = copy.deepcopy(point_2)
        point_3[2] = top_z
        point_4 = copy.deepcopy(top_point)
        # 第二层标注
        point_5 = copy.deepcopy(point_4)
        point_5[1] = bottom_hole_loc[1]
        point_6 = copy.deepcopy(point_5)
        point_6[1] = max_y
        first_dimension_loc = [
            [point_2, point_1],
            [point_3, point_2],
            [point_4, point_3],
        ]
        second_dimension_loc = [
            [point_4, point_1],
            [point_5, point_4],
            [point_6, point_5],
        ]
        # 信息合集
        dimension_info = {}
        dimension_info["first_floor"] = first_dimension_loc
        dimension_info["second_floor"] = second_dimension_loc
        return dimension_info

    def get_stair_hole_rein_rebar_outline_points(self):
        """
        获取楼梯孔洞加强筋引出线
        :return:
        """
        rein_rebar_loc = self.get_stair_hole_rein_rebar_left_projection()  # 获取楼梯加强钢筋轮廓点
        limit_z = self.h2 / 2
        # 顶部加强筋和底部加强筋
        top_rein_rebar = []
        bottom_rein_rebar = []
        for rebar in rein_rebar_loc:
            if rebar[0][0][2] > limit_z:
                for seg in rebar:
                    top_rein_rebar.append(seg)
            else:
                for seg in rebar:
                    bottom_rein_rebar.append(seg)
        min_y = self.lb_d
        min_z = self.h2
        for seg in bottom_rein_rebar:
            for point in seg:
                if point[1] < min_y:
                    min_y = point[1]
                if point[2] < min_z:
                    min_z = point[2]
        # 引出线
        outline_length_1 = 100
        outline_length_2 = 150
        theta = math.pi / 3
        point_1 = [bottom_rein_rebar[0][0][0], min_y, min_z]
        point_2 = copy.deepcopy(point_1)
        point_2[1] += -outline_length_1 * math.cos(theta)
        point_2[2] += -outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += -outline_length_2
        rein_rebar_diam = self.hole_rein_rebar_model.get_rebar_diameter()
        rebar_type = self.slab_struct.material.rebar_name
        text_loc = copy.deepcopy(point_3)
        text = "2" + str(rebar_type) + "-" + str(rein_rebar_diam) + "(预留洞加强筋)"
        outline_points = [[point_1, point_2], [point_2, point_3]]
        outline_text = [text_loc, text]
        # 引出线信息
        outline_info = {}
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info

    def get_stair_hole_rein_rebar_top_dimension_points(self):
        """
        获取楼梯孔洞加强筋俯视尺寸标注点
        :return:
        """
        # 获取孔洞坐标点
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        # 获取孔洞加强筋俯视坐标点
        rein_rebar_top_points = self.get_stair_hole_rein_rebar_bounding_points()
        # 获取竖向中部点
        top_seg = []
        bottom_seg = []
        for seg in rein_rebar_top_points:
            if len(seg) == 2 and abs(seg[0][1] - seg[1][1]) < self.drawing_precision:
                if seg[0][0] > hole_loc[0]:
                    top_seg.append(seg[0])
                    top_seg.append(seg[1])
                else:
                    bottom_seg.append(seg[0])
                    bottom_seg.append(seg[1])
        # 获取左侧顶部点和底部点
        top_point = [
            (top_seg[0][0] + top_seg[1][0]) / 2,
            (top_seg[0][1] + top_seg[1][1]) / 2,
            (top_seg[0][2] + top_seg[1][2]) / 2,
        ]
        bottom_point = [
            (bottom_seg[0][0] + bottom_seg[1][0]) / 2,
            (bottom_seg[0][1] + bottom_seg[1][1]) / 2,
            (bottom_seg[0][2] + bottom_seg[1][2]) / 2,
        ]
        bottom_left_point = [0, 0, 0]
        if bottom_seg[0][0] > bottom_seg[1][0]:
            bottom_left_point[0] = bottom_seg[1][0]
            bottom_left_point[1] = bottom_seg[1][1]
            bottom_left_point[2] = bottom_seg[1][2]
        else:
            bottom_left_point[0] = bottom_seg[0][0]
            bottom_left_point[1] = bottom_seg[0][1]
            bottom_left_point[2] = bottom_seg[0][2]
        min_line_y = self.lb_d
        min_arc_y = self.lb_d
        # 确定直线的最小y坐标值
        for seg in rein_rebar_top_points:
            if len(seg) == 2:
                for point in seg:
                    if point[1] < min_line_y:
                        min_line_y = point[1]
        # 确定圆弧的最小y坐标值
        for seg in rein_rebar_top_points:
            if len(seg) > 2:
                for point in seg:
                    if point[1] < min_arc_y:
                        min_arc_y = point[1]
        point_1 = copy.deepcopy(bottom_left_point)
        point_2 = copy.deepcopy(point_1)
        point_2[1] = min_line_y
        point_3 = copy.deepcopy(point_2)
        point_3[1] = min_arc_y
        dimension_loc = [
            [bottom_point, top_point],
            [point_2, point_1],
            [point_3, point_2],
        ]
        return dimension_loc

    def get_stair_slide_rein_rebar_top_hatch_points(self):
        """
        获取楼梯滑动铰加强筋俯视填充点集
        :return:
        """
        # 获取孔洞坐标点
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        hole_seg = self.get_stair_slide_hole_edge_segment_points()
        top_seg = hole_seg[0]
        mid_left_seg = hole_seg[1]
        mid_right_seg = hole_seg[2]
        bottom_seg = hole_seg[3]
        # 计算圆的直径
        top_diam = abs(top_seg[0][1] - top_seg[1][1])
        bottom_diam = abs(bottom_seg[0][1] - bottom_seg[1][1])
        point_1 = [0, 0, 0]
        point_2 = [0, 0, 0]
        if mid_left_seg[0][1] > mid_left_seg[1][1]:
            point_1[0] = mid_left_seg[1][0]
            point_1[1] = mid_left_seg[1][1]
            point_1[2] = mid_left_seg[1][2]
        else:
            point_1[0] = mid_left_seg[0][0]
            point_1[1] = mid_left_seg[0][1]
            point_1[2] = mid_left_seg[0][2]
        if mid_right_seg[0][1] > mid_right_seg[1][1]:
            point_2[0] = mid_right_seg[1][0]
            point_2[1] = mid_right_seg[1][1]
            point_2[2] = mid_right_seg[1][2]
        else:
            point_2[0] = mid_right_seg[0][0]
            point_2[1] = mid_right_seg[0][1]
            point_2[2] = mid_right_seg[0][2]
        mid_diam = abs(point_1[1] - point_2[1])
        edge = (mid_diam - bottom_diam) / 2
        draw_circle = [[list(hole_loc), list(hole_loc)], [top_diam, mid_diam]]
        # 绘制特殊线
        point_1 = copy.deepcopy(hole_loc)
        point_1[0] += mid_diam / 2
        point_2 = copy.deepcopy(hole_loc)
        point_2[1] += -(mid_diam / 2 - edge)
        point_3 = copy.deepcopy(point_1)
        point_3[0] += -mid_diam
        draw_spline = [[point_1, point_2, point_3], [list(hole_loc), mid_diam, 0, 180]]
        # 绘制引出线
        point_s_1 = copy.deepcopy(hole_loc)
        outline_length_1 = 150
        outline_length_2 = 150
        theta = math.pi / 3
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[0] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        text_loc = copy.deepcopy(point_s_3)
        text = "销键预留洞口" + "Φ" + str(int(top_diam)) + "(" + str(int(bottom_diam)) + ")"
        outline_text = [text_loc, text]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        # 填充信息
        hatch_info = {}
        hatch_info["draw_circle"] = draw_circle
        hatch_info["draw_spline"] = draw_spline
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_fix_rein_rebar_top_hatch_points(self):
        """
        获取楼梯固定铰加强筋俯视填充点集
        :return:
        """
        # 获取孔洞坐标点
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        hole_seg = self.get_stair_fix_hole_edge_segment_points()
        top_seg = hole_seg[0]
        bottom_seg = hole_seg[1]
        # 计算圆的直径
        top_diam = abs(top_seg[0][1] - top_seg[1][1])
        bottom_diam = abs(bottom_seg[0][1] - bottom_seg[1][1])
        edge = (top_diam - bottom_diam) / 4
        draw_circle = [[list(hole_loc)], [top_diam]]
        # 绘制特殊线
        point_1 = copy.deepcopy(hole_loc)
        point_1[0] += top_diam / 2
        point_2 = copy.deepcopy(hole_loc)
        point_2[1] += top_diam / 2 - edge
        point_3 = copy.deepcopy(point_1)
        point_3[0] += -top_diam
        draw_spline = [[point_1, point_2, point_3], [list(hole_loc), top_diam, 0, 180]]
        # 绘制引出线
        point_s_1 = copy.deepcopy(hole_loc)
        outline_length_1 = 150
        outline_length_2 = 150
        theta = math.pi / 3
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += outline_length_1 * math.cos(theta)
        point_s_2[0] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += outline_length_2
        text_loc = copy.deepcopy(point_s_3)
        text = "销键预留洞口" + "Φ" + str(int(top_diam)) + "(" + str(int(bottom_diam)) + ")"
        outline_text = [text_loc, text]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        # 填充信息
        hatch_info = {}
        hatch_info["draw_circle"] = draw_circle
        hatch_info["draw_spline"] = draw_spline
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info


class StairTopHoleReinRebarViewData(object):
    """
    获取楼梯顶部孔洞加强筋视图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.hole_loc_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.hole_rein_rebar_model = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.drawing_precision = 0.001
        self.hatch_offset_value = 5  # spline偏移值
        self.theta = math.pi / 6  # 折断线倾斜角度
        self.rebar_config = RebarConfig()
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.t = self.slab_struct.geometric.thickness
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_solid_long_cut_plane_point(self):
        """
        获取楼梯实体纵向剖切平面点
        :return:
        """
        hole_loc = self.hole_loc_info.get_hole_loc()
        limit_x = self.b0 / 2
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2
        current_point = [0, 0, 0]
        for point in hole_loc:
            if point.x < limit_x and point.y > limit_y:
                current_point[0] = point.x
                current_point[1] = point.y
                current_point[2] = point.z
        return current_point

    def get_stair_solid_long_cut_drawing(self):
        """
        获取楼梯纵向剖切数据：形成纵向剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_and_ear_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_solid_hole_cut_drawing(self):
        """
        获取楼梯孔洞剖切数据：形成孔洞剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_stair_all_hole_model()
        current_point = self.get_stair_solid_long_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(1, 0, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    @staticmethod
    def choose_inner_point_of_bounding_box(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    def get_stair_hole_rein_rebar_cut_point(self):
        """
        获取楼梯孔洞加强筋剖切点
        :return:
        """
        hole_rebar_loc = self.hole_rein_rebar_model.get_rebar_model()
        limit_x = self.b0 / 2
        limit_z = self.h2 + self.h - self.h1 / 2
        current_point = [0, 0, 0]
        for rebar in hole_rebar_loc:
            key_point = rebar[0]
            if key_point.x < limit_x and key_point.z > limit_z:
                current_point[0] = key_point.x
                current_point[1] = key_point.y
                current_point[2] = key_point.z
        return current_point

    def get_stair_hole_rein_rebar_profile_points(self):
        """
        获取孔洞加强筋剖切轮廓点
        :return:
        """
        current_point = self.get_stair_hole_rein_rebar_cut_point()  # 剖切点
        solid_model = self.composite_model.get_hole_rein_rebar_model()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 0, 1),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_hole_rein_rebar_bounding_box_loc(self):
        """
        获取楼梯孔洞加强筋包围框坐标点
        :return:
        """
        current_point = self.get_stair_hole_rein_rebar_cut_point()
        point_1 = [0, self.lb_d + self.ln, current_point[2]]
        point_2 = [self.b0 / 2, self.lb_d + self.ln, current_point[2]]
        point_3 = [self.b0 / 2, self.lb_d + self.ln + self.lt_d, current_point[2]]
        point_4 = [0, self.lb_d + self.ln + self.lt_d, current_point[2]]
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_hole_rein_rebar_bounding_points(self):
        """
        获取楼梯孔洞加强筋包围框内的点
        :return:
        """
        bounding_box_loc = self.get_stair_hole_rein_rebar_bounding_box_loc()
        cut_profile_points = self.get_stair_hole_rein_rebar_profile_points()
        bounding_box_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_box_points

    def get_stair_top_left_hole_bounding_points(self):
        """
        获取楼梯顶部左侧孔洞包围框
        :return:
        """
        hole_loc = self.get_stair_solid_long_cut_plane_point()  # 获取楼梯孔洞剖切点
        total_l = self.lb_d + self.ln + self.lt_d
        total_h = self.h2 + self.h
        point_1 = [hole_loc[0], total_l, total_h + self.h1]
        point_2 = [hole_loc[0], total_l, 0]
        point_3 = [hole_loc[0], total_l - self.lt_d, 0]
        point_4 = [hole_loc[0], total_l - self.lt_d, total_h + self.h1]
        bounding_box_loc = [point_1, point_2, point_3, point_4]
        return bounding_box_loc

    def get_stair_top_left_hole_bounding_profile_points(self):
        """
        获取楼梯顶端左侧孔洞包围框内轮廓点
        :return:
        """
        bounding_box_loc = self.get_stair_top_left_hole_bounding_points()  # 包围框
        hole_profile_points = self.get_stair_solid_hole_cut_drawing()  # 开始获取孔洞轮廓点
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, hole_profile_points
        )
        return bounding_profile_points

    def get_stair_top_local_profile_bounding_box(self):
        """
        获取楼梯顶端局部轮廓包围框
        :return:
        """
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        tabu_h = self.tabu_h  # 踏步高度
        tabu_b = self.tabu_b  # 踏步宽度
        add_z = 20
        add_y = 20
        length_1 = (tabu_h + add_z) / math.cos(self.theta)
        length_2 = ((2 * tabu_b / 3) * math.sin(self.theta) + self.t) + length_1
        total_h = self.h2 + self.h
        total_l = self.lb_d + self.ln + self.lt_d
        point_1 = [hole_loc[0], total_l + add_y, total_h + add_z]
        point_2 = [
            hole_loc[0],
            total_l - self.lt_d - 2 * tabu_b / 3 - length_1 * math.sin(self.theta),
            total_h + add_z,
        ]
        point_3 = [
            hole_loc[0],
            total_l
            - self.lt_d
            - 2 * tabu_b / 3
            + (length_2 - length_1) * math.sin(self.theta),
            total_h - tabu_h - (length_2 - length_1) * math.cos(self.theta),
        ]
        point_4 = [hole_loc[0], point_1[1], point_3[2]]
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_top_local_bounding_profile_points(self):
        """
        获取楼梯顶端局部包围轮廓点
        :return:
        """
        bounding_box_loc = self.get_stair_top_local_profile_bounding_box()
        cut_profile_points = self.get_stair_solid_long_cut_drawing()
        bounding_profile_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_profile_points

    def get_stair_top_right_boundary_points(self):
        """
        获取楼梯顶端右侧侧折断线点
        :return:
        """
        profile_points = (
            self.get_stair_top_local_bounding_profile_points()
        )  # 获取纵向切割图形底部包围框内的点
        min_y = self.lb_d + self.ln + self.lt_d
        min_z = self.h2 + self.h
        for seg in profile_points:
            for point in seg:
                if min_y > point[1]:
                    min_y = point[1]
                if min_z > point[2]:
                    min_z = point[2]
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        for seg in profile_points:
            for point in seg:
                if abs(min_y - point[1]) < self.drawing_precision:
                    top_point[0] = point[0]
                    top_point[1] = point[1]
                    top_point[2] = point[2]
                if abs(min_z - point[2]) < self.drawing_precision:
                    bottom_point[0] = point[0]
                    bottom_point[1] = point[1]
                    bottom_point[2] = point[2]
        draw_points = [top_point, bottom_point]
        return draw_points

    def get_stair_top_right_breakline_points(self):
        """
        获取楼梯底部左侧折断线点
        :return:
        """
        # 折断线特殊形状
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        special_points = self.get_stair_top_right_boundary_points()  # 获取楼梯顶部边界特殊点
        top_point = special_points[0]  # 顶部点
        bottom_point = special_points[1]  # 底部点
        direction = np.array(
            np.array(bottom_point) - np.array(top_point)
        ) / np.linalg.norm(
            np.array(bottom_point) - np.array(top_point)
        )  # 顶到底
        base_dir = np.cross(direction, np.array([-1, 0, 0]))
        base_length_1 = offset * math.sin(theta)
        base_length_2 = offset * math.cos(theta)
        # 开始计算关键点
        point_2 = copy.deepcopy(top_point)
        point_1 = copy.deepcopy(point_2)
        _point_1 = np.array(point_1) + (-1) * direction * extend_l
        point_1 = _point_1.tolist()
        point_mid = [
            (top_point[0] + bottom_point[0]) / 2,
            (top_point[1] + bottom_point[1]) / 2,
            (top_point[2] + bottom_point[2]) / 2,
        ]
        point_mid_t = np.array(point_mid) + (-1) * direction * l_space / 2
        point_3 = point_mid_t.tolist()
        _point_4 = (
            np.array(point_3) + base_length_1 * direction + base_length_2 * base_dir
        )
        point_4 = _point_4.tolist()
        point_mid_b = np.array(point_mid) + direction * l_space / 2
        point_6 = point_mid_b.tolist()
        _point_5 = (
            np.array(point_6)
            + (-1) * direction * base_length_1
            + (-1) * base_dir * base_length_2
        )
        point_5 = _point_5.tolist()
        point_7 = copy.deepcopy(bottom_point)
        _point_8 = np.array(point_7) + direction * extend_l
        point_8 = _point_8.tolist()
        draw_points = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_points

    def get_stair_hole_rein_rebar_left_projection(self):
        """
        获取孔洞加强筋侧视图:矩形形状
        :return:List[List[List[List[float],List[float]]]]
        """
        rein_rebar_loc = self.hole_rein_rebar_model.get_rebar_model()  # 获取孔洞加强筋数据
        diam = self.hole_rein_rebar_model.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2
        limit_y = (self.ln + self.lb_d + self.lt_d) / 2
        left_loc = []  # 左侧孔洞位置
        for num in range(len(rein_rebar_loc)):
            current_rebar = rein_rebar_loc[num]  # 当前钢筋
            current_point = current_rebar[0]  # 当前点
            if current_point.x < limit_x and current_point.y > limit_y:
                left_loc.append(current_rebar)
        # 获取每根钢筋的角点
        total_vertex = []
        for num in range(len(left_loc)):
            current_rebar = left_loc[num]
            rebar_loc = get_U_rein_rebar_vertex(current_rebar, diam, limit_y)
            total_vertex.append(rebar_loc)
        return total_vertex

    def get_stair_slide_hole_edge_segment_points(self):
        """
        获取楼梯滑动支座孔洞边缘线段点：顶部、中部左侧、中部右侧、底部
        :return:
        """
        hole_profile_loc = self.get_stair_top_left_hole_bounding_profile_points()
        top_seg = []  # 顶端线段
        total_h = self.h2 + self.h
        for seg in hole_profile_loc:
            if (
                abs(seg[0][2] - total_h) < self.drawing_precision
                and abs(seg[1][2] - total_h) < self.drawing_precision
            ):
                top_seg.append(list(seg[0]))
                top_seg.append(list(seg[1]))
        # 筛选出中间点
        mid_seg = []  # 中部线段
        mid_z = total_h - self.h1 / 2
        for seg in hole_profile_loc:
            if (seg[0][2] > mid_z and seg[0][2] < (total_h - 2)) and (
                seg[1][2] > mid_z and seg[1][2] < (total_h - 2)
            ):
                if seg[0][1] > seg[1][1]:
                    mid_seg.append([seg[0], seg[1]])
                else:
                    mid_seg.append([seg[0], seg[1]])
        if mid_seg[0][0][1] < mid_seg[1][0][1]:
            _seg = mid_seg[0]
            mid_seg[0] = mid_seg[1]
            mid_seg[1] = _seg
        # 中部左侧线段和中部右侧线段
        mid_left_seg = mid_seg[0]
        mid_right_seg = mid_seg[1]
        mid_left_point_1 = [0, 0, 0]  # 中部左侧线段左侧点
        mid_left_point_2 = [0, 0, 0]  # 中部左侧线段右侧点
        mid_right_point_1 = [0, 0, 0]  # 中部右侧线段左侧点
        mid_right_point_2 = [0, 0, 0]  # 中部右侧线段右侧点
        # 筛选中部左侧线段
        if mid_left_seg[0][1] > mid_left_seg[1][1]:
            mid_left_point_1[0] = mid_left_seg[0][0]
            mid_left_point_1[1] = mid_left_seg[0][1]
            mid_left_point_1[2] = mid_left_seg[0][2]
            mid_left_point_2[0] = mid_left_seg[1][0]
            mid_left_point_2[1] = mid_left_seg[1][1]
            mid_left_point_2[2] = mid_left_seg[1][2]
        else:
            mid_left_point_1[0] = mid_left_seg[1][0]
            mid_left_point_1[1] = mid_left_seg[1][1]
            mid_left_point_1[2] = mid_left_seg[1][2]
            mid_left_point_2[0] = mid_left_seg[0][0]
            mid_left_point_2[1] = mid_left_seg[0][1]
            mid_left_point_2[2] = mid_left_seg[0][2]
        # 筛选中部右侧线段
        if mid_right_seg[0][1] > mid_right_seg[1][1]:
            mid_right_point_1[0] = mid_right_seg[0][0]
            mid_right_point_1[1] = mid_right_seg[0][1]
            mid_right_point_1[2] = mid_right_seg[0][2]
            mid_right_point_2[0] = mid_right_seg[1][0]
            mid_right_point_2[1] = mid_right_seg[1][1]
            mid_right_point_2[2] = mid_right_seg[1][2]
        else:
            mid_right_point_1[0] = mid_right_seg[1][0]
            mid_right_point_1[1] = mid_right_seg[1][1]
            mid_right_point_1[2] = mid_right_seg[1][2]
            mid_right_point_2[0] = mid_right_seg[0][0]
            mid_right_point_2[1] = mid_right_seg[0][1]
            mid_right_point_2[2] = mid_right_seg[0][2]
        # 筛选底部点
        bottom_left_point = [0, 0, 0]
        bottom_right_point = [0, 0, 0]
        for seg in hole_profile_loc:
            if seg[0][2] < mid_z and seg[1][2] < mid_z:
                if seg[0][1] > seg[1][1]:
                    bottom_left_point[0] = seg[0][0]
                    bottom_left_point[1] = seg[0][1]
                    bottom_left_point[2] = seg[0][2]
                    bottom_right_point[0] = seg[1][0]
                    bottom_right_point[1] = seg[1][1]
                    bottom_right_point[2] = seg[1][2]
                else:
                    bottom_left_point[0] = seg[1][0]
                    bottom_left_point[1] = seg[1][1]
                    bottom_left_point[2] = seg[1][2]
                    bottom_right_point[0] = seg[0][0]
                    bottom_right_point[1] = seg[0][1]
                    bottom_right_point[2] = seg[0][2]
        mid_left_seg = [mid_left_point_1, mid_left_point_2]
        mid_right_seg = [mid_right_point_1, mid_right_point_2]
        bottom_seg = [bottom_left_point, bottom_right_point]
        seg_sets = [top_seg, mid_left_seg, mid_right_seg, bottom_seg]
        return seg_sets

    def get_stair_slide_hole_dimension_points(self):
        """
        获取楼梯滑动铰孔洞标注店
        :return:
        """
        # 楼梯底部左侧孔洞包围框轮廓点
        hole_seg = self.get_stair_slide_hole_edge_segment_points()
        total_l = self.lb_d + self.ln + self.lt_d
        # 计算顶部标注坐标点
        top_seg = hole_seg[0]  # 顶端线段
        top_left_point = [0, 0, 0]
        top_right_point = [0, 0, 0]
        # 判断两点的大小
        if top_seg[0][1] > top_seg[1][1]:
            top_left_point[0] = top_seg[0][0]
            top_left_point[1] = top_seg[0][1]
            top_left_point[2] = top_seg[0][2]
            top_right_point[0] = top_seg[1][0]
            top_right_point[1] = top_seg[1][1]
            top_right_point[2] = top_seg[1][2]
        else:
            top_left_point[0] = top_seg[1][0]
            top_left_point[1] = top_seg[1][1]
            top_left_point[2] = top_seg[1][2]
            top_right_point[0] = top_seg[0][0]
            top_right_point[1] = top_seg[0][1]
            top_right_point[2] = top_seg[0][2]
        # 特殊尺寸标注
        top_special_dimension = [top_left_point, top_right_point]
        top_special_text = "Ф" + str(
            int(round(abs(top_right_point[1] - top_left_point[1])))
        )
        # 筛选出中间点
        # 中部左侧线段和中部右侧线段
        mid_left_seg = hole_seg[1]
        mid_right_seg = hole_seg[2]
        mid_left_point_1 = [0, 0, 0]  # 中部左侧线段左侧点
        mid_left_point_2 = [0, 0, 0]  # 中部左侧线段右侧点
        mid_right_point_1 = [0, 0, 0]  # 中部右侧线段左侧点
        mid_right_point_2 = [0, 0, 0]  # 中部右侧线段右侧点
        # 筛选中部左侧线段
        if mid_left_seg[0][1] > mid_left_seg[1][1]:
            mid_left_point_1[0] = mid_left_seg[0][0]
            mid_left_point_1[1] = mid_left_seg[0][1]
            mid_left_point_1[2] = mid_left_seg[0][2]
            mid_left_point_2[0] = mid_left_seg[1][0]
            mid_left_point_2[1] = mid_left_seg[1][1]
            mid_left_point_2[2] = mid_left_seg[1][2]
        else:
            mid_left_point_1[0] = mid_left_seg[1][0]
            mid_left_point_1[1] = mid_left_seg[1][1]
            mid_left_point_1[2] = mid_left_seg[1][2]
            mid_left_point_2[0] = mid_left_seg[0][0]
            mid_left_point_2[1] = mid_left_seg[0][1]
            mid_left_point_2[2] = mid_left_seg[0][2]
        # 筛选中部右侧线段
        if mid_right_seg[0][1] > mid_right_seg[1][1]:
            mid_right_point_1[0] = mid_right_seg[0][0]
            mid_right_point_1[1] = mid_right_seg[0][1]
            mid_right_point_1[2] = mid_right_seg[0][2]
            mid_right_point_2[0] = mid_right_seg[1][0]
            mid_right_point_2[1] = mid_right_seg[1][1]
            mid_right_point_2[2] = mid_right_seg[1][2]
        else:
            mid_right_point_1[0] = mid_right_seg[1][0]
            mid_right_point_1[1] = mid_right_seg[1][1]
            mid_right_point_1[2] = mid_right_seg[1][2]
            mid_right_point_2[0] = mid_right_seg[0][0]
            mid_right_point_2[1] = mid_right_seg[0][1]
            mid_right_point_2[2] = mid_right_seg[0][2]
        # 顶部角点尺寸标注
        top_left_corner_point = copy.deepcopy(top_right_point)
        top_left_corner_point[1] = total_l
        mid_dimension_loc = [
            [mid_left_point_2, mid_left_point_1],
            [mid_right_point_2, mid_right_point_1],
            [top_left_corner_point, top_right_point],
        ]
        mid_special_dimension_loc = [mid_right_point_1, mid_left_point_2]
        mid_special_dimension_text = "Ф" + str(
            int(round(abs(mid_left_point_1[1] - mid_right_point_2[1])))
        )
        # 筛选出底部点
        bottom_seg = hole_seg[3]
        bottom_left_point = [0, 0, 0]
        bottom_right_point = [0, 0, 0]
        if bottom_seg[0][1] > bottom_seg[1][1]:
            bottom_left_point[0] = bottom_seg[0][0]
            bottom_left_point[1] = bottom_seg[0][1]
            bottom_left_point[2] = bottom_seg[0][2]
            bottom_right_point[0] = bottom_seg[1][0]
            bottom_right_point[1] = bottom_seg[1][1]
            bottom_right_point[2] = bottom_seg[1][2]
        else:
            bottom_left_point[0] = bottom_seg[1][0]
            bottom_left_point[1] = bottom_seg[1][1]
            bottom_left_point[2] = bottom_seg[1][2]
            bottom_right_point[0] = bottom_seg[0][0]
            bottom_right_point[1] = bottom_seg[0][1]
            bottom_right_point[2] = bottom_seg[0][2]
        bottom_special_dimension_loc = [bottom_right_point, bottom_left_point]
        bottom_special_dimension_text = "Ф" + str(
            int(round(abs(bottom_left_point[1] - bottom_right_point[1])))
        )
        special_info = [
            [
                top_special_dimension,
                mid_special_dimension_loc,
                bottom_special_dimension_loc,
            ],
            [
                top_special_text,
                mid_special_dimension_text,
                bottom_special_dimension_text,
            ],
        ]
        # 标注信息
        dimension_info = {}
        dimension_info["special_dimension"] = special_info
        dimension_info["basic_dimension"] = mid_dimension_loc
        return dimension_info

    def get_stair_fix_hole_edge_segment_points(self):
        """
        开始楼梯固定支座边缘线段点：顶部线段、底部线段
        :return:
        """
        hole_profile_loc = self.get_stair_top_left_hole_bounding_profile_points()
        limit_z = self.h2 + self.h - self.h1 / 2
        top_seg = []
        bottom_seg = []
        for seg in hole_profile_loc:
            if abs(seg[0][2] - seg[1][2]) < self.drawing_precision:
                if seg[0][2] > limit_z:
                    top_seg.append(seg[0])
                    top_seg.append(seg[1])
                else:
                    bottom_seg.append(seg[0])
                    bottom_seg.append(seg[1])
        segment_sets = [top_seg, bottom_seg]
        return segment_sets

    def get_stair_fix_hole_dimension_points(self):
        """
        获取楼梯固定节点孔洞标注点
        :return:
        """
        # 楼梯底部左侧孔洞包围框轮廓点
        hole_seg = self.get_stair_fix_hole_edge_segment_points()
        top_seg = hole_seg[0]
        bottom_seg = hole_seg[1]
        # 获取顶部关键点
        top_left_point = [0, 0, 0]
        top_right_point = [0, 0, 0]
        if top_seg[0][1] > top_seg[1][1]:
            top_left_point[0] = top_seg[0][0]
            top_left_point[1] = top_seg[0][1]
            top_left_point[2] = top_seg[0][2]
            top_right_point[0] = top_seg[1][0]
            top_right_point[1] = top_seg[1][1]
            top_right_point[2] = top_seg[1][2]
        else:
            top_left_point[0] = top_seg[1][0]
            top_left_point[1] = top_seg[1][1]
            top_left_point[2] = top_seg[1][2]
            top_right_point[0] = top_seg[0][0]
            top_right_point[1] = top_seg[0][1]
            top_right_point[2] = top_seg[0][2]
        # 获取底部关键点
        bottom_left_point = [0, 0, 0]
        bottom_right_point = [0, 0, 0]
        if bottom_seg[0][1] > bottom_seg[1][1]:
            bottom_left_point[0] = bottom_seg[0][0]
            bottom_left_point[1] = bottom_seg[0][1]
            bottom_left_point[2] = bottom_seg[0][2]
            bottom_right_point[0] = bottom_seg[1][0]
            bottom_right_point[1] = bottom_seg[1][1]
            bottom_right_point[2] = bottom_seg[1][2]
        else:
            bottom_left_point[0] = bottom_seg[1][0]
            bottom_left_point[1] = bottom_seg[1][1]
            bottom_left_point[2] = bottom_seg[1][2]
            bottom_right_point[0] = bottom_seg[0][0]
            bottom_right_point[1] = bottom_seg[0][1]
            bottom_right_point[2] = bottom_seg[0][2]
        top_dimension_text = "Ф" + str(
            int(round(abs(top_right_point[1] - top_left_point[1])))
        )
        bottom_dimension_text = "Ф" + str(
            int(round(abs(bottom_right_point[1] - bottom_left_point[1])))
        )
        special_dimension = [
            [
                [top_left_point, top_right_point],
                [bottom_right_point, bottom_left_point],
            ],
            [top_dimension_text, bottom_dimension_text],
        ]
        return special_dimension

    def get_stair_profile_dimension_points(self):
        """
        获取楼梯轮廓标注点
        :return:
        """
        # 获取孔洞坐标点
        bottom_hole_loc = self.get_stair_solid_long_cut_plane_point()  # 获取楼梯实体纵向切割点
        rein_rebar_loc = self.get_stair_hole_rein_rebar_left_projection()  # 获取楼梯加强钢筋轮廓点
        top_profile_points = (
            self.get_stair_top_local_bounding_profile_points()
        )  # 获取楼梯顶部轮廓点
        limit_z = self.h2 + self.h - self.h1 / 2
        # 顶部加强筋和底部加强筋
        top_rein_rebar = []
        bottom_rein_rebar = []
        for rebar in rein_rebar_loc:
            if rebar[0][0][2] > limit_z:
                for seg in rebar:
                    top_rein_rebar.append(seg)
            else:
                for seg in rebar:
                    bottom_rein_rebar.append(seg)
        # 孔洞加强筋的z值
        top_z = 0
        bottom_z = 0
        min_y = self.lb_d + self.ln + self.lt_d  # 底部加强钢筋的最大y坐标值
        for seg in top_rein_rebar:
            if abs(seg[0][1] - seg[1][1]) < self.drawing_precision:
                top_z = (seg[0][2] + seg[1][2]) / 2
        for seg in bottom_rein_rebar:
            if abs(seg[0][1] - seg[1][1]) < self.drawing_precision:
                bottom_z = (seg[0][2] + seg[1][2]) / 2
        for seg in bottom_rein_rebar:
            for point in seg:
                if point[1] < min_y:
                    min_y = point[1]
        max_y = 0
        for seg in top_profile_points:
            for point in seg:
                if point[1] > max_y:
                    max_y = point[1]
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        for seg in top_profile_points:
            if (
                abs(seg[0][1] - max_y) < self.drawing_precision
                and abs(seg[1][1] - max_y) < self.drawing_precision
            ):
                if seg[0][2] > seg[1][2]:
                    top_point[0] = seg[0][0]
                    top_point[1] = seg[0][1]
                    top_point[2] = seg[0][2]
                    bottom_point[0] = seg[1][0]
                    bottom_point[1] = seg[1][1]
                    bottom_point[2] = seg[1][2]
                else:
                    top_point[0] = seg[1][0]
                    top_point[1] = seg[1][1]
                    top_point[2] = seg[1][2]
                    bottom_point[0] = seg[0][0]
                    bottom_point[1] = seg[0][1]
                    bottom_point[2] = seg[0][2]
        # 第一层标注点
        # 从右侧底到顶
        point_1 = copy.deepcopy(list(bottom_point))
        point_2 = copy.deepcopy(point_1)
        point_2[2] = bottom_z
        point_3 = copy.deepcopy(point_2)
        point_3[2] = top_z
        point_4 = copy.deepcopy(top_point)
        # 第二层标注
        point_5 = copy.deepcopy(point_4)
        point_5[1] = bottom_hole_loc[1]
        point_6 = copy.deepcopy(point_5)
        point_6[1] = max_y
        first_dimension_loc = [
            [point_1, point_2],
            [point_2, point_3],
            [point_3, point_4],
        ]
        second_dimension_loc = [
            [point_1, point_4],
            [point_4, point_5],
            [point_5, point_6],
        ]
        # 信息合集
        dimension_info = {}
        dimension_info["first_floor"] = first_dimension_loc
        dimension_info["second_floor"] = second_dimension_loc
        return dimension_info

    def get_stair_hole_rein_rebar_outline_points(self):
        """
        获取楼梯孔洞加强筋引出线
        :return:
        """
        rein_rebar_loc = self.get_stair_hole_rein_rebar_left_projection()  # 获取楼梯加强钢筋轮廓点
        limit_z = self.h2 + self.h - self.h1 / 2
        # 顶部加强筋和底部加强筋
        top_rein_rebar = []
        bottom_rein_rebar = []
        for rebar in rein_rebar_loc:
            if rebar[0][0][2] > limit_z:
                for seg in rebar:
                    top_rein_rebar.append(seg)
            else:
                for seg in rebar:
                    bottom_rein_rebar.append(seg)
        max_y = 0
        min_z = self.h2 + self.h
        for seg in bottom_rein_rebar:
            for point in seg:
                if point[1] > max_y:
                    max_y = point[1]
                if point[2] < min_z:
                    min_z = point[2]
        # 引出线
        outline_length_1 = 100
        outline_length_2 = 150
        theta = math.pi / 3
        point_1 = [bottom_rein_rebar[0][0][0], max_y, min_z]
        point_2 = copy.deepcopy(point_1)
        point_2[1] += outline_length_1 * math.cos(theta)
        point_2[2] += -outline_length_1 * math.sin(theta)
        point_3 = copy.deepcopy(point_2)
        point_3[1] += outline_length_2
        rein_rebar_diam = self.hole_rein_rebar_model.get_rebar_diameter()
        rebar_type = self.slab_struct.material.rebar_name
        text_loc = copy.deepcopy(point_3)
        text = "2" + str(rebar_type) + "-" + str(rein_rebar_diam) + "(预留洞加强筋)"
        outline_points = [[point_1, point_2], [point_2, point_3]]
        outline_text = [text_loc, text]
        # 引出线信息
        outline_info = {}
        outline_info["outline_points"] = outline_points
        outline_info["outline_text"] = outline_text
        return outline_info

    def get_stair_hole_rein_rebar_top_dimension_points(self):
        """
        获取楼梯孔洞加强筋俯视尺寸标注点
        :return:
        """
        # 获取孔洞坐标点
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        # 获取孔洞加强筋俯视坐标点
        rein_rebar_top_points = self.get_stair_hole_rein_rebar_bounding_points()
        # 获取竖向中部点
        top_seg = []
        bottom_seg = []
        for seg in rein_rebar_top_points:
            if len(seg) == 2 and abs(seg[0][1] - seg[1][1]) < self.drawing_precision:
                if seg[0][0] > hole_loc[0]:
                    top_seg.append(seg[0])
                    top_seg.append(seg[1])
                else:
                    bottom_seg.append(seg[0])
                    bottom_seg.append(seg[1])
        # 获取左侧顶部点和底部点
        top_point = [
            (top_seg[0][0] + top_seg[1][0]) / 2,
            (top_seg[0][1] + top_seg[1][1]) / 2,
            (top_seg[0][2] + top_seg[1][2]) / 2,
        ]
        bottom_point = [
            (bottom_seg[0][0] + bottom_seg[1][0]) / 2,
            (bottom_seg[0][1] + bottom_seg[1][1]) / 2,
            (bottom_seg[0][2] + bottom_seg[1][2]) / 2,
        ]
        bottom_left_point = [0, 0, 0]
        if bottom_seg[0][0] > bottom_seg[1][0]:
            bottom_left_point[0] = bottom_seg[1][0]
            bottom_left_point[1] = bottom_seg[1][1]
            bottom_left_point[2] = bottom_seg[1][2]
        else:
            bottom_left_point[0] = bottom_seg[0][0]
            bottom_left_point[1] = bottom_seg[0][1]
            bottom_left_point[2] = bottom_seg[0][2]
        max_line_y = 0
        max_arc_y = 0
        # 确定直线的最大y坐标值
        for seg in rein_rebar_top_points:
            if len(seg) == 2:
                for point in seg:
                    if point[1] > max_line_y:
                        max_line_y = point[1]
        # 确定圆弧的最大y坐标值
        for seg in rein_rebar_top_points:
            if len(seg) > 2:
                for point in seg:
                    if point[1] > max_arc_y:
                        max_arc_y = point[1]
        point_1 = copy.deepcopy(bottom_left_point)
        point_2 = copy.deepcopy(point_1)
        point_2[1] = max_line_y
        point_3 = copy.deepcopy(point_2)
        point_3[1] = max_arc_y
        dimension_loc = [
            [top_point, bottom_point],
            [point_1, point_2],
            [point_2, point_3],
        ]
        return dimension_loc

    def get_stair_slide_rein_rebar_top_hatch_points(self):
        """
        获取楼梯滑动铰加强筋俯视填充点集
        :return:
        """
        # 获取孔洞坐标点
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        hole_seg = self.get_stair_slide_hole_edge_segment_points()
        top_seg = hole_seg[0]
        mid_left_seg = hole_seg[1]
        mid_right_seg = hole_seg[2]
        bottom_seg = hole_seg[3]
        # 计算圆的直径
        top_diam = abs(top_seg[0][1] - top_seg[1][1])
        bottom_diam = abs(bottom_seg[0][1] - bottom_seg[1][1])
        point_1 = [0, 0, 0]
        point_2 = [0, 0, 0]
        if mid_left_seg[0][1] > mid_left_seg[1][1]:
            point_1[0] = mid_left_seg[1][0]
            point_1[1] = mid_left_seg[1][1]
            point_1[2] = mid_left_seg[1][2]
        else:
            point_1[0] = mid_left_seg[0][0]
            point_1[1] = mid_left_seg[0][1]
            point_1[2] = mid_left_seg[0][2]
        if mid_right_seg[0][1] > mid_right_seg[1][1]:
            point_2[0] = mid_right_seg[1][0]
            point_2[1] = mid_right_seg[1][1]
            point_2[2] = mid_right_seg[1][2]
        else:
            point_2[0] = mid_right_seg[0][0]
            point_2[1] = mid_right_seg[0][1]
            point_2[2] = mid_right_seg[0][2]
        mid_diam = abs(point_1[1] - point_2[1])
        edge = (mid_diam - bottom_diam) / 2
        draw_circle = [[list(hole_loc), list(hole_loc)], [top_diam, mid_diam]]
        # 绘制特殊线
        point_1 = copy.deepcopy(hole_loc)
        point_1[0] += mid_diam / 2
        point_2 = copy.deepcopy(hole_loc)
        point_2[1] += -(mid_diam / 2 - edge)
        point_3 = copy.deepcopy(point_1)
        point_3[0] += -mid_diam
        draw_spline = [[point_1, point_2, point_3], [list(hole_loc), mid_diam, 0, 180]]
        # 绘制引出线
        point_s_1 = copy.deepcopy(hole_loc)
        outline_length_1 = 150
        outline_length_2 = 150
        theta = math.pi / 3
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_1 * math.cos(theta)
        point_s_2[0] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        text_loc = copy.deepcopy(point_s_3)
        text = "销键预留洞口" + "Φ" + str(int(top_diam)) + "(" + str(int(bottom_diam)) + ")"
        outline_text = [text_loc, text]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        # 填充信息
        hatch_info = {}
        hatch_info["draw_circle"] = draw_circle
        hatch_info["draw_spline"] = draw_spline
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info

    def get_stair_fix_rein_rebar_top_hatch_points(self):
        """
        获取楼梯固定铰加强筋俯视填充点集
        :return:
        """
        # 获取孔洞坐标点
        hole_loc = self.get_stair_solid_long_cut_plane_point()
        hole_seg = self.get_stair_fix_hole_edge_segment_points()
        top_seg = hole_seg[0]
        bottom_seg = hole_seg[1]
        # 计算圆的直径
        top_diam = abs(top_seg[0][1] - top_seg[1][1])
        bottom_diam = abs(bottom_seg[0][1] - bottom_seg[1][1])
        edge = (top_diam - bottom_diam) / 4
        draw_circle = [[list(hole_loc)], [top_diam]]
        # 绘制特殊线
        point_1 = copy.deepcopy(hole_loc)
        point_1[0] += top_diam / 2
        point_2 = copy.deepcopy(hole_loc)
        point_2[1] += -(top_diam / 2 - edge)
        point_3 = copy.deepcopy(point_1)
        point_3[0] += -top_diam
        draw_spline = [[point_1, point_2, point_3], [list(hole_loc), top_diam, 0, 180]]
        # 绘制引出线
        point_s_1 = copy.deepcopy(hole_loc)
        outline_length_1 = 150
        outline_length_2 = 150
        theta = math.pi / 3
        point_s_2 = copy.deepcopy(point_s_1)
        point_s_2[1] += -outline_length_1 * math.cos(theta)
        point_s_2[0] += outline_length_1 * math.sin(theta)
        point_s_3 = copy.deepcopy(point_s_2)
        point_s_3[1] += -outline_length_2
        text_loc = copy.deepcopy(point_s_3)
        text = "销键预留洞口" + "Φ" + str(int(top_diam)) + "(" + str(int(bottom_diam)) + ")"
        outline_text = [text_loc, text]
        outline_points = [[point_s_1, point_s_2], [point_s_2, point_s_3]]
        # 填充信息
        hatch_info = {}
        hatch_info["draw_circle"] = draw_circle
        hatch_info["draw_spline"] = draw_spline
        hatch_info["outline_points"] = outline_points
        hatch_info["outline_text"] = outline_text
        return hatch_info


class StairRailEmbeddedDetailViewData(object):
    """
    获取栏杆预埋件详图数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.hole_loc_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.composite_model = BuildMergeAndCutModel(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.rail_embedded_info = RailingEmbeddedPart(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.drawing_precision = 0.001
        self.hatch_offset_value = 5  # spline偏移值
        self.theta = math.pi / 6  # 折断线倾斜角度
        self.rebar_config = RebarConfig()
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
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

    @staticmethod
    def get_solid_cut_drawing(plane, cut_model):
        """
        通过一个平面剖切一个实体，形成剖切图
        :param plane: 剖切平面
        :param cut_model: 待切割实体
        :return:
        """
        from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section

        wire_model = BRepAlgoAPI_Section(cut_model, plane)  # 获取平面剖切后的线框图
        return wire_model

    def get_stair_rail_embedded_weld_cut_plane_point(self):
        """
        获取楼梯栏杆预埋件焊板剖切平面点
        :return:
        """
        rail_embedded_loc = self.rail_embedded_info.get_rail_embedded_part_loc()
        rail_embedded_config = self.rail_embedded_info.get_rail_datas()
        current_loc = rail_embedded_loc[1]
        thick = rail_embedded_config.t
        cut_point = [current_loc.x, current_loc.y, current_loc.z - thick / 2]
        return cut_point

    def get_stair_rail_embedded_weld_horizon_cut_drawing(self):
        """
        获取楼梯栏杆埋件焊板水平剖切数据：形成水平剖切轮廓
        :return:List[List[Tuple[float]]]
        """
        solid_model = self.composite_model.get_rail_embedded_weld_shape()
        current_point = self.get_stair_rail_embedded_weld_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 0, 1),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    @staticmethod
    def choose_inner_point_of_bounding_box_xz(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  # 判断线段是否在封闭多边形内部
                result_1 = pointInPolygon_xz(point_1, bounding_box_polyline)
                result_2 = pointInPolygon_xz(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    @staticmethod
    def choose_inner_point_of_bounding_box(
        bounding_box_loc: List[List[float]], cut_profile_point: List[List[Tuple[float]]]
    ):
        """
        选择包围框内部的点集合:yz平面
        """
        bounding_box_polyline = Polyline()  # 创建多边形
        for point in bounding_box_loc:
            p1 = Point3D()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = point[2]
            bounding_box_polyline.addPoint(p1)
        polyline = []
        for segment in cut_profile_point:
            current_seg = []
            total_num = len(segment)  # 判断点数量
            if total_num > 2:  # 曲线段的解决办法
                for num in range(total_num):
                    current_point = copy.deepcopy(segment[num])  # 获取线段当前点
                    point_ = Point3D()
                    point_.x = current_point[0]
                    point_.y = current_point[1]
                    point_.z = current_point[2]
                    result = pointInPolygon(
                        point_, bounding_box_polyline
                    )  # 判断点是否在多边形包围框内部
                    if result != 0:  # 保留包围框内部及包围框上的点
                        current_seg.append(current_point)
            elif total_num == 2:
                judge_seg = [list(segment[0]), list(segment[1])]
                point_1 = Point3D()
                point_1.x = segment[0][0]
                point_1.y = segment[0][1]
                point_1.z = segment[0][2]
                point_2 = Point3D()
                point_2.x = segment[1][0]
                point_2.y = segment[1][1]
                point_2.z = segment[1][2]
                intersection_points = segmentInPolygonPoint(
                    judge_seg, bounding_box_polyline
                )  #
                result_1 = pointInPolygon(point_1, bounding_box_polyline)  #
                result_2 = pointInPolygon(point_2, bounding_box_polyline)
                if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
                    current_seg.append(segment[0])
                    current_seg.append(segment[1])
                elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
                    current_seg.append(segment[0])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
                    current_seg.append(segment[1])
                    if len(intersection_points) == 1:
                        point_ = intersection_points[0]
                        current_seg.append((point_.x, point_.y, point_.z))
                    else:
                        logger.debug("无法构成线段！")
                elif (
                    result_1 == 0 and result_2 == 0 and len(intersection_points) == 2
                ):  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
                    # 相交两点
                    point_1 = intersection_points[0]
                    point_2 = intersection_points[1]
                    current_seg.append((point_1.x, point_1.y, point_1.z))
                    current_seg.append((point_2.x, point_2.y, point_2.z))
                else:  # 其它情况，两点可能相离。
                    pass
            else:
                logger.debug("生成的点集合不满足要求！")
            if len(current_seg) != 0:
                polyline.append(current_seg)
        return polyline

    # @staticmethod
    # def choose_inner_point_of_bounding_box_xz(bounding_box_loc: List[List[float]],
    #                                           cut_profile_point: List[List[Tuple[float]]]):
    #     """
    #     选择包围框内部的点集合:xz平面
    #     """
    #     bounding_box_polyline = Polyline()  # 创建多边形
    #     for point in bounding_box_loc:
    #         p1 = Point3D()
    #         p1.x = point[0]
    #         p1.y = point[1]
    #         p1.z = point[2]
    #         bounding_box_polyline.addPoint(p1)
    #     polyline = []
    #     for segment in cut_profile_point:
    #         current_seg = []
    #         total_num = len(segment)  # 判断点数量
    #         if total_num > 2:  # 曲线段的解决办法
    #             for num in range(total_num):
    #                 current_point = copy.deepcopy(segment[num])  # 获取线段当前点
    #                 point_ = Point3D()
    #                 point_.x = current_point[0]
    #                 point_.y = current_point[1]
    #                 point_.z = current_point[2]
    #                 result = pointInPolygon_xz(point_, bounding_box_polyline)  # 判断点是否在多边形包围框内部
    #                 if result != 0:  # 保留包围框内部及包围框上的点
    #                     current_seg.append(current_point)
    #         elif total_num == 2:
    #             judge_seg = [list(segment[0]), list(segment[1])]
    #             point_1 = Point3D()
    #             point_1.x = segment[0][0]
    #             point_1.y = segment[0][1]
    #             point_1.z = segment[0][2]
    #             point_2 = Point3D()
    #             point_2.x = segment[1][0]
    #             point_2.y = segment[1][1]
    #             point_2.z = segment[1][2]
    #             intersection_points = segmentInPolygonPoint(judge_seg, bounding_box_polyline)  #
    #             result_1 = pointInPolygon_xz(point_1, bounding_box_polyline)  #
    #             result_2 = pointInPolygon_xz(point_2, bounding_box_polyline)
    #             if result_1 != 0 and result_2 != 0:  # 两点皆在包围框内部
    #                 current_seg.append(segment[0])
    #                 current_seg.append(segment[1])
    #             elif result_1 != 0 and result_2 == 0:  # 第一点再包围框内，第二点再包围框外
    #                 current_seg.append(segment[0])
    #                 if len(intersection_points) == 1:
    #                     point_ = intersection_points[0]
    #                     current_seg.append((point_.x, point_.y, point_.z))
    #                 else:
    #                     logger.debug("无法构成线段！")
    #             elif result_1 == 0 and result_2 != 0:  # 第一点再包围框外，第二点在包围框内
    #                 current_seg.append(segment[1])
    #                 if len(intersection_points) == 1:
    #                     point_ = intersection_points[0]
    #                     current_seg.append((point_.x, point_.y, point_.z))
    #                 else:
    #                     logger.debug("无法构成线段！")
    #             elif result_1 == 0 and result_2 == 0 and len(
    #                     intersection_points) == 2:  # 两点都不在包围框内,线段与包围框是相离还是相交，相交则取出两交点，相离则不保留该线段上的点。
    #                 # 相交两点
    #                 point_1 = intersection_points[0]
    #                 point_2 = intersection_points[1]
    #                 current_seg.append((point_1.x, point_1.y, point_1.z))
    #                 current_seg.append((point_2.x, point_2.y, point_2.z))
    #             else:  # 其它情况，两点可能相离。
    #                 pass
    #         else:
    #             logger.debug("生成的点集合不满足要求！")
    #         if len(current_seg) != 0:
    #             polyline.append(current_seg)
    #     return polyline

    def get_stair_rail_embedded_weld_horizon_bounding_box_points(self):
        """
        获取楼梯栏杆预埋件焊板包围框
        :return:
        """
        cut_point = self.get_stair_rail_embedded_weld_cut_plane_point()
        tabu_b = self.tabu_b
        point_1 = [cut_point[0] - self.b0 / 2, cut_point[1] - tabu_b / 2, cut_point[2]]
        point_2 = copy.deepcopy(point_1)
        point_2[0] += self.b0
        point_3 = copy.deepcopy(point_2)
        point_3[1] += tabu_b
        point_4 = copy.deepcopy(point_3)
        point_4[0] += -self.b0
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_rail_embedded_weld_horizon_bounding_profile_points(self):
        """
        获取楼梯栏杆预埋件焊板包围轮廓点
        :return:
        """
        bounding_box_loc = (
            self.get_stair_rail_embedded_weld_horizon_bounding_box_points()
        )
        cut_profile_points = self.get_stair_rail_embedded_weld_horizon_cut_drawing()
        bounding_box_points = self.choose_inner_point_of_bounding_box(
            bounding_box_loc, cut_profile_points
        )
        return bounding_box_points

    def get_stair_rail_embedded_U_rebar_transverse_cut_plane_point(self):
        """
        获取楼梯栏杆预埋件U型钢筋横向剖切平面点
        :return:
        """
        origin_point = self.get_stair_rail_embedded_weld_cut_plane_point()  # 初始位置点
        rail_embedded_config = self.rail_embedded_info.get_rail_datas()
        edge_c = rail_embedded_config.c  # 钢筋边距
        edge_a = rail_embedded_config.a  # 横向边长-y方向
        space_l = (edge_a - 2 * edge_c) / 2
        cut_point = [origin_point[0], origin_point[1] - space_l, origin_point[2]]
        return cut_point

    def get_stair_rail_embedded_weld_transverse_cut_plane_point(self):
        """
        获取楼梯栏杆预埋件焊板横向剖切平面点
        :return:
        """
        origin_point = self.get_stair_rail_embedded_weld_cut_plane_point()  # 初始位置点--
        rail_embedded_config = self.rail_embedded_info.get_rail_datas()
        edge_c = rail_embedded_config.c  # 钢筋边距
        edge_a = rail_embedded_config.a  # 横向边长--y方向
        space_l = (edge_a - 2 * edge_c) / 2
        cut_point = [origin_point[0], origin_point[1] - space_l, origin_point[2]]
        return cut_point

    def get_stair_rail_embedded_U_rebar_transverse_cut_drawing(self):
        """
        获取楼梯栏杆预埋件U型钢筋横向剖切数据
        :return:
        """
        solid_model = self.composite_model.get_rail_embedded_U_rebar_shape()
        current_point = (
            self.get_stair_rail_embedded_U_rebar_transverse_cut_plane_point()
        )
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_rail_embedded_U_rebar_transverse_bounding_box_points(self):
        """
        获取楼梯栏杆预埋件U型钢筋横向包围框
        :return:
        """
        origin_point = self.get_stair_rail_embedded_U_rebar_transverse_cut_plane_point()
        tabu_b = self.tabu_b  # 踏步宽度
        total_h = self.h2 + self.h
        point_1 = copy.deepcopy(origin_point)
        point_1[0] += -self.b0 / 3
        point_1[2] = total_h
        point_2 = copy.deepcopy(point_1)
        point_2[2] = 0
        point_3 = copy.deepcopy(point_2)
        point_3[0] += 2 * self.b0 / 3
        point_4 = copy.deepcopy(point_3)
        point_4[2] = total_h
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_rail_embedded_U_rebar_transverse_bounding_profile_points(self):
        """
        获取楼梯栏杆预埋件U型钢筋横向向包围框内轮廓点
        :return:
        """
        bounding_box_loc = (
            self.get_stair_rail_embedded_U_rebar_transverse_bounding_box_points()
        )
        cut_profile_points = (
            self.get_stair_rail_embedded_U_rebar_transverse_cut_drawing()
        )
        bounding_box_points = self.choose_inner_point_of_bounding_box_xz(
            bounding_box_loc, cut_profile_points
        )
        return bounding_box_points

    def get_stair_rail_embedded_rabbet_transverse_cut_profile_points(self):
        """
        获取楼梯栏杆企口横向剖面轮廓形状点
        :return:
        """
        solid_model = self.composite_model.get_stair_solid_and_rail_rabbet_model()
        current_point = self.get_stair_rail_embedded_weld_transverse_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_rail_embedded_bounding_profile_change_profile_points(self):
        """
        开始楼梯栏杆埋件包围框改变后的轮廓点:仅改变x坐标
        :return:
        """
        cut_profile_points = (
            self.get_stair_rail_embedded_rabbet_transverse_cut_profile_points()
        )
        min_x = 2 * self.b0
        max_z = 0
        min_z = self.h2 + self.h
        for seg in cut_profile_points:
            for point in seg:
                if point[0] < min_x:
                    min_x = point[0]
                if point[2] < min_z:
                    min_z = point[2]
                if point[2] > max_z:
                    max_z = point[2]
        value_z = abs(max_z - min_z)
        change_profile_points = []
        for seg in cut_profile_points:
            if abs(max(seg[0][0], seg[1][0]) - min_x) > self.drawing_precision:
                change_profile_points.append([list(seg[0]), list(seg[1])])
        for seg in change_profile_points:
            if abs(seg[0][0] - min_x) < self.drawing_precision:
                seg[0][0] = 2 * self.b0 / 3
            if abs(seg[1][0] - min_x) < self.drawing_precision:
                seg[1][0] = 2 * self.b0 / 3
        for seg in change_profile_points:
            if abs(seg[0][2] - min_z) < self.drawing_precision:
                seg[0][2] += value_z / 2
            if abs(seg[1][2] - min_z) < self.drawing_precision:
                seg[1][2] += value_z / 2
        return change_profile_points

    @staticmethod
    def get_axis_mirror_points(
        segments: List[List[Tuple]], axis: str, point: List[float]
    ):
        """
        获取多线段关于轴对称后的点集合
        :param segments: List[List[Tuple[float],Tuple[float]]
        :param axis:
        :param point:
        :return:
        """
        if axis == "x":
            index = 0
        elif axis == "y":
            index = 1
        else:
            index = 2
        for seg in segments:
            for num in range(len(seg)):
                current_point = list(seg[num])
                add_ = point[index] - current_point[index]
                current_point[index] += 2 * add_
                seg[num] = tuple(current_point)
        return segments

    def adjust_z_value_to_form_profile_points(self, segments: List[List[Tuple]]):
        """
        减少实体下部轮廓高度
        :param segments:
        :return:
        """
        max_z = 0
        min_z = self.h2 + self.h
        for seg in segments:
            for point in seg:
                if point[2] < min_z:
                    min_z = point[2]
                if point[2] > max_z:
                    max_z = point[2]
        value_z = abs(max_z - min_z)
        for seg in segments:
            for num in range(len(seg)):
                current_p = list(seg[num])
                if abs(current_p[2] - min_z) < self.drawing_precision:
                    current_p[2] += value_z / 3
                seg[num] = tuple(current_p)
        return segments

    def get_stair_rail_embedded_rabbet_transverse_bounding_profile_points(self):
        """
        获取楼梯栏杆埋件企口包围框轮廓点
        :return:
        """
        bounding_box_loc = (
            self.get_stair_rail_embedded_weld_transverse_bounding_box_points()
        )
        cut_profile_points = (
            self.get_stair_rail_embedded_rabbet_transverse_cut_profile_points()
        )
        cut_plane_point = (
            self.get_stair_rail_embedded_weld_transverse_cut_plane_point()
        )  # 剖切平面点
        if cut_plane_point[0] < self.b0 / 2:
            bounding_box_points = self.choose_inner_point_of_bounding_box_xz(
                bounding_box_loc, cut_profile_points
            )
            index = "x"
            bounding_box_points = self.get_axis_mirror_points(
                bounding_box_points, index, cut_plane_point
            )
            bounding_box_points = self.adjust_z_value_to_form_profile_points(
                bounding_box_points
            )
        else:
            bounding_box_points = (
                self.get_stair_rail_embedded_bounding_profile_change_profile_points()
            )
        return bounding_box_points

    def get_stair_rail_embedded_rabbet_transverse_breakline_points(self):
        """
        获取楼梯栏杆埋件企口横向折断线点
        :return:
        """
        # 折断线基本参数
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        profile_points = (
            self.get_stair_rail_embedded_rabbet_transverse_bounding_profile_points()
        )
        min_x = self.b0  # 最小x值
        for seg in profile_points:
            for point in seg:
                if min_x > point[0]:
                    min_x = point[0]
        boundary_points = []
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        for seg in profile_points:
            for point in seg:
                if abs(point[0] - min_x) < self.drawing_precision:
                    boundary_points.append(list(point))
        if len(boundary_points) != 2:
            logger.debug("边缘点的数量有误！")
        else:
            if boundary_points[0][2] > boundary_points[1][2]:
                top_point[0] = boundary_points[0][0]
                top_point[1] = boundary_points[0][1]
                top_point[2] = boundary_points[0][2]
                bottom_point[0] = boundary_points[1][0]
                bottom_point[1] = boundary_points[1][1]
                bottom_point[2] = boundary_points[1][2]
            else:
                top_point[0] = boundary_points[1][0]
                top_point[1] = boundary_points[1][1]
                top_point[2] = boundary_points[1][2]
                bottom_point[0] = boundary_points[0][0]
                bottom_point[1] = boundary_points[0][1]
                bottom_point[2] = boundary_points[0][2]
        # 开始设置关键点
        mid_point = [
            (top_point[0] + bottom_point[0]) / 2,
            (top_point[1] + bottom_point[1]) / 2,
            (top_point[2] + bottom_point[2]) / 2,
        ]
        point_1 = copy.deepcopy(top_point)
        point_1[2] += extend_l
        point_2 = copy.deepcopy(top_point)
        point_3 = copy.deepcopy(top_point)
        point_3[2] = mid_point[2] + l_space / 2
        point_4 = copy.deepcopy(point_3)
        point_4[0] += offset * math.cos(theta)
        point_4[2] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_3)
        point_5[0] += -offset * math.cos(theta)
        point_5[2] += -l_space + offset * math.sin(theta)
        point_6 = copy.deepcopy(point_3)
        point_6[2] += -l_space
        point_7 = copy.deepcopy(bottom_point)
        point_8 = copy.deepcopy(point_7)
        point_8[2] += -extend_l
        draw_lines = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
            point_8,
        ]
        return draw_lines

    def get_stair_rail_embedded_U_rebar_horizon_cut_plane_point(self):
        """
        获取楼梯栏杆埋件U型钢筋水平剖切平面点
        :return:
        """
        origin_point = self.get_stair_rail_embedded_weld_cut_plane_point()
        rail_embedded_config = self.rail_embedded_info.get_rail_datas()
        thick = rail_embedded_config.t  # 焊接钢板厚度
        current_point = copy.deepcopy(origin_point)
        current_point[2] += -thick
        return current_point

    def get_stair_rail_embedded_U_rebar_horizon_bounding_box_points(self):
        """
        获取楼梯栏杆埋件U型钢筋水平剖切包围框点
        :return:
        """
        current_point = self.get_stair_rail_embedded_U_rebar_horizon_cut_plane_point()
        tabu_b = self.tabu_b  # 踏步宽度
        point_1 = copy.deepcopy(current_point)
        point_1[0] = self.b0 / 2
        point_1[1] += tabu_b / 2
        point_2 = copy.deepcopy(point_1)
        point_2[1] += -tabu_b
        point_3 = copy.deepcopy(point_2)
        point_3[0] += self.b0 / 2
        point_4 = copy.deepcopy(point_3)
        point_4[1] += tabu_b
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_rail_embedded_U_rebar_horizon_points(self):
        """
        获取楼梯栏杆埋件U型钢筋水平轮廓点----
        :return:
        """
        solid_model = self.composite_model.get_rail_embedded_U_rebar_shape()
        current_point = self.get_stair_rail_embedded_U_rebar_horizon_cut_plane_point()
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 0, 1),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_rail_embedded_weld_transverse_cut_drawing(self):
        """
        获取楼梯栏杆埋件焊板横向剖切数据
        :return:
        """
        solid_model = self.composite_model.get_rail_embedded_weld_shape()
        current_point = (
            self.get_stair_rail_embedded_weld_transverse_cut_plane_point()
        )  # 横向剖切点相同
        plane = gp_Pln(
            gp_Pnt(current_point[0], current_point[1], current_point[2]),
            gp_Dir(0, 1, 0),
        )  # 剖切平面
        compound_ = self.get_solid_cut_drawing(
            plane, solid_model
        ).Shape()  # TopoDS_compound
        edge = []
        if compound_:
            edge += list(TopologyExplorer(compound_).edges())
        points_ = []
        for edge_ in edge:
            points_3d = discretize_edge(edge_, self.drawing_precision)  # 转换成点
            points_.append(points_3d)
        return points_

    def get_stair_rail_embedded_weld_transverse_bounding_box_points(self):
        """
        获取楼梯栏杆埋件焊板横向包围框点
        :return:
        """
        cut_plane_point = self.get_stair_rail_embedded_weld_transverse_cut_plane_point()
        total_h = self.h2 + self.h
        point_1 = copy.deepcopy(cut_plane_point)
        point_1[0] += -self.b0 / 5
        point_1[2] = total_h
        point_2 = copy.deepcopy(point_1)
        point_2[2] = 0
        point_3 = copy.deepcopy(point_2)
        point_3[0] += 2 * self.b0 / 5
        point_4 = copy.deepcopy(point_3)
        point_4[2] = total_h
        bounding_box_loc = [point_1, point_2, point_3, point_4, point_1]
        return bounding_box_loc

    def get_stair_rail_embedded_weld_transverse_bounding_profile_points(self):
        """
        获取楼梯栏杆预埋件焊板横向包围框内轮廓点
        :return:
        """
        bounding_box_loc = (
            self.get_stair_rail_embedded_weld_transverse_bounding_box_points()
        )
        cut_profile_points = (
            self.get_stair_rail_embedded_weld_transverse_cut_drawing()
        )  #
        bounding_box_points = self.choose_inner_point_of_bounding_box_xz(
            bounding_box_loc, cut_profile_points
        )
        return bounding_box_points

    def get_stair_rail_embedded_U_rebar_horizon_bounding_profile_type_points(self):
        """
        获取楼梯栏杆埋件U型钢筋水平包围框内的轮廓点:获取每个圆的线段点
        :return:
        """
        current_point = self.get_stair_rail_embedded_U_rebar_horizon_cut_plane_point()
        rebar_profile_points = (
            self.get_stair_rail_embedded_U_rebar_horizon_points()
        )  # 钢筋水平轮廓点
        limit_x = current_point[0]
        limit_y = current_point[1]
        update_rebar_profile_points_r = []
        update_rebar_profile_points_l = []  # 筛选后的列表
        # 若左右侧均有埋件，则需要分左右考虑
        if len(rebar_profile_points) > 4:  # 钢筋数目多，表示两侧皆有栏杆埋件
            for num in range(len(rebar_profile_points)):
                point_ = list(rebar_profile_points[num][0])
                if point_[0] > self.b0 / 2:
                    update_rebar_profile_points_r.append(rebar_profile_points[num])
                else:
                    update_rebar_profile_points_l.append(rebar_profile_points[num])
        if len(rebar_profile_points) > 4:
            if current_point[0] > self.b0 / 2:
                update_rebar_profile_points = copy.deepcopy(
                    update_rebar_profile_points_r
                )
            else:
                update_rebar_profile_points = copy.deepcopy(
                    update_rebar_profile_points_l
                )
        else:
            update_rebar_profile_points = copy.deepcopy(rebar_profile_points)
        top_left_rebar = []  # 顶部左侧钢筋
        top_right_rebar = []  # 顶部右侧钢筋
        bottom_left_rebar = []  # 底部左侧钢筋
        bottom_right_rebar = []  # 底部右侧钢筋
        for seg in update_rebar_profile_points:
            if (seg[0][0] < limit_x and seg[1][0] < limit_x) and (
                seg[0][1] > limit_y and seg[1][1] > limit_y
            ):
                bottom_left_rebar.append(seg)
            elif (seg[0][0] > limit_x and seg[1][0] > limit_x) and (
                seg[0][1] > limit_y and seg[1][1] > limit_y
            ):
                top_left_rebar.append(seg)
            elif (seg[0][0] > limit_x and seg[1][0] > limit_x) and (
                seg[0][1] < limit_y and seg[1][1] < limit_y
            ):
                top_right_rebar.append(seg)
            else:
                bottom_right_rebar.append(seg)
        total_type_rebar_points = [
            top_left_rebar,
            top_right_rebar,
            bottom_left_rebar,
            bottom_right_rebar,
        ]
        return total_type_rebar_points

    @staticmethod
    def calculate_segments_about_circle(seg_points):
        """
        线段点集合--形成圆的线段组合
        :param seg_points:
        :return:
        """
        loc_x = 0
        loc_y = 0
        loc_z = 0
        num = len(seg_points[0])  # 点的数量
        for seg in seg_points:
            for num in range(len(seg) - 1):
                point = seg[num]
                loc_x += point[0]
                loc_y += point[1]
                loc_z += point[2]
        center = [round(loc_x / num, 6), round(loc_y / num, 6), round(loc_z / num, 6)]
        point_ = seg_points[0][0]
        radius = np.linalg.norm(np.array(center) - np.array(list(point_)))
        diameter = round(radius * 2)
        return center, diameter

    def get_stair_rail_embedded_total_U_rebar_center_diameter_data(self):
        """
        获取楼梯栏杆埋件所有钢筋圆心和直径
        :return:
        """
        total_rebar_points = (
            self.get_stair_rail_embedded_U_rebar_horizon_bounding_profile_type_points()
        )
        current_point = self.get_stair_rail_embedded_weld_cut_plane_point()
        rail_info = self.rail_embedded_info.get_rail_datas()
        diameter = rail_info.fi  # 钢筋直径
        edge_c = rail_info.c
        loc_y = rail_info.a
        loc_x = rail_info.b
        top_left_p = copy.deepcopy(current_point)
        top_left_p[0] += -(loc_x - 2 * edge_c) / 2
        top_left_p[1] += (loc_y - 2 * edge_c) / 2
        top_right_p = copy.deepcopy(current_point)
        top_right_p[0] += (loc_x - 2 * edge_c) / 2
        top_right_p[1] += (loc_y - 2 * edge_c) / 2
        bottom_left_p = copy.deepcopy(current_point)
        bottom_left_p[0] += -(loc_x - 2 * edge_c) / 2
        bottom_left_p[1] += -(loc_y - 2 * edge_c) / 2
        bottom_right_p = copy.deepcopy(current_point)
        bottom_right_p[0] += (loc_x - 2 * edge_c) / 2
        bottom_right_p[1] += -(loc_y - 2 * edge_c) / 2
        rebar_center = [top_left_p, top_right_p, bottom_left_p, bottom_right_p]  # 钢筋圆心
        rebar_diameter = [diameter, diameter, diameter, diameter]  # 钢筋直径
        return rebar_center, rebar_diameter

    def get_stair_rail_embedded_U_rebar_horizon_profile_points(self):
        """
        获取楼梯栏杆埋件U型钢筋轮廓点
        :return:
        """
        (
            rebar_center,
            rebar_diameter,
        ) = self.get_stair_rail_embedded_total_U_rebar_center_diameter_data()
        # 绘制直线点
        top_left_p = rebar_center[0]
        top_left_d = rebar_diameter[0]
        top_right_p = rebar_center[1]
        top_right_d = rebar_diameter[1]
        bottom_left_p = rebar_center[2]
        bottom_left_d = rebar_diameter[2]
        bottom_right_p = rebar_center[3]
        bottom_right_d = rebar_diameter[3]
        # 顶部两钢筋直线
        top_p1 = copy.deepcopy(top_left_p)
        top_p1[1] += top_left_d / 2
        top_p2 = copy.deepcopy(top_left_p)
        top_p2[1] += -top_left_d / 2
        top_p3 = copy.deepcopy(top_right_p)
        top_p3[1] += top_right_d / 2
        top_p4 = copy.deepcopy(top_right_p)
        top_p4[1] += -top_right_d / 2
        # 底部两钢筋直线
        bottom_p1 = copy.deepcopy(bottom_left_p)
        bottom_p1[1] += bottom_left_d / 2
        bottom_p2 = copy.deepcopy(bottom_left_p)
        bottom_p2[1] += -bottom_left_d / 2
        bottom_p3 = copy.deepcopy(bottom_right_p)
        bottom_p3[1] += bottom_right_d / 2
        bottom_p4 = copy.deepcopy(bottom_right_p)
        bottom_p4[1] += -bottom_right_d / 2
        draw_lines = [
            [top_p1, top_p3],
            [top_p2, top_p4],
            [bottom_p1, bottom_p3],
            [bottom_p2, bottom_p4],
        ]  #
        draw_circles = [rebar_center, rebar_diameter]
        rebar_draw_info = {}
        rebar_draw_info["draw_circles"] = draw_circles
        rebar_draw_info["draw_lines"] = draw_lines
        return rebar_draw_info

    def get_stair_rail_embedded_weld_profile_boundary_points(self):
        """
        获取楼梯栏杆埋件焊板轮廓边界点
        :return:
        """
        weld_profile_points = (
            self.get_stair_rail_embedded_weld_horizon_bounding_profile_points()
        )  # 获取焊板轮廓点
        min_x = self.b0
        max_x = 0
        min_y = self.lb_d + self.ln + self.lt_d
        max_y = 0
        current_z = weld_profile_points[0][0][2]
        for seg in weld_profile_points:
            for point in seg:
                if min_x > point[0]:
                    min_x = point[0]
                if min_y > point[1]:
                    min_y = point[1]
                if max_x < point[0]:
                    max_x = point[0]
                if max_y < point[1]:
                    max_y = point[1]
        top_left_point = [min_x, max_y, current_z]
        top_right_point = [max_x, max_y, current_z]
        bottom_left_point = [min_x, min_y, current_z]
        bottom_right_point = [max_x, min_y, current_z]
        bounding_box_loc = [
            top_left_point,
            top_right_point,
            bottom_left_point,
            bottom_right_point,
        ]
        return bounding_box_loc

    def get_stair_rail_embedded_left_view_dimension_points(self):
        """
        获取楼梯栏杆埋件左侧图标注点
        :return:
        """
        (
            rebar_center,
            rebar_diameter,
        ) = self.get_stair_rail_embedded_total_U_rebar_center_diameter_data()
        boundary_points = (
            self.get_stair_rail_embedded_weld_profile_boundary_points()
        )  # 楼梯栏杆埋件焊板轮廓边界点
        point_1 = boundary_points[0]
        point_2 = boundary_points[1]
        point_3 = boundary_points[2]
        point_4 = boundary_points[3]
        # 钢筋坐标点--由右向左逆时针
        point_s_1 = copy.deepcopy(rebar_center[1])
        point_s_1[1] = point_2[1]
        point_s_2 = copy.deepcopy(rebar_center[0])
        point_s_2[1] = point_1[1]
        point_s_3 = copy.deepcopy(rebar_center[0])
        point_s_3[0] = point_1[0]
        point_s_4 = copy.deepcopy(rebar_center[2])
        point_s_4[0] = point_1[0]
        dimension_loc = [
            [point_2, point_s_1],
            [point_s_1, point_s_2],
            [point_s_2, point_1],
            [point_1, point_s_3],
            [point_s_3, point_s_4],
            [point_s_4, point_3],
        ]
        return dimension_loc

    def get_stair_rail_embedded_left_view_outline_points(self):
        """
        获取楼梯栏杆埋件左侧视图引出线点
        :return:
        """
        outline_length = 60  # 引出线长度
        boundary_points = (
            self.get_stair_rail_embedded_weld_profile_boundary_points()
        )  # 楼梯栏杆埋件焊板轮廓边界点
        point_1 = boundary_points[0]
        point_2 = boundary_points[1]
        point_3 = boundary_points[2]
        point_4 = boundary_points[3]
        left_point = [
            (point_1[0] + point_3[0]) / 2,
            (point_1[1] + point_3[1]) / 2,
            (point_1[2] + point_3[2]) / 2,
        ]
        right_point = [
            (point_2[0] + point_4[0]) / 2,
            (point_2[1] + point_4[1]) / 2,
            (point_2[2] + point_4[2]) / 2,
        ]
        left_p1 = copy.deepcopy(left_point)
        left_p1[0] += -2 * outline_length
        left_p2 = copy.deepcopy(left_p1)
        left_p2[0] += -outline_length
        right_p1 = copy.deepcopy(right_point)
        right_p1[0] += outline_length / 2
        right_p2 = copy.deepcopy(right_p1)
        right_p2[0] += outline_length
        left_text = "A"
        right_text = "A"
        left_mid_loc = [
            (left_p1[0] + left_p2[0]) / 2,
            (left_p1[1] + left_p2[1]) / 2,
            (left_p1[2] + left_p2[2]) / 2,
        ]
        right_mid_loc = [
            (right_p1[0] + right_p2[0]) / 2,
            (right_p1[1] + right_p2[1]) / 2,
            (right_p1[2] + right_p2[2]) / 2,
        ]
        draw_lines = [[left_p1, left_p2], [right_p1, right_p2]]
        special_text = [[left_mid_loc, right_mid_loc], [left_text, right_text]]
        # 信息汇总
        outline_info = {}
        outline_info["draw_lines"] = draw_lines
        outline_info["special_text"] = special_text
        return outline_info

    def get_stair_rail_left_view_place_points(self):
        """
        获取楼梯栏杆左侧视图放置点
        :return:
        """
        rail_info = self.rail_embedded_info.get_rail_datas()
        total_y = rail_info.a  # 栏杆埋件y向长度
        total_x = rail_info.b  # 栏杆埋件x向长度
        loc_x_1 = -2 * max(total_x, total_y)
        loc_x_2 = 0
        loc_x_3 = self.b0 / 2
        value_x = [loc_x_1, loc_x_2, loc_x_3]
        return value_x

    def get_stair_rail_embedded_mid_view_dimension_points(self):
        """
        获取楼梯栏杆埋件中部视图尺寸标注点
        :return:
        """
        weld_transverse_profile = (
            self.get_stair_rail_embedded_weld_transverse_bounding_profile_points()
        )  # 焊板横向轮廓点
        rebar_transverse_profile = (
            self.get_stair_rail_embedded_U_rebar_transverse_bounding_profile_points()
        )  # U型钢筋横向轮廓点
        # 获取焊板横向轮廓上最小x坐标点
        weld_min_x = self.b0
        rebar_min_z = self.h2 + self.h
        for seg in weld_transverse_profile:
            for point in seg:
                if weld_min_x > point[0]:
                    weld_min_x = point[0]
        for seg in rebar_transverse_profile:
            for point in seg:
                if rebar_min_z > point[2]:
                    rebar_min_z = point[2]
        # 获取焊板左侧线段
        weld_left_seg = []
        for seg in weld_transverse_profile:
            if (
                abs(seg[0][0] - weld_min_x) < self.drawing_precision
                and abs(seg[1][0] - weld_min_x) < self.drawing_precision
            ):
                weld_left_seg.append(seg[0])
                weld_left_seg.append(seg[1])
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        if len(weld_left_seg) != 2:
            logger.debug("获取焊板左侧边点数目有误！")
        else:
            if weld_left_seg[0][2] > weld_left_seg[1][2]:
                top_point[0] = weld_left_seg[0][0]
                top_point[1] = weld_left_seg[0][1]
                top_point[2] = weld_left_seg[0][2]
                bottom_point[0] = weld_left_seg[1][0]
                bottom_point[1] = weld_left_seg[1][1]
                bottom_point[2] = weld_left_seg[1][2]
            else:
                top_point[0] = weld_left_seg[1][0]
                top_point[1] = weld_left_seg[1][1]
                top_point[2] = weld_left_seg[1][2]
                bottom_point[0] = weld_left_seg[0][0]
                bottom_point[1] = weld_left_seg[0][1]
                bottom_point[2] = weld_left_seg[0][2]
        point_3 = copy.deepcopy(top_point)
        point_3[2] = rebar_min_z
        dimension_loc = [[top_point, bottom_point], [bottom_point, point_3]]
        return dimension_loc

    def get_stair_rail_embedded_mid_view_hatch_outline_points(self):
        """
        获取栏杆埋件中部视图填充和引出线坐标点
        :return:
        """
        weld_thickness = 5  # 焊接厚度
        weld_transverse_profile = (
            self.get_stair_rail_embedded_weld_transverse_bounding_profile_points()
        )  # 焊板横向轮廓点
        rebar_transverse_profile = (
            self.get_stair_rail_embedded_U_rebar_transverse_bounding_profile_points()
        )  # U型钢筋横向轮廓点
        # 获取焊板右下角点
        current_y = weld_transverse_profile[0][0][1]
        weld_min_z = self.h2 + self.h
        weld_max_x = 0
        for seg in weld_transverse_profile:
            for point in seg:
                if weld_min_z > point[2]:
                    weld_min_z = point[2]
                if weld_max_x < point[0]:
                    weld_max_x = point[0]
        weld_right_bottom_point = [weld_max_x, current_y, weld_min_z]
        rebar_max_z = 0
        for seg in rebar_transverse_profile:
            for point in seg:
                if rebar_max_z < point[2]:
                    rebar_max_z = point[2]
        rebar_top_seg = []
        for seg in rebar_transverse_profile:
            if (
                len(seg) == 2
                and abs(seg[0][2] - rebar_max_z) < self.drawing_precision
                and abs(seg[1][2] - rebar_max_z) < self.drawing_precision
            ):
                rebar_top_seg.append(seg)
        if rebar_top_seg[0][0][0] > rebar_top_seg[1][0][0]:
            rebar_right_seg = [rebar_top_seg[0][0], rebar_top_seg[0][1]]
            rebar_left_seg = [rebar_top_seg[1][0], rebar_top_seg[1][1]]
        else:
            rebar_right_seg = [rebar_top_seg[1][0], rebar_top_seg[1][1]]
            rebar_left_seg = [rebar_top_seg[0][0], rebar_top_seg[0][1]]
        # 判断各线段内各点大小
        left_p1 = [0, 0, 0]
        left_p2 = [0, 0, 0]
        right_p1 = [0, 0, 0]
        right_p2 = [0, 0, 0]
        if rebar_left_seg[0][0] > rebar_left_seg[1][0]:
            left_p1[0] = rebar_left_seg[1][0]
            left_p1[1] = rebar_left_seg[1][1]
            left_p1[2] = rebar_left_seg[1][2]
            left_p2[0] = rebar_left_seg[0][0]
            left_p2[1] = rebar_left_seg[0][1]
            left_p2[2] = rebar_left_seg[0][2]
        else:
            left_p1[0] = rebar_left_seg[0][0]
            left_p1[1] = rebar_left_seg[0][1]
            left_p1[2] = rebar_left_seg[0][2]
            left_p2[0] = rebar_left_seg[1][0]
            left_p2[1] = rebar_left_seg[1][1]
            left_p2[2] = rebar_left_seg[1][2]
        if rebar_right_seg[0][0] > rebar_right_seg[1][0]:
            right_p1[0] = rebar_right_seg[1][0]
            right_p1[1] = rebar_right_seg[1][1]
            right_p1[2] = rebar_right_seg[1][2]
            right_p2[0] = rebar_right_seg[0][0]
            right_p2[1] = rebar_right_seg[0][1]
            right_p2[2] = rebar_right_seg[0][2]
        else:
            right_p1[0] = rebar_right_seg[0][0]
            right_p1[1] = rebar_right_seg[0][1]
            right_p1[2] = rebar_right_seg[0][2]
            right_p2[0] = rebar_right_seg[1][0]
            right_p2[1] = rebar_right_seg[1][1]
            right_p2[2] = rebar_right_seg[1][2]
        left_mid_p = [
            (left_p1[0] + left_p2[0]) / 2,
            (left_p1[1] + left_p2[1]) / 2,
            (left_p1[2] + left_p2[2]) / 2,
        ]
        right_mid_p = [
            (right_p1[0] + right_p2[0]) / 2,
            (right_p1[1] + right_p2[1]) / 2,
            (right_p1[2] + right_p2[2]) / 2,
        ]
        rebar_mid_p = [0, 0, 0]
        for seg in rebar_transverse_profile:
            if (
                len(seg) == 2
                and seg[0][0] > right_mid_p[0]
                and seg[1][0] > right_mid_p[0]
            ):
                rebar_mid_p[0] = (seg[0][0] + seg[1][0]) / 2
                rebar_mid_p[1] = (seg[0][1] + seg[1][1]) / 2
                rebar_mid_p[2] = (seg[0][2] + seg[1][2]) / 2
        # 填充点集合
        # 左侧填充点
        point_1_l = copy.deepcopy(left_p1)
        point_1_l[0] += -weld_thickness
        point_2_l = copy.deepcopy(left_p1)
        point_2_l[2] += -weld_thickness
        point_3_l = copy.deepcopy(left_p2)
        point_3_l[2] += -weld_thickness
        point_4_l = copy.deepcopy(left_p2)
        point_4_l[0] += weld_thickness
        # 右侧填充点
        point_1_r = copy.deepcopy(right_p1)
        point_1_r[0] += -weld_thickness
        point_2_r = copy.deepcopy(right_p1)
        point_2_r[2] += -weld_thickness
        point_3_r = copy.deepcopy(right_p2)
        point_3_r[2] += -weld_thickness
        point_4_r = copy.deepcopy(right_p2)
        point_4_r[0] += weld_thickness
        left_hatch = [point_1_l, point_2_l, point_3_l, point_4_l]
        right_hatch = [point_1_r, point_2_r, point_3_r, point_4_r]
        # 绘制引出线
        outline_length_1 = 50
        outline_length_2 = 50
        theta = math.pi / 3
        # 焊板引出线
        point_w_1 = copy.deepcopy(weld_right_bottom_point)
        point_w_2 = copy.deepcopy(point_w_1)
        point_w_2[0] += outline_length_1 * math.cos(theta)
        point_w_2[2] += -outline_length_1 * math.sin(theta)
        point_w_3 = copy.deepcopy(point_w_2)
        point_w_3[0] += outline_length_2
        outline_weld = [[point_w_1, point_w_2], [point_w_2, point_w_3]]
        text_loc_1 = point_w_2
        text_1 = "Q345B"
        outline_text_w = [text_loc_1, text_1]
        # 钢筋引出线
        point_r_1 = copy.deepcopy(rebar_mid_p)
        point_r_2 = copy.deepcopy(point_r_1)
        point_r_2[0] += outline_length_1 * math.cos(theta)
        point_r_2[2] += -outline_length_1 * math.sin(theta)
        point_r_3 = copy.deepcopy(point_r_2)
        point_r_3[0] += outline_length_2
        outline_rebar = [[point_r_1, point_r_2], [point_r_2, point_r_3]]
        rail_info = self.rail_embedded_info.get_rail_datas()
        diameter = rail_info.fi  # 钢筋直径
        text_loc_2 = point_r_2
        text_2 = "Ф" + str(diameter)
        outline_text_r = [text_loc_2, text_2]
        hatch_info = {}
        hatch_info["left_hatch"] = left_hatch
        hatch_info["right_hatch"] = right_hatch
        hatch_info["outline_weld"] = outline_weld
        hatch_info["outline_rebar"] = outline_rebar
        hatch_info["outline_text_w"] = outline_text_w
        hatch_info["outline_text_r"] = outline_text_r
        return hatch_info

    def get_stair_rail_embedded_right_view_dimension_points(self):
        """
        获取楼梯栏杆埋件右侧视图尺寸标注点
        :return:
        """
        weld_transverse_profile = (
            self.get_stair_rail_embedded_weld_transverse_bounding_profile_points()
        )  # 焊板横向轮廓点
        rebar_transverse_profile = (
            self.get_stair_rail_embedded_U_rebar_transverse_bounding_profile_points()
        )  # U型钢筋横向轮廓点
        # 获取焊板横向轮廓上最小x坐标点
        weld_min_x = self.b0  # 焊板横向轮廓x的最小值
        weld_max_x = 0  # 焊板横向轮廓x的最大值
        rebar_min_z = self.h2 + self.h
        for seg in weld_transverse_profile:
            for point in seg:
                if weld_min_x > point[0]:
                    weld_min_x = point[0]
                if weld_max_x < point[0]:
                    weld_max_x = point[0]
        for seg in rebar_transverse_profile:
            for point in seg:
                if rebar_min_z > point[2]:
                    rebar_min_z = point[2]
        # 获取焊板左侧线段
        weld_left_seg = []
        for seg in weld_transverse_profile:
            if (
                abs(seg[0][0] - weld_min_x) < self.drawing_precision
                and abs(seg[1][0] - weld_min_x) < self.drawing_precision
            ):
                weld_left_seg.append(seg[0])
                weld_left_seg.append(seg[1])
        top_point = [0, 0, 0]
        bottom_point = [0, 0, 0]
        if len(weld_left_seg) != 2:
            logger.debug("获取焊板左侧边点数目有误！")
        else:
            if weld_left_seg[0][2] > weld_left_seg[1][2]:
                top_point[0] = weld_left_seg[0][0]
                top_point[1] = weld_left_seg[0][1]
                top_point[2] = weld_left_seg[0][2]
                bottom_point[0] = weld_left_seg[1][0]
                bottom_point[1] = weld_left_seg[1][1]
                bottom_point[2] = weld_left_seg[1][2]
            else:
                top_point[0] = weld_left_seg[1][0]
                top_point[1] = weld_left_seg[1][1]
                top_point[2] = weld_left_seg[1][2]
                bottom_point[0] = weld_left_seg[0][0]
                bottom_point[1] = weld_left_seg[0][1]
                bottom_point[2] = weld_left_seg[0][2]
        # 获取楼梯横向轮廓边界点
        profile_points = (
            self.get_stair_rail_embedded_rabbet_transverse_bounding_profile_points()
        )
        stair_min_x = self.b0
        stair_max_x = 0
        stair_min_z = self.h2 + self.h
        stair_max_z = 0
        for seg in profile_points:
            for point in seg:
                if stair_min_x > point[0]:
                    stair_min_x = point[0]
                if stair_max_x < point[0]:
                    stair_max_x = point[0]
                if stair_min_z > point[2]:
                    stair_min_z = point[2]
                if stair_max_z < point[2]:
                    stair_max_z = point[2]
        stair_left_point = [0, 0, 0]
        stair_right_point = [0, 0, 0]
        for seg in profile_points:
            if (
                abs(seg[0][0] - stair_min_x) < self.drawing_precision
                and abs(seg[0][2] - stair_max_z) < self.drawing_precision
            ):
                stair_left_point[0] = seg[1][0]
                stair_left_point[1] = seg[1][1]
                stair_left_point[2] = seg[1][2]
            if (
                abs(seg[1][0] - stair_min_x) < self.drawing_precision
                and abs(seg[1][2] - stair_max_z) < self.drawing_precision
            ):
                stair_left_point[0] = seg[0][0]
                stair_left_point[1] = seg[0][1]
                stair_left_point[2] = seg[0][2]
            if (
                abs(seg[0][0] - stair_max_x) < self.drawing_precision
                and abs(seg[0][2] - stair_max_z) < self.drawing_precision
            ):
                stair_right_point[0] = seg[1][0]
                stair_right_point[1] = seg[1][1]
                stair_right_point[2] = seg[1][2]
            if (
                abs(seg[1][0] - stair_max_x) < self.drawing_precision
                and abs(seg[1][2] - stair_max_z) < self.drawing_precision
            ):
                stair_right_point[0] = seg[0][0]
                stair_right_point[1] = seg[0][1]
                stair_right_point[2] = seg[0][2]
        # 左侧轮廓标注点
        left_point_1 = copy.deepcopy(top_point)
        left_point_1[2] = stair_max_z
        left_point_2 = copy.deepcopy(top_point)
        left_point_3 = copy.deepcopy(bottom_point)
        left_point_4 = copy.deepcopy(top_point)
        left_point_4[2] = rebar_min_z
        # 顶部轮廓标注点
        top_point_1 = [stair_max_x, profile_points[0][0][1], stair_max_z]
        top_point_2 = copy.deepcopy(stair_right_point)
        top_point_3 = copy.deepcopy(top_point_2)
        top_point_3[0] = weld_max_x
        top_point_4 = copy.deepcopy(stair_left_point)
        top_point_4[0] = weld_min_x
        top_point_5 = copy.deepcopy(stair_left_point)
        # 右侧特殊标注
        right_point_1 = copy.deepcopy(top_point_1)
        right_point_1[2] = stair_min_z
        right_point_2 = copy.deepcopy(top_point_1)
        basic_dimension_loc = [
            [top_point_1, top_point_2],
            [top_point_2, top_point_3],
            [top_point_3, top_point_4],
            [top_point_4, top_point_5],
            [left_point_1, left_point_2],
            [left_point_2, left_point_3],
            [left_point_3, left_point_4],
        ]
        text = "梯板厚度"
        special_dimension_loc = [[right_point_1, right_point_2], text]
        dimension_info = {}
        dimension_info["basic_dimension_loc"] = basic_dimension_loc
        dimension_info["special_dimension_loc"] = special_dimension_loc
        return dimension_info
