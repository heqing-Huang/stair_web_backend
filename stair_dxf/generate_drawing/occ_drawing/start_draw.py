"""
开始绘图，获取数据
"""
import copy

import numpy as np

from stair_dxf.stair_design.countrebar import (
    BottomLongitudinalRebar,
    TopLongitudinalRebar,
    MidDistributionRebar,
    BottomEdgeLongitudinalRebar,
    TopEdgeLongitudinalRebar,
    HoistingReinforceLongitudinalRebar,
    HoistingReinforcePointRebar,
    BottomEdgeReinforceRebar,
    BottomEdgeStirrup,
    TopEdgeReinforceRebar,
    HoleReinforceRebar,
    DemoldEmbeddedPartsLoc,
    HoistingEmbeddedPartsLoc,
    RailingEmbeddedPart,
    HoleLocation,
    TopEdgeStirrup,
    WaterDripLoc,
    StepSlotLoc,
    InternalCorner,
    ExternalCorner,
    LadderBeamAndSlabLoc,
    ConnectEmbeddedPartLoc,
)
from stair_dxf.stair_design.sidegeomentry import StairGeometry

import math
from typing import List, Dict
import logging
import time

logger = logging.getLogger(__name__)


class OCCData(object):
    """
    建立三维模型所需数据，所有数据皆为深化设计后的参数。
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ) -> None:
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.geom_solid = StairGeometry(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.generate_basic_data()  # 初始化几何数据
        self.generate_whole_rebar_datas()  # 产生钢筋坐标信息
        self.generate_construct_datas()  # 产生构造信息
        self.generate_embedded_part_datas()  # 产生预埋件信息

    def generate_basic_data(self):
        """
        产生设计所需基本参数，更新设计需要的参数
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

    def generate_construct_datas(self):
        """
        产生构造信息：孔洞定位信息，滴水线槽信息，防滑槽信息
        :return:
        """
        # 建立孔洞信息
        self.hole_info: HoleLocation = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        # 建立防滑槽信息
        self.step_slot_info: StepSlotLoc = StepSlotLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        # 建立滴水线槽信息
        self.water_drip_info: WaterDripLoc = WaterDripLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        self.internal_info: InternalCorner = InternalCorner(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 阴角信息
        self.external_info: ExternalCorner = ExternalCorner(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 阳角信息
        # 平台板和平台梁及连接节点信息
        self.ladder_beam_slab_loc = LadderBeamAndSlabLoc(
            self.slab_struct, self.detail_slab
        )  # 楼梯顶部和底部平台梁和平台板的位置信息

        self.connect_embedded_part_loc = ConnectEmbeddedPartLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 连接预埋件部件信息

    def generate_whole_rebar_datas(self):
        """
        产生countrebar中所有钢筋数据
        :return:
        """
        self.mid_distribute_rebar = MidDistributionRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_long_rebar = TopLongitudinalRebar(
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
        )
        self.bottom_edge_long_rebar = BottomEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_edge_long_rebar = TopEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.bottom_edge_stir = BottomEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )

        self.top_edge_stir = TopEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )

        self.hole_rein_rebar = HoleReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.hoist_rein_long_rebar = HoistingReinforceLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.hoist_rein_point_rebar = HoistingReinforcePointRebar(
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
        )
        self.bottom_edge_rein_rebar = BottomEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )

    def generate_embedded_part_datas(self):
        """
        产生所有预埋件数据
        :return:
        """
        # 创建栏杆预埋件数据
        self.railing_datas = RailingEmbeddedPart(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        # 创建吊装预埋件信息
        self.hoist_embedded_part = HoistingEmbeddedPartsLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
        # 创建脱模预埋件信息
        self.demold_embedded_part = DemoldEmbeddedPartsLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

    @staticmethod
    def get_hinge_datas(point, height, diams):
        """
        得到孔洞定位点，圆台高度，圆台顶端和底端直径数据，用于OCC实现三维实体模型。
        :param point:孔洞圆台底部中心点
        :param height:孔孔圆台高度
        :param diams:孔洞圆台底部和顶部直径数组
        :return:
        """
        num = len(diams)  # 圆台的数量
        hole = []
        for i in range(num):
            point_ = copy.deepcopy(point)
            _point = [point_.x, point_.y, point_.z]
            if i >= 1:
                _point[2] += height[i - 1]
            hole.append([_point, height[i], diams[i]])
        return hole

    @staticmethod
    def transform_polyline_to_arcline(points, bending_radius, diam):
        """
        多段线形成圆弧的方法：起点和终点不变，中间弯折点变换成起弯点，圆弧段，结束弯折点，平直段
        多段线--->平直段+圆弧段+平直段
        测试钢筋类型：直角弯折，钝角弯折，锐角弯折三种情况，钢筋所在位置任意
        :param points: [起点，弯折点，终点]
        :param bending_radius: 弯箍半径
        :param diam: 钢筋直径
        :return:
        """
        points_ = copy.deepcopy(points)
        point_s = points_[0]  # 起点
        point_m = points_[1]  # 弯折点
        point_e = points_[2]  # 终点
        vector_s_m = np.array(np.array(point_m) - np.array(point_s))  # 起点和弯折点形成的向量
        vector_m_e = np.array(np.array(point_e) - np.array(point_m))  # 终点和弯折点形成的向量
        single_vec_s_m = vector_s_m / np.linalg.norm(vector_s_m)  # 向量单位化
        single_vec_m_e = vector_m_e / np.linalg.norm(vector_m_e)  # 向量单位化
        length_1 = np.linalg.norm(vector_s_m)  # 第一段的长度
        length_2 = np.linalg.norm(vector_m_e)  # 第二段的长度
        cos_theta = np.dot(vector_s_m, vector_m_e) / (length_1 * length_2)
        theta = np.arccos(cos_theta)  # 转化为弧度制
        line_length = (
            2 * math.tan(theta / 2) * (bending_radius + diam / 2)
        )  # 起弯点和结束弯点间的轴心线长度
        arc_length = theta * (bending_radius + diam / 2)  # 起弯点和折弯点的圆弧长度
        point_m_1 = np.array(
            np.array(point_m) - single_vec_s_m * line_length / 2
        )  # 起弯点
        point_m_2 = np.array(
            np.array(point_m) + single_vec_m_e * line_length / 2
        )  # 结束弯折点
        point_path = [point_s, point_m_1.tolist(), point_m_2.tolist(), point_e]  # 路径点
        # 计算弯箍圆心
        plane_vec = np.cross(single_vec_s_m, single_vec_m_e)  # 垂直与钢筋所在平面
        vertical_line = np.cross(plane_vec, single_vec_s_m)  # 起点和起弯点形成直线的法线（指向弯箍半径的圆心）
        radius_set = [bending_radius, diam / 2]
        return point_path, radius_set

    def get_stair_left_vertex_data(self):
        """
        获取混凝土左侧角点数据
        :return:
        """
        return self.geom_solid.get_concrete_body_left()

    def get_stair_bottom_ear_left_data(self):
        """
        得到楼梯底端挑耳左侧角点数据
        :return:
        """
        return self.geom_solid.get_bottom_ear_left()

    def get_stair_top_ear_left_data(self):
        """
        得到楼梯顶端挑耳左侧角点数据
        :return:
        """
        return self.geom_solid.get_top_ear_left()

    def get_stair_all_width(self):
        """
        得到楼梯所有部位的宽度大小，形如[楼梯中部宽度，顶端宽度，底端宽度]
        :return:
        """
        # 楼梯主体宽度
        stair_width = self.b0
        # 楼梯底端挑耳角点和总长度
        bottom_width = self.b2
        # 楼梯顶端挑耳角点和总长度
        top_width = self.b1
        total_width = [stair_width, top_width, bottom_width]
        return total_width

    def get_stair_internal_data(self):
        """
        获取楼梯阴角坐标数据：由下到上
        :return:
        """
        internal_loc = self.internal_info.get_internal_corner_location()  # 阴角位置
        internal_config = self.internal_info.get_internal_corner_config()  # 阴角配置信息
        loc = []
        information = {}
        for num in range(len(internal_loc)):
            current_point = internal_loc[num]
            loc.append([current_point.x, current_point.y, current_point.z])
        information["location"] = loc
        information["radius"] = internal_config["radius"]
        return information

    def get_stair_external_data(self):
        """
        获取楼梯阳角坐标数据：由下到上
        :return:
        """
        external_loc = self.external_info.get_external_corner_location()  # 阳角位置
        external_config = self.external_info.get_external_corner_config()  # 阳角配置信息
        loc = []
        information = {}
        for num in range(len(external_loc)):
            current_point = external_loc[num]
            loc.append([current_point.x, current_point.y, current_point.z])
        information["location"] = loc
        information["side"] = external_config["side"]
        return information

    def get_hole_data(self):
        """
        获取孔洞数据
        :return:
        """
        # 获取孔洞数据

        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置数据
        hole_shape_info = self.hole_info.get_hole_geometry_shape()  # 孔洞大小
        fixed_hinge_info = hole_shape_info["fixed_hinge"]  # 固定铰形状大小
        slide_hinge_info = hole_shape_info["slide_hinge"]  # 滑动铰形状大小
        fix_diams = [[fixed_hinge_info["d2"], fixed_hinge_info["c2"]]]
        slide_diams = [
            [slide_hinge_info["f1"], slide_hinge_info["d1"]],
            [slide_hinge_info["e1"], slide_hinge_info["c1"]],
        ]
        hole_config_info = self.hole_info.get_hole_config()
        top_hole_type = hole_config_info["top_type"]
        bottom_hole_type = hole_config_info["bottom_type"]
        draw_hole_datas = []  # occ绘图需要所有数据
        if top_hole_type == 0:  # 固定铰
            top_hole_1 = hole_loc[2]
            top_hole_2 = hole_loc[3]
            height = [self.h1]
            single_hole_1 = self.get_hinge_datas(top_hole_1, height, fix_diams)
            single_hole_2 = self.get_hinge_datas(top_hole_2, height, fix_diams)
            draw_hole_datas.extend(single_hole_1)
            draw_hole_datas.extend(single_hole_2)
        elif top_hole_type == 1:  # 滑动铰
            top_hole_1 = hole_loc[2]
            top_hole_2 = hole_loc[3]
            height = [self.h1 - slide_hinge_info["h1"], slide_hinge_info["h1"]]
            single_hole_1 = self.get_hinge_datas(top_hole_1, height, slide_diams)
            single_hole_2 = self.get_hinge_datas(top_hole_2, height, slide_diams)
            draw_hole_datas.extend(single_hole_1)
            draw_hole_datas.extend(single_hole_2)

        if bottom_hole_type == 0:  # 固定铰
            bottom_hole_1 = hole_loc[0]
            bottom_hole_2 = hole_loc[1]
            height = [self.h2]
            single_hole_1 = self.get_hinge_datas(bottom_hole_1, height, fix_diams)
            single_hole_2 = self.get_hinge_datas(bottom_hole_2, height, fix_diams)
            draw_hole_datas.extend(single_hole_1)
            draw_hole_datas.extend(single_hole_2)
        elif bottom_hole_type == 1:  # 滑动铰
            bottom_hole_1 = hole_loc[0]
            bottom_hole_2 = hole_loc[1]
            height = [self.h2 - slide_hinge_info["h1"], slide_hinge_info["h1"]]
            single_hole_1 = self.get_hinge_datas(bottom_hole_1, height, slide_diams)
            single_hole_2 = self.get_hinge_datas(bottom_hole_2, height, slide_diams)
            draw_hole_datas.extend(single_hole_1)
            draw_hole_datas.extend(single_hole_2)
        return draw_hole_datas

    def get_step_slot_datas(self):
        """
        得到防滑槽的数据
        :return:
        """
        step_slot_loc = self.step_slot_info.get_step_slot_location()  # 防滑槽坐标点
        step_slot_config = self.step_slot_info.get_step_slot_configuration()  # 防滑槽配置信息
        step_slot = {}  # 防滑槽信息
        design_mode = step_slot_config["design_mode"]  # 设计模式
        fact_loc = []
        if design_mode != 2:  # 设计模式不为无防滑槽
            for num in range(len(step_slot_loc)):
                current_point = copy.deepcopy(step_slot_loc[num])  # 当前点
                fact_loc.append([current_point.x, current_point.y, current_point.z])
        else:  # 无防滑槽的模式
            pass
        step_slot["location"] = fact_loc
        step_slot["config"] = step_slot_config
        return step_slot

    def get_water_drip_datas(self):
        """
        滴水线槽数据
        :return:
        """
        water_drip_loc = self.water_drip_info.get_water_drip_location()  # 滴水线槽坐标点
        water_drip_config = (
            self.water_drip_info.get_water_drip_configuration()
        )  # 滴水线槽配置信息
        water_drip = {}  # 滴水线槽配置信息
        water_loc = []  # 滴水线槽的位置
        design_location = water_drip_config["design_location"]
        design_mode = water_drip_config["design_mode"]  # 设计模式
        if design_mode == 2:  # 无滴水线槽
            pass
        else:
            if design_location == 2:  # 两侧皆有滴水线槽数据
                for num in range(len(water_drip_loc)):
                    points_ = copy.deepcopy(water_drip_loc[num])
                    _water_loc = []
                    for iter in range(len(points_)):
                        current_point = copy.deepcopy(points_[iter])
                        _water_loc.append(
                            [current_point.x, current_point.y, current_point.z]
                        )
                    water_loc.append(_water_loc)
            else:  # 仅一侧有滴水线槽数据
                for num in range(len(water_drip_loc)):
                    current_point = copy.deepcopy(water_drip_loc[num])
                    water_loc.append(
                        [current_point.x, current_point.y, current_point.z]
                    )
        water_drip["location"] = water_loc
        water_drip["config"] = water_drip_config
        return water_drip

    def get_single_railing_embedded_model(self):
        """
        单个栏杆预埋件OCC模型
        :return:
        """
        rail_info = self.railing_datas.get_rail_datas()  # 得到栏杆预埋件型号数据
        single_rail_info = {}
        single_rail_info["direction"] = [0, 0, -1]  # 栏杆方向
        point = [0, 0, 0]
        weld_box = {}
        weld_box["location"] = point
        weld_box["shape"] = [rail_info.b, rail_info.a, rail_info.t]
        single_rail_info["weld_box"] = weld_box
        distance_x = rail_info.b - 2 * rail_info.c
        distance_y = rail_info.a - 2 * rail_info.c
        # 所有钢筋的位置
        u_rebar_loc = []
        # 第一根U型钢筋的位置
        point_1 = [-distance_x / 2, -distance_y / 2, -rail_info.t]  # 左侧U形
        point_2 = [point_1[0], point_1[1], point_1[2] - rail_info.d]
        point_2_3 = [point_2[0] + distance_y / 2, point_2[1], point_2[2]]
        point_3 = [point_2[0] + distance_y, point_2[1], point_2[2]]
        point_4 = [point_3[0], point_3[1], point_3[2] + rail_info.d]
        u_rebar_1 = [point_1, point_2, point_2_3, point_3, point_4]
        u_rebar_loc.append(u_rebar_1)  # 第一根U形钢筋
        # 第二根U形钢筋的位置
        u_rebar_2 = []
        for i in range(len(u_rebar_1)):
            point_ = copy.deepcopy(u_rebar_1[i])
            point_[1] += distance_y
            u_rebar_2.append(point_)
        u_rebar_loc.append(u_rebar_2)
        rebar_diam = rail_info.fi / 2  # 钢筋的半径
        bending_radius = 2.5 * rail_info.fi  # TODO 弯箍半径，弯箍半径取为2.5倍的钢筋直径
        layout_rebar_info = []
        for num in range(len(u_rebar_loc)):
            rebar_loc = copy.deepcopy(u_rebar_loc[num])
            rebar_loc_1 = [rebar_loc[0], rebar_loc[1], rebar_loc[2]]
            rebar_loc_2 = [rebar_loc[2], rebar_loc[3], rebar_loc[4]]
            point_path_1, rebar_set_1 = self.transform_polyline_to_arcline(
                rebar_loc_1, bending_radius, rebar_diam
            )  # 第一段的放样路径
            point_path_2, rebar_set_2 = self.transform_polyline_to_arcline(
                rebar_loc_2, bending_radius, rebar_diam
            )  # 第二段的放样路径
            layout_rebar_info.append([point_path_1, rebar_set_1])
            layout_rebar_info.append([point_path_2, rebar_set_2])
        rebar_shape = {}
        rebar_shape["path"] = layout_rebar_info
        # 钢筋信息
        single_rail_info["rebar"] = rebar_shape
        return single_rail_info

    def get_railing_embedded_datas(self):
        """
        得到栏杆预埋件OCC绘图数据：各预埋件坐标点，
        :return:
        """
        rail_locs = self.railing_datas.get_rail_embedded_part_loc()  # 得到栏杆预埋件的位置
        model_inf = {}  # 绘制模型信息
        rail_loc_lists = []  # 栏杆预埋件的位置
        for num in range(len(rail_locs)):
            point_ = copy.deepcopy(rail_locs[num])
            loc_ = [point_.x, point_.y, point_.z]  # 栏杆预埋件的位置坐标
            rail_loc_lists.append(loc_)
        model_inf["location"] = rail_loc_lists  # 栏杆预埋件的位置
        single_rail = self.get_single_railing_embedded_model()
        model_inf["layout"] = single_rail
        return model_inf

    def get_rail_embedded_rabbet_datas(self):
        """
        获取栏杆预埋件企口数据
        """
        rail_rabbet_info = {}  # 栏杆预埋件企口信息
        rabbet_loc = self.railing_datas.get_rail_embedded_rabbet_loc()  # 获取栏杆预埋件位置数据
        rabbet_shape = self.railing_datas.get_rail_rabbet_config()  # 获取栏杆预埋件配置信息
        rabbet_change_loc = []
        for rabbet in rabbet_loc:
            rabbet_change_loc.append([rabbet.x, rabbet.y, rabbet.z])
        rail_rabbet_info["location"] = rabbet_change_loc  # 获取企口坐标点
        rail_rabbet_info["rabbet_shape"] = rabbet_shape  # 获取企口配置信息
        return rail_rabbet_info

    def get_u_type_rebar_datas(self, _points, bending_radius, diam):
        """
        得到U型钢筋的OCC形状绘制数据
        :param _points: 四点
        :param bending_radius:弯箍半径
        :param diam: 钢筋直径
        :return:
        """
        points_ = copy.deepcopy(_points)
        point_1 = points_[0]  # 起点
        point_2 = points_[1]  # 中间点1
        point_3 = points_[2]  # 中间点2
        point_4 = points_[3]  # 终点
        vect_12 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )
        vect_23 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
            np.array(point_3) - np.array(point_2)
        )
        vect_34 = np.array(np.array(point_4) - np.array(point_3)) / np.linalg.norm(
            np.array(point_4) - np.array(point_3)
        )
        # 弯曲角1
        cos_theta_1 = np.dot(vect_12, vect_23)
        theta_1 = np.arccos(cos_theta_1)  # 转化为弧度制
        # 弯曲角2
        cos_theta_2 = np.dot(vect_23, vect_34)
        theta_2 = np.arccos(cos_theta_2)  # 转化为弧度制
        total_radius = bending_radius + diam / 2  # 弯箍半径
        length_1 = math.tan(theta_1 / 2) * total_radius  #
        length_2 = math.tan(theta_2 / 2) * total_radius  #
        point_2_1 = np.array(point_2) + length_1 * (-vect_12)
        point_2_2 = np.array(point_2) + length_1 * vect_23
        point_3_1 = np.array(point_3) + length_2 * (-vect_23)
        point_3_2 = np.array(point_3) + length_2 * vect_34
        location = [
            point_1,
            point_2_1.tolist(),
            point_2_2.tolist(),
            point_3_1.tolist(),
            point_3_2.tolist(),
            point_4,
        ]
        return location

    def get_all_mid_distribute_rebar(self):
        """
        获取所有中部分布筋实际下料形状数据
        :return:
        """
        mid_rebar_info = self.mid_distribute_rebar.get_double_rebar_model()
        rebar_infos = {}  # 钢筋信息集
        mid_rebar_locs = []  # 钢筋坐标点集合
        diam = self.mid_distribute_rebar.get_rebar_diameter()  # 中部分布筋的直径
        bending_radius = self.mid_distribute_rebar.get_bending_diameter()  # 中部分布筋的弯箍直径
        rebar_path_sets = []  # 钢筋路径集合
        # 遍历每根钢筋
        for num in range(len(mid_rebar_info)):
            single_rebar = copy.deepcopy(mid_rebar_info[num])
            rebar_ = []
            for point in single_rebar:
                point_ = copy.deepcopy(point)
                rebar_.append([point_.x, point_.y, point_.z])
            mid_rebar_locs.append(rebar_)
            single_rebar_path = self.get_u_type_rebar_datas(
                rebar_, bending_radius, diam
            )  # 钢筋的路径
            rebar_path_sets.append(single_rebar_path)
        rebar_infos["radius"] = diam / 2
        rebar_infos["path"] = rebar_path_sets
        return rebar_infos

    def get_z_type_rebar_datas(self, _points, bending_radius, diam):
        """
        得到z型钢筋数据
        :param _points:钢筋坐标点
        :param bending_radius: 钢筋弯箍半径
        :param diam: 钢筋直径
        :return:
        """
        points = copy.deepcopy(_points)  # 钢筋坐标点
        point_1 = points[0]
        point_2 = points[1]
        point_3 = points[2]
        point_4 = points[3]
        vect_12 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )
        vect_23 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
            np.array(point_3) - np.array(point_2)
        )
        vect_34 = np.array(np.array(point_4) - np.array(point_3)) / np.linalg.norm(
            np.array(point_4) - np.array(point_3)
        )
        # 弯曲角1
        cos_theta_1 = np.dot(vect_12, vect_23)
        theta_1 = np.arccos(cos_theta_1)  # 转化为弧度制
        # 弯曲角2
        cos_theta_2 = np.dot(vect_23, vect_34)
        theta_2 = np.arccos(cos_theta_2)  # 转化为弧度制
        total_radius = bending_radius + diam / 2  # 弯箍半径
        #
        length_1 = math.tan(theta_1 / 2) * total_radius
        length_2 = math.tan(theta_2 / 2) * total_radius
        point_2_1 = np.array(point_2) + length_1 * (-vect_12)
        point_2_2 = np.array(point_2) + length_1 * vect_23
        point_3_1 = np.array(point_3) + length_2 * (-vect_23)
        point_3_2 = np.array(point_3) + length_2 * vect_34
        location = [
            point_1,
            point_2_1.tolist(),
            point_2_2.tolist(),
            point_3_1.tolist(),
            point_3_2.tolist(),
            point_4,
        ]
        return location

    def get_five_sections_type_rebar_datas(self, _points, bending_radius, diam):
        """
        得到五段钢筋的数据---吊点加强纵筋
        :param _points:钢筋坐标点
        :param bending_radius: 钢筋弯箍半径
        :param diam: 钢筋直径
        :return:
        """
        points = copy.deepcopy(_points)  # 钢筋坐标点
        point_1 = points[0]
        point_2 = points[1]
        point_3 = points[2]
        point_4 = points[3]
        point_5 = points[4]
        vect_12 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )
        vect_23 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
            np.array(point_3) - np.array(point_2)
        )
        vect_34 = np.array(np.array(point_4) - np.array(point_3)) / np.linalg.norm(
            np.array(point_4) - np.array(point_3)
        )
        vect_45 = np.array(np.array(point_5) - np.array(point_4)) / np.linalg.norm(
            np.array(point_5) - np.array(point_4)
        )
        # 弯曲角1
        cos_theta_1 = np.dot(vect_12, vect_23)
        theta_1 = np.arccos(cos_theta_1)  # 转化为弧度制
        # 弯曲角2
        cos_theta_2 = np.dot(vect_23, vect_34)
        theta_2 = np.arccos(cos_theta_2)  # 转化为弧度制
        # 弯曲角3
        cos_theta_3 = np.dot(vect_34, vect_45)
        theta_3 = np.arccos(cos_theta_3)  # 转化为弧度制
        total_radius = bending_radius + diam / 2  # 弯箍半径
        #
        length_1 = math.tan(theta_1 / 2) * total_radius
        length_2 = math.tan(theta_2 / 2) * total_radius
        length_3 = math.tan(theta_3 / 2) * total_radius

        point_2_1 = np.array(point_2) + length_1 * (-vect_12)
        point_2_2 = np.array(point_2) + length_1 * vect_23
        point_3_1 = np.array(point_3) + length_2 * (-vect_23)
        point_3_2 = np.array(point_3) + length_2 * vect_34
        point_4_1 = np.array(point_4) + length_3 * (-vect_34)
        point_4_2 = np.array(point_4) + length_3 * vect_45
        location = [
            point_1,
            point_2_1.tolist(),
            point_2_2.tolist(),
            point_3_1.tolist(),
            point_3_2.tolist(),
            point_4_1.tolist(),
            point_4_2.tolist(),
            point_5,
        ]
        return location

    def get_closed_type_rebar_datas(self, _points, bending_radius, diam):
        """
        得到封闭箍筋的数据
        :param _points:钢筋坐标点
        :param bending_radius: 钢筋弯箍半径
        :param diam: 钢筋直径
        :return:
        """
        points = copy.deepcopy(_points)  # 钢筋坐标点
        point_1 = points[0]
        point_2 = points[1]
        point_3 = points[2]
        point_4 = points[3]
        vect_12 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )
        vect_23 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
            np.array(point_3) - np.array(point_2)
        )
        vect_34 = np.array(np.array(point_4) - np.array(point_3)) / np.linalg.norm(
            np.array(point_4) - np.array(point_3)
        )
        vect_41 = np.array(np.array(point_1) - np.array(point_4)) / np.linalg.norm(
            np.array(point_1) - np.array(point_4)
        )
        # 弯曲角1
        cos_theta_1 = np.dot(vect_12, vect_23)
        theta_1 = np.arccos(cos_theta_1)  # 转化为弧度制
        # 弯曲角2
        cos_theta_2 = np.dot(vect_23, vect_34)
        theta_2 = np.arccos(cos_theta_2)  # 转化为弧度制
        # 弯曲角3
        cos_theta_3 = np.dot(vect_34, vect_41)
        theta_3 = np.arccos(cos_theta_3)  # 转化为弧度制
        # 弯曲角4
        cos_theta_4 = np.dot(vect_41, vect_12)
        theta_4 = np.arccos(cos_theta_4)  # 转化为弧度制
        total_radius = bending_radius + diam / 2  # 弯箍半径
        #
        length_1 = math.tan(theta_1 / 2) * total_radius
        length_2 = math.tan(theta_2 / 2) * total_radius
        length_3 = math.tan(theta_3 / 2) * total_radius
        length_4 = math.tan(theta_4 / 2) * total_radius
        # 弯折点
        point_2_1 = np.array(point_2) + length_1 * (-vect_12)
        point_2_2 = np.array(point_2) + length_1 * vect_23
        point_3_1 = np.array(point_3) + length_2 * (-vect_23)
        point_3_2 = np.array(point_3) + length_2 * vect_34
        point_4_1 = np.array(point_4) + length_3 * (-vect_34)
        point_4_2 = np.array(point_4) + length_3 * vect_41
        point_1_1 = np.array(point_1) + length_4 * (-vect_41)
        point_1_2 = np.array(point_1) + length_4 * vect_12

        location = [
            point_1_2.tolist(),
            point_2_1.tolist(),
            point_2_2.tolist(),
            point_3_1.tolist(),
            point_3_2.tolist(),
            point_4_1.tolist(),
            point_4_2.tolist(),
            point_1_1.tolist(),
        ]
        return location

    def get_L_type_rebar_datas(self, _points, bending_radius, diam):
        """
        得到L型钢筋数据
        :param _points:钢筋坐标点
        :param bending_radius: 钢筋弯箍半径
        :param diam: 钢筋直径
        :return:
        """
        points = copy.deepcopy(_points)  # 钢筋坐标点
        point_1 = points[0]
        point_2 = points[1]
        point_3 = points[2]
        vect_12 = np.array(np.array(point_2) - np.array(point_1)) / np.linalg.norm(
            np.array(point_2) - np.array(point_1)
        )
        vect_23 = np.array(np.array(point_3) - np.array(point_2)) / np.linalg.norm(
            np.array(point_3) - np.array(point_2)
        )
        # 弯曲角1
        cos_theta_1 = np.dot(vect_12, vect_23)
        theta_1 = np.arccos(cos_theta_1)  # 转化为弧度制
        total_radius = bending_radius + diam / 2  # 弯箍半径
        #
        length_1 = math.tan(theta_1 / 2) * total_radius
        point_2_1 = np.array(point_2) + length_1 * (-vect_12)
        point_2_2 = np.array(point_2) + length_1 * vect_23
        location = [point_1, point_2_1.tolist(), point_2_2.tolist(), point_3]
        return location

    def get_bottom_long_rebar_datas(self):
        """
        得到底部纵筋生成OCC模型的数据
        :return:
        """
        bottom_rebar_loc = self.bottom_long_rebar.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.bottom_long_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.bottom_long_rebar.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(bottom_rebar_loc)):
            points_ = copy.deepcopy(bottom_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_L_type_rebar_datas(locs_, bending_radius, rebar_diam)
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_top_long_rebar_datas(self):
        """
        计算上部纵筋生成OCC模型的数据
        :return:
        """
        top_rebar_loc = self.top_long_rebar.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.top_long_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.top_long_rebar.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(top_rebar_loc)):
            points_ = copy.deepcopy(top_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_z_type_rebar_datas(locs_, bending_radius, rebar_diam)
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_bottom_edge_long_rebar_datas(self):
        """
        计算底部边缘纵筋生成OCC模型的数据
        :return:
        """
        bottom_edge_rebar_loc = (
            self.bottom_edge_long_rebar.get_rebar_model()
        )  # 获取钢筋的坐标点
        rebar_diam = self.bottom_edge_long_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        rebar_info = []
        for num in range(len(bottom_edge_rebar_loc)):
            points_ = copy.deepcopy(bottom_edge_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_info.append(locs_)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_top_edge_long_rebar_datas(self):
        """
        计算顶部边缘纵筋生成OCC模型的数据
        :return:
        """
        top_edge_rebar_loc = self.top_edge_long_rebar.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.top_edge_long_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        rebar_info = []
        for num in range(len(top_edge_rebar_loc)):
            points_ = copy.deepcopy(top_edge_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_info.append(locs_)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_top_edge_stir_datas(self):
        """
        计算顶部边缘箍筋生成OCC模型的数据
        :return:
        """
        top_stir_loc = self.top_edge_stir.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.top_edge_stir.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.top_edge_stir.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(top_stir_loc)):
            points_ = copy.deepcopy(top_stir_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_closed_type_rebar_datas(
                locs_, bending_radius, rebar_diam
            )
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_bottom_edge_stir_datas(self):
        """
        计算底部边缘箍筋生成OCC模型的数据
        :return:
        """
        bottom_stir_loc = self.bottom_edge_stir.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.bottom_edge_stir.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.bottom_edge_stir.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(bottom_stir_loc)):
            points_ = copy.deepcopy(bottom_stir_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_closed_type_rebar_datas(
                locs_, bending_radius, rebar_diam
            )
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_hole_rein_rebar_datas(self):
        """
        计算孔洞加强钢筋生成OCC模型的数据
        :return:
        """
        hole_rein_rebar = self.hole_rein_rebar.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        rebar_info = []
        for num in range(len(hole_rein_rebar)):
            points_ = copy.deepcopy(hole_rein_rebar[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            locs_.remove(locs_[2])  # 移除中间点，便于利用L型函数形成OCC模型
            rebar_info.append(locs_)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_hoist_rein_long_rebar_datas(self):
        """
        计算吊点加强纵筋生成OCC模型的数据
        :return:
        """
        hoist_rein_rebar = self.hoist_rein_long_rebar.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.hoist_rein_long_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.hoist_rein_long_rebar.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(hoist_rein_rebar)):
            points_ = copy.deepcopy(hoist_rein_rebar[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_five_sections_type_rebar_datas(
                locs_, bending_radius, rebar_diam
            )
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_hoist_rein_point_rebar_datas(self):
        """
        吊装加强点筋生成OCC模型的数据
        :return:
        """
        hoist_rein_point_rebar_loc = (
            self.hoist_rein_point_rebar.get_rebar_model()
        )  # 获取钢筋的坐标点
        rebar_diam = self.hoist_rein_point_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        rebar_info = []
        for num in range(len(hoist_rein_point_rebar_loc)):
            points_ = copy.deepcopy(hoist_rein_point_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_info.append(locs_)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_top_edge_rein_rebar_datas(self):
        """
        计算上部边缘加强生成OCC模型的数据
        :return:
        """
        top_rein_rebar_loc = self.top_edge_rein_rebar.get_rebar_model()  # 获取钢筋的坐标点
        rebar_diam = self.top_edge_rein_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.top_edge_rein_rebar.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(top_rein_rebar_loc)):
            points_ = copy.deepcopy(top_rein_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_z_type_rebar_datas(locs_, bending_radius, rebar_diam)
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_bottom_edge_rein_rebar_datas(self):
        """
        计算下部边缘加强生成OCC模型的数据
        :return:
        """
        bottom_rein_rebar_loc = (
            self.bottom_edge_rein_rebar.get_rebar_model()
        )  # 获取钢筋的坐标点
        rebar_diam = self.bottom_edge_rein_rebar.get_rebar_diameter()  # 获取钢筋的直径
        rebar_sets = {}
        bending_radius = self.bottom_edge_rein_rebar.get_bending_diameter()  # 钢筋弯箍半径
        rebar_info = []
        for num in range(len(bottom_rein_rebar_loc)):
            points_ = copy.deepcopy(bottom_rein_rebar_loc[num])  # 当前钢筋的路径点
            locs_ = []
            for i in range(len(points_)):
                point = copy.deepcopy(points_[i])
                locs_.append([point.x, point.y, point.z])
            rebar_loc = self.get_L_type_rebar_datas(locs_, bending_radius, rebar_diam)
            rebar_info.append(rebar_loc)
        rebar_sets["path"] = rebar_info
        rebar_sets["radius"] = rebar_diam / 2
        return rebar_sets

    def get_hoist_embedded_part_datas(self):
        """
        获取吊装预埋件数据，用于生成OCC模型
        :return:
        """
        hoist_location = (
            self.hoist_embedded_part.get_hoist_embedded_part_loc()
        )  # 吊装预埋件坐标位置
        hoist_info_sets = (
            self.hoist_embedded_part.get_hoist_embedded_part_info()
        )  # 吊装预埋件信息集合
        location = []
        for num in range(len(hoist_location)):
            hoist_loc_ = copy.deepcopy(hoist_location[num])
            loc_ = [hoist_loc_.x, hoist_loc_.y, hoist_loc_.z]  # 预埋件标志点
            theta = 0  # 预埋件旋转角--逆时针为正
            rotate_axis = [0, 0, 1]  # 预埋件旋转轴
            location.append([loc_, theta, rotate_axis])
        hoist_info_sets["location"] = location
        return hoist_info_sets

    def get_demold_embedded_part_datas(self):
        """
        获取脱模预埋件数据，用于生成OCC模型
        :return:
        """
        demold_location = (
            self.demold_embedded_part.get_demold_embedded_part_loc()
        )  # 脱模预埋件坐标位置
        demold_info_sets = self.demold_embedded_part.get_demold_part_info()  # 脱模预埋件信息集合
        pouring_way = demold_info_sets["pouring_way"]  # 浇筑方式
        location = []  # 预埋件位置信息
        for num in range(len(demold_location)):
            hoist_loc_ = copy.deepcopy(demold_location[num])
            loc_ = [hoist_loc_.x, hoist_loc_.y, hoist_loc_.z]  # 预埋件标志点
            if pouring_way == 1:  # 立式浇筑立式脱模
                theta = math.pi / 2  # 预埋件旋转角--逆时针为正
                rotate_axis = [0, 1, 0]  # 预埋件旋转轴
            elif pouring_way == 2:  # 卧式浇筑卧式脱模
                cos_theta = self.cos
                theta_0 = np.arccos(cos_theta)
                theta = theta_0 - math.pi  # 预埋件旋转角--逆时针为正
                rotate_axis = [1, 0, 0]  # 预埋件旋转轴
            else:  # 立式浇筑卧式脱模
                if num == 0 or num == 1:  # 侧向
                    theta = math.pi / 2  # 预埋件旋转角--逆时针为正
                    rotate_axis = [0, 1, 0]  # 预埋件旋转轴
                else:  # 卧式方向
                    cos_theta = self.cos
                    theta_0 = np.arccos(cos_theta)
                    theta = theta_0 - math.pi  # 预埋件旋转角--逆时针为正
                    rotate_axis = [1, 0, 0]  # 预埋件旋转轴
            location.append([loc_, theta, rotate_axis])
        demold_info_sets["location"] = location
        return demold_info_sets

    def get_ladder_beam_and_slab_datas(self):
        """
        获取楼梯平台梁和平台板数据
        :return:
        """
        ladder_beam_and_slab_total_info = {}  # 楼梯平台梁和平台板全部信息
        top_beam_info = {}  # 顶部平台梁信息
        bottom_beam_info = {}  # 底部平台梁信息
        top_slab_info = {}  # 顶部平台板信息
        bottom_slab_info = {}  # 底部平台板信息
        # 获取基本数据信息
        bottom_beam_loc = (
            self.ladder_beam_slab_loc.get_stair_bottom_beam_loc()
        )  # 获取楼梯底部平台梁的位置信息
        bottom_beam_stretch_length = (
            self.ladder_beam_slab_loc.get_stair_bottom_beam_stretch_length()
        )  # 获取楼梯底部平台梁拉伸长度
        top_beam_loc = (
            self.ladder_beam_slab_loc.get_stair_top_beam_loc()
        )  # 获取楼梯顶部平台梁的位置信息
        top_beam_stretch_length = (
            self.ladder_beam_slab_loc.get_stair_top_beam_stretch_length()
        )  # 获取楼梯顶部平台梁拉伸长度
        bottom_slab_loc = (
            self.ladder_beam_slab_loc.get_stair_bottom_slab_loc()
        )  # 获取楼梯底部平台板位置信息
        bottom_slab_stretch_length = (
            self.ladder_beam_slab_loc.get_stair_bottom_slab_stretch_length()
        )  # 获取楼梯底部平台板拉伸长度
        top_slab_loc = (
            self.ladder_beam_slab_loc.get_stair_top_slab_loc()
        )  # 获取楼梯顶部平台板位置信息
        top_slab_stretch_length = (
            self.ladder_beam_slab_loc.get_stair_top_slab_stretch_length()
        )  # 获取楼梯顶部平台板拉伸长度
        # 变换构件坐标信息
        bottom_beam_loc_c = []
        top_beam_loc_c = []
        bottom_slab_loc_c = []
        top_slab_loc_c = []
        # 改变楼梯平台梁的坐标信息
        for point_b, point_t in zip(bottom_beam_loc, top_beam_loc):
            bottom_beam_loc_c.append([point_b.x, point_b.y, point_b.z])
            top_beam_loc_c.append([point_t.x, point_t.y, point_t.z])
        # 改变楼梯平台板的坐标信息
        for point_b, point_t in zip(bottom_slab_loc, top_slab_loc):
            bottom_slab_loc_c.append([point_b.x, point_b.y, point_b.z])
            top_slab_loc_c.append([point_t.x, point_t.y, point_t.z])
        # 保存平台板和平台梁信息
        top_beam_info["location"] = top_beam_loc_c
        top_beam_info["direction"] = top_beam_stretch_length
        bottom_beam_info["location"] = bottom_beam_loc_c
        bottom_beam_info["direction"] = bottom_beam_stretch_length
        top_slab_info["location"] = top_slab_loc_c
        top_slab_info["direction"] = top_slab_stretch_length
        bottom_slab_info["location"] = bottom_slab_loc_c
        bottom_slab_info["direction"] = bottom_slab_stretch_length
        ladder_beam_and_slab_total_info["top_beam"] = top_beam_info
        ladder_beam_and_slab_total_info["bottom_beam"] = bottom_beam_info
        ladder_beam_and_slab_total_info["top_slab"] = top_slab_info
        ladder_beam_and_slab_total_info["bottom_slab"] = bottom_slab_info
        return ladder_beam_and_slab_total_info

    def get_connect_embedded_part_datas(self):
        """
        获取连接预埋件数据
        :return:
        """
        embedded_total_info = {}  # 预埋件全部信息
        # 获取楼梯连接孔洞信息
        loc_info = self.connect_embedded_part_loc.get_stair_connect_hole_loc()
        element_info = (
            self.connect_embedded_part_loc.get_stair_connect_element_local_info()
        )  # 获取楼梯部件信息集
        top_shape = element_info["top_shape"]
        top_rebar_diam = top_shape.rebar_anchor_diameter  # 获取顶部锚固钢筋的直径
        bottom_shape = element_info["bottom_shape"]
        bottom_rebar_diam = bottom_shape.rebar_anchor_diameter  # 获取底部锚固钢筋的直径
        # 获取连接锚固钢筋坐标信息
        anchor_rebar_info = (
            self.connect_embedded_part_loc.get_stair_connect_rebar_local_location()
        )  # 获取连接锚固钢筋局部位置信息
        nut_info = (
            self.connect_embedded_part_loc.get_stair_connect_nut_local_location()
        )  # 获取螺纹局部位置信息
        shim_info = (
            self.connect_embedded_part_loc.get_stair_connect_shim_local_location()
        )  # 获取垫片局部位置信息
        # 连接锚固钢筋坐标信息
        anchor_rebar_info_c = {}
        top_rebar = anchor_rebar_info["top_rebar"]
        bottom_rebar = anchor_rebar_info["bottom_rebar"]
        top_anchor_rebar_info_loc = []
        bottom_anchor_rebar_info_loc = []
        for point_t, point_b in zip(top_rebar, bottom_rebar):
            top_anchor_rebar_info_loc.append([point_t.x, point_t.y, point_t.z])
            bottom_anchor_rebar_info_loc.append([point_b.x, point_b.y, point_b.z])
        # 连接螺母信息
        nut_info_c = {}
        top_nut = nut_info["top_nut"]
        bottom_nut = nut_info["bottom_nut"]
        # 转换螺母局部坐标点
        nut_info_c["top_nut"] = [top_nut.x, top_nut.y, top_nut.z]
        nut_info_c["bottom_nut"] = [bottom_nut.x, bottom_nut.y, bottom_nut.z]
        # 连接垫片信息
        shim_info_c = {}
        top_shim = shim_info["top_shim"]
        bottom_shim = shim_info["bottom_shim"]
        # 转换垫片局部坐标点
        shim_info_c["top_shim"] = [top_shim.x, top_shim.y, top_shim.z]
        shim_info_c["bottom_shim"] = [bottom_shim.x, bottom_shim.y, bottom_shim.z]
        # 孔洞坐标信息
        hole_loc_info = {}
        top_bending_radius = 1.5 * top_rebar_diam  # 顶部锚固钢筋弯箍半径
        bottom_bending_radius = 1.5 * bottom_rebar_diam  # 底部锚固钢筋弯箍半径
        top_anchor_rebar_bending_loc = self.get_L_type_rebar_datas(
            top_anchor_rebar_info_loc, top_bending_radius, top_rebar_diam
        )
        bottom_anchor_rebar_bending_loc = self.get_L_type_rebar_datas(
            bottom_anchor_rebar_info_loc, bottom_bending_radius, bottom_rebar_diam
        )
        anchor_rebar_info_c["top_rebar"] = top_anchor_rebar_bending_loc
        anchor_rebar_info_c["bottom_rebar"] = bottom_anchor_rebar_bending_loc
        # 计算孔洞位置信息
        top_hole_loc = loc_info["top_hole_loc"]  # 顶部孔洞位置信息
        bottom_hole_loc = loc_info["bottom_hole_loc"]  # 底部孔洞信息
        top_hole_loc_c = []  # 改变后的顶部孔洞位置
        bottom_hole_loc_c = []  # 改变后的底部孔洞位置
        for point_t, point_b in zip(top_hole_loc, bottom_hole_loc):  # 遍历顶部孔洞和底部孔洞位置
            top_hole_loc_c.append([point_t.x, point_t.y, point_t.z])
            bottom_hole_loc_c.append([point_b.x, point_b.y, point_b.z])
        hole_loc_info["top_hole"] = top_hole_loc_c
        hole_loc_info["bottom_hole"] = bottom_hole_loc_c
        embedded_total_info["hole_location"] = hole_loc_info
        embedded_total_info["element_info"] = element_info
        embedded_total_info["anchor_rebar_location"] = anchor_rebar_info_c
        embedded_total_info["nut"] = nut_info_c
        embedded_total_info["shim"] = shim_info_c
        return embedded_total_info
