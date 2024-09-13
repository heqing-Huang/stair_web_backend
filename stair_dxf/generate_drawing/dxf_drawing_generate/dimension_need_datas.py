"""
  本模块用于标注所需基础数据
"""
import copy

import numpy as np

from stair_dxf.stair_design.countrebar import (
    HoleLocation,
    HoistingEmbeddedPartsLoc,
    StepSlotLoc,
    TopEdgeStirrup,
    TopEdgeLongitudinalRebar,
    BottomEdgeLongitudinalRebar,
    BottomLongitudinalRebar,
    TopLongitudinalRebar,
    MidDistributionRebar,
    BottomEdgeStirrup,
    HoistingReinforceLongitudinalRebar,
    HoistingReinforcePointRebar,
    HoleReinforceRebar,
    TopEdgeReinforceRebar,
    BottomEdgeReinforceRebar,
)
from stair_dxf.stair_design.sidegeomentry import StairGeometry

from stair_dxf.generate_drawing.dxf_drawing_generate.basic_method import rotation_3d
import math


class TopViewDimensionData(object):
    """
    俯视图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_basic_class()
        self.generate_dimension_data()
        self.generate_hole_dimension_points()

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

    def generate_dimension_data(self):
        """
        产生数据
        :return:
        """
        self.stair_geom = StairGeometry(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 产生楼梯几何数据
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 产生楼梯孔洞位置数据
        self.hoist_embedded_info = HoistingEmbeddedPartsLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 产生吊装预埋件位置数据
        self.step_slot_info = StepSlotLoc(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 产生防滑槽位置数据

    def generate_hole_dimension_points(self):
        """
        产生孔洞标注点：逆时针方向放置
        标注分内层、中部、外层三部分
        :return:dict
        """
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置点
        limit_y = 0
        limit_x = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            limit_y += current_point.y
            limit_x += current_point.x
        limit_y = limit_y / len(hole_loc)  # 区分顶部和底部的坐标点
        limit_x = limit_x / len(hole_loc)  # 区分左侧和右侧的坐标点
        top_loc = []
        bottom_loc = []
        # 筛选出顶部和底部孔洞
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            curr_point_list = [current_point.x, current_point.y, current_point.z]
            if curr_point_list[1] > limit_y:
                top_loc.append(curr_point_list)
            else:
                bottom_loc.append(curr_point_list)
        # 将顶部和底部孔洞点对为逆时针方向
        if top_loc[0][0] < limit_x:
            top_loc.reverse()
        if bottom_loc[0][0] > limit_x:
            bottom_loc.reverse()
        # 存储所有待标注的点集合
        dimension_loc = []
        # 顶部和底部中间标注位置
        top_mid_loc = top_loc
        dimension_loc.append(top_mid_loc)
        bottom_mid_loc = bottom_loc
        dimension_loc.append(bottom_mid_loc)
        # 顶部左侧和右侧标注位置
        # 顶部左侧标注位置
        top_left_point = copy.deepcopy(top_loc[1])
        top_left_point[0] = 0
        top_left_loc = [top_loc[1], top_left_point]  #
        dimension_loc.append(top_left_loc)
        # 顶部右侧标注位置
        top_right_point = copy.deepcopy(top_loc[0])
        top_right_point[0] = self.b0
        top_right_loc = [top_right_point, top_loc[0]]  #
        dimension_loc.append(top_right_loc)
        # 顶部挑耳标注位置，并不一定会有挑耳
        top_ear_point = copy.deepcopy(top_right_point)
        top_ear_point[0] += self.b1
        top_ear_loc = [top_ear_point, top_right_point]  #
        dimension_loc.append(top_ear_loc)
        # 底部左侧和右侧标注位置
        # 底部左侧标注位置
        bottom_left_point = copy.deepcopy(bottom_loc[0])
        bottom_left_point[0] = 0
        bottom_left_loc = [bottom_left_point, bottom_loc[0]]  #
        dimension_loc.append(bottom_left_loc)
        # 底部右侧标注位置
        bottom_right_point = copy.deepcopy(bottom_loc[1])
        bottom_right_point[0] = self.b0
        bottom_right_loc = [bottom_loc[1], bottom_right_point]  #
        dimension_loc.append(bottom_right_loc)
        # 底部挑耳标注位置，并不一定会有挑耳
        bottom_ear_point = copy.deepcopy(bottom_right_point)
        bottom_ear_point[0] += self.b2
        bottom_ear_loc = [bottom_right_point, bottom_ear_point]  #
        dimension_loc.append(bottom_ear_loc)
        # 顶部孔洞左侧和右侧纵向标注位置
        # 顶部孔洞左侧纵向标注位置
        top_left_loc_s = copy.deepcopy(top_left_point)
        top_left_loc_s[1] = self.lb_d + self.lt_d + self.ln
        top_left_horizon_loc = [top_left_loc_s, top_left_point]
        dimension_loc.append(top_left_horizon_loc)
        # 顶部孔洞右侧纵向标注位置
        top_right_loc_e = copy.deepcopy(top_right_point)
        top_right_loc_e[1] = self.lb_d + self.lt_d + self.ln
        top_right_horizon_loc = [top_right_point, top_right_loc_e]
        dimension_loc.append(top_right_horizon_loc)
        # 底部孔洞左右侧纵向标注位置
        # 底部孔洞左侧纵向标注位置
        bottom_left_loc_e = copy.deepcopy(bottom_left_point)
        bottom_left_loc_e[1] = 0
        bottom_left_horizon_loc = [bottom_left_point, bottom_left_loc_e]
        dimension_loc.append(bottom_left_horizon_loc)
        # 底部孔洞右侧纵向标注位置
        bottom_right_loc_s = copy.deepcopy(bottom_right_point)
        bottom_right_loc_s[1] = 0
        bottom_right_horizon_loc = [bottom_right_loc_s, bottom_right_point]
        dimension_loc.append(bottom_right_horizon_loc)
        dim_loc = {}  # 获取数据
        dim_loc["location"] = dimension_loc
        dim_loc["floor"] = "first floor"  # 标注层数
        return dim_loc

    def generate_stair_solid_dimension_points(self):
        """
        产生楼梯实体标注点:逆时针方向,[[x1,y1,z1],[x2,y2,z2]]
        :return:
        """
        # 获取楼梯基础数据
        b0 = self.b0  # 楼梯宽度
        b1 = self.b1  # 楼梯顶部挑耳
        b2 = self.b2  # 楼梯底部挑耳
        lb = self.lb_d  # 底端上边长
        lt = self.lt_d  # 顶端上边长
        ln = self.ln  # 楼梯净跨
        total_l = lb + ln + lt  # 楼梯总长
        total_h = 0  # 由于投影的因素，楼梯z坐标为0
        # 创建坐标点
        solid_stair = {}  # 楼梯实体坐标点
        # 第二层标注
        # 顶端上部标注坐标点
        top_top_dimension_loc = [[b0 + b1, total_l, total_h], [0, total_l, total_h]]
        # 底端下部标注坐标点
        bottom_bottom_dimension_loc = [[0, 0, total_h], [b0 + b2, 0, total_h]]
        # 中部右侧标注坐标点
        max_b = max(b1, b2)
        right_dimension_loc = [[b0 + max_b, 0, total_h], [b0 + max_b, total_l, total_h]]
        solid_stair["second floor"] = [
            top_top_dimension_loc,
            bottom_bottom_dimension_loc,
            right_dimension_loc,
        ]
        # 第一层标注
        right_bottom_dimension_loc = [
            [b0 + max_b, 0, total_h],
            [b0 + max_b, lb, total_h],
        ]
        right_top_dimension_loc = [
            [b0 + max_b, lb + ln, total_h],
            [b0 + max_b, total_l, total_h],
        ]
        solid_stair["first floor"] = [
            right_bottom_dimension_loc,
            right_top_dimension_loc,
        ]
        # 特殊第一层标注---文本标注
        right_mid_dimension_loc = [
            [b0 + max_b, lb, total_h],
            [b0 + max_b, lb + ln, total_h],
        ]
        text = str(int(self.tabu_b)) + "x" + str(self.n) + "=" + str(self.ln)
        solid_stair["special floor"] = [[right_mid_dimension_loc], text]
        return solid_stair

    def generate_hoist_embedded_dimension_points(self):
        """
        产生吊装预埋标注点
        :return:
        """
        hoist_loc = (
            self.hoist_embedded_info.get_hoist_embedded_part_loc()
        )  # 获取吊装预埋件的坐标点
        tabu_b = self.tabu_b  # 获取踏步数
        b0 = self.b0  # 楼梯宽度
        # 区分左侧顶部和底部，右侧顶部和底部的吊装预埋件，便于绘图
        left_top_loc = []
        left_bottom_loc = []
        right_top_loc = []
        right_bottom_loc = []
        limit_x = 0
        limit_y = 0
        for num in range(len(hoist_loc)):
            limit_x += hoist_loc[num].x
            limit_y += hoist_loc[num].y
        limit_x = limit_x / len(hoist_loc)
        limit_y = limit_y / len(hoist_loc)
        for num in range(len(hoist_loc)):
            if hoist_loc[num].x < limit_x and hoist_loc[num].y < limit_y:
                left_bottom_loc.extend(
                    [hoist_loc[num].x, hoist_loc[num].y, hoist_loc[num].z]
                )
            elif hoist_loc[num].x < limit_x and hoist_loc[num].y > limit_y:
                left_top_loc.extend(
                    [hoist_loc[num].x, hoist_loc[num].y, hoist_loc[num].z]
                )
            elif hoist_loc[num].x > limit_x and hoist_loc[num].y < limit_y:
                right_bottom_loc.extend(
                    [hoist_loc[num].x, hoist_loc[num].y, hoist_loc[num].z]
                )
            else:
                right_top_loc.extend(
                    [hoist_loc[num].x, hoist_loc[num].y, hoist_loc[num].z]
                )
        # 将z坐标置为0
        left_bottom_loc[2] = 0
        left_top_loc[2] = 0
        right_bottom_loc[2] = 0
        right_top_loc[2] = 0
        # 对于左侧底部吊装预埋件
        left_bottom_y_1 = copy.deepcopy(left_bottom_loc)
        left_bottom_y_1[0] = 0
        left_bottom_y_1[1] += tabu_b / 2
        left_bottom_y_2 = copy.deepcopy(left_bottom_loc)
        left_bottom_y_2[0] = 0
        left_bottom_y_3 = copy.deepcopy(left_bottom_loc)
        left_bottom_y_3[0] = 0
        left_bottom_y_3[1] -= tabu_b / 2
        # 所有待标注的线段
        all_dimension_loc = [
            [left_bottom_y_1, left_bottom_y_2],
            [left_bottom_y_2, left_bottom_y_3],
            [left_bottom_y_2, left_bottom_loc],
        ]
        # 对于左侧顶部吊装预埋件
        left_top_y_1 = copy.deepcopy(left_top_loc)
        left_top_y_1[0] = 0
        left_top_y_1[1] += tabu_b / 2
        left_top_y_2 = copy.deepcopy(left_top_loc)
        left_top_y_2[0] = 0
        left_top_y_3 = copy.deepcopy(left_top_loc)
        left_top_y_3[0] = 0
        left_top_y_3[1] -= tabu_b / 2
        all_dimension_loc.append([left_top_y_1, left_top_y_2])
        all_dimension_loc.append([left_top_y_2, left_top_y_3])
        all_dimension_loc.append([left_top_y_2, left_top_loc])
        # 对于右侧底部吊装预埋件
        right_bottom_y_1 = copy.deepcopy(right_bottom_loc)
        right_bottom_y_1[0] = b0
        right_bottom_y_1[1] -= tabu_b / 2
        right_bottom_y_2 = copy.deepcopy(right_bottom_loc)
        right_bottom_y_2[0] = b0
        right_bottom_y_3 = copy.deepcopy(right_bottom_loc)
        right_bottom_y_3[0] = b0
        right_bottom_y_3[1] += tabu_b / 2
        all_dimension_loc.append([right_bottom_y_1, right_bottom_y_2])
        all_dimension_loc.append([right_bottom_y_2, right_bottom_y_3])
        all_dimension_loc.append([right_bottom_loc, right_bottom_y_2])
        # 对于右侧顶部吊装预埋件
        right_top_y_1 = copy.deepcopy(right_top_loc)
        right_top_y_1[0] = b0
        right_top_y_1[1] -= tabu_b / 2
        right_top_y_2 = copy.deepcopy(right_top_loc)
        right_top_y_2[0] = b0
        right_top_y_3 = copy.deepcopy(right_top_loc)
        right_top_y_3[0] = b0
        right_top_y_3[1] += tabu_b / 2
        all_dimension_loc.append([right_top_y_1, right_top_y_2])
        all_dimension_loc.append([right_top_y_2, right_top_y_3])
        all_dimension_loc.append([right_top_loc, right_top_y_2])
        return all_dimension_loc

    def generate_step_slot_dimension_points(self):
        """
        产生防滑槽标注位置信息
        :return:
        """
        step_slot_loc = self.step_slot_info.get_step_slot_location()
        mid_index = int(len(step_slot_loc) / 2)
        b0 = self.b0  # 楼梯宽度
        dimension_point = step_slot_loc[mid_index]
        point_1 = [dimension_point.x, dimension_point.y, dimension_point.z]  # 待标注点
        point_0 = copy.deepcopy(point_1)
        point_0[0] = 0
        point_2 = [b0 - dimension_point.x, dimension_point.y, dimension_point.z]
        point_3 = copy.deepcopy(point_2)
        point_3[0] = b0
        # 标注位置
        dimension_loc = [[point_0, point_1], [point_2, point_3]]
        return dimension_loc

    def generate_hole_dimension_text_points(self):
        """
        获取孔洞标注文本位置点
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置点
        hole_shape = self.hole_info.get_hole_config()  # 获取孔洞直径信息
        top_top_diam = hole_shape["top_top_dia"]  # 顶端上部直径
        top_bottom_diam = hole_shape["top_bottom_dia"]  # 顶端下部直径
        bottom_top_diam = hole_shape["bottom_top_dia"]  # 底端上部直径
        bottom_bottom_diam = hole_shape["bottom_bottom_dia"]  # 底端下部直径
        # 顶部孔洞说明
        top_hole_dimension = (
            "销键预留洞Φ" + str(top_top_diam) + "(" + str(top_bottom_diam) + ")"
        )
        # 底部孔洞说明
        bottom_hole_dimension = (
            "销键预留洞Φ" + str(bottom_top_diam) + "(" + str(bottom_bottom_diam) + ")"
        )
        limit_y = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            limit_y += current_point.y
        limit_y = limit_y / len(hole_loc)  # 区分顶部和底部的坐标点
        top_loc = []
        bottom_loc = []
        # 筛选出顶部和底部孔洞
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            curr_point_list = [current_point.x, current_point.y, current_point.z]
            if curr_point_list[1] > limit_y:
                top_loc.append(curr_point_list)
            else:
                bottom_loc.append(curr_point_list)
        hole_locations = [top_loc, bottom_loc]
        return hole_locations, top_hole_dimension, bottom_hole_dimension


class BottomViewDimensionData(object):
    """
    仰视图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_basic_class()
        self.generate_dimension_data()
        self.generate_hole_dimension_points()

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

    def generate_dimension_data(self):
        """
        产生数据
        :return:
        """
        self.stair_geom = StairGeometry(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 产生楼梯几何数据
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )  # 产生楼梯孔洞位置数据

    def generate_hole_dimension_points(self):
        """
        产生孔洞标注点：逆时针方向放置
        标注分内层、中部、外层三部分
        :return:dict
        """
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置点
        limit_y = 0
        limit_x = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            limit_y += current_point.y
            limit_x += current_point.x
        limit_y = limit_y / len(hole_loc)  # 区分顶部和底部的坐标点
        limit_x = limit_x / len(hole_loc)  # 区分左侧和右侧的坐标点
        top_loc = []
        bottom_loc = []
        # 筛选出顶部和底部孔洞
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            curr_point_list = [current_point.x, current_point.y, current_point.z]
            if curr_point_list[1] > limit_y:
                top_loc.append(curr_point_list)
            else:
                bottom_loc.append(curr_point_list)
        # 将顶部和底部孔洞点对为逆时针方向
        if top_loc[0][0] < limit_x:
            top_loc.reverse()
        if bottom_loc[0][0] > limit_x:
            bottom_loc.reverse()
        # 存储所有待标注的点集合
        dimension_loc = []
        # 顶部和底部中间标注位置
        top_mid_loc = top_loc
        dimension_loc.append(top_mid_loc)
        bottom_mid_loc = bottom_loc
        dimension_loc.append(bottom_mid_loc)
        # 顶部左侧和右侧标注位置
        # 顶部左侧标注位置
        top_left_point = copy.deepcopy(top_loc[1])
        top_left_point[0] = 0
        top_left_loc = [top_loc[1], top_left_point]  #
        dimension_loc.append(top_left_loc)
        # 顶部右侧标注位置
        top_right_point = copy.deepcopy(top_loc[0])
        top_right_point[0] = self.b0
        top_right_loc = [top_right_point, top_loc[0]]  #
        dimension_loc.append(top_right_loc)
        # 顶部挑耳标注位置，并不一定会有挑耳
        top_ear_point = copy.deepcopy(top_right_point)
        top_ear_point[0] += self.b1
        top_ear_loc = [top_ear_point, top_right_point]  #
        dimension_loc.append(top_ear_loc)
        # 底部左侧和右侧标注位置
        # 底部左侧标注位置
        bottom_left_point = copy.deepcopy(bottom_loc[0])
        bottom_left_point[0] = 0
        bottom_left_loc = [bottom_left_point, bottom_loc[0]]  #
        dimension_loc.append(bottom_left_loc)
        # 底部右侧标注位置
        bottom_right_point = copy.deepcopy(bottom_loc[1])
        bottom_right_point[0] = self.b0
        bottom_right_loc = [bottom_loc[1], bottom_right_point]  #
        dimension_loc.append(bottom_right_loc)
        # 底部挑耳标注位置，并不一定会有挑耳
        bottom_ear_point = copy.deepcopy(bottom_right_point)
        bottom_ear_point[0] += self.b2
        bottom_ear_loc = [bottom_right_point, bottom_ear_point]  #
        dimension_loc.append(bottom_ear_loc)
        # 顶部孔洞左侧纵向标注位置
        top_left_loc_s = copy.deepcopy(top_loc[0])
        top_left_loc_e = copy.deepcopy(top_loc[0])
        top_left_loc_s[1] = self.lb_d + self.lt_d + self.ln
        top_left_horizon_loc = [top_left_loc_s, top_left_loc_e]
        dimension_loc.append(top_left_horizon_loc)
        # 底部孔洞左侧纵向标注位置
        bottom_left_loc_s = copy.deepcopy(bottom_loc[0])
        bottom_left_loc_e = copy.deepcopy(bottom_loc[0])
        bottom_left_loc_e[1] = 0
        bottom_left_horizon_loc = [bottom_left_loc_s, bottom_left_loc_e]
        dimension_loc.append(bottom_left_horizon_loc)
        dim_loc = {}  # 获取数据
        dim_loc["location"] = dimension_loc
        dim_loc["floor"] = "first floor"  # 标注层数
        return dim_loc

    def generate_stair_solid_dimension_points(self):
        """
        产生楼梯实体标注点:逆时针方向,[[x1,y1,z1],[x2,y2,z2]]
        :return:
        """
        # 获取楼梯基础数据
        b0 = self.b0  # 楼梯宽度
        b1 = self.b1  # 楼梯顶部挑耳
        b2 = self.b2  # 楼梯底部挑耳
        lb = self.lb_d  # 底端上边长
        lt = self.lt_d  # 顶端上边长
        ln = self.ln  # 楼梯净跨
        l1b = self.l1b  # 楼梯底端下边长
        l1t = self.l1t  # 楼梯顶端下边长
        max_lb = l1b
        total_l = lb + ln + lt  # 楼梯总长
        total_h = 0  # 由于投影的因素，楼梯z坐标为0
        # 创建坐标点
        solid_stair = {}  # 楼梯实体标注坐标点
        # 第二层标注
        # 顶端上部标注坐标点
        top_top_dimension_loc = [[b0 + b1, total_l, total_h], [0, total_l, total_h]]
        # 底端下部标注坐标点
        bottom_bottom_dimension_loc = [[0, 0, total_h], [b0 + b2, 0, total_h]]
        # 中部左侧第一层标注
        left_top_dimension_loc = [[0, total_l, total_h], [0, total_l - l1t, total_h]]
        left_mid_dimension_loc = [[0, total_l - l1t, total_h], [0, max_lb, total_h]]
        left_bottom_dimension_loc = [[0, max_lb, total_h], [0, 0, total_h]]
        # 中部右侧标注坐标点
        # 第一层标注点
        max_b = max(b1, b2)
        max_t = lt if b1 > 0 else l1t
        right_bottom_in_dimension_loc = [
            [b0 + max_b, 0, total_h],
            [b0 + max_b, max_lb, total_h],
        ]
        right_mid_in_dimension_loc = [
            [b0 + max_b, max_lb, total_h],
            [b0 + max_b, total_l - max_t, total_h],
        ]
        right_top_in_dimension_loc = [
            [b0 + max_b, total_l - max_t, total_h],
            [b0 + max_b, total_l, total_h],
        ]
        # 第二层标注点
        right_out_dimension_loc = [
            [b0 + max_b, 0, total_h],
            [b0 + max_b, total_l, total_h],
        ]
        # 合并标注点
        solid_stair["first floor"] = [
            right_bottom_in_dimension_loc,
            right_mid_in_dimension_loc,
            right_top_in_dimension_loc,
            left_top_dimension_loc,
            left_mid_dimension_loc,
            left_bottom_dimension_loc,
        ]
        solid_stair["second floor"] = [
            top_top_dimension_loc,
            bottom_bottom_dimension_loc,
            right_out_dimension_loc,
        ]

        return solid_stair

    def generate_hole_dimension_text_points(self):
        """
        获取孔洞标注文本位置点
        :return:
        """
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置点
        hole_shape = self.hole_info.get_hole_config()  # 获取孔洞直径信息
        top_top_diam = hole_shape["top_top_dia"]  # 顶端上部直径
        top_bottom_diam = hole_shape["top_bottom_dia"]  # 顶端下部直径
        bottom_top_diam = hole_shape["bottom_top_dia"]  # 底端上部直径
        bottom_bottom_diam = hole_shape["bottom_bottom_dia"]  # 底端下部直径
        # 顶部孔洞说明
        top_hole_dimension = (
            "销键预留洞Φ" + str(top_top_diam) + "(" + str(top_bottom_diam) + ")"
        )
        # 底部孔洞说明
        bottom_hole_dimension = (
            "销键预留洞Φ" + str(bottom_top_diam) + "(" + str(bottom_bottom_diam) + ")"
        )
        limit_y = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            limit_y += current_point.y
        limit_y = limit_y / len(hole_loc)  # 区分顶部和底部的坐标点
        top_loc = []
        bottom_loc = []
        # 筛选出顶部和底部孔洞
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            curr_point_list = [current_point.x, current_point.y, current_point.z]
            if curr_point_list[1] > limit_y:
                top_loc.append(curr_point_list)
            else:
                bottom_loc.append(curr_point_list)
        hole_locations = [top_loc, bottom_loc]
        return hole_locations, top_hole_dimension, bottom_hole_dimension


class LeftViewDimensionData(object):
    """
    左侧视图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
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

    def generate_stair_solid_dimension_standard_point(self):
        """
        产生楼梯实体标注点
        :return:
        """
        # 获取基础数据
        lb = self.lb_d
        l1b = self.l1b
        max_l1b = max(l1b, lb)
        h = self.h
        h1 = self.h1
        h2 = self.h2
        lt = self.lt_d
        l1t = self.l1t
        tan_theta = self.tan
        ln = self.ln  # 楼梯净跨
        total_H = h2 + h  # 楼梯总高
        total_L = lb + ln + lt  # 楼梯总长
        loc_x = 0
        # 顶端厚度方向标注点
        top_left_h = [[loc_x, total_L, total_H], [loc_x, total_L, total_H - h1]]
        # 顶端下部标注点
        top_bottom_l = [
            [loc_x, total_L, total_H - h1],
            [loc_x, total_L - l1t, total_H - h1],
        ]
        # 底端下部标注点
        bottom_bottom_l = [[loc_x, max_l1b, 0], [loc_x, 0, 0]]
        # 底端右侧标注点
        bottom_right_h = [[loc_x, 0, 0], [loc_x, 0, h2]]
        result = [top_left_h, top_bottom_l, bottom_bottom_l, bottom_right_h]
        return result

    def generate_stair_solid_dimension_xie_special_point(self):
        """
        获取楼梯斜向特殊标注点
        :return:
        """
        lb = self.lb_d
        l1b = self.l1b
        h = self.h
        h1 = self.h1
        h2 = self.h2
        lt = self.lt_d
        l1t = self.l1t
        tan_theta = self.tan
        ln = self.ln  # 楼梯净跨
        total_H = h2 + h  # 楼梯总高
        total_L = lb + ln + lt  # 楼梯总长
        loc_x = 0
        # 中部下部斜向标注
        if lb < l1b:
            special_loc = [loc_x, l1b, 0]
        else:
            delta_h = abs(l1b - lb) * tan_theta
            special_loc = [loc_x, lb, delta_h]
        mid_bottom_l = [
            [
                [loc_x, total_L - l1t, total_H - h1],
                [special_loc[0], special_loc[1], special_loc[2]],
            ]
        ]
        return mid_bottom_l

    def get_hole_dimension_point(self):
        """
        获取孔洞定位点
        :return:
        """
        # 获取基础数据
        hole_loc = self.hole_info.get_hole_loc()
        lt = self.lt_d
        lb = self.lb_d
        total_L = self.lb_d + self.ln + self.lt_d  # 楼梯总长
        total_H = self.h2 + self.h  # 楼梯总的高度
        loc_x = 0
        top_hole_loc = []
        bottom_hole_loc = []
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            if current_point.y > limit_y:
                top_hole_loc.append(current_point)
            else:
                bottom_hole_loc.append(current_point)
        # 顶部孔洞位置
        top_point = top_hole_loc[0]
        point_t_1 = [loc_x, top_point.y, total_H]
        point_t_2 = [loc_x, total_L, total_H]
        point_t_0 = [loc_x, total_L - lt, total_H]
        # 底部孔洞位置
        bottom_point = bottom_hole_loc[0]
        point_b_1 = [loc_x, bottom_point.y, total_H]
        point_b_0 = [loc_x, 0, total_H]
        point_b_2 = [loc_x, lb, total_H]
        result = [
            [point_b_0, point_b_1],
            [point_b_1, point_b_2],
            [point_t_0, point_t_1],
            [point_t_1, point_t_2],
        ]
        return result

    def get_hoist_dimension_point(self):
        """
        获取吊装预埋件位置标注点
        :return:
        """
        hoist_loc = self.hoist_loc.get_hoist_embedded_part_loc()  # 获取吊装预埋件坐标位置
        tabu_b = self.tabu_b  # 踏步宽度
        limit_x = self.b0 / 2
        left_loc = []
        for num in range(len(hoist_loc)):
            current_point = hoist_loc[num]  # 当前吊装预埋件位置
            if current_point.x < limit_x:
                left_loc.append([current_point.x, current_point.y, current_point.z])
        dimension_loc = []
        for num in range(len(left_loc)):
            point_1 = copy.deepcopy(left_loc[num])
            point_0 = copy.deepcopy(point_1)
            point_0[1] -= tabu_b / 2
            point_2 = copy.deepcopy(point_1)
            point_2[1] += tabu_b / 2
            dimension_loc.append([point_0, point_1])
            dimension_loc.append([point_1, point_2])
        return dimension_loc

    def get_stair_dimension_thickness_point(self):
        """
        标注楼梯板厚
        :return:
        """
        import numpy as np

        num = int(self.n / 2)
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        thickness = self.t
        vector_1 = np.array([0, tabu_b, tabu_h])
        vector_2 = np.array([-1, 0, 0])
        # 形成法向量
        normal = np.cross(vector_2, vector_1) / (
            np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
        )
        loc_x = 0
        loc_y = self.lb_d + num * self.tabu_b
        loc_z = self.h2 + num * self.tabu_h
        # 形成点
        point_1 = [loc_x, loc_y, loc_z]
        point_2 = normal * thickness + np.array(point_1)
        point_2 = point_2.tolist()
        dimension_point = [point_1, point_2]
        return dimension_point

    def get_special_dimension_text(self):
        """
        获取特殊标注数据
        :return:
        """
        # 获取基础数据
        lb = self.lb_d
        l1b = self.l1b
        h = self.h
        h2 = self.h2
        lt = self.lt_d
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        ln = self.ln  # 楼梯净跨
        total_H = h2 + h  # 楼梯总高
        total_L = lb + ln + lt  # 楼梯总长
        loc_x = 0
        # 中间上部标注点
        mid_top_loc = [[loc_x, lb, total_H], [loc_x, total_L - lt, total_H]]
        mid_top_text = str(tabu_b) + "X" + str(self.n - 1) + "=" + str(ln)
        # 右侧顶部标注点
        left_top_loc = [[loc_x, 0, h2], [loc_x, 0, total_H]]
        left_top_text = str(tabu_h) + "X" + str(self.n) + "=" + str(h)
        result = [[mid_top_loc, mid_top_text], [left_top_loc, left_top_text]]
        return result


class RightViewDimensionData(object):
    """
    右侧视图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
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


class StairSectionOneViewDimensionData(object):
    """
    获取楼梯剖面1-1视图标注数据
    """

    def __init__(self, slab_struct, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
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

    def get_stair_hole_dimension_point(self):
        """
        对楼梯实体和孔洞进行标注
        :return:
        """
        # 获取基础数据
        hole_loc = self.hole_info.get_hole_loc()
        h1 = self.h1
        total_H = self.h2 + self.h  # 楼梯总的高度
        loc_y = 0
        total_B = self.b0 + self.b1
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        limit_x = self.b0 / 2
        top_left_hole_x = 0
        top_right_hole_x = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            if current_point.y > limit_y and current_point.x < limit_x:  # 顶部左侧孔洞位置
                top_left_hole_x = current_point.x  # 顶部左侧孔洞位置
            elif current_point.y > limit_y and current_point.x > limit_x:  # 顶部右侧孔洞位置
                top_right_hole_x = current_point.x  # 顶部右侧孔洞位置
        # 第一层标注
        # 剖切图左侧标注点
        left_top_1 = [0, loc_y, total_H]
        left_top_2 = [0, loc_y, total_H - h1]
        # 剖切图顶部标注点
        top_right_0 = [total_B, loc_y, total_H]  # 顶部右侧标注点
        top_right_1 = [top_right_hole_x, loc_y, total_H]  # 顶部右侧孔洞标注点
        top_left_0 = [top_left_hole_x, loc_y, total_H]  # 顶部左侧孔洞标注点
        top_left_1 = [0, loc_y, total_H]  # 顶部孔洞标注点

        # 第二层标注点
        top_external_1 = [total_B, loc_y, total_H]  # 顶端外侧右端点
        top_external_2 = [0, loc_y, total_H]  # 顶端外侧左端点
        first_floor = [
            [left_top_1, left_top_2],
            [top_right_0, top_right_1],
            [top_left_0, top_left_1],
            [top_right_1, top_left_0],
        ]
        second_floor = [[top_external_1, top_external_2]]
        # 标注信息
        dimension_loc = {}
        dimension_loc["first floor"] = first_floor
        dimension_loc["second floor"] = second_floor

        return dimension_loc

    def get_hole_dimension_point(self):
        """
        获取孔洞定位点,对孔洞进行单独标注
        注意：由于连接孔洞类型的不同，生成的标注点对可能存在空的情况，在进行标注时需要排除。
        :return:
        """
        # 获取基础数据
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置信息
        hole_config = self.hole_info.get_hole_config()  # 获取孔洞配置信息
        h1 = self.h1
        total_H = self.h2 + self.h  # 楼梯总的高度
        loc_y = 0
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        limit_x = self.b0 / 2
        top_left_hole_x = 0
        top_right_hole_x = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            if current_point.y > limit_y and current_point.x < limit_x:  # 顶部左侧孔洞位置
                top_left_hole_x = current_point.x  # 顶部左侧孔洞位置
            elif current_point.y > limit_y and current_point.x > limit_x:  # 顶部右侧孔洞位置
                top_right_hole_x = current_point.x  # 顶部右侧孔洞位置
        # 孔洞位置
        left_top_loc = [top_left_hole_x, loc_y, total_H]  # 左侧顶部位置
        right_top_loc = [top_right_hole_x, loc_y, total_H]  # 右侧顶部位置
        left_bottom_loc = [top_left_hole_x, loc_y, total_H - h1]  # 左侧底部位置
        right_bottom_loc = [top_right_hole_x, loc_y, total_H - h1]  # 右侧底部位置
        # 获取顶端上部孔洞直径和顶端下部孔洞直径
        top_diam = hole_config["top_top_dia"]
        bottom_diam = hole_config["top_bottom_dia"]
        # 获取顶端右侧上部孔洞标注点
        right_top_dimension_1 = copy.deepcopy(right_top_loc)
        right_top_dimension_1[0] += top_diam / 2
        right_top_dimension_2 = copy.deepcopy(right_top_loc)
        right_top_dimension_2[0] -= top_diam / 2
        # 获取顶端右侧下部孔洞标注点
        right_bottom_dimension_1 = copy.deepcopy(right_bottom_loc)
        right_bottom_dimension_1[0] -= bottom_diam / 2
        right_bottom_dimension_2 = copy.deepcopy(right_bottom_loc)
        right_bottom_dimension_2[0] += bottom_diam / 2
        # 获取顶端左侧上部孔洞标注点
        left_top_dimension_1 = copy.deepcopy(left_top_loc)
        left_top_dimension_1[0] += top_diam / 2
        left_top_dimension_2 = copy.deepcopy(left_top_loc)
        left_top_dimension_2[0] -= top_diam / 2
        # 获取顶端左侧下部孔洞标注点
        left_bottom_dimension_1 = copy.deepcopy(left_bottom_loc)
        left_bottom_dimension_1[0] -= bottom_diam / 2
        left_bottom_dimension_2 = copy.deepcopy(left_bottom_loc)
        left_bottom_dimension_2[0] += bottom_diam / 2
        # 孔洞顶端标注文本
        top_dimension_text = "Φ" + str(top_diam)
        # 孔洞底端标注文本
        bottom_dimension_text = "Φ" + str(bottom_diam)
        special_dimension = []
        hole_type = hole_config["top_type"]

        # 生成孔洞特殊位置
        if hole_type == 1:  # 滑动铰支座
            # 获取数据
            dimension_h = hole_config["top_h"]
            diam_1 = hole_config["top_top_dia_1"]
            diam_2 = hole_config["top_bottom_dia_1"]
            # 左侧中心点
            left_center = copy.deepcopy(left_top_loc)
            left_center[2] -= dimension_h
            left_p1 = [left_center[0] - diam_1 / 2, left_center[1], left_center[2]]
            left_p2 = [left_center[0] - diam_2 / 2, left_center[1], left_center[2]]
            left_p3 = [left_center[0] + diam_2 / 2, left_center[1], left_center[2]]
            left_p4 = [left_center[0] + diam_1 / 2, left_center[1], left_center[2]]

            # 右侧中心点
            right_center = copy.deepcopy(right_top_loc)
            right_center[2] -= dimension_h
            right_p1 = [right_center[0] - diam_1 / 2, right_center[1], right_center[2]]
            right_p2 = [right_center[0] - diam_2 / 2, right_center[1], right_center[2]]
            right_p3 = [right_center[0] + diam_2 / 2, right_center[1], right_center[2]]
            right_p4 = [right_center[0] + diam_1 / 2, right_center[1], right_center[2]]
            special_dimension.append([left_p2, left_p1])
            special_dimension.append([left_p4, left_p3])
            special_dimension.append([right_p2, right_p1])
            special_dimension.append([right_p4, right_p3])
        # 合并数据
        top_dimension_loc = [
            [left_top_dimension_1, left_top_dimension_2],
            [right_top_dimension_1, right_top_dimension_2],
        ]
        bottom_dimension_loc = [
            [left_bottom_dimension_1, left_bottom_dimension_2],
            [right_bottom_dimension_1, right_bottom_dimension_2],
        ]
        total_dimension_info = [
            [top_dimension_loc, bottom_dimension_loc],
            [top_dimension_text, bottom_dimension_text],
            special_dimension,
        ]
        return total_dimension_info

    def get_special_signal_dimension_point(self):
        """
        获取特殊符号标注点和文本
        :return:
        """
        special_length = 36  # 三角形的高
        h1 = self.h1
        b0 = self.b0
        total_H = self.h2 + self.h
        loc_y = 0
        # 标注位置
        # 标注符号顶部点位置
        top_loc_p1 = [b0 / 2, loc_y, total_H]
        top_loc_p2 = [b0 / 2 - special_length, loc_y, total_H + special_length]
        top_loc_p3 = [b0 / 2 + special_length, loc_y, total_H + special_length]
        # 标注符号底部点位置
        bottom_loc_p1 = [b0 / 2, loc_y, total_H - h1]
        bottom_loc_p2 = [b0 / 2 - special_length, loc_y, total_H - h1 - special_length]
        bottom_loc_p3 = [b0 / 2 + special_length, loc_y, total_H - h1 - special_length]
        # 标注文本
        top_text = "楼梯顶面"
        top_text_loc = top_loc_p3
        bottom_text = "楼梯底面"
        bottom_text_loc = [
            bottom_loc_p3[0],
            bottom_loc_p3[1],
            bottom_loc_p3[2] - 1.5 * special_length,
        ]
        # 标注符号位置
        signal_loc = [
            [top_loc_p1, top_loc_p2],
            [top_loc_p1, top_loc_p3],
            [top_loc_p2, top_loc_p3],
            [bottom_loc_p1, bottom_loc_p2],
            [bottom_loc_p1, bottom_loc_p3],
            [bottom_loc_p2, bottom_loc_p3],
        ]
        # 标注文本及内容
        dimension_text = [[top_text, bottom_text], [top_text_loc, bottom_text_loc]]
        return signal_loc, dimension_text


class StairSectionTwoViewDimensionData(object):
    """
    获取楼梯剖面2-2视图标注数据
    """

    def __init__(self, slab_struct, detail_slab, struct_book, detail_book):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )
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

    def get_stair_hole_dimension_point(self):
        """
        对楼梯底部剖切实体和孔洞进行标注
        :return:
        """
        # 获取基础数据
        hole_loc = self.hole_info.get_hole_loc()
        h2 = self.h2
        loc_y = 0
        total_B = self.b0 + self.b2
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        limit_x = self.b0 / 2
        bottom_left_hole_x = 0
        bottom_right_hole_x = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            if current_point.y < limit_y and current_point.x < limit_x:  # 底部左侧孔洞位置
                bottom_left_hole_x = current_point.x  # 底部左侧孔洞位置
            elif current_point.y < limit_y and current_point.x > limit_x:  # 底部右侧孔洞位置
                bottom_right_hole_x = current_point.x  # 底部右侧孔洞位置
        # 第一层标注
        # 剖切图左侧标注点
        left_top_1 = [0, loc_y, h2]
        left_top_2 = [0, loc_y, 0]
        # 剖切图顶部标注点
        top_right_0 = [total_B, loc_y, h2]  # 顶部右侧标注点
        top_right_1 = [bottom_right_hole_x, loc_y, h2]  # 顶部右侧孔洞标注点
        top_left_0 = [bottom_left_hole_x, loc_y, h2]  # 顶部左侧孔洞标注点
        top_left_1 = [0, loc_y, h2]  # 顶部孔洞标注点

        # 第二层标注点
        top_external_1 = [total_B, loc_y, h2]  # 底端端外侧右端点
        top_external_2 = [0, loc_y, h2]  # 底端端外侧左端点
        first_floor = [
            [left_top_1, left_top_2],
            [top_right_0, top_right_1],
            [top_left_0, top_left_1],
            [top_right_1, top_left_0],
        ]
        second_floor = [[top_external_1, top_external_2]]
        # 标注信息
        dimension_loc = {}
        dimension_loc["first floor"] = first_floor
        dimension_loc["second floor"] = second_floor

        return dimension_loc

    def get_hole_dimension_point(self):
        """
        获取孔洞定位点,对孔洞进行单独标注
        注意：由于连接孔洞类型的不同，生成的标注点对可能存在空的情况，在进行标注时需要排除。
        :return:
        """
        # 获取基础数据
        hole_loc = self.hole_info.get_hole_loc()  # 获取孔洞位置信息
        hole_config = self.hole_info.get_hole_config()  # 获取孔洞配置信息
        h2 = self.h2  # 楼梯底端板厚度
        loc_y = 0
        limit_y = (self.lb_d + self.lt_d + self.ln) / 2
        limit_x = self.b0 / 2
        bottom_left_hole_x = 0
        bottom_right_hole_x = 0
        for num in range(len(hole_loc)):
            current_point = hole_loc[num]
            if current_point.y < limit_y and current_point.x < limit_x:  # 底部左侧孔洞位置
                bottom_left_hole_x = current_point.x  # 底部左侧孔洞位置
            elif current_point.y < limit_y and current_point.x > limit_x:  # 底部右侧孔洞位置
                bottom_right_hole_x = current_point.x  # 底部右侧孔洞位置
        # 孔洞位置
        left_top_loc = [bottom_left_hole_x, loc_y, h2]  # 左侧顶部位置
        right_top_loc = [bottom_right_hole_x, loc_y, h2]  # 右侧顶部位置
        left_bottom_loc = [bottom_left_hole_x, loc_y, 0]  # 左侧底部位置
        right_bottom_loc = [bottom_right_hole_x, loc_y, 0]  # 右侧底部位置
        # 获取顶底端上部孔洞直径和底端下部孔洞直径
        top_diam = hole_config["bottom_top_dia"]
        bottom_diam = hole_config["bottom_bottom_dia"]
        # 获取底端右侧上部孔洞标注点
        right_top_dimension_1 = copy.deepcopy(right_top_loc)
        right_top_dimension_1[0] += top_diam / 2
        right_top_dimension_2 = copy.deepcopy(right_top_loc)
        right_top_dimension_2[0] -= top_diam / 2
        # 获取底端右侧下部孔洞标注点
        right_bottom_dimension_1 = copy.deepcopy(right_bottom_loc)
        right_bottom_dimension_1[0] -= bottom_diam / 2
        right_bottom_dimension_2 = copy.deepcopy(right_bottom_loc)
        right_bottom_dimension_2[0] += bottom_diam / 2
        # 获取底端左侧上部孔洞标注点
        left_top_dimension_1 = copy.deepcopy(left_top_loc)
        left_top_dimension_1[0] += top_diam / 2
        left_top_dimension_2 = copy.deepcopy(left_top_loc)
        left_top_dimension_2[0] -= top_diam / 2
        # 获取底端左侧下部孔洞标注点
        left_bottom_dimension_1 = copy.deepcopy(left_bottom_loc)
        left_bottom_dimension_1[0] -= bottom_diam / 2
        left_bottom_dimension_2 = copy.deepcopy(left_bottom_loc)
        left_bottom_dimension_2[0] += bottom_diam / 2
        # 孔洞顶端标注文本
        top_dimension_text = "Φ" + str(top_diam)
        # 孔洞底端标注文本
        bottom_dimension_text = "Φ" + str(bottom_diam)
        special_dimension = []
        hole_type = hole_config["bottom_type"]

        # 生成孔洞特殊位置
        if hole_type == 1:  # 滑动铰支座
            # 获取数据
            dimension_h = hole_config["bottom_h"]
            diam_1 = hole_config["bottom_top_dia_1"]
            diam_2 = hole_config["bottom_bottom_dia_1"]
            # 左侧中心点
            left_center = copy.deepcopy(left_top_loc)
            left_center[2] -= dimension_h
            left_p1 = [left_center[0] - diam_1 / 2, left_center[1], left_center[2]]
            left_p2 = [left_center[0] - diam_2 / 2, left_center[1], left_center[2]]
            left_p3 = [left_center[0] + diam_2 / 2, left_center[1], left_center[2]]
            left_p4 = [left_center[0] + diam_1 / 2, left_center[1], left_center[2]]

            # 右侧中心点
            right_center = copy.deepcopy(right_top_loc)
            right_center[2] -= dimension_h
            right_p1 = [right_center[0] - diam_1 / 2, right_center[1], right_center[2]]
            right_p2 = [right_center[0] - diam_2 / 2, right_center[1], right_center[2]]
            right_p3 = [right_center[0] + diam_2 / 2, right_center[1], right_center[2]]
            right_p4 = [right_center[0] + diam_1 / 2, right_center[1], right_center[2]]
            special_dimension.append([left_p1, left_p2])
            special_dimension.append([left_p3, left_p4])
            special_dimension.append([right_p1, right_p2])
            special_dimension.append([right_p3, right_p4])
        # 合并数据
        top_dimension_loc = [
            [left_top_dimension_1, left_top_dimension_2],
            [right_top_dimension_1, right_top_dimension_2],
        ]
        bottom_dimension_loc = [
            [left_bottom_dimension_1, left_bottom_dimension_2],
            [right_bottom_dimension_1, right_bottom_dimension_2],
        ]
        total_dimension_info = [
            [top_dimension_loc, bottom_dimension_loc],
            [top_dimension_text, bottom_dimension_text],
            special_dimension,
        ]
        return total_dimension_info

    def get_special_signal_dimension_point(self):
        """
        获取特殊符号标注点和文本
        :return:
        """
        special_length = 36  # 三角形的高
        h2 = self.h2
        b0 = self.b0
        loc_y = 0
        # 标注位置
        # 标注符号顶部点位置
        top_loc_p1 = [b0 / 2, loc_y, h2]
        top_loc_p2 = [b0 / 2 - special_length, loc_y, h2 + special_length]
        top_loc_p3 = [b0 / 2 + special_length, loc_y, h2 + special_length]
        # 标注符号底部点位置
        bottom_loc_p1 = [b0 / 2, loc_y, 0]
        bottom_loc_p2 = [b0 / 2 - special_length, loc_y, 0 - special_length]
        bottom_loc_p3 = [b0 / 2 + special_length, loc_y, 0 - special_length]
        # 标注文本
        top_text = "楼梯顶面"
        top_text_loc = top_loc_p3
        bottom_text = "楼梯底面"
        bottom_text_loc = [
            bottom_loc_p3[0],
            bottom_loc_p3[1],
            bottom_loc_p3[2] - 1.5 * special_length,
        ]
        # 标注符号位置
        signal_loc = [
            [top_loc_p1, top_loc_p2],
            [top_loc_p1, top_loc_p3],
            [top_loc_p2, top_loc_p3],
            [bottom_loc_p1, bottom_loc_p2],
            [bottom_loc_p1, bottom_loc_p3],
            [bottom_loc_p2, bottom_loc_p3],
        ]
        # 标注文本及内容
        dimension_text = [[top_text, bottom_text], [top_text_loc, bottom_text_loc]]
        return signal_loc, dimension_text


class ReinforceViewDimensionData(object):
    """
    配筋视图标注数据获取
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.get_construct_info()  # 获取构造信息
        self.get_rebar_config_info()  # 获取钢筋配置信息
        self.generate_basic_class()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.t = self.slab_struct.geometric.thickness
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

    def get_construct_info(self):
        """
        获取配置信息
        :return:
        """
        self.hole_info = HoleLocation(
            self.slab_struct, self.detail_slab, self.struct_book, self.detail_book
        )

    def get_rebar_config_info(self):
        """
        获取钢筋配置信息
        :return:
        """
        self.mid_distribute_rebar = MidDistributionRebar(
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
        self.bottom_edge_stir = BottomEdgeStirrup(
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
        self.bottom_edge_long_rebar = BottomEdgeLongitudinalRebar(
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

    def get_stair_solid_top_dimension_point(self):
        """
        获取楼梯实体顶部标注点
        :return:List[List[List[float]]]
        """
        # 获取顶部边缘箍筋信息
        self.top_edge_stir_loc = self.top_edge_stir.get_rebar_model()  # 获取顶部边缘箍筋坐标
        self.top_edge_stir_diam = self.top_edge_stir.get_rebar_diameter()  # 获取顶部边缘箍筋直径
        # 获取顶部边缘纵筋信息
        self.top_edge_long_rebar_loc = (
            self.top_edge_long_rebar.get_rebar_model()
        )  # 获取顶部边缘纵筋坐标
        # 获取基础数据
        total_H = self.h2 + self.h  # 楼梯总高
        total_L = self.lb_d + self.ln + self.lt_d  # 楼梯总长
        total_l1 = self.lb_d + self.ln
        loc_x = 0
        total_h1 = total_H - self.h1
        left_top_h = total_h1  # 左侧顶端顶部高度
        left_bottom_h = total_H  # 左侧顶端底部高度
        top_stir_loc = self.top_edge_stir_loc[0]  # 获取顶部边缘箍筋坐标系列点
        # 遍历顶部边缘箍筋点，获取最大的z和最小的z
        for num in range(len(top_stir_loc)):
            current_point = top_stir_loc[num]
            if current_point.z < left_bottom_h:
                left_bottom_h = current_point.z
            if current_point.z > left_top_h:
                left_top_h = current_point.z
        # 筛选出顶部边缘纵向钢筋上部和下部
        top_top_y = [total_l1]
        limit_z = (total_H + total_h1) / 2
        for num in range(len(self.top_edge_long_rebar_loc)):
            current_point = self.top_edge_long_rebar_loc[num][0]
            if current_point.z > limit_z:
                top_top_y.append(current_point.y)
        top_top_y.append(total_L)
        # 左侧顶端标注点
        left_top_p1 = [loc_x, total_L, total_H]
        left_top_p2 = [loc_x, total_L, left_top_h + self.top_edge_stir_diam / 2]
        left_top_p3 = [loc_x, total_L, left_bottom_h - self.top_edge_stir_diam / 2]
        left_top_p4 = [loc_x, total_L, total_h1]
        # 顶端标注点
        top_dimension_point = []
        for num in range(len(top_top_y) - 1):
            top_dimension_point.append(
                [[loc_x, top_top_y[num], total_H], [loc_x, top_top_y[num + 1], total_H]]
            )
        top_dimension_point.append([left_top_p1, left_top_p2])
        top_dimension_point.append([left_top_p2, left_top_p3])
        top_dimension_point.append([left_top_p3, left_top_p4])
        return top_dimension_point

    def get_stair_solid_bottom_dimension_point(self):
        """
        获取楼梯实体底部标注点
        :return:List[List[List[float]]]
        """
        # 获取底部边缘纵筋信息
        self.bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar.get_rebar_model()
        )  # 获取底部边缘纵筋坐标
        h2 = self.h2
        limit_z = (0 + self.h2) / 2
        loc_x = 0
        bottom_top_y = [0]
        for num in range(len(self.bottom_edge_long_rebar_loc)):
            current_point = self.bottom_edge_long_rebar_loc[num][0]
            if current_point.z > limit_z:
                bottom_top_y.append(current_point.y)
        bottom_top_y.append(self.lb_d)
        # 底端标注点
        bottom_dimension_point = []
        for num in range(len(bottom_top_y) - 1):
            bottom_dimension_point.append(
                [[loc_x, bottom_top_y[num], h2], [loc_x, bottom_top_y[num + 1], h2]]
            )
        return bottom_dimension_point

    def get_draw_cut_signal_dimension_point(self):
        """
        获取绘制剖切符号标注点
        :return:List[line_loc,dimension_text],line_loc=List[List[float]],dimension_text = [list[float],list[float],str]
        """
        signal_length = 80  # 剖切符长度
        signal_edge = 100  # 剖切符边距
        loc_x = 0
        total_h = self.h2 + self.h  # 楼梯高度
        total_h1 = self.h2 + self.h - self.h1  # 楼梯顶端下边高
        thickness = self.t  # 板厚度
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        h2 = self.h2
        h1 = self.h1
        current_n = int(self.n / 2)
        # 剖切线标志点
        top_loc = [loc_x, self.lb_d + self.ln + 2 * self.lt_d / 3, total_h]
        mid_loc = [loc_x, self.lb_d + current_n * tabu_b, h2 + current_n * tabu_h]
        bottom_loc = [loc_x, 2 * self.lb_d / 3, h2]
        # 顶部剖切线
        top_p1 = [top_loc[0], top_loc[1], top_loc[2] + signal_edge + signal_length]
        top_p2 = [top_loc[0], top_loc[1], top_loc[2] + signal_edge]
        top_p3 = [top_loc[0], top_loc[1], top_loc[2] - h1 - signal_edge]
        top_p4 = [top_loc[0], top_loc[1], top_loc[2] - h1 - signal_edge - signal_length]
        top_text_p1 = [
            (top_p1[0] + top_p2[0]) / 2,
            (top_p1[1] + top_p2[1]) / 2,
            (top_p1[2] + top_p2[2]) / 2,
        ]
        top_text_p2 = [
            (top_p3[0] + top_p4[0]) / 2,
            (top_p3[1] + top_p4[1]) / 2,
            (top_p3[2] + top_p4[2]) / 2,
        ]
        top_text_content = "a"
        # 底部剖切线
        bottom_p1 = [
            bottom_loc[0],
            bottom_loc[1],
            bottom_loc[2] + signal_edge + signal_length,
        ]
        bottom_p2 = [bottom_loc[0], bottom_loc[1], bottom_loc[2] + signal_edge]
        bottom_p3 = [bottom_loc[0], bottom_loc[1], bottom_loc[2] - h2 - signal_edge]
        bottom_p4 = [
            bottom_loc[0],
            bottom_loc[1],
            bottom_loc[2] - h1 - signal_edge - signal_length,
        ]
        bottom_text_p1 = [
            (bottom_p1[0] + bottom_p2[0]) / 2,
            (bottom_p1[1] + bottom_p2[1]) / 2,
            (bottom_p1[2] + bottom_p2[2]) / 2,
        ]
        bottom_text_p2 = [
            (bottom_p3[0] + bottom_p4[0]) / 2,
            (bottom_p3[1] + bottom_p4[1]) / 2,
            (bottom_p3[2] + bottom_p4[2]) / 2,
        ]
        bottom_text_content = "c"
        # 中部剖切线
        vector_1 = np.array([0, tabu_b, tabu_h])
        vector_2 = np.array([-1, 0, 0])
        direction_add = np.cross(vector_1, vector_2) / (
            np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
        )
        direction_minus = direction_add * (-1)
        # 点1
        mid_p1_a = np.array(mid_loc) + direction_add * (signal_edge + signal_length)
        mid_p1 = mid_p1_a.tolist()
        # 点2
        mid_p2_a = np.array(mid_loc) + direction_add * (signal_edge)
        mid_p2 = mid_p2_a.tolist()
        # 点3
        mid_p3_a = np.array(mid_loc) + direction_minus * (thickness + signal_edge)
        mid_p3 = mid_p3_a.tolist()
        # 点4
        mid_p4_a = np.array(mid_loc) + direction_minus * (
            thickness + signal_edge + signal_length
        )
        mid_p4 = mid_p4_a.tolist()
        mid_text_p1 = [
            (mid_p1[0] + mid_p2[0]) / 2,
            (mid_p1[1] + mid_p2[1]) / 2,
            (mid_p1[2] + mid_p2[2]) / 2,
        ]
        mid_text_p2 = [
            (mid_p3[0] + mid_p4[0]) / 2,
            (mid_p3[1] + mid_p4[1]) / 2,
            (mid_p3[2] + mid_p4[2]) / 2,
        ]
        mid_text_content = "b"
        line_loc = [
            [top_p1, top_p2],
            [top_p3, top_p4],
            [mid_p1, mid_p2],
            [mid_p3, mid_p4],
            [bottom_p1, bottom_p2],
            [bottom_p3, bottom_p4],
        ]
        dimension_text = [
            [top_text_p1, top_text_p2, top_text_content],
            [mid_text_p1, mid_text_p2, mid_text_content],
            [bottom_text_p1, bottom_text_p2, bottom_text_content],
        ]
        dimension_result = [line_loc, dimension_text]
        return dimension_result

    def get_bottom_long_rebar_outline_shape(self):
        """
        获取底部纵筋引出线形状
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        bottom_long_loc = self.bottom_long_rebar.get_rebar_model()  # 获取钢筋位置模型
        bottom_rebar_diam = self.bottom_long_rebar.get_rebar_diameter()
        rebar_symbol = self.bottom_long_rebar.get_rebar_config_info()  # 钢筋符号
        bottom_rebar = bottom_long_loc[0]  # 获取单根钢筋
        rebar_point_1 = [
            bottom_rebar[-2].x,
            bottom_rebar[-2].y,
            bottom_rebar[-2].z,
        ]  # 点1
        rebar_point_2 = [
            bottom_rebar[-1].x,
            bottom_rebar[-1].y,
            bottom_rebar[-1].z,
        ]  # 点2
        vector_12 = np.array(np.array(rebar_point_2) - np.array(rebar_point_1))
        special_point_0 = np.array(
            np.array(2 * vector_12 / 3) + np.array(rebar_point_1)
        )
        point_1 = special_point_0.tolist()
        point_2 = [
            point_1[0],
            point_1[1] + outline_length_1 * math.cos(theta),
            point_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0], point_2[1] + outline_length_2, point_2[2]]
        text_end = "①"
        text_mid = (
            "下部纵筋"
            + str(rebar_symbol)
            + str(bottom_rebar_diam)
            + "@"
            + str(self.struct_book.spacing_1_actual)
        )
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_top_long_rebar_outline_shape(self):
        """
        获取顶部纵筋引出线形状
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        top_long_loc = self.top_long_rebar.get_rebar_model()  # 获取钢筋位置模型
        top_rebar_diam = self.top_long_rebar.get_rebar_diameter()
        rebar_symbol = self.top_long_rebar.get_rebar_config_info()  # 钢筋符号
        top_rebar = top_long_loc[0]  # 获取单根钢筋
        rebar_point_1 = top_rebar[1]  # 点1
        rebar_point_2 = top_rebar[2]  # 点2
        point_1 = [
            (rebar_point_1.x + rebar_point_2.x) / 2,
            (rebar_point_1.y + rebar_point_2.y) / 2,
            (rebar_point_1.z + rebar_point_2.z) / 2,
        ]
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta),
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "②"
        text_mid = (
            "上部纵筋"
            + str(rebar_symbol)
            + str(top_rebar_diam)
            + "@"
            + str(self.struct_book.spacing_2_actual)
        )
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_mid_distribute_rebar_outline_shape(self):
        """
        获取中部分布筋引出线形状数据点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        mid_distribute_loc = (
            self.mid_distribute_rebar.get_double_rebar_model()
        )  # 获取双层钢筋模型
        mid_rebar_diam = self.mid_distribute_rebar.get_rebar_diameter()  # 获取钢筋直径
        mid_rebar_symbol = self.mid_distribute_rebar.get_rebar_config_info()  # 获取钢筋符号
        dimension_num = int(len(mid_distribute_loc) / 2)  # 中部钢筋数量位置
        current_rebar_1 = mid_distribute_loc[dimension_num]  # 当前钢筋
        current_rebar_2 = mid_distribute_loc[dimension_num + 1]  # 下下一根钢筋
        # 第二第三点坐标
        current_point_2 = current_rebar_1[1]
        current_point_3 = current_rebar_1[2]
        next_point_2 = current_rebar_2[1]
        next_point_3 = current_rebar_2[2]
        # 标志点
        point_2 = [
            (current_point_2.x + current_point_3.x) / 2,
            (current_point_2.y + current_point_3.y) / 2,
            (current_point_2.z + current_point_3.z) / 2,
        ]
        point_1 = [
            (next_point_2.x + next_point_3.x) / 2,
            (next_point_2.y + next_point_3.y) / 2,
            (next_point_2.z + next_point_3.z) / 2,
        ]
        point_3 = [point_2[0], point_2[1], point_2[2] - outline_length_1]
        point_4 = [point_3[0], point_3[1] + outline_length_2, point_3[2]]
        text_end = "③"
        text_mid = (
            "中部分布筋"
            + str(mid_rebar_symbol)
            + str(mid_rebar_diam)
            + "@"
            + str(self.struct_book.spacing_3_actual)
        )
        dimension_result = [
            [[point_1, point_3], [point_2, point_3], [point_3, point_4]],
            [[point_3, text_mid], [point_4, text_end]],
        ]
        return dimension_result

    def get_bottom_edge_long_rebar_outline_shape(self):
        """
        获取底端边缘纵筋引出线标注点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        bottom_edge_long_loc = self.bottom_edge_long_rebar.get_rebar_model()  # 钢筋的坐标点
        rebar_diam = self.bottom_edge_long_rebar.get_rebar_diameter()  # 钢筋的直径
        rebar_symbol = self.bottom_edge_long_rebar.get_rebar_config_info()  # 获取钢筋符号
        limit_z = self.h2 / 2  # 筛选出顶部和顶部的底端边缘纵向钢筋
        point_1 = [0, self.lb_d, 0]  # 引出线首点
        for seg in bottom_edge_long_loc:
            current_rebar = seg
            current_point = current_rebar[0]  # 获取当前钢筋的首点
            if current_point.z > limit_z:  # 筛选出上部钢筋
                if current_point.y < point_1[1]:
                    point_1[0] = current_point.x
                    point_1[1] = current_point.y
                    point_1[2] = current_point.z
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta),
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "④"
        text_mid = (
            "底端边缘纵筋"
            + str(rebar_symbol)
            + str(rebar_diam)
            + "@"
            + str(self.bottom_edge_long_rebar.get_rebar_spacing())
        )
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_top_edge_long_rebar_outline_shape(self):
        """
        获取顶端边缘纵筋引出线标注点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        top_edge_long_loc = self.top_edge_long_rebar.get_rebar_model()  # 钢筋的坐标点
        rebar_diam = self.top_edge_long_rebar.get_rebar_diameter()  # 钢筋的直径
        rebar_symbol = self.top_edge_long_rebar.get_rebar_config_info()  # 获取钢筋符号
        limit_z = self.h + self.h2 - self.h1 / 2  # 筛选出顶部和顶部的顶端边缘纵向钢筋
        point_1 = [0, self.ln + self.lb_d + self.lt_d, 0]  # 引出线首点
        for seg in top_edge_long_loc:
            current_rebar = seg
            current_point = current_rebar[0]  # 获取当前钢筋的首点
            if current_point.z > limit_z:  # 筛选出上部钢筋
                if current_point.y < point_1[1]:
                    point_1[0] = current_point.x
                    point_1[1] = current_point.y
                    point_1[2] = current_point.z
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta),
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "⑤"
        text_mid = (
            "顶端边缘纵筋"
            + str(rebar_symbol)
            + str(rebar_diam)
            + "@"
            + str(self.top_edge_long_rebar.get_rebar_spacing())
        )
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_bottom_edge_stir_outline_shape(self):
        """
        获取底部边缘箍筋
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        bottom_edge_stir_loc = self.bottom_edge_stir.get_rebar_model()  # 获取底部边缘箍筋坐标点
        bottom_edge_stir_diam = self.bottom_edge_stir.get_rebar_diameter()  # 获取底部边缘箍筋直径
        rebar_symbol = self.bottom_edge_stir.get_rebar_config_info()  # 获取钢筋强度等级符号
        current_rebar = bottom_edge_stir_loc[0]  # 获取首根钢筋
        rebar_character_point = []  # 钢筋特征点
        limit_y = self.lb_d / 2
        for num in range(len(current_rebar)):
            current_point = current_rebar[num]
            if current_point.y < limit_y:
                rebar_character_point.append(current_point)
        sum_z = 0
        for num in range(len(rebar_character_point)):
            current_point = rebar_character_point[num]
            sum_z += current_point.z
        point_1 = [
            rebar_character_point[0].x,
            rebar_character_point[0].y,
            sum_z / len(rebar_character_point),
        ]
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta) / 2,
            point_1[2] + outline_length_1 * math.sin(theta) / 2,
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "⑥"
        text_mid = (
            "底端边缘箍筋"
            + str(rebar_symbol)
            + str(bottom_edge_stir_diam)
            + "@"
            + str(self.bottom_edge_stir.get_rebar_spacing())
        )
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_top_edge_stir_outline_shape(self):
        """
        获取顶端边缘箍筋
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4
        top_edge_stir_loc = self.top_edge_stir.get_rebar_model()  # 获取顶部边缘箍筋坐标点
        top_edge_stir_diam = self.top_edge_stir.get_rebar_diameter()  # 获取顶部边缘箍筋直径
        rebar_symbol = self.top_edge_stir.get_rebar_config_info()  # 获取钢筋强度等级符号
        current_rebar = top_edge_stir_loc[0]  # 获取首根钢筋
        rebar_character_point = []  # 钢筋特征点
        limit_y = self.lb_d + self.ln + self.lt_d / 2
        for num in range(len(current_rebar)):
            current_point = current_rebar[num]
            if current_point.y < limit_y:
                rebar_character_point.append(current_point)
        sum_z = 0
        for num in range(len(rebar_character_point)):
            current_point = rebar_character_point[num]
            sum_z += current_point.z
        point_1 = [
            rebar_character_point[0].x,
            rebar_character_point[0].y,
            sum_z / len(rebar_character_point),
        ]
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta) / 2,
            point_1[2] + outline_length_1 * math.sin(theta) / 2,
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "⑨"
        text_mid = (
            "顶端边缘箍筋"
            + str(rebar_symbol)
            + str(top_edge_stir_diam)
            + "@"
            + str(self.top_edge_stir.get_rebar_spacing())
        )
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_hole_rein_rebar_outline_shape(self):
        """
        获取孔洞加强筋引出线坐标
        :return:
        """
        outline_length_1 = 250
        outline_length_2 = 300
        theta = math.pi / 3  # 引出线夹角
        hole_rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        # 基础数据
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2
        limit_z = (self.h2) / 2
        limit_x = self.b0 / 2
        choosed_rebar = []  # 初始化被选定的钢筋
        for num in range(len(hole_rein_rebar_loc)):
            current_rebar = hole_rein_rebar_loc[num]
            current_point = current_rebar[0]  # 获取孔洞加强筋上的点
            if (
                current_point.x <= limit_x
                and current_point.y <= limit_y
                and current_point.z >= limit_z
            ):
                choosed_rebar.append(current_rebar)
        rebar_point_1 = choosed_rebar[0][0]  # 获取加强筋第一点
        rebar_point_2 = choosed_rebar[0][1]  # 获取加强筋第二点
        # 开始获取引出线坐标点
        point_1 = [
            (rebar_point_1.x + rebar_point_2.x) / 2,
            (rebar_point_1.y + rebar_point_2.y) / 2,
            (rebar_point_1.z + rebar_point_2.z) / 2,
        ]
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta),
            point_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "⑦"
        text_mid = "孔洞加强筋"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_hoist_rein_long_rebar_outline_shape(self):
        """
        吊装加强纵筋
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 3  # 引出线倾斜角
        hoist_rein_long_rebar_loc = (
            self.hoist_rein_long_rebar.get_rebar_model()
        )  # 获取吊点加强纵筋坐标
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2
        limit_x = self.b0 / 2
        dimension_rebar = []  # 筛选出待标注吊装预埋纵筋
        for num in range(len(hoist_rein_long_rebar_loc)):
            current_rebar = hoist_rein_long_rebar_loc[num]
            current_point = current_rebar[0]  # 获取吊装加强筋上任意点
            if current_point.y > limit_y and current_point.x < limit_x:
                dimension_rebar.append(current_rebar)
        rebar_point_2 = dimension_rebar[0][1]
        rebar_point_3 = dimension_rebar[0][2]
        point_1 = [
            (rebar_point_2.x + rebar_point_3.x) / 2,
            (rebar_point_2.y + rebar_point_3.y) / 2,
            (rebar_point_2.z + rebar_point_3.z) / 2,
        ]
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta),
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "⑧a"
        text_mid = "吊点加强纵筋"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result

    def get_hoist_rein_point_rebar_outline_shape(self):
        """
        获取吊点加强点筋
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 300
        theta = math.pi / 4  # 引出线倾斜角
        hoist_rein_point_rebar_loc = (
            self.hoist_rein_point_rebar.get_rebar_model()
        )  # 吊点加强点筋坐标
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2
        dimension_rebar = []  # 待标注的钢筋
        for num in range(len(hoist_rein_point_rebar_loc)):
            current_rebar = hoist_rein_point_rebar_loc[num]
            current_point = current_rebar[0]
            if current_point.y <= limit_y:
                dimension_rebar.append(current_rebar)
        rebar_point_1 = dimension_rebar[0][0]
        rebar_point_2 = dimension_rebar[0][1]
        point_1 = [
            (rebar_point_1.x + rebar_point_2.x) / 2,
            (rebar_point_1.y + rebar_point_2.y) / 2,
            (rebar_point_1.z + rebar_point_2.z) / 2,
        ]
        point_2 = [
            point_1[0],
            point_1[1] - outline_length_1 * math.cos(theta) / 2,
            point_1[2] + outline_length_1 * math.sin(theta) / 2,
        ]
        point_3 = [point_2[0], point_2[1] - outline_length_2, point_2[2]]
        text_end = "⑧b"
        text_mid = "吊点加强点筋"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_2, text_mid], [point_3, text_end]],
        ]
        return dimension_result


class StairRebarSectionAToAViewDimensionData(object):
    """
    楼梯A-A剖面图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_basic_class()
        self.generate_rebar_config()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.t = self.slab_struct.geometric.thickness
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

    def generate_rebar_config(self):
        """
        产生钢筋配置信息
        :return:
        """
        self.top_edge_long_rebar_info = TopEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 上部边缘纵筋
        self.top_edge_stir_info = TopEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_edge_rein_rebar_info = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.bottom_edge_rein_rebar_info = BottomEdgeReinforceRebar(
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

    def get_stair_solid_profile_dimension_point(self):
        """
        获取楼梯轮廓标注点
        :return:
        """
        # 第二层标注点
        loc_y = 0
        total_H = self.h2 + self.h
        total_h1 = total_H - self.h1
        total_W = self.b0 + self.b1
        # 顶部左侧标注点
        top_left_p1 = [0, loc_y, total_H]
        top_left_p2 = [0, loc_y, total_h1]
        # 顶部上侧标注
        top_top_p1 = [total_W, loc_y, total_H]
        top_top_p2 = [0, loc_y, total_H]
        dimension_points = [[top_left_p1, top_left_p2], [top_top_p1, top_top_p2]]
        return dimension_points

    def get_stair_top_edge_long_rebar_dimension_point(self):
        """
        获取顶部顶部边缘纵筋标注点
        :return:
        """
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar_info.get_rebar_model()
        )  # 获取钢筋数据
        total_H = self.h2 + self.h
        total_h1 = total_H - self.h1
        loc_y = 0
        max_z = 0  # 最大z坐标值
        min_z = self.h2 + self.h  # 最小z坐标值
        # 筛选出最大和最小的z坐标值
        for rebar in top_edge_long_rebar_loc:
            current_point = rebar[0]
            if max_z < current_point.z:
                max_z = current_point.z
            if min_z > current_point.z:
                min_z = current_point.z
        left_p1 = [0, loc_y, total_H]
        left_p2 = [0, loc_y, max_z]
        left_p3 = [0, loc_y, min_z]
        left_p4 = [0, loc_y, total_h1]
        dimension_points = [[left_p1, left_p2], [left_p2, left_p3], [left_p3, left_p4]]
        return dimension_points

    def get_stair_top_edge_stir_dimension_point(self):
        """
        获取顶部边缘箍筋标注点
        :return:
        """
        top_edge_stir_loc = self.top_edge_stir_info.get_rebar_model()  # 获取顶部边缘箍筋坐标点
        total_w = self.b0 + self.b1
        loc_y = 0
        loc_z = self.h2 + self.h
        range_x = [0]
        for rebar in top_edge_stir_loc:
            current_point = rebar[0]  # 获取当前点
            range_x.append(current_point.x)
        range_x.append(total_w)
        range_x.sort(reverse=True)  # 将列表中的元素由大到小排列
        dimension_points = []
        for num in range(len(range_x) - 1):
            current_x = range_x[num]
            next_x = range_x[num + 1]
            point_1 = [current_x, loc_y, loc_z]
            point_2 = [next_x, loc_y, loc_z]
            dimension_points.append([point_1, point_2])
        return dimension_points

    def get_stair_top_edge_stir_outline_shape(self):
        """
        获取楼梯顶端边缘箍筋引出线形状
        :return:[直线段点，标注说明文档]
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 3
        top_edge_stir_loc = self.top_edge_stir_info.get_rebar_model()  # 获取顶端边缘箍筋坐标点
        current_num = int(len(top_edge_stir_loc) / 2)  # 获取顶端边缘箍筋位置
        current_rebar = top_edge_stir_loc[current_num]  # 当前钢筋
        loc_x = current_rebar[0].x
        loc_y = 0
        loc_z = 0
        for num in range(len(current_rebar)):
            current_point = current_rebar[num]  # 当前点
            loc_z += current_point.z
        loc_z = loc_z / len(current_rebar)
        point_1 = [loc_x, loc_y, loc_z]  # 当前点
        point_2 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0] + outline_length_2, point_2[1], point_2[2]]
        text_end = "⑨"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_result

    def get_stair_top_hole_rein_rebar_outline_shape(self):
        """
        获取楼梯顶部孔洞加强筋引出线形状
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 3
        rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        rebar_diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2  #
        limit_z = self.h2 + self.h - self.h1 / 2
        outline_point = [0, 0, 0]
        for rebar in rein_rebar_loc:
            current_point = rebar[2]
            if current_point.x < limit_x and current_point.z > limit_z:
                outline_point[0] = current_point.x
                outline_point[2] = current_point.z
        rebar_2 = rein_rebar_loc[0][1]
        rebar_4 = rein_rebar_loc[0][3]
        length = abs(rebar_4.x - rebar_2.x) + rebar_diam
        point_1 = [
            outline_point[0] + length / 2,
            outline_point[1],
            outline_point[2] - rebar_diam / 2,
        ]
        point_2 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0] + outline_length_2, point_2[1], point_2[2]]
        text_end = "⑦"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_result

    def get_stair_top_edge_long_rebar_outline_shape(self):
        """
        获取楼梯顶部边缘纵筋引出线数据
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar_info.get_rebar_model()
        )  # 获取顶部边缘纵筋坐标点
        max_z = 0  # 最大z坐标值
        min_z = self.h2 + self.h  # 最小z坐标值
        point_s = top_edge_long_rebar_loc[0][0]  # 起点
        point_e = top_edge_long_rebar_loc[0][1]  # 终点
        loc_x = (point_s.x + point_e.x) / 2
        loc_y = 0
        for num in range(len(top_edge_long_rebar_loc)):
            current_rebar = top_edge_long_rebar_loc[num]  # 当前钢筋
            current_point_s = current_rebar[0]  # 当前点
            if current_point_s.z > max_z:
                max_z = current_point_s.z
            if current_point_s.z < min_z:
                min_z = current_point_s.z
        point_1 = [loc_x, loc_y, max_z]
        point_2 = [loc_x, loc_y, min_z]
        point_3 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_4 = [point_3[0] + outline_length_2, point_3[1], point_3[2]]
        text_end = "⑤"
        dimension_result = [
            [[point_1, point_3], [point_2, point_3], [point_3, point_4]],
            [[point_4, text_end]],
        ]
        return dimension_result

    def get_stair_top_edge_rein_rebar_outline_shape(self):
        """
        顶端边缘加强筋引出线
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        top_edge_rein_rebar_loc = (
            self.top_edge_rein_rebar_info.get_rebar_model()
        )  # 获取顶部边缘加强筋的坐标点
        top_edge_rein_rebar_diam = (
            self.top_edge_rein_rebar_info.get_rebar_diameter()
        )  # 获取顶部边缘加强筋
        top_edge_stir_loc = self.top_edge_stir_info.get_rebar_model()  # 获取上部边缘箍筋的坐标点
        top_edge_stir_diam = self.top_edge_stir_info.get_rebar_diameter()  # 获取上部边缘箍筋的直径
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar_info.get_rebar_model()
        )  # 获取上部边缘纵筋的坐标点
        top_edge_long_rebar_diam = (
            self.top_edge_long_rebar_info.get_rebar_diameter()
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
        loc_x_l = (
            min_x  # + (top_edge_stir_diam + top_edge_rein_rebar_diam) / 2  # 左侧点位置
        )
        loc_x_r = (
            max_x  # - (top_edge_stir_diam + top_edge_rein_rebar_diam) / 2  # 右侧点的位置
        )
        # 标注左侧点
        point_l_1 = [loc_x_l, loc_y, loc_z]  # 左侧标志点
        point_l_2 = [
            point_l_1[0] - outline_length_1 * math.cos(theta),
            point_l_1[1],
            point_l_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_l_3 = [point_l_2[0] - outline_length_2, point_l_2[1], point_l_2[2]]
        text = "⑩-a"
        # 标注右侧点
        point_r_1 = [loc_x_r, loc_y, loc_z]  # 右侧标志点
        point_r_2 = [
            point_r_1[0] + outline_length_1 * math.cos(theta),
            point_r_1[1],
            point_r_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_r_3 = [point_r_2[0] + outline_length_2, point_r_2[1], point_r_2[2]]
        dimension_result = [
            [
                [point_l_1, point_l_2],
                [point_l_2, point_l_3],
                [point_r_1, point_r_2],
                [point_r_2, point_r_3],
            ],
            [[point_l_3, text], [point_r_3, text]],
        ]
        return dimension_result

    def get_stair_bottom_edge_rein_rebar_outline_shape(self):
        """
        获取楼梯底部边缘加强筋引出线坐标点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        bottom_edge_rein_rebar_loc = (
            self.bottom_edge_rein_rebar_info.get_rebar_model()
        )  # 获取底部边缘加强筋的坐标点
        bottom_edge_rein_rebar_diam = (
            self.bottom_edge_rein_rebar_info.get_rebar_diameter()
        )  # 获取底部边缘加强筋
        top_edge_stir_loc = self.top_edge_stir_info.get_rebar_model()  # 获取上部边缘箍筋的坐标点
        top_edge_stir_diam = self.top_edge_stir_info.get_rebar_diameter()  # 获取上部边缘箍筋的直径
        top_edge_long_rebar_loc = (
            self.top_edge_long_rebar_info.get_rebar_model()
        )  # 获取上部边缘纵筋的坐标点
        top_edge_long_rebar_diam = (
            self.top_edge_long_rebar_info.get_rebar_diameter()
        )  # 获取上部边缘纵筋的直径
        loc_y = 0
        min_z = self.h2 + self.h
        min_x = self.b0
        max_x = 0
        # 遍历顶部边缘纵筋，获取最大的z坐标值
        for num in range(len(top_edge_long_rebar_loc)):
            current_rebar = top_edge_long_rebar_loc[num]
            current_point = current_rebar[0]  # 获取首点
            if current_point.z < min_z:
                min_z = current_point.z
        # 遍历顶部边缘箍筋，获取最大和最小的x坐标值
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
        # 标注左侧点
        point_l_1 = [loc_x_l, loc_y, loc_z]  # 左侧标志点
        point_l_2 = [
            point_l_1[0] - outline_length_1 * math.cos(theta),
            point_l_1[1],
            point_l_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_l_3 = [point_l_2[0] - outline_length_2, point_l_2[1], point_l_2[2]]
        text = "⑩-b"
        # 标注右侧点
        point_r_1 = [loc_x_r, loc_y, loc_z]  # 右侧标志点
        point_r_2 = [
            point_r_1[0] + outline_length_1 * math.cos(theta),
            point_r_1[1],
            point_r_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_r_3 = [point_r_2[0] + outline_length_2, point_r_2[1], point_r_2[2]]
        dimension_result = [
            [
                [point_l_1, point_l_2],
                [point_l_2, point_l_3],
                [point_r_1, point_r_2],
                [point_r_2, point_r_3],
            ],
            [[point_l_3, text], [point_r_3, text]],
        ]
        return dimension_result


class StairRebarSectionBToBViewDimensionData(object):
    """
    楼梯B-B剖面图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_basic_class()
        self.generate_rebar_config()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.t = self.slab_struct.geometric.thickness
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

    def generate_rebar_config(self):
        """
        产生钢筋配置信息
        :return:
        """
        self.bottom_long_rebar_info = BottomLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_long_rebar_info = TopLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.mid_distribute_rebar_info = MidDistributionRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_edge_rein_rebar_info = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.bottom_edge_rein_rebar_info = BottomEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )

    def get_stair_solid_middle_cut_plane_point(self):
        """
        获取楼梯实体中部剖切面上点
        :return:
        """
        # 获取基础数据
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        loc_x = 0
        loc_y = self.lb_d + current_num * tabu_b
        loc_z = self.h2 + current_num * tabu_h
        current_point = [loc_x, loc_y, loc_z]
        return current_point

    def get_stair_solid_middle_section_vector(self):
        """
        获取楼梯实体中部
        :return:
        """
        # 获取基础数据
        tabu_b = self.tabu_b  # 踏步宽度
        tabu_h = self.tabu_h  # 踏步高度
        current_num = int(self.n / 2)
        vector_1 = np.array([-1, 0, 0])
        vector_2 = np.array([0, tabu_b, tabu_h]) / np.linalg.norm([0, tabu_b, tabu_h])
        normal = vector_2  # 斜向平面法线方向
        direction = np.cross(vector_1, vector_2)
        return normal, direction

    def get_stair_solid_profile_first_floor_dimension_point(self):
        """
        开始进行楼梯轮廓第一层尺寸标注点：顶部纵筋和底部纵筋间距和边距
        :return:
        """
        # 获取基础数据

        current_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()
        protect_c = self.cover  # 保护层厚度
        length_1 = self.t - 2 * protect_c
        # 左侧第一层标注点
        left_point_1 = copy.deepcopy(current_point)
        left_point_2_m = np.array(left_point_1) + direction * protect_c
        left_point_2 = left_point_2_m.tolist()
        left_point_3_m = np.array(left_point_2) + direction * length_1
        left_point_3 = left_point_3_m.tolist()
        left_point_4_m = np.array(left_point_3) + direction * protect_c
        left_point_4 = left_point_4_m.tolist()
        dimension_points = [
            [left_point_1, left_point_2],
            [left_point_2, left_point_3],
            [left_point_3, left_point_4],
        ]
        return dimension_points

    def get_stair_solid_profile_second_floor_dimension_point(self):
        """
        获取楼梯实体轮廓第二层尺寸标注点
        :return:
        """
        # 获取基础数据
        current_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()
        # 左侧第二层标注点
        left_point_1 = copy.deepcopy(current_point)
        left_point_2_m = np.array(left_point_1) + direction * self.t
        left_point_2 = left_point_2_m.tolist()
        # 顶部第二层标注点
        top_point_2 = copy.deepcopy(left_point_1)
        top_point_1 = [top_point_2[0] + self.b0, top_point_2[1], top_point_2[2]]
        dimension_points = [[left_point_1, left_point_2], [top_point_1, top_point_2]]
        return dimension_points

    def get_bottom_long_rebar_dimension_point(self):
        """
        获得底部纵筋尺寸标注点
        :return:
        """
        # 获取基础数据
        current_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()
        bottom_point_m = np.array(current_point) + direction * self.t
        bottom_point = bottom_point_m.tolist()
        range_x = [0, self.b0]
        bottom_long_rebar_loc = (
            self.bottom_long_rebar_info.get_rebar_model()
        )  # 获取底部纵筋坐标点
        # 遍历并获取x坐标
        for rebar in bottom_long_rebar_loc:
            current_point = rebar[0]  # 获取点
            range_x.append(current_point.x)
        range_x.sort()
        dimension_loc = []
        for num in range(len(range_x) - 1):
            loc_x_1 = range_x[num]
            loc_x_2 = range_x[num + 1]
            point_1 = [loc_x_1, bottom_point[1], bottom_point[2]]
            point_2 = [loc_x_2, bottom_point[1], bottom_point[2]]
            dimension_loc.append([point_1, point_2])
        return dimension_loc

    def get_top_long_rebar_dimension_point(self):
        """
        获得顶部纵筋尺寸标注点
        :return:
        """
        # 获取基础数据
        top_point = self.get_stair_solid_middle_cut_plane_point()
        range_x = [0, self.b0]
        top_long_rebar_loc = self.top_long_rebar_info.get_rebar_model()  # 获取顶部纵筋坐标点
        # 遍历并获取x坐标
        for rebar in top_long_rebar_loc:
            current_point = rebar[0]  # 获取点
            range_x.append(current_point.x)
        range_x.sort(reverse=True)
        dimension_loc = []
        for num in range(len(range_x) - 1):
            loc_x_1 = range_x[num]
            loc_x_2 = range_x[num + 1]
            point_1 = [loc_x_1, top_point[1], top_point[2]]
            point_2 = [loc_x_2, top_point[1], top_point[2]]
            dimension_loc.append([point_1, point_2])
        return dimension_loc

    def get_stair_bottom_long_rebar_outline_shape(self):
        """
        获取楼梯底部纵筋引出线坐标点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        current_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()
        protect_c = self.cover
        # 该点与下部纵筋的y,z值相同
        bottom_long_rebar_loc = (
            self.bottom_long_rebar_info.get_rebar_model()
        )  # 获取底部纵向钢筋位置
        bottom_long_rebar_diam = self.bottom_long_rebar_info.get_rebar_diameter()
        next_point_m = np.array(current_point) + direction * (
            self.t - protect_c - bottom_long_rebar_diam / 2
        )
        next_point = next_point_m.tolist()
        current_num = int(len(bottom_long_rebar_loc) / 3)
        current_rebar = bottom_long_rebar_loc[current_num]  # 当前钢筋
        loc_x = current_rebar[0].x
        # 开始计算关键点
        point_1 = [loc_x, next_point[1], next_point[2]]
        current_dir_1 = rotation_3d(
            np.asarray([1, 0, 0]), np.asarray(normal.tolist()), theta
        )
        point_2_m = np.array(point_1) + current_dir_1 * outline_length_1
        point_2 = point_2_m.tolist()
        current_dir_2 = rotation_3d(
            np.asarray([1, 0, 0]), np.asarray(normal.tolist()), 0
        )
        point_3_m = np.array(point_2) + current_dir_2 * outline_length_2
        point_3 = point_3_m.tolist()
        text_end = "①"
        dimension_loc = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_loc

    def get_stair_top_long_rebar_outline_shape(self):
        """
        获取楼梯顶部纵筋引出线坐标点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        current_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()
        protect_c = self.cover
        # 该点与下部纵筋的y,z值相同
        top_long_rebar_loc = self.top_long_rebar_info.get_rebar_model()  # 获取顶部纵向钢筋位置
        top_long_rebar_diam = (
            self.top_long_rebar_info.get_rebar_diameter()
        )  # 获取顶部纵向钢筋的直径
        next_point_m = np.array(current_point) + direction * (
            protect_c + top_long_rebar_diam / 2
        )
        next_point = next_point_m.tolist()

        current_num = int(len(top_long_rebar_loc) / 3)
        current_rebar = top_long_rebar_loc[current_num]  # 当前钢筋
        loc_x = current_rebar[0].x
        # 开始计算关键点
        point_1 = [loc_x, next_point[1], next_point[2]]
        current_dir_1 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), theta)
        point_2_m = np.array(point_1) + current_dir_1 * outline_length_1
        point_2 = point_2_m.tolist()
        current_dir_2 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), 0)
        point_3_m = np.array(point_2) + current_dir_2 * outline_length_2
        point_3 = point_3_m.tolist()
        text_end = "②"
        dimension_loc = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_loc

    def get_stair_solid_mid_distribute_rebar_feature_point(self):
        """
        获取楼梯实体中部分布筋特征点：将全局坐标系转化为局部坐标系下的基点，用于与B-B配筋剖切图相匹配
        :return:
        """
        mid_distribute_rebar_loc = (
            self.mid_distribute_rebar_info.get_double_rebar_model()
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
        mid_distribute_rebar_loc = (
            self.mid_distribute_rebar_info.get_double_rebar_model()
        )  # 获取中部分布筋坐标点
        current_num = int(len(mid_distribute_rebar_loc) / 2)  # 中部分布筋中间位置
        current_rebar = mid_distribute_rebar_loc[current_num]  # 当前钢筋
        next_rebar = mid_distribute_rebar_loc[current_num + 1]  # 下一根钢筋
        current_p1 = current_rebar[1]
        current_p2 = current_rebar[2]
        next_p1 = next_rebar[1]
        next_p2 = next_rebar[2]
        current_point = [
            (current_p1.x + current_p2.x) / 2,
            (current_p1.y + current_p2.y) / 2,
            (current_p1.z + current_p2.z) / 2,
        ]
        next_point = [
            (next_p1.x + next_p2.x) / 2,
            (next_p1.y + next_p2.y) / 2,
            (next_p1.z + next_p2.z) / 2,
        ]
        base_ = self.get_stair_solid_middle_cut_plane_point()  # 变换后的基点
        normal, direction = self.get_stair_solid_middle_section_vector()
        base_m = np.array(base_) + direction * self.t
        base_point = base_m.tolist()
        special_points = (
            self.get_stair_solid_mid_distribute_rebar_feature_point()
        )  # 变换前的基点
        special_1 = special_points[0]  # 变换前基点1
        special_2 = special_points[1]  # 变换前基点2
        # 转换当前钢筋
        transform_point_1 = (
            np.array(current_point) - np.array(special_1) + np.array(base_point)
        )
        # 转换下一钢筋
        transform_point_2 = (
            np.array(next_point) - np.array(special_2) + np.array(base_point)
        )
        dimension_result = [transform_point_1, transform_point_2]
        return dimension_result

    def get_stair_solid_mid_distribute_rebar_outline_shape(self):
        """
        获取楼梯中部分布筋引出线坐标点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 6
        mid_rebar_loc = self.get_stair_solid_mid_distribute_rebar_match_point()
        normal, direction = self.get_stair_solid_middle_section_vector()  # 法向量与方向向量
        dir_reverse = direction * (-1)
        point_1 = mid_rebar_loc[0]  # 获取当前钢筋的坐标点
        point_2 = mid_rebar_loc[1]  # 获取下根钢筋的坐标点
        current_dir_1 = rotation_3d(
            np.asarray(dir_reverse.tolist()), np.asarray(normal), -theta
        )
        point_3_m = np.array(point_1) + current_dir_1 * outline_length_1
        point_3 = point_3_m.tolist()
        current_dir_2 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), 0)
        point_4_m = np.array(point_3) + current_dir_2 * outline_length_2
        point_4 = point_4_m.tolist()
        text_end = "③"
        dimension_point = [
            [[point_1, point_3], [point_2, point_3], [point_3, point_4]],
            [[point_4, text_end]],
        ]
        return dimension_point

    def get_stair_solid_top_edge_rein_rebar_outline_shape(self):
        """
        获取顶部边缘加强筋引出线形状
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta_1 = math.pi / 3
        theta_2 = math.pi / 6
        top_edge_rein_rebar_loc = self.top_edge_rein_rebar_info.get_rebar_model()
        top_edge_rein_rebar_diam = self.top_edge_rein_rebar_info.get_rebar_diameter()
        range_x = []
        # 获取顶部边缘加强筋的x坐标
        for rebar in top_edge_rein_rebar_loc:
            current_point = rebar[0]
            range_x.append(current_point.x)
        range_x.sort()  # 从小到大
        key_point = top_edge_rein_rebar_loc[0][1]
        point_ = [0, key_point.y, key_point.z]
        special_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()  # 法向量与方向向量
        vector_1 = np.array(special_point) - np.array(point_)
        length = abs(np.dot(vector_1, direction))
        point_s_m = np.array(special_point) + direction * length
        point_s = point_s_m.tolist()
        # 标志点
        point_l_1 = [point_s[0] + range_x[0], point_s[1], point_s[2]]
        point_r_1 = [point_s[0] + range_x[1], point_s[1], point_s[2]]
        dir_l_1 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), theta_1)
        dir_l_2 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), 0)
        dir_r_1 = rotation_3d(np.asarray([1, 0, 0]), np.asarray(normal), -theta_2)
        dir_r_2 = rotation_3d(np.asarray([1, 0, 0]), np.asarray(normal), 0)
        #
        point_l_2_m = np.array(point_l_1) + dir_l_1 * outline_length_1
        point_l_2 = point_l_2_m.tolist()
        point_l_3_m = np.array(point_l_2) + dir_l_2 * outline_length_2
        point_l_3 = point_l_3_m.tolist()
        point_r_2_m = np.array(point_r_1) + dir_r_1 * outline_length_1
        point_r_2 = point_r_2_m.tolist()
        point_r_3_m = np.array(point_r_2) + dir_r_2 * outline_length_2
        point_r_3 = point_r_3_m.tolist()
        text_end = "⑩-a"
        dimension_point = [
            [
                [point_l_1, point_l_2],
                [point_l_2, point_l_3],
                [point_r_1, point_r_2],
                [point_r_2, point_r_3],
            ],
            [[point_l_3, text_end], [point_r_3, text_end]],
        ]
        return dimension_point

    def get_stair_solid_bottom_edge_rein_rebar_outline_shape(self):
        """
        获取楼梯底部边缘加强筋引出线
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta_1 = math.pi / 3
        theta_2 = math.pi / 6
        bottom_edge_rein_rebar_loc = self.bottom_edge_rein_rebar_info.get_rebar_model()
        bottom_edge_rein_rebar_diam = (
            self.bottom_edge_rein_rebar_info.get_rebar_diameter()
        )
        range_x = []
        # 获取底部边缘加强筋的x坐标
        for rebar in bottom_edge_rein_rebar_loc:
            current_point = rebar[0]
            range_x.append(current_point.x)
        range_x.sort()  # 从小到大
        key_point = bottom_edge_rein_rebar_loc[0][1]
        point_ = [0, key_point.y, key_point.z]
        special_point = self.get_stair_solid_middle_cut_plane_point()
        normal, direction = self.get_stair_solid_middle_section_vector()  # 法向量与方向向量
        vector_1 = np.array(special_point) - np.array(point_)
        length = abs(np.dot(vector_1, direction))
        point_s_m = np.array(special_point) + direction * length
        point_s = point_s_m.tolist()
        # 标志点
        point_l_1 = [point_s[0] + range_x[0], point_s[1], point_s[2]]
        point_r_1 = [point_s[0] + range_x[1], point_s[1], point_s[2]]
        dir_l_1 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), -theta_1)
        dir_l_2 = rotation_3d(np.asarray([-1, 0, 0]), np.asarray(normal), 0)
        dir_r_1 = rotation_3d(np.asarray([1, 0, 0]), np.asarray(normal), theta_2)
        dir_r_2 = rotation_3d(np.asarray([1, 0, 0]), np.asarray(normal), 0)
        #
        point_l_2_m = np.array(point_l_1) + dir_l_1 * outline_length_1
        point_l_2 = point_l_2_m.tolist()
        point_l_3_m = np.array(point_l_2) + dir_l_2 * outline_length_2
        point_l_3 = point_l_3_m.tolist()
        point_r_2_m = np.array(point_r_1) + dir_r_1 * outline_length_1
        point_r_2 = point_r_2_m.tolist()
        point_r_3_m = np.array(point_r_2) + dir_r_2 * outline_length_2
        point_r_3 = point_r_3_m.tolist()
        text_end = "⑩-b"
        dimension_point = [
            [
                [point_l_1, point_l_2],
                [point_l_2, point_l_3],
                [point_r_1, point_r_2],
                [point_r_2, point_r_3],
            ],
            [[point_l_3, text_end], [point_r_3, text_end]],
        ]
        return dimension_point


class StairRebarSectionCToCViewDimensionData(object):
    """
    楼梯A-A剖面图标注数据
    """

    def __init__(
        self, slab_struct, detail_slab, struct_book, detail_book, rebar_for_bim
    ):
        self.slab_struct = slab_struct
        self.detail_slab = detail_slab
        self.struct_book = struct_book
        self.detail_book = detail_book
        self.rebar_for_bim = rebar_for_bim
        self.generate_basic_class()
        self.generate_rebar_config()

    def generate_basic_class(self):
        """
        产生基本类模板基础数据
        :return:
        """
        self.n = self.slab_struct.geometric.steps_number
        self.tabu_h = self.struct_book.steps_h
        self.tabu_b = self.struct_book.steps_b
        self.t = self.slab_struct.geometric.thickness
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

    def generate_rebar_config(self):
        """
        产生钢筋配置信息
        :return:
        """
        self.bottom_edge_long_rebar_info = BottomEdgeLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )  # 下部边缘纵筋
        self.bottom_edge_stir_info = BottomEdgeStirrup(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.top_edge_rein_rebar_info = TopEdgeReinforceRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )
        self.bottom_edge_rein_rebar_info = BottomEdgeReinforceRebar(
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
        self.bottom_long_rebar_info = BottomLongitudinalRebar(
            self.slab_struct,
            self.detail_slab,
            self.struct_book,
            self.detail_book,
            self.rebar_for_bim,
        )

    def get_stair_solid_profile_dimension_point(self):
        """
        获取楼梯轮廓标注点
        :return:
        """
        # 第二层标注点
        loc_y = 0
        total_H = self.h2
        total_h1 = 0
        total_W = self.b0 + self.b2
        # 底部左侧标注点
        bottom_left_p1 = [0, loc_y, total_H]
        bottom_left_p2 = [0, loc_y, total_h1]
        # 底部上侧标注
        bottom_top_p1 = [total_W, loc_y, total_H]
        bottom_top_p2 = [0, loc_y, total_H]
        dimension_points = [
            [bottom_left_p1, bottom_left_p2],
            [bottom_top_p1, bottom_top_p2],
        ]
        return dimension_points

    def get_stair_bottom_edge_long_rebar_dimension_point(self):
        """
        获取底部边缘纵筋标注点
        :return:
        """
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar_info.get_rebar_model()
        )  # 获取钢筋数据
        total_H = self.h2
        total_h1 = 0
        loc_y = 0
        max_z = 0  # 最大z坐标值
        min_z = self.h2  # 最小z坐标值
        # 筛选出最大和最小的z坐标值
        for rebar in bottom_edge_long_rebar_loc:
            current_point = rebar[0]
            if max_z < current_point.z:
                max_z = current_point.z
            if min_z > current_point.z:
                min_z = current_point.z
        left_p1 = [0, loc_y, total_H]
        left_p2 = [0, loc_y, max_z]
        left_p3 = [0, loc_y, min_z]
        left_p4 = [0, loc_y, total_h1]
        dimension_points = [[left_p1, left_p2], [left_p2, left_p3], [left_p3, left_p4]]
        return dimension_points

    def get_stair_bottom_edge_stir_dimension_point(self):
        """
        获取底部边缘箍筋标注点
        :return:
        """
        bottom_edge_stir_loc = (
            self.bottom_edge_stir_info.get_rebar_model()
        )  # 获取底部边缘箍筋坐标点
        total_w = self.b0 + self.b2
        loc_y = 0
        loc_z = self.h2
        range_x = [0]
        for rebar in bottom_edge_stir_loc:
            current_point = rebar[0]  # 获取当前点
            range_x.append(current_point.x)
        range_x.append(total_w)
        range_x.sort(reverse=True)  # 将列表中的元素由大到小排列
        dimension_points = []
        for num in range(len(range_x) - 1):
            current_x = range_x[num]
            next_x = range_x[num + 1]
            point_1 = [current_x, loc_y, loc_z]
            point_2 = [next_x, loc_y, loc_z]
            dimension_points.append([point_1, point_2])
        return dimension_points

    def get_bottom_long_rebar_outline_shape(self):
        """
        获取下部纵筋的引出线形状
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        rebar_loc = self.bottom_long_rebar_info.get_rebar_model()  # 获取下部纵筋的坐标点
        current_num = int(3 * len(rebar_loc) / 5)
        current_rebar = rebar_loc[current_num]
        limit_y = (self.lb_d + self.ln + self.lt_d) / 2
        current_points = []
        for num in range(len(current_rebar)):
            current_point = current_rebar[num]
            if current_point.y < limit_y:
                current_points.append(current_point)
        loc_x = 0
        loc_y = 0
        loc_z = 0
        for num in range(len(current_points)):
            point_ = current_points[num]
            loc_x += point_.x
            loc_z += point_.z
        loc_x = loc_x / len(current_points)
        loc_z = loc_z / len(current_points)
        point_1 = [loc_x, loc_y, loc_z]  # 标志首点
        point_2 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0] + outline_length_2, point_2[1], point_2[2]]
        text_end = "①"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_result

    def get_stair_bottom_edge_stir_outline_shape(self):
        """
        获取楼梯底端边缘箍筋引出线形状
        :return:[直线段点，标注说明文档]
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 3
        bottom_edge_stir_loc = (
            self.bottom_edge_stir_info.get_rebar_model()
        )  # 获取底端边缘箍筋坐标点
        current_num = int(len(bottom_edge_stir_loc) / 2)  # 获取顶端边缘箍筋位置
        current_rebar = bottom_edge_stir_loc[current_num]  # 当前钢筋
        loc_x = current_rebar[0].x
        loc_y = 0
        loc_z = 0
        for num in range(len(current_rebar)):
            current_point = current_rebar[num]  # 当前点
            loc_z += current_point.z
        loc_z = loc_z / len(current_rebar)
        point_1 = [loc_x, loc_y, loc_z]  # 当前点
        point_2 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0] + outline_length_2, point_2[1], point_2[2]]
        text_end = "⑥"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_result

    def get_stair_bottom_hole_rein_rebar_outline_shape(self):
        """
        获取楼梯底部孔洞加强筋引出线形状
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 3
        rein_rebar_loc = self.hole_rein_rebar.get_rebar_model()  # 获取孔洞加强筋数据
        rebar_diam = self.hole_rein_rebar.get_rebar_diameter()  # 获取孔洞加强钢筋的直径
        # 准备基础数据
        limit_x = self.b0 / 2  #
        max_z = self.h2
        min_z = self.h2 / 2
        outline_point = [0, 0, 0]
        for rebar in rein_rebar_loc:
            current_point = rebar[2]
            if (
                current_point.x < limit_x
                and current_point.z > min_z
                and current_point.z < max_z
            ):
                outline_point[0] = current_point.x
                outline_point[2] = current_point.z
        rebar_2 = rein_rebar_loc[0][1]
        rebar_4 = rein_rebar_loc[0][3]
        length = abs(rebar_4.x - rebar_2.x) + rebar_diam
        point_1 = [
            outline_point[0] + length / 2,
            outline_point[1],
            outline_point[2] - rebar_diam / 2,
        ]
        point_2 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_3 = [point_2[0] + outline_length_2, point_2[1], point_2[2]]
        text_end = "⑦"
        dimension_result = [
            [[point_1, point_2], [point_2, point_3]],
            [[point_3, text_end]],
        ]
        return dimension_result

    def get_stair_bottom_edge_long_rebar_outline_shape(self):
        """
        获取楼梯底部边缘纵筋引出线数据
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 3
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar_info.get_rebar_model()
        )  # 获取底部边缘纵筋坐标点
        max_z = 0  # 最大z坐标值
        min_z = self.h2  # 最小z坐标值
        point_s = bottom_edge_long_rebar_loc[0][0]  # 起点
        point_e = bottom_edge_long_rebar_loc[0][1]  # 终点
        loc_x = (point_s.x + point_e.x) / 2
        loc_y = 0
        for num in range(len(bottom_edge_long_rebar_loc)):
            current_rebar = bottom_edge_long_rebar_loc[num]  # 当前钢筋
            current_point_s = current_rebar[0]  # 当前点
            if current_point_s.z > max_z:
                max_z = current_point_s.z
            if current_point_s.z < min_z:
                min_z = current_point_s.z
        point_1 = [loc_x, loc_y, max_z]
        point_2 = [loc_x, loc_y, min_z]
        point_3 = [
            point_1[0] + outline_length_1 * math.cos(theta),
            point_1[1],
            point_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_4 = [point_3[0] + outline_length_2, point_3[1], point_3[2]]
        text_end = "④"
        dimension_result = [
            [[point_1, point_3], [point_2, point_3], [point_3, point_4]],
            [[point_4, text_end]],
        ]
        return dimension_result

    def get_stair_top_edge_rein_rebar_outline_shape(self):
        """
        顶端边缘加强筋引出线
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        top_edge_rein_rebar_loc = (
            self.top_edge_rein_rebar_info.get_rebar_model()
        )  # 获取顶部边缘加强筋的坐标点
        top_edge_rein_rebar_diam = (
            self.top_edge_rein_rebar_info.get_rebar_diameter()
        )  # 获取顶部边缘加强筋
        bottom_edge_stir_loc = (
            self.bottom_edge_stir_info.get_rebar_model()
        )  # 获取下部边缘箍筋的坐标点
        bottom_edge_stir_diam = (
            self.bottom_edge_stir_info.get_rebar_diameter()
        )  # 获取下部边缘箍筋的直径
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar_info.get_rebar_model()
        )  # 获取下部边缘纵筋的坐标点
        bottom_edge_long_rebar_diam = (
            self.bottom_edge_long_rebar_info.get_rebar_diameter()
        )  # 获取下部边缘纵筋的直径
        loc_y = 0
        max_z = 0
        min_x = self.b0
        max_x = 0
        # 遍历顶部边缘纵筋，获取最大的z坐标值
        for num in range(len(bottom_edge_long_rebar_loc)):
            current_rebar = bottom_edge_long_rebar_loc[num]
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
        loc_z = max_z - (bottom_edge_long_rebar_diam + top_edge_rein_rebar_diam) / 2
        loc_x_l = (
            min_x  # + (bottom_edge_stir_diam + top_edge_rein_rebar_diam) / 2  # 左侧点位置
        )
        loc_x_r = (
            max_x  # - (bottom_edge_stir_diam + top_edge_rein_rebar_diam) / 2  # 右侧点的位置
        )
        # 标注左侧点
        point_l_1 = [loc_x_l, loc_y, loc_z]  # 左侧标志点
        point_l_2 = [
            point_l_1[0] - outline_length_1 * math.cos(theta),
            point_l_1[1],
            point_l_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_l_3 = [point_l_2[0] - outline_length_2, point_l_2[1], point_l_2[2]]
        text = "⑩-a"
        # 标注右侧点
        point_r_1 = [loc_x_r, loc_y, loc_z]  # 右侧标志点
        point_r_2 = [
            point_r_1[0] + outline_length_1 * math.cos(theta),
            point_r_1[1],
            point_r_1[2] + outline_length_1 * math.sin(theta),
        ]
        point_r_3 = [point_r_2[0] + outline_length_2, point_r_2[1], point_r_2[2]]
        dimension_result = [
            [
                [point_l_1, point_l_2],
                [point_l_2, point_l_3],
                [point_r_1, point_r_2],
                [point_r_2, point_r_3],
            ],
            [[point_l_3, text], [point_r_3, text]],
        ]
        return dimension_result

    def get_stair_bottom_edge_rein_rebar_outline_shape(self):
        """
        获取楼梯底部边缘加强筋引出线坐标点
        :return:
        """
        outline_length_1 = 100
        outline_length_2 = 25
        theta = math.pi / 4
        bottom_edge_rein_rebar_loc = (
            self.bottom_edge_rein_rebar_info.get_rebar_model()
        )  # 获取底部边缘加强筋的坐标点
        bottom_edge_rein_rebar_diam = (
            self.bottom_edge_rein_rebar_info.get_rebar_diameter()
        )  # 获取底部边缘加强筋
        bottom_edge_stir_loc = (
            self.bottom_edge_stir_info.get_rebar_model()
        )  # 获取下部边缘箍筋的坐标点
        bottom_edge_stir_diam = (
            self.bottom_edge_stir_info.get_rebar_diameter()
        )  # 获取下部边缘箍筋的直径
        bottom_edge_long_rebar_loc = (
            self.bottom_edge_long_rebar_info.get_rebar_model()
        )  # 获取下部边缘纵筋的坐标点
        bottom_edge_long_rebar_diam = (
            self.bottom_edge_long_rebar_info.get_rebar_diameter()
        )  # 获取下部边缘纵筋的直径
        loc_y = 0
        min_z = self.h2
        min_x = self.b0
        max_x = 0
        # 遍历底部边缘纵筋，获取最小的z坐标值
        for num in range(len(bottom_edge_rein_rebar_loc)):
            current_rebar = bottom_edge_rein_rebar_loc[num]
            current_point = current_rebar[0]  # 获取首点
            if current_point.z < min_z:
                min_z = current_point.z
        # 遍历底部边缘箍筋，获取最大和最小的x坐标值
        for num in range(len(bottom_edge_rein_rebar_loc)):
            current_rebar = bottom_edge_rein_rebar_loc[num]
            for point in current_rebar:
                if point.x > max_x:
                    max_x = point.x
                if point.x < min_x:
                    min_x = point.x
        loc_z = (
            min_z  # + (bottom_edge_long_rebar_diam + bottom_edge_rein_rebar_diam) / 2
        )
        loc_x_l = min_x  # + (bottom_edge_stir_diam + bottom_edge_rein_rebar_diam) / 2  # 左侧点位置
        loc_x_r = max_x  # - (bottom_edge_stir_diam + bottom_edge_rein_rebar_diam) / 2  # 右侧点的位置
        # 标注左侧点
        point_l_1 = [loc_x_l, loc_y, loc_z]  # 左侧标志点
        point_l_2 = [
            point_l_1[0] - outline_length_1 * math.cos(theta),
            point_l_1[1],
            point_l_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_l_3 = [point_l_2[0] - outline_length_2, point_l_2[1], point_l_2[2]]
        text = "⑩-b"
        # 标注右侧点
        point_r_1 = [loc_x_r, loc_y, loc_z]  # 右侧标志点
        point_r_2 = [
            point_r_1[0] + outline_length_1 * math.cos(theta),
            point_r_1[1],
            point_r_1[2] - outline_length_1 * math.sin(theta),
        ]
        point_r_3 = [point_r_2[0] + outline_length_2, point_r_2[1], point_r_2[2]]
        dimension_result = [
            [
                [point_l_1, point_l_2],
                [point_l_2, point_l_3],
                [point_r_1, point_r_2],
                [point_r_2, point_r_3],
            ],
            [[point_l_3, text], [point_r_3, text]],
        ]
        return dimension_result
