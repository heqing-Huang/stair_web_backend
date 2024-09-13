"""
# File       : Rebar_layout.py
# Time       ：2022/9/21 9:25
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""
import time
from typing import Tuple

import numpy as np
import copy

from stair_structure.model import StructuralDesign, StructuralDesignResult
from stair_detailed.models import RailDesignMode, DetailedDesign, DetailedDesignResult
from dc_rebar import *
from .APF_compute import compute_aer, compute_Attract, compute_r
from .tools import rotation_matrix_from_vectors
from .rebar_data import RebarData
from .collision_detection import StairObstacle, StairFCLModel
from .Fcl_models import Agent
from .models import RebarforBIM
from .path_adjustment import PointOffset, point_offset_rebar


def find_next_point(
    bar_dia, fcl_model, current_point_find, offset, action_1, action_2, vecN
):
    if offset == 0:
        L1 = 0
        L2 = 0
        current_point_1 = copy.deepcopy(current_point_find)
        current_point_2 = copy.deepcopy(current_point_find)
        num_1 = 0
        current_point_1 = current_point_1 + action_1
        check_point_1 = current_point_1 + vecN
        agent = Agent(size=bar_dia, position=check_point_1)
        check_ob = fcl_model.collision_agent(agent)
        if check_ob == 0:
            L1 = 1
        else:
            while check_ob == 1:
                num_1 = num_1 + 1
                current_point_1 = current_point_1 + action_1
                check_point_1 = current_point_1 + vecN
                agent = Agent(size=bar_dia, position=check_point_1)
                check_ob = fcl_model.collision_agent(agent)
                if check_ob == 0:
                    L1 = 1
                    break
                if num_1 == 300:
                    L1 = 0
                    break
        current_point_1 = current_point_1 + vecN

        num_2 = 0
        current_point_2 = current_point_2 + action_2
        check_point_2 = current_point_2 + vecN
        agent = Agent(size=bar_dia, position=check_point_2)
        check_ob = fcl_model.collision_agent(agent)
        if check_ob == 0:
            L2 = 1
        else:
            while check_ob == 1:
                num_2 = num_2 + 1
                current_point_2 = current_point_2 + action_2
                check_point_2 = current_point_2 + vecN
                agent = Agent(size=bar_dia, position=check_point_2)
                check_ob = fcl_model.collision_agent(agent)
                if check_ob == 0:
                    L2 = 1
                    break
                if num_2 == 300:
                    L2 = 0
                    break
        current_point_2 = current_point_2 + vecN
        if num_1 <= num_2 or (not L2 and L1):
            next_point = current_point_1
        elif num_1 > num_2 or (not L1 and L2):
            next_point = current_point_2
        else:
            raise Exception("钢筋无法完成排布")
        return next_point
    if offset == 1:
        L1 = 0
        current_point_1 = copy.deepcopy(current_point_find)
        num_1 = 0
        current_point_1 = current_point_1 + action_1
        check_point_1 = current_point_1 + vecN
        agent = Agent(size=bar_dia, position=check_point_1)
        check_ob = fcl_model.collision_agent(agent)
        if check_ob == 0:
            L1 = 1
        else:
            while check_ob == 1:
                num_1 = num_1 + 1
                current_point_1 = current_point_1 + action_1
                check_point_1 = current_point_1 + vecN
                agent = Agent(size=bar_dia, position=check_point_1)
                check_ob = fcl_model.collision_agent(agent)
                if check_ob == 0:
                    L1 = 1
                    break
                if num_1 == 300:
                    L1 = 0
                    break
        if not L1:
            raise Exception("钢筋无法完成排布")
        current_point_1 = current_point_1 + vecN
        next_point = current_point_1
        return next_point
    if offset == 2:
        L2 = 0
        current_point_2 = copy.deepcopy(current_point_find)
        num_2 = 0
        current_point_2 = current_point_2 + action_2
        check_point_2 = current_point_2 + vecN
        agent = Agent(size=bar_dia, position=check_point_2)
        check_ob = fcl_model.collision_agent(agent)
        if check_ob == 0:
            L2 = 1
        else:
            while check_ob == 1:
                num_2 = num_2 + 1
                current_point_2 = current_point_2 + action_2
                check_point_2 = current_point_2 + vecN
                agent = Agent(size=bar_dia, position=check_point_2)
                check_ob = fcl_model.collision_agent(agent)
                if check_ob == 0:
                    L2 = 1
                    break
                if num_2 == 300:
                    L2 = 0
                    break
        if not L2:
            raise Exception("钢筋无法完成排布")
        current_point_2 = current_point_2 + vecN
        next_point = current_point_2
        return next_point


def APF(st_p, object_p, fcl_model, bar_dia, offset, normal_vector, qa=1):
    """
    势能场法APF
    """
    step_length = 3
    init_dir = (object_p - st_p) / np.linalg.norm((object_p - st_p))  # 指定初始搜索方向（单位向量）
    label = np.argmax(np.abs(object_p - st_p))  # 确定相差最大的轴为优先移动方向（x0 y1 z2）
    st_p_tem = st_p + bar_dia / 2 * init_dir
    object_p_tem = object_p - bar_dia / 2 * init_dir
    label_for_opt = np.argmax(normal_vector)
    action_1 = normal_vector
    action_2 = -normal_vector
    vecN = step_length * init_dir
    bar_cell = [list(st_p)]
    current_point = copy.deepcopy(st_p_tem)
    bar_cell.append(current_point.tolist())
    for kstep in range(5000):
        att_r = compute_r(current_point, object_p_tem)
        att_azimuth, att_elevation = compute_aer(
            current_point, object_p_tem, att_r
        )  # 计算方位角和仰角
        Fatx, Faty, Fatz = compute_Attract(
            qa, att_azimuth, att_elevation, att_r
        )  # 计算三个轴方向的引力分量
        Fa = np.array([Fatx, Faty, Fatz])
        vec1 = step_length * Fa / np.sqrt(np.sum(Fa * Fa))  # 将引力向量归一化并乘以步长
        check_point = copy.deepcopy(current_point)
        check_point = check_point + vec1
        agent = Agent(size=bar_dia, position=check_point)
        check_ob = fcl_model.collision_agent(agent)
        if check_ob == 1:  # 发生碰撞
            if len(bar_cell) == 0:
                bar_cell = bar_cell + [current_point.tolist()]
            current_point_find = np.array(bar_cell[-1])
            next_point = find_next_point(
                bar_dia, fcl_model, current_point_find, offset, action_1, action_2, vecN
            )
            new_point = next_point.tolist()
            bar_cell = bar_cell + [new_point]
            current_point = np.array(bar_cell[-1])
        else:
            current_point = current_point + vec1
            bar_cell.append(list(current_point))
        object_p_tem[label_for_opt] = current_point[label_for_opt]
        if label == 0:
            if abs(current_point[0] - object_p_tem[0]) <= step_length:
                break
        if label == 1:
            if abs(current_point[1] - object_p_tem[1]) <= step_length:
                break
        if label == 2:
            if abs(current_point[2] - object_p_tem[2]) <= step_length:
                break
    bar_cell = bar_cell + [object_p.tolist()]
    return bar_cell


def APF_rebar(fcl_model, bar, bar_dia, offset, normal_vector, qa=1):
    """
    势能场法APF_钢筋
    :param normal_vector:
    :param bar:
    :param fcl_model:
    :param bar_dia:
    :param offset:
    """
    action_1 = normal_vector
    action_2 = -normal_vector

    check_rebar = copy.deepcopy(bar)
    rebar_fcls = fcl_model.new_rebar_fcl(check_rebar, bar_dia)  # 返回带Rebar_fcl对象的列表
    rebar_objs = fcl_model.new_obj(rebar_fcls)  # 返回带fcl碰撞检测对象的列表
    check_ob = fcl_model.collision_check_add(rebar_objs)
    if check_ob == 1:  # 发生碰撞
        if offset == 0:
            L1 = 0
            L2 = 0
            current_rebar_1 = copy.deepcopy(bar)
            current_rebar_2 = copy.deepcopy(bar)
            num_1 = 0
            current_rebar_1 = current_rebar_1 + action_1
            rebar_fcls = fcl_model.new_rebar_fcl(current_rebar_1, bar_dia)
            rebar_objs = fcl_model.new_obj(rebar_fcls)
            check_ob = fcl_model.collision_check_add(rebar_objs)
            if check_ob == 0:
                L1 = 1
            else:
                while check_ob == 1:
                    num_1 = num_1 + 1
                    current_rebar_1 = current_rebar_1 + action_1
                    rebar_fcls = fcl_model.new_rebar_fcl(current_rebar_1, bar_dia)
                    rebar_objs = fcl_model.new_obj(rebar_fcls)
                    check_ob = fcl_model.collision_check_add(rebar_objs)
                    if check_ob == 0:
                        L1 = 1
                        break
                    if num_1 == 300:
                        L1 = 0
                        break

            num_2 = 0
            current_rebar_2 = current_rebar_2 + action_2
            rebar_fcls = fcl_model.new_rebar_fcl(current_rebar_2, bar_dia)
            rebar_objs = fcl_model.new_obj(rebar_fcls)
            check_ob = fcl_model.collision_check_add(rebar_objs)
            if check_ob == 0:
                L2 = 1
            else:
                while check_ob == 1:
                    num_2 = num_2 + 1
                    current_rebar_2 = current_rebar_2 + action_2
                    rebar_fcls = fcl_model.new_rebar_fcl(current_rebar_2, bar_dia)
                    rebar_objs = fcl_model.new_obj(rebar_fcls)
                    check_ob = fcl_model.collision_check_add(rebar_objs)
                    if check_ob == 0:
                        L2 = 1
                        break
                    if num_2 == 300:
                        L2 = 0
                        break
            if num_1 <= num_2 or (not L2 and L1):
                rebar_set = current_rebar_1
            elif num_1 > num_2 or (not L1 and L2):
                rebar_set = current_rebar_2
            else:
                raise Exception("钢筋无法完成排布")
            return rebar_set
    else:
        rebar_set = check_rebar
        return rebar_set


def long_rebar_data_convert(rebar_list, rebar_dia):
    """
    整理钢筋数据
    :param rebar_list:
    :param rebar_dia:
    :return:
    """
    rebar_computation = []
    rebar_dias = np.full([len(rebar_list)], rebar_dia, dtype=int)
    for i in range(len(rebar_list)):
        rebar_sets = []
        rebar_set = []
        for j in range(len(rebar_list[i])):
            rebar_set.append(
                [rebar_list[i][j].x, rebar_list[i][j].y, rebar_list[i][j].z]
            )
        rebar_sets.append(rebar_set)  # 一根钢筋的[x1, y1, z1, x2, y2, z2.........]
        rebar_computation = rebar_computation + rebar_sets
    rebar_computation = np.array(rebar_computation)
    return rebar_computation, rebar_dias


def mid_rebar_convert(mid_rebar_computation):
    """
    整理分布钢筋数据
    :param mid_rebar_computation: n*4*3
    :return:
    """
    mid_rebar_set = []
    for i in range(int(len(mid_rebar_computation) / 2)):
        mid_rebar_set = mid_rebar_set + [
            [
                [
                    mid_rebar_computation[2 * i][1][0],
                    mid_rebar_computation[2 * i][1][1],
                    mid_rebar_computation[2 * i][1][2],
                ],
                [
                    mid_rebar_computation[2 * i][2][0],
                    mid_rebar_computation[2 * i][2][1],
                    mid_rebar_computation[2 * i][2][2],
                ],
            ],
            [
                [
                    mid_rebar_computation[2 * i + 1][1][0],
                    mid_rebar_computation[2 * i + 1][1][1],
                    mid_rebar_computation[2 * i + 1][1][2],
                ],
                [
                    mid_rebar_computation[2 * i + 1][2][0],
                    mid_rebar_computation[2 * i + 1][2][1],
                    mid_rebar_computation[2 * i + 1][2][2],
                ],
            ],
        ]
    mid_rebar_set = np.array(mid_rebar_set)
    return mid_rebar_set


def dis_rebar_data_convert(rebar_list, rebar_dia):
    """
    整理分布钢筋数据
    :param rebar_list:
    :param rebar_dia:
    :return:
    """
    rebar_computation = []
    rebar_dias = np.full([len(rebar_list)], rebar_dia, dtype=int)
    for i in range(int(len(rebar_list) / 2)):
        rebar_sets = [
            [
                [
                    rebar_list[2 * i][1].x,
                    rebar_list[2 * i][1].y,
                    rebar_list[2 * i][1].z,
                ],
                [
                    rebar_list[2 * i][2].x,
                    rebar_list[2 * i][2].y,
                    rebar_list[2 * i][2].z,
                ],
            ],
            [
                [
                    rebar_list[2 * i + 1][1].x,
                    rebar_list[2 * i + 1][1].y,
                    rebar_list[2 * i + 1][1].z,
                ],
                [
                    rebar_list[2 * i + 1][2].x,
                    rebar_list[2 * i + 1][2].y,
                    rebar_list[2 * i + 1][2].z,
                ],
            ],
        ]
        rebar_computation = rebar_computation + rebar_sets
    rebar_computation = np.array(rebar_computation)
    return rebar_computation, rebar_dias


class RebarSet:
    def __init__(
        self, fcl_model, rebar_list, rebar_dias, normal_vector=np.array([1, 0, 0])
    ):

        self.rebar_list = rebar_list
        self.rebar_output = copy.deepcopy(rebar_list)
        self.rebar_dias = rebar_dias
        self.fcl_model = fcl_model
        self.normal_vector = normal_vector

    def long_layout(self):
        if len(self.rebar_list) != len(self.rebar_dias):
            raise Exception("钢筋根数和直径数量不匹配")
        offset_label = np.argmax(np.abs(self.normal_vector))
        rebar_cell = [[] for _ in range(len(self.rebar_list))]  # 存放钢筋路径点
        for i in range(len(self.rebar_list)):
            rebar = copy.deepcopy(self.rebar_list[i])  # 第i根钢筋的控制点列表
            rebar_cell[i].append(rebar[0].tolist())  # 第i根钢筋的起始点加入钢筋路径
            origin = copy.deepcopy(rebar[0][offset_label])  # 偏移 初始点
            offset = 0  # 偏移方向
            for j in range(len(rebar) - 1):
                start_point = copy.deepcopy(rebar[j])
                object_point = copy.deepcopy(rebar[j + 1])
                rebar_points = APF(
                    st_p=start_point,
                    object_p=object_point,
                    fcl_model=self.fcl_model,
                    bar_dia=self.rebar_dias[i],
                    offset=offset,
                    normal_vector=self.normal_vector,
                )
                points_offset = PointOffset(rebar_points)
                points_offset.featurepoint()  # 提取特征点（弯折）
                rebar_points_offset = (
                    points_offset.long_picknewpoint()
                )  # 拉直（重新定义起止点的x坐标）
                rebar_cell[i].append(rebar_points_offset[-1])
                if offset == 0:
                    if rebar_points_offset[0][offset_label] - origin > 0:
                        offset = 1
                    elif rebar_points_offset[0][offset_label] - origin < 0:
                        offset = 2
                    else:
                        offset = 0
                rebar[:, [offset_label]] = rebar_points_offset[-1][offset_label]  # 拉直

            rebar_cell[i] = point_offset_rebar(
                rebar_cell[i], origin, offset_label
            )  # 拉直
            self.fcl_model.add_rebar(rebar=rebar, dia=self.rebar_dias[i])
            self.rebar_output[i] = rebar_cell[i]

    def dis_layout_bottom(self):
        """底部分布纵筋"""
        if len(self.rebar_list) != len(self.rebar_dias):
            raise Exception("钢筋根数和直径数量不匹配")
        offset_label = np.argmax(np.abs(self.normal_vector))  # 1
        for i in range(int(len(self.rebar_list) / 2)):
            rebar_bottom = copy.deepcopy(self.rebar_list[2 * i])
            rebar_top = copy.deepcopy(self.rebar_list[2 * i + 1])
            origin = copy.deepcopy(rebar_bottom[0][offset_label])  # 初始点y
            offset = 0  # 偏移方向
            # 下部钢筋
            start_point_bottom = copy.deepcopy(rebar_bottom[0])
            object_point_bottom = copy.deepcopy(rebar_bottom[1])
            rebar_points_bottom = APF(
                st_p=start_point_bottom,
                object_p=object_point_bottom,
                fcl_model=self.fcl_model,
                bar_dia=self.rebar_dias[2 * i],
                offset=offset,
                normal_vector=self.normal_vector,
            )
            points_offset_bottom = PointOffset(rebar_points_bottom)
            points_offset_bottom.featurepoint()
            rebar_points_offset_bottom = points_offset_bottom.dis_picknewpoint()
            rebar_bottom_output = np.array(rebar_points_offset_bottom)
            offset_bottom = rebar_bottom_output[0] - start_point_bottom

            if offset == 0:
                if rebar_points_offset_bottom[0][offset_label] - origin > 0:
                    offset = 1
                elif rebar_points_offset_bottom[0][offset_label] - origin < 0:
                    offset = 2
                else:
                    offset = 0
            # 上部钢筋
            start_point_top = copy.deepcopy(rebar_top[0]) + offset_bottom
            object_point_top = copy.deepcopy(rebar_top[1]) + offset_bottom
            rebar_points_top = APF(
                st_p=start_point_top,
                object_p=object_point_top,
                fcl_model=self.fcl_model,
                bar_dia=self.rebar_dias[2 * i + 1],
                offset=offset,
                normal_vector=self.normal_vector,
            )
            points_offset_top = PointOffset(rebar_points_top)
            points_offset_top.featurepoint()
            rebar_points_offset_top = points_offset_top.dis_picknewpoint()
            rebar_top_out = np.array(rebar_points_offset_top)
            offset_top = rebar_top_out[0] - start_point_top
            rebar_bottom_output = rebar_bottom_output + offset_top
            self.fcl_model.add_rebar(
                rebar=rebar_bottom_output, dia=self.rebar_dias[2 * i]
            )
            self.rebar_output[2 * i] = rebar_bottom_output
            self.fcl_model.add_rebar(
                rebar=rebar_top_out, dia=self.rebar_dias[2 * i + 1]
            )
            self.rebar_output[2 * i + 1] = rebar_top_out

    def dis_layout(self):
        """中间分布筋"""
        if len(self.rebar_list) != len(self.rebar_dias):
            raise Exception("钢筋根数和直径数量不匹配")
        for i in range(int(len(self.rebar_list) / 2)):
            rebar_bottom = copy.deepcopy(self.rebar_list[2 * i])  # 2*3
            rebar_top = copy.deepcopy(self.rebar_list[2 * i + 1])
            offset = 0  # 偏移方向
            # 下部钢筋
            start_point_bottom = copy.deepcopy(rebar_bottom[0])  # 1*3
            object_point_bottom = copy.deepcopy(rebar_bottom[1])
            start_point_top = copy.deepcopy(rebar_top[0])
            object_point_top = copy.deepcopy(rebar_top[1])
            rebar = np.array(
                [
                    start_point_bottom,
                    object_point_bottom,
                    object_point_top,
                    start_point_top,
                ]
            )  # 4*3

            rebar_points = APF_rebar(
                fcl_model=self.fcl_model,
                bar=rebar,
                bar_dia=self.rebar_dias[2 * i],
                offset=offset,
                normal_vector=self.normal_vector,
            )

            rebar_bottom_output = np.array([rebar_points[0], rebar_points[1]])
            rebar_top_out = np.array([rebar_points[3], rebar_points[2]])
            self.fcl_model.add_rebar(
                rebar=rebar_bottom_output, dia=self.rebar_dias[2 * i]
            )
            self.rebar_output[2 * i] = rebar_bottom_output
            self.fcl_model.add_rebar(
                rebar=rebar_top_out, dia=self.rebar_dias[2 * i + 1]
            )
            self.rebar_output[2 * i + 1] = rebar_top_out

    def dis_layout_top(self):
        """顶部分布纵筋"""
        if len(self.rebar_list) != len(self.rebar_dias):
            raise Exception("钢筋根数和直径数量不匹配")
        offset_label = np.argmax(np.abs(self.normal_vector))  # 1
        for i in range(int(len(self.rebar_list) / 2)):
            rebar_bottom = copy.deepcopy(self.rebar_list[2 * i])
            rebar_top = copy.deepcopy(self.rebar_list[2 * i + 1])
            origin = copy.deepcopy(rebar_bottom[0][offset_label])  # 偏移 初始点

            for j in range(2):

                offset = j + 1  # 偏移方向
                # 下部钢筋
                start_point_bottom = copy.deepcopy(rebar_bottom[0])
                object_point_bottom = copy.deepcopy(rebar_bottom[1])
                rebar_points_bottom = APF(
                    st_p=start_point_bottom,
                    object_p=object_point_bottom,
                    fcl_model=self.fcl_model,
                    bar_dia=self.rebar_dias[2 * i],
                    offset=offset,
                    normal_vector=self.normal_vector,
                )
                points_offset_bottom = PointOffset(rebar_points_bottom)
                points_offset_bottom.featurepoint()
                rebar_points_offset_bottom = points_offset_bottom.dis_picknewpoint()
                rebar_bottom_output = np.array(rebar_points_offset_bottom)
                offset_bottom = rebar_bottom_output[0] - start_point_bottom
                # 上部钢筋
                start_point_top = copy.deepcopy(rebar_top[0]) + offset_bottom
                object_point_top = copy.deepcopy(rebar_top[1]) + offset_bottom
                rebar_points_top = APF(
                    st_p=start_point_top,
                    object_p=object_point_top,
                    fcl_model=self.fcl_model,
                    bar_dia=self.rebar_dias[2 * i + 1],
                    offset=offset,
                    normal_vector=self.normal_vector,
                )
                points_offset_top = PointOffset(rebar_points_top)
                points_offset_top.featurepoint()
                rebar_points_offset_top = points_offset_top.dis_picknewpoint()
                rebar_top_out = np.array(rebar_points_offset_top)
                offset_top = rebar_top_out[0] - start_point_top
                rebar_bottom_output = rebar_bottom_output + offset_top
                if offset == 1:
                    rebar_top_out_left = rebar_top_out
                    rebar_bottom_output_left = rebar_bottom_output
                    offset_left = abs(rebar_bottom_output[0][offset_label] - origin)
                else:
                    rebar_top_out_right = rebar_top_out
                    rebar_bottom_output_right = rebar_bottom_output
                    offset_right = abs(rebar_bottom_output[0][offset_label] - origin)
            if offset_left <= offset_right:
                rebar_bottom_output = rebar_bottom_output_left
                rebar_top_out = rebar_top_out_left
            else:
                rebar_bottom_output = rebar_bottom_output_right
                rebar_top_out = rebar_top_out_right
            self.fcl_model.add_rebar(
                rebar=rebar_bottom_output, dia=self.rebar_dias[2 * i]
            )
            self.rebar_output[2 * i] = rebar_bottom_output
            self.fcl_model.add_rebar(
                rebar=rebar_top_out, dia=self.rebar_dias[2 * i + 1]
            )
            self.rebar_output[2 * i + 1] = rebar_top_out

    def stirrup_layout(self):
        if len(self.rebar_list) != len(self.rebar_dias):
            raise Exception("钢筋根数和直径数量不匹配")
        offset_label = np.argmax(np.abs(self.normal_vector))
        for i in range(len(self.rebar_list)):
            rebar = copy.deepcopy(self.rebar_list[i])  # 5*3
            origin = copy.deepcopy(rebar[0][offset_label])  # 偏移 初始点
            offset = 0  # 偏移方向
            rebar_set = APF_rebar(
                fcl_model=self.fcl_model,
                bar=rebar,
                bar_dia=self.rebar_dias[i],
                offset=offset,
                normal_vector=self.normal_vector,
            )
            self.fcl_model.add_rebar(rebar=rebar_set, dia=self.rebar_dias[i])
            self.rebar_output[i] = rebar_set.tolist()


def mid_rebar_for_BIM(rebar_bottom, rebar_top):
    """还原中部分布筋的四个控制点"""
    rebar_bottom_point = [
        [rebar_bottom[0][0], rebar_top[0][1], rebar_top[0][2]],
        rebar_bottom[0].tolist(),
        rebar_bottom[1].tolist(),
        [rebar_bottom[1][0], rebar_top[0][1], rebar_top[0][2]],
    ]

    rebar_top_point = [
        [rebar_top[0][0], rebar_bottom[0][1], rebar_bottom[0][2]],
        rebar_top[0].tolist(),
        rebar_top[1].tolist(),
        [rebar_top[1][0], rebar_bottom[0][1], rebar_bottom[0][2]],
    ]
    return rebar_bottom_point, rebar_top_point


def rebar_layout(
    structure_design: StructuralDesign,
    structure_design_result: StructuralDesignResult,
    detailed_design: DetailedDesign,
    detailed_design_result: DetailedDesignResult,
):
    """
    钢筋排布函数
    :param structure_design:  结构设计参数
    :param structure_design_result: 结构设计结果
    :param detailed_design:  深化设计参数
    :param detailed_design_result: 深化设计结果
    :return: 用于BIM建模的钢筋数据
    """
    rebar_for_BIM = RebarforBIM()  # 创建用于生成钢筋BIM模型的实例
    fcl_model = StairFCLModel()  # 创建fcl碰撞检测模型
    detailed_design = detailed_design_result.detailed_design
    stair_obstacle = StairObstacle(
        structure_design=structure_design,
        structure_design_result=structure_design_result,
        detailed_design=detailed_design,
        detailed_design_result=detailed_design_result,
    )

    # 保护层障碍物
    cover_objs = stair_obstacle.get_cover()  # 包含多个障碍物对象的列表
    fcl_model.add_obj(cover_objs)  # 障碍对象添加到fcl_model里geoms和objs去

    # 孔洞障碍物
    hole_objs = stair_obstacle.get_hole()
    fcl_model.add_obj(hole_objs)
    if detailed_design.inserts_detailed.rail_design_mode == RailDesignMode.MANUAL:
        # 栏杆预埋件
        railing_objs = stair_obstacle.get_railing()
        fcl_model.add_obj(railing_objs)

    # 吊装预埋件
    lifting_objs = stair_obstacle.get_lifting()
    fcl_model.add_obj(lifting_objs)

    # 脱模预埋件
    demolding_objs = stair_obstacle.get_demolding()
    fcl_model.add_obj(demolding_objs)

    # 钢筋数据
    rebar_data = RebarData(
        structure_design=structure_design,
        structure_design_result=structure_design_result,
        detailed_design=detailed_design,
        detailed_design_result=detailed_design_result,
    )
    # 孔洞加强钢筋
    hole_rebar_objs, hole_rebar_BIM = rebar_data.get_hole_rebar()
    rebar_for_BIM.hole_rebar = hole_rebar_BIM
    fcl_model.add_obj(hole_rebar_objs)

    # 吊装加强钢筋纵筋
    (
        lifting_longitudinal_rebar_objs,
        lifting_longitudinal_rebar_BIM,
    ) = rebar_data.get_lifting_longitudinal_rebar()
    rebar_for_BIM.lifting_longitudinal_rebar = lifting_longitudinal_rebar_BIM
    fcl_model.add_obj(lifting_longitudinal_rebar_objs)

    # 吊装加强钢筋点筋
    (
        lifting_point_rebar_objs,
        lifting_point_rebar_BIM,
    ) = rebar_data.get_lifting_point_rebar()
    rebar_for_BIM.lifting_point_rebar = lifting_point_rebar_BIM
    fcl_model.add_obj(lifting_point_rebar_objs)

    # 计算

    start = time.time()
    # 下部加强纵筋边筋
    bottom_edge_reinforce_rebar = rebar_data.get_bottom_edge_reinforce_rebar()
    bottom_edge_rein_rebar_computation = np.array(
        bottom_edge_reinforce_rebar
    )  # [[[...], [...], [...]], [[...], [...], [...]]]
    bottom_edge_rein_rebar_dias = (
        np.ones(len(bottom_edge_reinforce_rebar))
        * rebar_data.bottom_edge_reinforce_rebar_diameter
    )
    bottom_edge_rein_rebar_layout = RebarSet(
        fcl_model, bottom_edge_rein_rebar_computation, bottom_edge_rein_rebar_dias
    )
    bottom_edge_rein_rebar_layout.long_layout()  # 1. 将调直后的钢筋添加到现有的fcl_model中 2. 更新self.rebar_output
    fcl_model = bottom_edge_rein_rebar_layout.fcl_model
    bottom_edge_rein_rebar_BIM = []
    for (
        rebar_points
    ) in bottom_edge_rein_rebar_layout.rebar_output:  # rebar_output中包含多根钢筋的路径点
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.bottom_edge_reinforce_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        bottom_edge_rein_rebar_BIM.append(rebar)
    rebar_for_BIM.bottom_edge_rein_rebar = bottom_edge_rein_rebar_BIM

    # 上部加强纵筋边筋
    top_edge_reinforce_rebar = rebar_data.get_top_edge_reinforce_rebar()
    top_edge_rein_rebar_computation = np.array(top_edge_reinforce_rebar)
    top_edge_rein_rebar_dias = (
        np.ones(len(bottom_edge_reinforce_rebar))
        * rebar_data.top_edge_reinforce_rebar_diameter
    )
    top_edge_rein_rebar_layout = RebarSet(
        fcl_model, top_edge_rein_rebar_computation, top_edge_rein_rebar_dias
    )
    top_edge_rein_rebar_layout.long_layout()
    fcl_model = top_edge_rein_rebar_layout.fcl_model
    top_edge_rein_rebar_BIM = []
    for rebar_points in top_edge_rein_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.top_edge_reinforce_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        top_edge_rein_rebar_BIM.append(rebar)
    rebar_for_BIM.top_edge_rein_rebar = top_edge_rein_rebar_BIM

    # 底部钢筋
    bottom_rebar = rebar_data.get_bottom_rebar()
    bottom_rebar_computation = np.array(bottom_rebar)
    bottom_rebar_dias = np.ones(len(bottom_rebar)) * rebar_data.bottom_rebar_diameter
    bottom_rebar_layout = RebarSet(
        fcl_model, bottom_rebar_computation, bottom_rebar_dias
    )
    bottom_rebar_layout.long_layout()
    fcl_model = bottom_rebar_layout.fcl_model
    bottom_rebar_BIM = []
    for rebar_points in bottom_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.bottom_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        bottom_rebar_BIM.append(rebar)
    rebar_for_BIM.bottom_rebar = bottom_rebar_BIM

    # 顶部钢筋
    top_rebar = rebar_data.get_top_rebar()
    top_rebar_computation = np.array(top_rebar)
    top_rebar_dias = np.ones(len(top_rebar)) * rebar_data.top_rebar_diameter
    top_rebar_layout = RebarSet(fcl_model, top_rebar_computation, top_rebar_dias)
    top_rebar_layout.long_layout()
    fcl_model = top_rebar_layout.fcl_model
    top_rebar_BIM = []
    for rebar_points in top_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.top_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        top_rebar_BIM.append(rebar)
    rebar_for_BIM.top_rebar = top_rebar_BIM

    # 下部加强箍筋
    bottom_edge_stirrup_rebar = rebar_data.get_bottom_edge_stirrup_rebar()
    bottom_edge_stirrup_rebar_computation = np.array(bottom_edge_stirrup_rebar)  # n*5*3
    bottom_edge_stirrup_rebar_dias = (
        np.ones(len(bottom_edge_stirrup_rebar))
        * rebar_data.bottom_edge_stirrup_diameter
    )
    bottom_edge_stirrup_rebar_layout = RebarSet(
        fcl_model, bottom_edge_stirrup_rebar_computation, bottom_edge_stirrup_rebar_dias
    )
    bottom_edge_stirrup_rebar_layout.stirrup_layout()
    fcl_model = bottom_edge_stirrup_rebar_layout.fcl_model
    bottom_edge_stirrup_rebar_BIM = []
    for rebar_points in bottom_edge_stirrup_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.bottom_edge_stirrup_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        bottom_edge_stirrup_rebar_BIM.append(rebar)
    rebar_for_BIM.bottom_edge_stirrup_rebar = bottom_edge_stirrup_rebar_BIM

    # 上部加强箍筋
    top_edge_stirrup_rebar = rebar_data.get_top_edge_stirrup_rebar()
    top_edge_stirrup_rebar_computation = np.array(top_edge_stirrup_rebar)
    top_edge_stirrup_rebar_dias = (
        np.ones(len(top_edge_stirrup_rebar)) * rebar_data.top_edge_stirrup_diameter
    )
    top_edge_stirrup_rebar_layout = RebarSet(
        fcl_model, top_edge_stirrup_rebar_computation, top_edge_stirrup_rebar_dias
    )
    top_edge_stirrup_rebar_layout.stirrup_layout()
    fcl_model = top_edge_stirrup_rebar_layout.fcl_model
    top_edge_stirrup_rebar_BIM = []
    for rebar_points in top_edge_stirrup_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.top_edge_stirrup_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        top_edge_stirrup_rebar_BIM.append(rebar)
    rebar_for_BIM.top_edge_stirrup_rebar = top_edge_stirrup_rebar_BIM

    # 下部加强筋（分布）
    bottom_rein_rebar = rebar_data.get_bottom_rein_rebar()
    bottom_rein_rebar_computation = np.array(bottom_rein_rebar)
    bottom_rein_rebar_dias = (
        np.ones(len(bottom_rein_rebar))
        * rebar_data.bottom_edge_longitudinal_rebar_diameter
    )
    bottom_rein_rebar_normal_vector = np.array([0, 1, 0])
    bottom_rein_rebar_layout = RebarSet(
        fcl_model,
        bottom_rein_rebar_computation,
        bottom_rein_rebar_dias,
        normal_vector=bottom_rein_rebar_normal_vector,
    )
    bottom_rein_rebar_layout.dis_layout_bottom()
    fcl_model = bottom_rein_rebar_layout.fcl_model
    bottom_rein_rebar_BIM = []
    for rebar_points in bottom_rein_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.bottom_edge_longitudinal_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        bottom_rein_rebar_BIM.append(rebar)
    rebar_for_BIM.bottom_rein_rebar = bottom_rein_rebar_BIM

    # 上部加强筋 （分布）
    top_rein_rebar = rebar_data.get_top_rein_rebar()
    top_rein_rebar_computation = np.array(top_rein_rebar)  # n*2*3
    top_rein_rebar_dias = (
        np.ones(len(top_rein_rebar)) * rebar_data.top_edge_longitudinal_rebar_diameter
    )
    top_rein_rebar_normal_vector = np.array([0, 1, 0])
    top_rein_rebar_layout = RebarSet(
        fcl_model,
        top_rein_rebar_computation,
        top_rein_rebar_dias,
        normal_vector=top_rein_rebar_normal_vector,
    )
    top_rein_rebar_layout.dis_layout_top()
    fcl_model = top_rein_rebar_layout.fcl_model
    top_rein_rebar_BIM = []
    for rebar_points in top_rein_rebar_layout.rebar_output:
        rebar_point_list = rebar_points.tolist()
        rebar = Rebar(
            radius=int(rebar_data.top_edge_longitudinal_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list),
        )
        top_rein_rebar_BIM.append(rebar)
    rebar_for_BIM.top_rein_rebar = top_rein_rebar_BIM

    # 分布筋
    mid_rebar = rebar_data.get_mid_rebar()
    mid_rebar_computation = np.array(mid_rebar)  # n * 4 * 3
    mid_rebar_dias = (
        np.ones(len(mid_rebar)) * rebar_data.mid_distribution_rebar_diameter
    )
    vector1 = mid_rebar_computation[0][1] - mid_rebar_computation[0][0]
    vector2 = mid_rebar_computation[0][2] - mid_rebar_computation[0][0]
    mid_rebar_normal_vector = np.cross(vector1, vector2) / np.linalg.norm(
        np.cross(vector1, vector2)
    )
    mid_rebar_computation = mid_rebar_convert(
        mid_rebar_computation
    )  # 分布筋被转换为只有x向的两根钢筋 n * 2 * 3(每个分布筋上下2段包含在n内，每一段包含左右2点，每点3个维度)
    mid_rebar_layout = RebarSet(
        fcl_model,
        mid_rebar_computation,
        mid_rebar_dias,
        normal_vector=mid_rebar_normal_vector,
    )
    mid_rebar_layout.dis_layout()
    for i in range(int(len(mid_rebar_layout.rebar_output) / 2)):
        original_rebar_bottom = mid_rebar_layout.rebar_output[2 * i]
        original_rebar_top = mid_rebar_layout.rebar_output[2 * i + 1]
        rebar_point_list_bottom, rebar_point_list_top = mid_rebar_for_BIM(
            original_rebar_bottom, original_rebar_top
        )  # 4*3
        rebar = Rebar(
            radius=int(rebar_data.mid_distribution_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list_bottom),
        )
        rebar_for_BIM.mid_rebar.append(rebar)
        rebar = Rebar(
            radius=int(rebar_data.mid_distribution_rebar_diameter / 2),
            poly=IndexedPolyCurve(rebar_point_list_top),
        )
        rebar_for_BIM.mid_rebar.append(rebar)
    fcl_model = mid_rebar_layout.fcl_model
    stop = time.time()

    print(stop - start)
    return rebar_for_BIM
