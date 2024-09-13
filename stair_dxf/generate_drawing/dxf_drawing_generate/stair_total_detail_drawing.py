"""
预制楼梯深化设计图纸
生成楼梯的DXF文件
  Block,Layer,Linetype---块、图层、线型
  Current Object Scale、Paper Space Linetype Scale、Model Space Linetype Scale(CETLSCALE、LTSCALE、PSLTSCALE、MSLTSCALE)
"""

import copy

import numpy as np

from stair_dxf.generate_drawing.occ_drawing.solid_projection import (
    StairProjectionDrawingData,
    StairTopViewData,
    StairBottomViewData,
    StairLeftViewData,
    StairRightViewData,
    StairSectionOneViewData,
    StairSectionTwoViewData,
    StairReinforceViewData,
    StairRebarSectionAToAViewData,
    StairRebarSectionBToBViewData,
    StairRebarSectionCToCViewData,
    StairStepSlotLeftViewData,
    StairStepSlotTopViewData,
    StairBottomInstallNodeViewData,
    StairTopInstallNodeViewData,
    StairDoubleSideWallJointLongViewData,
    StairDoubleSideWallJointTransverseViewData,
    StairBottomHoleReinRebarViewData,
    StairTopHoleReinRebarViewData,
    StairRailEmbeddedDetailViewData,
)

from stair_dxf.generate_drawing.dxf_drawing_generate.basic_method import (
    get_circle_radius_and_center,
    adjust_drawing_scale,
    radian_bt_vectors,
    rotation_3d,
    transform_point_from_yz_to_xy,
    transform_point_from_xz_to_xy,
    transform_point_from_yz_to_xy_s,
    rotation_point_from_base_to_theta,
    calculate_normal_vector,
    transform_point_from_xy_to_yx,
)
from stair_dxf.generate_drawing.dxf_drawing_generate.dimension_need_datas import (
    TopViewDimensionData,
    BottomViewDimensionData,
    LeftViewDimensionData,
    StairSectionOneViewDimensionData,
    StairSectionTwoViewDimensionData,
    ReinforceViewDimensionData,
    StairRebarSectionAToAViewDimensionData,
    StairRebarSectionBToBViewDimensionData,
    StairRebarSectionCToCViewDimensionData,
)
import math
import ezdxf
from ezdxf.enums import TextEntityAlignment

from ezdxf.document import Drawing

from stair_detailed.models import DetailedDesign, DetailedDesignResult
from stair_rebar_layout.models import RebarforBIM
from stair_structure.model import StructuralDesign, StructuralDesignResult


class StairTopView(object):
    """
    生成楼梯dxf俯视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_projection_data = StairTopViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.top_view_dimension_data = TopViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.rotate_angle = math.pi / 2  # 图形旋转角度，初始方向和最终放置方向旋转的角度
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.top_view = self.dxf_doc
        self.model_space = self.top_view.modelspace()  # 创建模型空间

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def create_total_block(self):
        """
        产生所有图层块
        :return:
        """
        self.Top_Shape = self.top_view.blocks.new("top_view")  # 添加视图块
        self.Rein_rebar = self.top_view.blocks.new("top_view_rein_rebar")  # 添加加强筋块
        self.Dimension = self.top_view.blocks.new("top_view_dimension")  # 添加标注块
        self.Hole_dimension = self.top_view.blocks.new("top_view_hole_dim")  # 添加孔洞位置块
        self.Text_Dim = self.top_view.blocks.new("top_view_text_dim")  # 添加孔洞说明文本标注块

    def begin_draw_standard_profile_points(self, profile_points, layer: str):
        """
        开始绘制轮廓点
        :param profile_points:轮廓点
        :param layer: 图层
        :return:
        """
        for num in range(len(profile_points)):
            profile_points[num][0] = list(profile_points[num][0])
            profile_points[num][1] = list(profile_points[num][1])
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (segment[0][0], segment[0][1], segment[0][2]),
                (segment[1][0], segment[1][1], segment[1][2]),
            ]  # 将三维点转化为平面点
            self.Top_Shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_polyline(self):
        """
        开始绘制多段线
        :return:
        """
        points = (
            self.stair_projection_data.get_stair_solid_projection()
        )  # [[(),()],[(),()]]
        layer_name = "G_Outline"
        self.begin_draw_standard_profile_points(points, layer_name)

    def begin_draw_circle(self):
        """
        开始绘制圆
        :return:
        """
        hole_points = self.stair_projection_data.get_hole_projection()  # 获取孔洞投影数据
        layer_name = "G_Outline"
        for num in range(len(hole_points)):
            single_hole = hole_points[num]
            center, radius = get_circle_radius_and_center(single_hole)
            center_list = list(center)
            center_list = adjust_drawing_scale(center_list, 1 / self.scale)  # 调整坐标的比例
            radius *= 1 / self.scale  # 调整圆半径比例
            self.Top_Shape.add_circle(
                center_list, radius, dxfattribs={"layer": layer_name}
            )

    def begin_draw_step_slot(self):
        """
        开始绘制防滑槽
        :return:
        """
        points = self.stair_projection_data.get_step_slot_projection()
        layer_name = "G_Step_Slot"
        self.begin_draw_standard_profile_points(points, layer_name)

    def begin_draw_hoist_embedded_diagram(self):
        """
        绘制吊装预埋件示意图
        :return:
        """
        (
            hoist_points,
            radius_sets,
        ) = self.stair_projection_data.get_hoist_embedded_projection()
        for num in range(len(radius_sets)):
            radius_sets[num] *= 1 / self.scale
        for index in range(len(hoist_points)):
            hoist_points[index] = list(hoist_points[index])
            hoist_points[index] = adjust_drawing_scale(
                hoist_points[index], 1 / self.scale
            )  # 调整坐标点的比例
        for num in range(len(hoist_points)):
            current_point = list(hoist_points[num])
            current_point[2] = 0
            total_length = radius_sets[0] + radius_sets[1]
            point_1 = copy.deepcopy(current_point)
            point_2 = copy.deepcopy(current_point)
            point_3 = copy.deepcopy(current_point)
            point_4 = copy.deepcopy(current_point)
            # 生成吊装预埋件示意图的轴线
            point_1[0] = point_1[0] - total_length
            point_2[0] = point_2[0] + total_length
            point_3[1] = point_3[1] - total_length
            point_4[1] = point_4[1] + total_length
            self.Top_Shape.add_circle(
                tuple(current_point), radius_sets[0], dxfattribs={"layer": "G_Embedded"}
            )
            self.Top_Shape.add_circle(
                tuple(current_point), radius_sets[1], dxfattribs={"layer": "G_Embedded"}
            )
            self.Top_Shape.add_line(
                tuple(point_1),
                tuple(point_2),
                dxfattribs={"layer": "G_Embedded_Subline"},
            )
            self.Top_Shape.add_line(
                tuple(point_3),
                tuple(point_4),
                dxfattribs={"layer": "G_Embedded_Subline"},
            )

    def begin_draw_hole_rein_rebar(self):
        """
        绘制孔洞加强筋投影图
        :return:None
        """
        hole_rein_rebar = self.stair_projection_data.get_hole_rein_rebar_section()
        # 将三维坐标点转化为二维坐标点
        hole_rein_rebar_fact = []
        for num in range(len(hole_rein_rebar)):
            current_u_loc = hole_rein_rebar[num]
            curr_seg = []
            for segment in current_u_loc:
                points_ = []
                for point in segment:
                    points_.append((point[0], point[1], 0))
                curr_seg.append(points_)
            hole_rein_rebar_fact.append(curr_seg)
        for num in range(len(hole_rein_rebar_fact)):
            current_rebar_loc = hole_rein_rebar[num]
            for segment in current_rebar_loc:
                if len(segment) < 3:
                    for k in range(len(segment) - 1):
                        point_1 = list(segment[k])
                        point_2 = list(segment[k + 1])
                        # 缩放比例
                        point_1 = adjust_drawing_scale(point_1, 1 / self.scale)
                        point_2 = adjust_drawing_scale(point_2, 1 / self.scale)
                    self.Rein_rebar.add_line(
                        point_1, point_2, dxfattribs={"layer": "G_Rebar"}
                    )
                else:
                    point_0 = list(segment[0])
                    point_1 = list(segment[-1])
                    # 缩放比例
                    point_0 = adjust_drawing_scale(point_0, 1 / self.scale)
                    point_1 = adjust_drawing_scale(point_1, 1 / self.scale)
                    center = (
                        (point_0[0] + point_1[0]) / 2,
                        (point_0[1] + point_1[1]) / 2,
                        (point_0[2] + point_1[2]) / 2,
                    )
                    radius = abs(point_0[0] - point_1[0]) / 2
                    limit_y = (self.lb_d + self.lt_d + self.ln) / 2 * 1 / self.scale
                    if center[1] > limit_y:  # 顶部圆孔
                        self.Rein_rebar.add_arc(
                            center, radius, 0, 180, dxfattribs={"layer": "G_Rebar"}
                        )
                    else:
                        self.Rein_rebar.add_arc(
                            center, radius, -180, 0, dxfattribs={"layer": "G_Rebar"}
                        )

    def begin_dimension_hole_location(self):
        """
        开始标注孔洞的位置
        :return:
        """

        hole_dimension_info = (
            self.top_view_dimension_data.generate_hole_dimension_points()
        )
        # 标注偏移量
        offset = self.dimension_offset
        floor_loc = hole_dimension_info["floor"]  # 获取标注所在层数
        hole_dimension_loc = hole_dimension_info["location"]  # 获取标注坐标点
        # 通过标注所在层数，确定最终的标注偏移量
        if floor_loc == "first floor":
            offset *= 0.5
        # 调整比例
        for num in range(len(hole_dimension_loc)):
            hole_dimension_loc[num][0] = list(hole_dimension_loc[num][0])
            hole_dimension_loc[num][1] = list(hole_dimension_loc[num][1])
            hole_dimension_loc[num][0] = adjust_drawing_scale(
                hole_dimension_loc[num][0], 1 / self.scale
            )
            hole_dimension_loc[num][1] = adjust_drawing_scale(
                hole_dimension_loc[num][1], 1 / self.scale
            )
            hole_dimension_loc[num][0] = list(hole_dimension_loc[num][0])
            hole_dimension_loc[num][1] = list(hole_dimension_loc[num][1])

        for num in range(len(hole_dimension_loc)):
            current_object = hole_dimension_loc[num]
            start_point = current_object[0]
            end_point = current_object[1]
            start_point[2] = 0
            end_point[2] = 0

            start_point = tuple(start_point)
            end_point = tuple(end_point)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            value = np.linalg.norm(np.array(end_point) - np.array(start_point))
            if value != 0:  # 排除无挑耳的情况
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * (offset) + np.array(list(end_point))
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                    dim = self.Dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.Dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        distance=offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()

    def begin_dimension_solid_stair_location(self):
        """
        开始楼梯实体位置标注
        :return:
        """
        solid_stair_dimension = (
            self.top_view_dimension_data.generate_stair_solid_dimension_points()
        )
        first_dimension = solid_stair_dimension["first floor"]  # 单层尺寸标注位置
        second_dimension = solid_stair_dimension["second floor"]  # 第二层尺寸标注位置
        special_dimension = solid_stair_dimension["special floor"]  # 特殊层尺寸标注位置
        special_dimension_loc = special_dimension[0]  # 特殊尺寸标注位置
        special_dimension_text = special_dimension[1]  # 特殊尺寸标注文本
        # 第一层标注对象
        for segment in first_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(start_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 第二层标注对象
        for segment in second_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 1
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * (current_offset) + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 特殊层标注对象
        for segment in special_dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * (current_offset) + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_dimension_hoist_location(self):
        """
        开始标注吊装预埋件位置尺寸
        :return:
        """
        hoist_dimension = (
            self.top_view_dimension_data.generate_hoist_embedded_dimension_points()
        )  # 产生吊装位置标注点
        for num in range(len(hoist_dimension)):
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            segment = copy.deepcopy(hoist_dimension[num])
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * (current_offset) + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_dimension_step_slot_location(self):
        """
        获取防滑槽的标注位置
        :return:
        """
        step_slot_loc = (
            self.top_view_dimension_data.generate_step_slot_dimension_points()
        )  # 产生防滑槽坐标点
        for num in range(len(step_slot_loc)):
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.1
            segment = copy.deepcopy(step_slot_loc[num])
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * (current_offset) + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_hole_type_notes(self):
        """
        孔洞类型说明
        :return:
        """
        (
            hole_text_loc,
            top_hole_text,
            bottom_hole_text,
        ) = self.top_view_dimension_data.generate_hole_dimension_text_points()
        top_text_loc = hole_text_loc[0]  # 顶部文本位置
        bottom_text_loc = hole_text_loc[1]  # 底部文本位置
        current_offset_1 = self.dimension_offset  # 偏移量
        current_offset_2 = 1 * self.dimension_offset  # 偏移量
        for num in range(len(top_text_loc)):
            current_point = top_text_loc[num]
            top_text_loc[num] = list(
                adjust_drawing_scale(current_point, 1 / self.scale)
            )
        for num in range(len(bottom_text_loc)):
            current_point = bottom_text_loc[num]
            bottom_text_loc[num] = list(
                adjust_drawing_scale(current_point, 1 / self.scale)
            )
        top_mid_loc = copy.deepcopy(top_text_loc[0])
        top_max_x = max(top_text_loc[0][0], top_text_loc[1][0])
        top_mid_loc[0] = top_max_x
        bottom_mid_loc = copy.deepcopy(bottom_text_loc[0])
        bottom_max_x = max(bottom_text_loc[0][0], bottom_text_loc[1][0])
        bottom_mid_loc[0] = bottom_max_x
        # 顶部文字开始处
        top_mid_loc[1] += current_offset_1
        # 顶部文字结束处
        top_end_loc = copy.deepcopy(top_mid_loc)
        top_end_loc[1] += current_offset_2
        # 底部文字开始处
        bottom_mid_loc[1] -= current_offset_1
        # 底部文字结束处
        bottom_end_loc = copy.deepcopy(bottom_mid_loc)
        bottom_end_loc[1] -= current_offset_2
        # 旋转点
        theta = math.pi / 2  # 逆时针旋转角度--弧度制
        rotate_axis = np.asarray([0, 0, 1])  # 旋转轴
        # 对顶部孔洞文本标注进行旋转
        for num in range(len(top_text_loc)):
            current_point = copy.deepcopy(top_text_loc[num])
            top_text_loc[num] = rotation_3d(
                np.array(current_point), rotate_axis, theta
            ).tolist()
        # 对底部文本标注进行旋转
        for num in range(len(bottom_text_loc)):
            current_point = copy.deepcopy(bottom_text_loc[num])
            bottom_text_loc[num] = rotation_3d(
                np.array(current_point), rotate_axis, theta
            ).tolist()
        # 顶部文字开始处进行旋转
        top_mid_loc = rotation_3d(np.array(top_mid_loc), rotate_axis, theta).tolist()
        # 顶部文字结束处进行旋转
        top_end_loc = rotation_3d(np.array(top_end_loc), rotate_axis, theta).tolist()
        # 底部文字开始处进行旋转
        bottom_mid_loc = rotation_3d(
            np.array(bottom_mid_loc), rotate_axis, theta
        ).tolist()
        # 底部文字结束处进行旋转
        bottom_end_loc = rotation_3d(
            np.array(bottom_end_loc), rotate_axis, theta
        ).tolist()
        # 所有要标注的点
        dimension_text_loc = [
            [top_text_loc[0], top_mid_loc],
            [top_text_loc[1], top_mid_loc],
            [top_mid_loc, top_end_loc],
            [bottom_text_loc[0], bottom_mid_loc],
            [bottom_text_loc[1], bottom_mid_loc],
            [bottom_mid_loc, bottom_end_loc],
        ]
        # 绘制直线
        for num in range(len(dimension_text_loc)):
            current_segment = copy.deepcopy(dimension_text_loc[num])
            start_point = tuple(current_segment[0])
            end_point = tuple(current_segment[1])
            self.Text_Dim.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": "G_Label"}
            )
        # 添加文本
        # 添加旋转后左侧文本
        top_mid_loc_t = tuple(top_mid_loc)
        self.Text_Dim.add_text(
            top_hole_text, height=self.note_text_size, dxfattribs={"layer": "G_Note"}
        ).set_placement(top_mid_loc_t, align=TextEntityAlignment.BOTTOM_RIGHT)
        # 添加旋转后右侧文本
        bottom_mid_loc_t = tuple(bottom_mid_loc)
        self.Text_Dim.add_text(
            bottom_hole_text, height=self.note_text_size, dxfattribs={"layer": "G_Note"}
        ).set_placement(bottom_mid_loc_t, align=TextEntityAlignment.BOTTOM_LEFT)

    def main_run_process(self):
        """
        主要运行过程，生成dxf文件
        :return:
        """
        self.begin_draw_polyline()  # 开始绘制多段线
        self.begin_draw_circle()  # 绘制圆
        self.begin_draw_step_slot()  # 开始绘制防滑槽
        self.begin_draw_hoist_embedded_diagram()  # 开始绘制吊装预埋件投影图
        self.begin_draw_hole_rein_rebar()  # 开始绘制孔洞加强钢筋的剖切图
        self.begin_dimension_hole_location()  # 孔洞位置标注
        self.begin_dimension_solid_stair_location()  # 楼梯实体对象标注
        self.begin_dimension_hoist_location()  # 开始标注吊装预埋件位置
        self.begin_dimension_step_slot_location()  # 开始标注防滑槽位置
        self.begin_hole_type_notes()  # 开始孔洞说明文档定位


class StairBottomView(object):
    """
    生成楼梯dxf仰视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_projection_data = StairBottomViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.bottom_view_dimension_data = BottomViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.rotate_angle = math.pi / 2  # 图形旋转角度，初始方向和最终放置方向旋转的角度
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        创建dxf文件
        :return:
        """
        self.bottom_view = self.dxf_doc
        self.model_space = self.bottom_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        """
        self.Bottom_Shape = self.bottom_view.blocks.new("bottom_view")  # 添加视图块
        self.Rein_rebar = self.bottom_view.blocks.new(
            "bottom_view_rein_rebar"
        )  # 添加加强筋块
        self.Hole_dimension = self.bottom_view.blocks.new(
            "bottom_view_hole_dim"
        )  # 添加孔洞位置块
        self.Text_Dim = self.bottom_view.blocks.new(
            "bottom_view_text_dim"
        )  # 添加孔洞说明文本标注块

    def begin_draw_standard_profile_points(self, profile_points, layer: str):
        """
        开始绘制轮廓点
        :param profile_points:轮廓点
        :param layer: 图层
        :return:
        """
        for num in range(len(profile_points)):
            profile_points[num][0] = list(profile_points[num][0])
            profile_points[num][1] = list(profile_points[num][1])
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.Bottom_Shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_polyline(self):
        """
        开始绘制楼梯主体多段线
        :return:
        """
        points = (
            self.stair_projection_data.get_stair_solid_projection()
        )  # [[(),()],[(),()]]
        layer_name = "G_Outline"
        self.begin_draw_standard_profile_points(points, layer_name)

    def begin_draw_circle(self):
        """
        开始绘制圆
        :return:
        """
        hole_points = self.stair_projection_data.get_hole_projection()  # 获取孔洞投影数据
        layer_name = "G_Outline"
        for num in range(len(hole_points)):
            single_hole = hole_points[num]
            center, radius = get_circle_radius_and_center(single_hole)
            center_list = list(center)
            center_list = adjust_drawing_scale(center_list, 1 / self.scale)  # 调整坐标的比例
            radius *= 1 / self.scale  # 调整圆半径比例
            self.Bottom_Shape.add_circle(
                center_list, radius, dxfattribs={"layer": layer_name}
            )

    def begin_draw_hole_rein_rebar(self):
        """
        绘制孔洞加强筋投影图
        :return:None
        """
        hole_rein_rebar = self.stair_projection_data.get_hole_rein_rebar_section()
        # 将三维坐标点转化为二维坐标点
        hole_rein_rebar_fact = []
        for num in range(len(hole_rein_rebar)):
            current_u_loc = hole_rein_rebar[num]
            curr_seg = []
            for segment in current_u_loc:
                points_ = []
                for point in segment:
                    points_.append((point[0], point[1], 0))
                curr_seg.append(points_)
            hole_rein_rebar_fact.append(curr_seg)
        for num in range(len(hole_rein_rebar_fact)):
            current_rebar_loc = hole_rein_rebar[num]
            for segment in current_rebar_loc:
                if len(segment) < 3:
                    for k in range(len(segment) - 1):
                        point_1 = list(segment[k])
                        point_2 = list(segment[k + 1])
                        # 缩放比例
                        point_1 = adjust_drawing_scale(point_1, 1 / self.scale)
                        point_2 = adjust_drawing_scale(point_2, 1 / self.scale)
                    self.Rein_rebar.add_line(
                        point_1, point_2, dxfattribs={"layer": "G_Rebar"}
                    )
                else:
                    point_0 = list(segment[0])
                    point_1 = list(segment[-1])
                    # 缩放比例
                    point_0 = adjust_drawing_scale(point_0, 1 / self.scale)
                    point_1 = adjust_drawing_scale(point_1, 1 / self.scale)
                    center = (
                        (point_0[0] + point_1[0]) / 2,
                        (point_0[1] + point_1[1]) / 2,
                        (point_0[2] + point_1[2]) / 2,
                    )
                    radius = abs(point_0[0] - point_1[0]) / 2
                    limit_y = (self.lb_d + self.lt_d + self.ln) / 2 * 1 / self.scale
                    if center[1] > limit_y:  # 顶部圆孔
                        self.Rein_rebar.add_arc(
                            center, radius, 0, 180, dxfattribs={"layer": "G_Rebar"}
                        )
                    else:
                        self.Rein_rebar.add_arc(
                            center, radius, -180, 0, dxfattribs={"layer": "G_Rebar"}
                        )

    def begin_dimension_hole_location(self):
        """
        开始标注孔洞的位置
        :return:
        """

        hole_dimension_info = (
            self.bottom_view_dimension_data.generate_hole_dimension_points()
        )
        # 标注偏移量
        offset = self.dimension_offset
        floor_loc = hole_dimension_info["floor"]  # 获取标注所在层数
        hole_dimension_loc = hole_dimension_info["location"]  # 获取标注坐标点
        # 通过标注所在层数，确定最终的标注偏移量
        if floor_loc == "first floor":
            offset *= 0.5
        # 调整比例
        for num in range(len(hole_dimension_loc)):
            hole_dimension_loc[num][0] = list(hole_dimension_loc[num][0])
            hole_dimension_loc[num][1] = list(hole_dimension_loc[num][1])
            hole_dimension_loc[num][0] = adjust_drawing_scale(
                hole_dimension_loc[num][0], 1 / self.scale
            )
            hole_dimension_loc[num][1] = adjust_drawing_scale(
                hole_dimension_loc[num][1], 1 / self.scale
            )
            hole_dimension_loc[num][0] = list(hole_dimension_loc[num][0])
            hole_dimension_loc[num][1] = list(hole_dimension_loc[num][1])
        # 开始绘制图形
        for num in range(len(hole_dimension_loc)):
            current_object = hole_dimension_loc[num]
            start_point = current_object[0]
            end_point = current_object[1]
            start_point[2] = 0
            end_point[2] = 0
            value = np.linalg.norm(np.array(end_point) - np.array(start_point))
            start_point = tuple(start_point)
            end_point = tuple(end_point)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if value != 0:  # 排除无挑耳的情况
                if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                    dim = self.Hole_dimension.add_linear_dim(
                        base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                    )
                else:
                    dim = self.Hole_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        distance=offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()

    def begin_dimension_solid_stair_location(self):
        """
        开始楼梯实体位置标注
        :return:
        """
        solid_stair_dimension = (
            self.bottom_view_dimension_data.generate_stair_solid_dimension_points()
        )
        first_dimension = solid_stair_dimension["first floor"]  # 单层尺寸标注位置
        second_dimension = solid_stair_dimension["second floor"]  # 第二层尺寸标注位置
        # 第一层标注对象
        for segment in first_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Hole_dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",  # 默认是1:100进行绘制，输入的数值是绘制的大小，实际大小需要乘以绘图比例。
                )
            else:
                dim = self.Hole_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 第二层标注对象
        for segment in second_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 1
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Hole_dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Hole_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",  # 按照1：100进行绘制
                )
            dim.render()

    def begin_hole_type_notes(self):
        """
        孔洞类型说明
        :return:
        """
        (
            hole_text_loc,
            top_hole_text,
            bottom_hole_text,
        ) = self.bottom_view_dimension_data.generate_hole_dimension_text_points()
        top_text_loc = hole_text_loc[0]  # 顶部文本位置
        bottom_text_loc = hole_text_loc[1]  # 底部文本位置
        current_offset_1 = -self.dimension_offset  # 偏移量
        current_offset_2 = -1 * self.dimension_offset  # 偏移量
        for num in range(len(top_text_loc)):
            current_point = top_text_loc[num]
            top_text_loc[num] = list(
                adjust_drawing_scale(current_point, 1 / self.scale)
            )
        for num in range(len(bottom_text_loc)):
            current_point = bottom_text_loc[num]
            bottom_text_loc[num] = list(
                adjust_drawing_scale(current_point, 1 / self.scale)
            )
        top_mid_loc = copy.deepcopy(top_text_loc[0])
        top_mid_loc[0] = (top_text_loc[0][0] + top_text_loc[1][0]) / 2
        bottom_mid_loc = copy.deepcopy(bottom_text_loc[0])
        bottom_mid_loc[0] = (bottom_text_loc[0][0] + bottom_text_loc[1][0]) / 2
        # 顶部文字开始处
        top_mid_loc[1] += current_offset_1
        # 顶部文字结束处
        top_end_loc = copy.deepcopy(top_mid_loc)
        top_end_loc[1] += current_offset_2
        # 底部文字开始处
        bottom_mid_loc[1] -= current_offset_1
        # 底部文字结束处
        bottom_end_loc = copy.deepcopy(bottom_mid_loc)
        bottom_end_loc[1] -= current_offset_2
        # 旋转点
        theta = math.pi / 2  # 逆时针旋转角度--弧度制
        rotate_axis = np.asarray([0, 0, 1])  # 旋转轴
        # 对顶部孔洞文本标注进行旋转
        for num in range(len(top_text_loc)):
            current_point = copy.deepcopy(top_text_loc[num])
            top_text_loc[num] = rotation_3d(
                np.array(current_point), rotate_axis, theta
            ).tolist()
        # 对底部文本标注进行旋转
        for num in range(len(bottom_text_loc)):
            current_point = copy.deepcopy(bottom_text_loc[num])
            bottom_text_loc[num] = rotation_3d(
                np.array(current_point), rotate_axis, theta
            ).tolist()
        # 顶部文字开始处进行旋转
        top_mid_loc = rotation_3d(np.array(top_mid_loc), rotate_axis, theta).tolist()
        # 顶部文字结束处进行旋转
        top_end_loc = rotation_3d(np.array(top_end_loc), rotate_axis, theta).tolist()
        # 底部文字开始处进行旋转
        bottom_mid_loc = rotation_3d(
            np.array(bottom_mid_loc), rotate_axis, theta
        ).tolist()
        # 底部文字结束处进行旋转
        bottom_end_loc = rotation_3d(
            np.array(bottom_end_loc), rotate_axis, theta
        ).tolist()
        # 所有要标注的点
        dimension_text_loc = [
            [top_text_loc[0], top_mid_loc],
            [top_text_loc[1], top_mid_loc],
            [top_mid_loc, top_end_loc],
            [bottom_text_loc[0], bottom_mid_loc],
            [bottom_text_loc[1], bottom_mid_loc],
            [bottom_mid_loc, bottom_end_loc],
        ]

        # 绘制直线
        for num in range(len(dimension_text_loc)):
            current_segment = copy.deepcopy(dimension_text_loc[num])
            start_point = tuple(current_segment[0])
            end_point = tuple(current_segment[1])
            self.Text_Dim.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": "G_Label"}
            )
        # 添加文本
        # 添加旋转后左侧文本
        top_mid_loc_t = tuple(top_mid_loc)
        self.Text_Dim.add_text(
            top_hole_text, height=self.note_text_size, dxfattribs={"layer": "G_Label"}
        ).set_placement(top_mid_loc_t, align=TextEntityAlignment.BOTTOM_LEFT)
        # 添加旋转后右侧文本
        bottom_mid_loc_t = tuple(bottom_mid_loc)
        self.Text_Dim.add_text(
            bottom_hole_text,
            height=self.note_text_size,
            dxfattribs={"layer": "G_Label"},
        ).set_placement(bottom_mid_loc_t, align=TextEntityAlignment.BOTTOM_RIGHT)

    def main_run_process(self):
        """
        开始主要的运行过程
        :return:
        """
        self.begin_draw_polyline()
        self.begin_draw_circle()
        self.begin_draw_hole_rein_rebar()
        self.begin_dimension_hole_location()
        self.begin_dimension_solid_stair_location()
        self.begin_hole_type_notes()


class StairLeftView(object):
    """
    生成楼梯dxf侧视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_projection_data = StairLeftViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.left_view_dimension_data = LeftViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.rotate_angle = math.pi / 2  # 图形旋转角度，初始方向和最终放置方向旋转的角度
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.left_view = self.dxf_doc
        self.model_space = self.left_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        """
        self.left_shape = self.left_view.blocks.new("left_view")  # 添加视图块
        self.Rein_rebar = self.left_view.blocks.new("left_view_rein_rebar")  # 添加加强筋块
        self.hole_shape = self.left_view.blocks.new("left_view_hole_profile")  # 添加孔洞轮廓线
        self.Dimension = self.left_view.blocks.new("left_view_dimension")  # 添加孔洞位置块

    def begin_draw_standard_profile_points(self, profile_points, layer: str):
        """
        开始绘制轮廓点
        :param profile_points:轮廓点
        :param layer: 图层
        :return:
        """
        # 变换绘制比例
        for num in range(len(profile_points)):
            if len(profile_points[num]) <= 2:
                profile_points[num][0] = transform_point_from_yz_to_xy(
                    list(profile_points[num][0])
                )  # 起点
                profile_points[num][1] = transform_point_from_yz_to_xy(
                    list(profile_points[num][1])
                )  # 终点
                profile_points[num][0] = adjust_drawing_scale(
                    profile_points[num][0], 1 / self.scale
                )
                profile_points[num][1] = adjust_drawing_scale(
                    profile_points[num][1], 1 / self.scale
                )
            else:  # 多线段组成的圆弧
                for k in range(len(profile_points[num])):
                    current_point = transform_point_from_yz_to_xy(
                        list(profile_points[num][k])
                    )  # 当前点
                    profile_points[num][k] = adjust_drawing_scale(
                        current_point, 1 / self.scale
                    )
        for segment in profile_points:
            if len(segment) <= 2:
                seg = [
                    (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                    (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
                ]  # 将三维点转化为平面点
                self.left_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})
            else:  # 线段点数超过2，形成圆弧
                for k in range(len(segment) - 1):
                    start_p = segment[k]  # 线段起点
                    end_p = segment[k + 1]  # 线段终点
                    seg = [
                        (abs(start_p[0]), abs(start_p[1]), abs(start_p[2])),
                        (abs(end_p[0]), abs(end_p[1]), abs(end_p[2])),
                    ]  # 将三维点转化为平面点
                    self.left_shape.add_line(
                        seg[0], seg[1], dxfattribs={"layer": layer}
                    )

    def begin_draw_polyline(self):
        """
        开始绘制楼梯主体多段线
        :return:
        """
        points = (
            self.stair_projection_data.get_stair_solid_left_cut_drawing()
        )  # [[(),()],[(),()]]  # 获取切割模型数据
        # 变换绘制比例
        layer_name = "G_Outline"
        self.begin_draw_standard_profile_points(points, layer_name)

    def begin_draw_hole_rein_rebar(self):
        """
        绘制孔洞加强筋投影图
        :return:None
        """
        hole_rein_rebar_loc = (
            self.stair_projection_data.get_stair_hole_rein_rebar_projection()
        )  # 获取孔洞加强筋数据
        # 图层
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(hole_rein_rebar_loc)):
            for k in range(len(hole_rein_rebar_loc[num])):
                hole_rein_rebar_loc[num][k][0] = transform_point_from_yz_to_xy(
                    list(hole_rein_rebar_loc[num][k][0])
                )  # 起点
                hole_rein_rebar_loc[num][k][1] = transform_point_from_yz_to_xy(
                    list(hole_rein_rebar_loc[num][k][1])
                )  # 终点
                hole_rein_rebar_loc[num][k][0] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][0], 1 / self.scale
                )
                hole_rein_rebar_loc[num][k][1] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][1], 1 / self.scale
                )
        for rebar in hole_rein_rebar_loc:
            for k in range(len(rebar)):
                segment = rebar[k]  # 钢筋当前段
                seg = [
                    (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                    (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
                ]  # 将三维点转化为平面点
                self.left_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_hoist_embedded_shape(self):
        """
        开始绘制吊装预埋件
        :return:
        """
        hoist_data = self.stair_projection_data.get_hoist_embedded_cut_shape()
        layer_name = "G_Embedded"
        # 变换绘制比例
        for num in range(len(hoist_data)):
            hoist_data[num][0] = transform_point_from_yz_to_xy(
                list(hoist_data[num][0])
            )  # 起点
            hoist_data[num][1] = transform_point_from_yz_to_xy(
                list(hoist_data[num][1])
            )  # 终点
            hoist_data[num][0] = adjust_drawing_scale(
                hoist_data[num][0], 1 / self.scale
            )
            hoist_data[num][1] = adjust_drawing_scale(
                hoist_data[num][1], 1 / self.scale
            )
        for segment in hoist_data:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.left_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_draw_hole_shape(self):
        """
        开始绘制孔洞形状
        :return:
        """
        hole_data = (
            self.stair_projection_data.get_stair_hole_left_cut_drawing()
        )  # 获取孔洞数据
        layer_name = "G_Hole_Outline"
        # 变换绘制比例
        for num in range(len(hole_data)):
            hole_data[num][0] = transform_point_from_yz_to_xy(
                list(hole_data[num][0])
            )  # 起点
            hole_data[num][1] = transform_point_from_yz_to_xy(
                list(hole_data[num][1])
            )  # 终点
            hole_data[num][0] = adjust_drawing_scale(hole_data[num][0], 1 / self.scale)
            hole_data[num][1] = adjust_drawing_scale(hole_data[num][1], 1 / self.scale)
        for segment in hole_data:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.left_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_dimension_solid_stair_location(self):
        """
        开始楼梯实体位置标注
        :return:
        """
        dimension_loc = (
            self.left_view_dimension_data.generate_stair_solid_dimension_standard_point()
        )  # 获取实体标注点
        # 第一层标注对象
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 旋转角度
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 获取楼梯斜向标注点
        special_loc = (
            self.left_view_dimension_data.generate_stair_solid_dimension_xie_special_point()
        )
        for segment in special_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 旋转角度
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            dim = self.Dimension.add_aligned_dim(
                p1=start_point,
                p2=end_point,
                distance=-current_offset,  # 负数代表反方向
                dimstyle="G_DS_20",  # 按照1：100进行绘制
            )
            dim.render()

    def begin_dimension_hole_location(self):
        """
        开始标注孔洞的位置
        :return:
        """
        hole_loc = self.left_view_dimension_data.get_hole_dimension_point()  # 孔洞标注
        # 第一层标注对象
        for segment in hole_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 旋转角度
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_dimension_hoist_location(self):
        """
        标注吊装预埋件位置
        :return:
        """
        hoist_loc = self.left_view_dimension_data.get_hoist_dimension_point()
        # 第一层标注对象
        for segment in hoist_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 旋转角度
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_dimension_stair_thickness_location(self):
        """
        开始标注楼梯厚度信息
        :return:
        """
        thick_loc = (
            self.left_view_dimension_data.get_stair_dimension_thickness_point()
        )  # 楼梯厚度标注点
        # 第一层标注对象
        # 调整该层标注相对实体偏移量
        current_offset = self.dimension_offset * 0.5
        start_point = transform_point_from_yz_to_xy(list(thick_loc[0]))
        end_point = transform_point_from_yz_to_xy(list(thick_loc[1]))
        # 调整点的比例
        start_point = adjust_drawing_scale(start_point, 1 / self.scale)
        end_point = adjust_drawing_scale(end_point, 1 / self.scale)
        # 旋转角度
        start_point = tuple(
            rotation_3d(
                np.asarray(list(start_point)), np.asarray([0, 0, 1]), self.rotate_angle
            )
        )
        end_point = tuple(
            rotation_3d(
                np.asarray(list(end_point)), np.asarray([0, 0, 1]), self.rotate_angle
            )
        )
        # 对尺寸数字反向内容进行特殊操作
        direction = calculate_normal_vector(start_point, end_point, 0)
        base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
        base_p = tuple(base_p_)  # 对齐点
        if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
            dim = self.Dimension.add_linear_dim(
                base=base_p,
                p1=start_point,
                p2=end_point,
                dimstyle="G_DS_20",
            )
        else:
            dim = self.Dimension.add_aligned_dim(
                p1=end_point,
                p2=start_point,
                distance=current_offset,
                dimstyle="G_DS_20",
            )
        dim.render()

    def begin_dimension_special_type_location(self):
        """
        特殊类型说明
        :return:
        """
        special_info = self.left_view_dimension_data.get_special_dimension_text()

        # 特殊层标注对象
        for num in range(len(special_info)):
            # 调整该层标注相对实体偏移量
            segment = special_info[num][0]
            current_text = special_info[num][1]
            current_offset = self.dimension_offset * 0.5
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 旋转角度
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.Dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",  # 默认是1:100进行绘制，输入的数值是绘制的大小，实际大小需要乘以绘图比例。
                )
            else:
                dim = self.Dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def main_run_process(self):
        """
        开始主要的运行过程
        :return:
        """
        self.begin_draw_polyline()
        self.begin_draw_hole_shape()
        self.begin_draw_hole_rein_rebar()
        self.begin_draw_hoist_embedded_shape()
        self.begin_dimension_solid_stair_location()
        self.begin_dimension_hole_location()
        self.begin_dimension_hoist_location()
        self.begin_dimension_stair_thickness_location()
        self.begin_dimension_special_type_location()


class StairReinforcementView(object):
    """
    生成楼梯dxf配筋图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_reinforce_data = StairReinforceViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯配筋数据
        self.reinforce_view_dimension_data = ReinforceViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.rein_view = self.dxf_doc
        self.model_space = self.rein_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.stair_shape = self.rein_view.blocks.new("stair_reinforce_view")  # 添加轮廓块
        self.rebar_shape = self.rein_view.blocks.new("stair_reinforce_rebar")  # 添加钢筋块
        self.embedded_shape = self.rein_view.blocks.new(
            "stair_reinforce_embedded"
        )  # 添加预埋件块
        self.dimension = self.rein_view.blocks.new("stair_reinforce_dimension")  # 添加标注块
        self.cut_signal = self.rein_view.blocks.new(
            "stair_reinforce_cut_signal"
        )  # 添加剖切符块
        self.outline_shape = self.rein_view.blocks.new(
            "stair_reinforce_outline"
        )  # 添加引出线块

    def begin_draw_polyline(self):
        """
        开始绘制楼梯主体多段线
        :return:
        """
        points = (
            self.stair_reinforce_data.get_stair_solid_profile_cut_drawing()
        )  # [[(),()],[(),()]]  # 获取切割模型数据
        layer_name = "G_Outline"
        # 变换绘制比例
        for num in range(len(points)):
            if len(points[num]) <= 2:
                points[num][0] = transform_point_from_yz_to_xy(
                    list(points[num][0])
                )  # 起点
                points[num][1] = transform_point_from_yz_to_xy(
                    list(points[num][1])
                )  # 终点
                points[num][0] = adjust_drawing_scale(points[num][0], 1 / self.scale)
                points[num][1] = adjust_drawing_scale(points[num][1], 1 / self.scale)
            else:  # 多线段组成的圆弧
                for k in range(len(points[num])):
                    current_point = transform_point_from_yz_to_xy(
                        list(points[num][k])
                    )  # 当前点
                    points[num][k] = adjust_drawing_scale(current_point, 1 / self.scale)
        for segment in points:
            if len(segment) <= 2:
                seg = [
                    (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                    (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
                ]  # 将三维点转化为平面点
                self.stair_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )
            else:  # 线段点数超过2，形成圆弧
                for k in range(len(segment) - 1):
                    start_p = segment[k]  # 线段起点
                    end_p = segment[k + 1]  # 线段终点
                    seg = [
                        (abs(start_p[0]), abs(start_p[1]), abs(start_p[2])),
                        (abs(end_p[0]), abs(end_p[1]), abs(end_p[2])),
                    ]  # 将三维点转化为平面点
                    self.stair_shape.add_line(
                        seg[0], seg[1], dxfattribs={"layer": layer_name}
                    )

    def begin_draw_hole_rein_rebar(self):
        """
        绘制孔洞加强筋投影图
        :return:None
        """
        hole_rein_rebar_loc = (
            self.stair_reinforce_data.get_stair_hole_rein_rebar_projection()
        )  # 获取孔洞加强筋数据
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(hole_rein_rebar_loc)):
            for k in range(len(hole_rein_rebar_loc[num])):
                hole_rein_rebar_loc[num][k][0] = transform_point_from_yz_to_xy(
                    list(hole_rein_rebar_loc[num][k][0])
                )  # 起点
                hole_rein_rebar_loc[num][k][1] = transform_point_from_yz_to_xy(
                    list(hole_rein_rebar_loc[num][k][1])
                )  # 终点
                hole_rein_rebar_loc[num][k][0] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][0], 1 / self.scale
                )
                hole_rein_rebar_loc[num][k][1] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][1], 1 / self.scale
                )
        for rebar in hole_rein_rebar_loc:
            for k in range(len(rebar)):
                segment = rebar[k]  # 钢筋当前段
                seg = [
                    (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                    (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_hoist_embedded_shape(self):
        """
        开始绘制吊装预埋件
        :return:
        """
        hoist_data = self.stair_reinforce_data.get_hoist_embedded_cut_shape()
        layer_name = "G_Embedded"
        # 变换绘制比例
        for num in range(len(hoist_data)):
            hoist_data[num][0] = transform_point_from_yz_to_xy(
                list(hoist_data[num][0])
            )  # 起点
            hoist_data[num][1] = transform_point_from_yz_to_xy(
                list(hoist_data[num][1])
            )  # 终点
            hoist_data[num][0] = adjust_drawing_scale(
                hoist_data[num][0], 1 / self.scale
            )
            hoist_data[num][1] = adjust_drawing_scale(
                hoist_data[num][1], 1 / self.scale
            )
        for segment in hoist_data:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.embedded_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_hole_shape(self):
        """
        开始绘制孔洞形状
        :return:
        """
        hole_data = (
            self.stair_reinforce_data.get_stair_hole_profile_cut_drawing()
        )  # 获取孔洞数据
        layer_name = "G_Hole_Outline"
        # 变换绘制比例
        for num in range(len(hole_data)):
            hole_data[num][0] = transform_point_from_yz_to_xy(
                list(hole_data[num][0])
            )  # 起点
            hole_data[num][1] = transform_point_from_yz_to_xy(
                list(hole_data[num][1])
            )  # 终点
            hole_data[num][0] = adjust_drawing_scale(hole_data[num][0], 1 / self.scale)
            hole_data[num][1] = adjust_drawing_scale(hole_data[num][1], 1 / self.scale)
        for segment in hole_data:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.stair_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_draw_bottom_long_rebar_shape(self):
        """
        开始绘制底部纵筋钢筋形状
        :return:
        """
        bottom_rebar_data = self.stair_reinforce_data.get_bottom_long_rebar_cut_shape()
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(bottom_rebar_data)):
            for k in range(len(bottom_rebar_data[num])):
                bottom_rebar_data[num][k] = transform_point_from_yz_to_xy(
                    list(bottom_rebar_data[num][k])
                )  # 终点
                bottom_rebar_data[num][k] = adjust_drawing_scale(
                    bottom_rebar_data[num][k], 1 / self.scale
                )
        for num in range(len(bottom_rebar_data)):
            current_rebar = bottom_rebar_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_top_long_rebar_shape(self):
        """
        开始绘制底部纵筋钢筋形状
        :return:
        """
        top_rebar_data = self.stair_reinforce_data.get_top_long_rebar_cut_shape()
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(top_rebar_data)):
            for k in range(len(top_rebar_data[num])):
                top_rebar_data[num][k] = transform_point_from_yz_to_xy(
                    list(top_rebar_data[num][k])
                )  # 终点
                top_rebar_data[num][k] = adjust_drawing_scale(
                    top_rebar_data[num][k], 1 / self.scale
                )
        for num in range(len(top_rebar_data)):
            current_rebar = top_rebar_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_mid_distribute_rebar_shape(self):
        """
        开始绘制中部分布筋形状
        :return:
        """
        mid_distribute_data = (
            self.stair_reinforce_data.get_mid_distribute_rebar_cut_shape()
        )
        layer_name = "G_Rebar"
        # 变换比例
        for num in range(len(mid_distribute_data)):
            for k in range(len(mid_distribute_data[num])):
                mid_distribute_data[num][k] = transform_point_from_yz_to_xy(
                    list(mid_distribute_data[num][k])
                )  # 终点
                mid_distribute_data[num][k] = adjust_drawing_scale(
                    mid_distribute_data[num][k], 1 / self.scale
                )
        for num in range(len(mid_distribute_data)):
            current_rebar = mid_distribute_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_bottom_edge_long_rebar_shape(self):
        """
        开始绘制底部边缘纵筋
        :return:
        """
        bottom_edge_long_rebar_data = (
            self.stair_reinforce_data.get_bottom_edge_long_rebar_cut_shape()
        )
        layer_name = "G_Rebar"
        # 变换比例
        for num in range(len(bottom_edge_long_rebar_data)):
            for k in range(len(bottom_edge_long_rebar_data[num])):
                bottom_edge_long_rebar_data[num][k] = transform_point_from_yz_to_xy(
                    list(bottom_edge_long_rebar_data[num][k])
                )  # 终点
                bottom_edge_long_rebar_data[num][k] = adjust_drawing_scale(
                    bottom_edge_long_rebar_data[num][k], 1 / self.scale
                )
        for num in range(len(bottom_edge_long_rebar_data)):
            current_rebar = bottom_edge_long_rebar_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_top_edge_long_rebar_shape(self):
        """
        开始绘制顶部边缘纵筋
        :return:
        """
        top_edge_long_rebar_data = (
            self.stair_reinforce_data.get_top_edge_long_rebar_cut_shape()
        )
        layer_name = "G_Rebar"
        for num in range(len(top_edge_long_rebar_data)):
            for k in range(len(top_edge_long_rebar_data[num])):
                top_edge_long_rebar_data[num][k] = transform_point_from_yz_to_xy(
                    list(top_edge_long_rebar_data[num][k])
                )  # 终点
                top_edge_long_rebar_data[num][k] = adjust_drawing_scale(
                    top_edge_long_rebar_data[num][k], 1 / self.scale
                )
        for num in range(len(top_edge_long_rebar_data)):
            current_rebar = top_edge_long_rebar_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def get_bottom_edge_stir_shape(self):
        """
        获取底部边缘箍筋形状
        :return:
        """
        bottom_edge_stir_data = (
            self.stair_reinforce_data.get_bottom_edge_stir_cut_shape()
        )
        layer_name = "G_Rebar"
        for num in range(len(bottom_edge_stir_data)):
            for k in range(len(bottom_edge_stir_data[num])):
                bottom_edge_stir_data[num][k] = transform_point_from_yz_to_xy(
                    list(bottom_edge_stir_data[num][k])
                )  # 终点
                bottom_edge_stir_data[num][k] = adjust_drawing_scale(
                    bottom_edge_stir_data[num][k], 1 / self.scale
                )
        for num in range(len(bottom_edge_stir_data)):
            current_rebar = bottom_edge_stir_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def get_top_edge_stir_shape(self):
        """
        获取顶部边缘箍筋形状
        :return:
        """
        top_edge_stir_data = self.stair_reinforce_data.get_top_edge_stir_cut_shape()
        layer_name = "G_Rebar"
        for num in range(len(top_edge_stir_data)):
            for k in range(len(top_edge_stir_data[num])):
                top_edge_stir_data[num][k] = transform_point_from_yz_to_xy(
                    list(top_edge_stir_data[num][k])
                )  # 终点
                top_edge_stir_data[num][k] = adjust_drawing_scale(
                    top_edge_stir_data[num][k], 1 / self.scale
                )
        for num in range(len(top_edge_stir_data)):
            current_rebar = top_edge_stir_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def get_hoist_rein_long_rebar_shape(self):
        """
        获取吊点加强纵筋形状
        :return:
        """
        hoist_rein_long_rebar_data = (
            self.stair_reinforce_data.get_hoist_rein_long_rebar_cut_shape()
        )
        layer_name = "G_Rebar"
        for num in range(len(hoist_rein_long_rebar_data)):
            for k in range(len(hoist_rein_long_rebar_data[num])):
                hoist_rein_long_rebar_data[num][k] = transform_point_from_yz_to_xy(
                    list(hoist_rein_long_rebar_data[num][k])
                )  # 终点
                hoist_rein_long_rebar_data[num][k] = adjust_drawing_scale(
                    hoist_rein_long_rebar_data[num][k], 1 / self.scale
                )
        for num in range(len(hoist_rein_long_rebar_data)):
            current_rebar = hoist_rein_long_rebar_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def get_hoist_rein_point_rebar_shape(self):
        """
        吊装加强点筋形状
        :return:
        """
        hoist_rein_point_rebar_data = (
            self.stair_reinforce_data.get_hoist_rein_point_rebar_cut_shape()
        )
        layer_name = "G_Rebar"
        for num in range(len(hoist_rein_point_rebar_data)):
            for k in range(len(hoist_rein_point_rebar_data[num])):
                hoist_rein_point_rebar_data[num][k] = transform_point_from_yz_to_xy(
                    list(hoist_rein_point_rebar_data[num][k])
                )  # 终点
                hoist_rein_point_rebar_data[num][k] = adjust_drawing_scale(
                    hoist_rein_point_rebar_data[num][k], 1 / self.scale
                )
        for num in range(len(hoist_rein_point_rebar_data)):
            current_rebar = hoist_rein_point_rebar_data[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def get_stair_solid_top_dimension_location(self):
        """
        开始楼梯实体顶部标注
        :return:
        """
        dimension_loc = (
            self.reinforce_view_dimension_data.get_stair_solid_top_dimension_point()
        )  # 获取实体标注点
        # 第一层标注对象
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            dim = self.dimension.add_aligned_dim(
                p1=end_point,
                p2=start_point,
                distance=current_offset,
                dimstyle="G_DS_20",
            )
            dim.render()

    def get_stair_solid_bottom_dimension_location(self):
        """
        开始楼梯实体底部标注
        :return:
        """
        dimension_loc = (
            self.reinforce_view_dimension_data.get_stair_solid_bottom_dimension_point()
        )  # 获取实体标注点
        # 第一层标注对象
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            dim = self.dimension.add_aligned_dim(
                p1=end_point,
                p2=start_point,
                distance=current_offset,
                dimstyle="G_DS_20",
            )
            dim.render()

    def get_stair_solid_cut_signal_dimension_location(self):
        """
        获取楼梯实体剖切符标注点位置
        :return:
        """
        cut_signal_info = (
            self.reinforce_view_dimension_data.get_draw_cut_signal_dimension_point()
        )  # 获取剖切符位置及文本信息
        line_loc = cut_signal_info[0]  # 剖切线位置
        dimension_text = cut_signal_info[1]  # 标注文本位置
        layer_name = "G_Cut_Line"
        # 开始绘制直线
        for segment in line_loc:
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            special_line = self.cut_signal.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_name}
            )
        for text_loc in dimension_text:
            text_loc_1 = transform_point_from_yz_to_xy(list(text_loc[0]))  # 文本位置
            text_loc_2 = transform_point_from_yz_to_xy(list(text_loc[1]))  # 文本位置
            text_loc_1 = adjust_drawing_scale(text_loc_1, 1 / self.scale)
            text_loc_2 = adjust_drawing_scale(text_loc_2, 1 / self.scale)
            text_content = text_loc[2]  # 文本内容
            self.cut_signal.add_text(
                text_content,
                height=self.note_text_size,
                rotation=-90,
                dxfattribs={"layer": "G_Note"},
            ).set_placement(
                text_loc_1, align=TextEntityAlignment.BOTTOM_RIGHT
            )  # 防止重叠
            self.cut_signal.add_text(
                text_content,
                height=self.note_text_size,
                rotation=-90,
                dxfattribs={"layer": "G_Note"},
            ).set_placement(
                text_loc_2, align=TextEntityAlignment.BOTTOM_RIGHT
            )  # 防止重叠

    def get_stair_solid_dimension_outline_yz_xy_text(
        self, dimension_info, line_layer, text_layer, text_loc
    ):
        """
        开始楼梯实体标注引出线
        :return:
        """
        dimension_line = dimension_info[0]  # 引出线绘制
        dimension_text = dimension_info[1]  # 引出线文字说明
        # 开始绘制直线
        for segment in dimension_line:
            start_point = transform_point_from_yz_to_xy(list(segment[0]))
            end_point = transform_point_from_yz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline_shape.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        for segment in dimension_text:
            point_loc = segment[0]  # 文本防止点
            point_loc = transform_point_from_yz_to_xy(list(point_loc))  # 文本位置
            point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
            text = segment[1]  # 文本内容
            self.outline_shape.add_text(
                text,
                height=self.note_text_size,
                rotation=-90,
                dxfattribs={"layer": text_layer},
            ).set_placement(point_loc, align=text_loc)

    def get_stair_solid_bottom_long_rebar_dimension_outline(self):
        """
        开始楼梯实体下部纵筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_bottom_long_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_RIGHT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_top_long_rebar_dimension_outline(self):
        """
        开始楼梯实体上部纵筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_top_long_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_mid_distribute_rebar_dimension_outline(self):
        """
        开始楼梯实体中部分布筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_mid_distribute_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_RIGHT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_bottom_edge_long_rebar_dimension_outline(self):
        """
        开始楼梯实体底端边缘纵筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_bottom_edge_long_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_top_edge_long_rebar_dimension_outline(self):
        """
        开始楼梯实体顶端边缘纵筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_top_edge_long_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_bottom_edge_stir_dimension_outline(self):
        """
        开始楼梯实体底端边缘箍筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_bottom_edge_stir_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_top_edge_stir_dimension_outline(self):
        """
        开始楼梯实体顶端边缘箍筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_top_edge_stir_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_hole_rein_rebar_dimension_outline(self):
        """
        开始楼梯实体孔洞加强筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_hole_rein_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_hoist_rein_long_rebar_dimension_outline(self):
        """
        开始楼梯实体吊装加强纵筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_hoist_rein_long_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def get_stair_solid_hoist_rein_point_rebar_dimension_outline(self):
        """
        开始楼梯实体吊装加强点筋标注引出线
        :return:
        """
        dimension_info = (
            self.reinforce_view_dimension_data.get_hoist_rein_point_rebar_outline_shape()
        )
        line_layer = "G_Outline"
        text_layer = "G_Note"
        text_loc = TextEntityAlignment.BOTTOM_LEFT
        self.get_stair_solid_dimension_outline_yz_xy_text(
            dimension_info, line_layer, text_layer, text_loc
        )

    def main_run_process(self):
        """
        开始主要的运行过程
        :return:
        """
        self.begin_draw_polyline()
        self.begin_draw_hole_shape()
        self.begin_draw_hole_rein_rebar()
        self.begin_draw_hoist_embedded_shape()
        self.begin_draw_bottom_long_rebar_shape()
        self.begin_draw_top_long_rebar_shape()
        self.begin_draw_mid_distribute_rebar_shape()
        self.begin_draw_bottom_edge_long_rebar_shape()
        self.begin_draw_top_edge_long_rebar_shape()
        self.get_bottom_edge_stir_shape()
        self.get_top_edge_stir_shape()
        self.get_hoist_rein_long_rebar_shape()
        self.get_hoist_rein_point_rebar_shape()
        self.get_stair_solid_top_dimension_location()
        self.get_stair_solid_bottom_dimension_location()
        self.get_stair_solid_cut_signal_dimension_location()
        self.get_stair_solid_bottom_long_rebar_dimension_outline()
        self.get_stair_solid_top_long_rebar_dimension_outline()
        self.get_stair_solid_mid_distribute_rebar_dimension_outline()
        self.get_stair_solid_bottom_edge_long_rebar_dimension_outline()
        self.get_stair_solid_top_edge_long_rebar_dimension_outline()
        self.get_stair_solid_bottom_edge_stir_dimension_outline()
        self.get_stair_solid_top_edge_stir_dimension_outline()
        self.get_stair_solid_hole_rein_rebar_dimension_outline()
        self.get_stair_solid_hoist_rein_long_rebar_dimension_outline()
        self.get_stair_solid_hoist_rein_point_rebar_dimension_outline()


class StairSectionOneView(object):
    """
    生成楼梯剖面1-1视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_section_data = StairSectionOneViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.section_view_dimension_data = StairSectionOneViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.section_view = self.dxf_doc
        self.model_space = self.section_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建图块
        """
        self.section_Shape = self.section_view.blocks.new("section_one_view")  # 添加视图块
        self.shape_dimension = self.section_view.blocks.new(
            "section_one_dimension"
        )  # 添加孔洞位置块
        self.special_signal = self.section_view.blocks.new(
            "section_one_one_signal"
        )  # 添加标高块

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for num in range(len(profile_points)):
            profile_points[num][0] = transform_point_from_xz_to_xy(
                list(profile_points[num][0])
            )  # 起点
            profile_points[num][1] = transform_point_from_xz_to_xy(
                list(profile_points[num][1])
            )  # 终点
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.section_Shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_polyline(self):
        """
        开始绘制楼梯1-1剖面轮廓图
        :return:
        """
        profile_points = (
            self.stair_section_data.get_stair_solid_section_view()
        )  # 获取楼梯剖面数据
        layer_name = "G_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_hole_shape(self):
        """
        开始绘制孔洞
        :return:
        """
        hole_points = self.stair_section_data.get_stair_hole_view_data()  # 获取楼梯剖面数据
        layer_name = "G_Hole_Outline"
        self.begin_draw_standard_polyline(hole_points, layer_name)

    def begin_dimension_solid_stair_location(self):
        """
        开始绘制楼梯剖面1-1图轮廓定位数据
        :return:
        """
        stair_loc = self.section_view_dimension_data.get_stair_hole_dimension_point()
        first_floor_loc = stair_loc["first floor"]  # 第一层标注点位置
        second_floor_loc = stair_loc["second floor"]  # 第二层标注点位置
        # 第一层标注对象
        for segment in first_floor_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = transform_point_from_xz_to_xy(list(segment[0]))  # 起点
            end_point = transform_point_from_xz_to_xy(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            dim = self.shape_dimension.add_aligned_dim(
                p1=end_point,
                p2=start_point,
                distance=current_offset,
                dimstyle="G_DS_20",
            )
            dim.render()
        # 第二层标注对象
        for segment in second_floor_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 1
            # 调整点的比例
            start_point = transform_point_from_xz_to_xy(list(segment[0]))  # 起点
            end_point = transform_point_from_xz_to_xy(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            dim = self.shape_dimension.add_aligned_dim(
                p1=end_point,
                p2=start_point,
                distance=current_offset,
                dimstyle="G_DS_20",
            )
            dim.render()

    def begin_dimension_hole_location(self):
        """
        开始标注孔洞位置
        :return:
        """
        hole_info = (
            self.section_view_dimension_data.get_hole_dimension_point()
        )  # 获取孔洞标注点
        hole_loc = hole_info[0]  # 孔洞位置
        hole_text = hole_info[1]  # 孔洞文本
        special_dimension = hole_info[2]  # 特殊标注
        for num in range(len(hole_loc)):
            current_loc = hole_loc[num]
            for segment in current_loc:
                # 调整该层标注相对实体偏移量
                current_offset = self.dimension_offset * 0.25
                start_point = transform_point_from_xz_to_xy(list(segment[0]))
                end_point = transform_point_from_xz_to_xy(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                    dim = self.shape_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        text=hole_text[num],
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.shape_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        text=hole_text[num],
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()
        # 滑动铰支座孔洞内部尺寸标注
        if len(special_dimension) != 0:
            for segment in special_dimension:
                # 调整该层标注相对实体偏移量
                current_offset = self.dimension_offset * 0.15
                start_point = transform_point_from_xz_to_xy(list(segment[0]))
                end_point = transform_point_from_xz_to_xy(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 布置标注块
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                    dim = self.shape_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.shape_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        distance=current_offset,
                        dimstyle="G_DS_20",  # 按照1：100进行绘制
                    )
                dim.render()

    def begin_dimension_special_text_location(self):
        """
        开始标注特殊文本
        :return:
        """
        (
            signal_loc,
            dimension_text,
        ) = (
            self.section_view_dimension_data.get_special_signal_dimension_point()
        )  # 获取特殊标志符点
        elevation_line_layer = "G_Elevation_Line"
        text_layer = "G_Note"
        # 绘制三角形
        for num in range(len(signal_loc)):
            signal_loc[num][0] = transform_point_from_xz_to_xy(
                list(signal_loc[num][0])
            )  # 起点
            signal_loc[num][1] = transform_point_from_xz_to_xy(
                list(signal_loc[num][1])
            )  # 终点
            signal_loc[num][0] = adjust_drawing_scale(
                signal_loc[num][0], 1 / self.scale
            )
            signal_loc[num][1] = adjust_drawing_scale(
                signal_loc[num][1], 1 / self.scale
            )
        for segment in signal_loc:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.special_signal.add_line(
                seg[0], seg[1], dxfattribs={"layer": elevation_line_layer}
            )
        # 添加文本
        for num in range(len(dimension_text[0])):
            current_point = dimension_text[1][num]
            current_point = transform_point_from_xz_to_xy(list(current_point))  # 终点
            current_point = adjust_drawing_scale(current_point, 1 / self.scale)
            text = dimension_text[0][num]
            if num != 1:
                self.special_signal.add_text(
                    text, height=self.note_text_size, dxfattribs={"layer": text_layer}
                ).set_placement(current_point, align=TextEntityAlignment.BOTTOM_RIGHT)
            else:
                self.special_signal.add_text(
                    text, height=self.note_text_size, dxfattribs={"layer": text_layer}
                ).set_placement(
                    current_point, align=TextEntityAlignment.BOTTOM_RIGHT
                )  # 防止重叠

    def main_run_process(self):
        """
        主要的运行过程
        :return:
        """
        self.begin_draw_polyline()  # 开始绘制楼梯轮廓
        self.begin_draw_hole_shape()  # 开始绘制孔洞形状
        self.begin_dimension_solid_stair_location()  # 开始标注剖面图轮廓
        self.begin_dimension_hole_location()  # 开始标注孔洞信息
        self.begin_dimension_special_text_location()  # 开始标注特殊文本


class StairSectionTwoView(object):
    """
    生成楼梯剖面2-2视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_section_data = StairSectionTwoViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.section_view_dimension_data = StairSectionTwoViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.section_view = self.dxf_doc
        self.model_space = self.section_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建图块
        """
        self.section_Shape = self.section_view.blocks.new("section_two_view")  # 添加视图块
        self.shape_dimension = self.section_view.blocks.new(
            "section_two_dimension"
        )  # 添加孔洞位置块
        self.special_signal = self.section_view.blocks.new(
            "section_two_two_signal"
        )  # 添加标高块

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for num in range(len(profile_points)):
            profile_points[num][0] = transform_point_from_xz_to_xy(
                list(profile_points[num][0])
            )  # 起点
            profile_points[num][1] = transform_point_from_xz_to_xy(
                list(profile_points[num][1])
            )  # 终点
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.section_Shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_polyline(self):
        """
        开始绘制楼梯2-2剖面轮廓图
        :return:
        """
        profile_points = (
            self.stair_section_data.get_stair_solid_section_view()
        )  # 获取楼梯剖面数据
        layer_name = "G_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_hole_shape(self):
        """
        开始绘制孔洞
        :return:
        """
        hole_points = self.stair_section_data.get_stair_hole_view_data()  # 获取楼梯剖面数据
        layer_name = "G_Hole_Outline"
        self.begin_draw_standard_polyline(hole_points, layer_name)

    def begin_dimension_solid_stair_location(self):
        """
        开始绘制楼梯剖面1-1图轮廓定位数据
        :return:
        """
        stair_loc = self.section_view_dimension_data.get_stair_hole_dimension_point()
        first_floor_loc = stair_loc["first floor"]  # 第一层标注点位置
        second_floor_loc = stair_loc["second floor"]  # 第二层标注点位置
        # 第一层标注对象
        for segment in first_floor_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = transform_point_from_xz_to_xy(list(segment[0]))  # 起点
            end_point = transform_point_from_xz_to_xy(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                dim = self.shape_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.shape_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 第二层标注对象
        for segment in second_floor_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 1
            # 调整点的比例
            start_point = transform_point_from_xz_to_xy(list(segment[0]))  # 起点
            end_point = transform_point_from_xz_to_xy(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                dim = self.shape_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.shape_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",  # 按照1：100进行绘制
                )
            dim.render()

    def begin_dimension_hole_location(self):
        """
        开始标注孔洞位置
        :return:
        """
        hole_info = (
            self.section_view_dimension_data.get_hole_dimension_point()
        )  # 获取孔洞标注点
        hole_loc = hole_info[0]  # 孔洞位置
        hole_text = hole_info[1]  # 孔洞文本
        special_dimension = hole_info[2]  # 特殊标注
        for num in range(len(hole_loc)):
            current_loc = hole_loc[num]
            for segment in current_loc:
                # 调整该层标注相对实体偏移量
                current_offset = self.dimension_offset * 0.25
                start_point = transform_point_from_xz_to_xy(list(segment[0]))
                end_point = transform_point_from_xz_to_xy(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                    dim = self.shape_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        text=hole_text[num],
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.shape_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        text=hole_text[num],
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()
        # 滑动铰支座孔洞内部尺寸标注
        if len(special_dimension) != 0:
            for segment in special_dimension:
                # 调整该层标注相对实体偏移量
                current_offset = self.dimension_offset * 0.15
                start_point = transform_point_from_xz_to_xy(list(segment[0]))
                end_point = transform_point_from_xz_to_xy(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 布置标注块
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                    dim = self.shape_dimension.add_linear_dim(
                        base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                    )
                else:
                    dim = self.shape_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        distance=current_offset,
                        dimstyle="G_DS_20",  # 按照1：100进行绘制
                    )
                dim.render()

    def begin_dimension_special_text_location(self):
        """
        开始标注特殊文本
        :return:
        """
        (
            signal_loc,
            dimension_text,
        ) = (
            self.section_view_dimension_data.get_special_signal_dimension_point()
        )  # 获取特殊标志符点
        elevation_line_layer = "G_Elevation_Line"
        text_layer = "G_Note"
        # 绘制三角形
        for num in range(len(signal_loc)):
            signal_loc[num][0] = transform_point_from_xz_to_xy(
                list(signal_loc[num][0])
            )  # 起点
            signal_loc[num][1] = transform_point_from_xz_to_xy(
                list(signal_loc[num][1])
            )  # 终点
            signal_loc[num][0] = adjust_drawing_scale(
                signal_loc[num][0], 1 / self.scale
            )
            signal_loc[num][1] = adjust_drawing_scale(
                signal_loc[num][1], 1 / self.scale
            )
        for segment in signal_loc:
            seg = [
                ((segment[0][0]), (segment[0][1]), (segment[0][2])),
                ((segment[1][0]), (segment[1][1]), (segment[1][2])),
            ]  # 将三维点转化为平面点,一般不取绝对值
            self.special_signal.add_line(
                seg[0], seg[1], dxfattribs={"layer": elevation_line_layer}
            )
        # 添加文本
        for num in range(len(dimension_text[0])):
            current_point = dimension_text[1][num]
            current_point = transform_point_from_xz_to_xy(list(current_point))  # 终点
            current_point = adjust_drawing_scale(current_point, 1 / self.scale)
            text = dimension_text[0][num]
            if num != 1:
                self.special_signal.add_text(
                    text, height=self.note_text_size, dxfattribs={"layer": text_layer}
                ).set_placement(current_point, align=TextEntityAlignment.BOTTOM_RIGHT)
            else:
                self.special_signal.add_text(
                    text, height=self.note_text_size, dxfattribs={"layer": text_layer}
                ).set_placement(
                    current_point, align=TextEntityAlignment.BOTTOM_RIGHT
                )  # 防止重叠

    def main_run_process(self):
        """
        主要的运行过程
        :return:
        """
        self.begin_draw_polyline()  # 开始绘制楼梯轮廓
        self.begin_draw_hole_shape()  # 开始绘制孔洞形状
        self.begin_dimension_solid_stair_location()  # 开始标注剖面图轮廓
        self.begin_dimension_hole_location()  # 开始标注孔洞信息
        self.begin_dimension_special_text_location()  # 开始标注特殊文本


class StairRebarSectionAToAView(object):
    """
    楼梯配筋剖面a-a视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_section_data = StairRebarSectionAToAViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.section_view_dimension_data = StairRebarSectionAToAViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        创建dxf文件
        :return:
        """
        self.section_view = self.dxf_doc
        self.model_space = self.section_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建块
        """
        self.section_shape = self.section_view.blocks.new("section_a_view")  # 添加视图块
        self.hole_shape = self.section_view.blocks.new("section_a_hole")  # 添加孔洞形状块
        self.rebar_shape = self.section_view.blocks.new("section_a_rebar")  # 添加钢筋形状块
        self.dimension = self.section_view.blocks.new("section_a_dimension")  # 添加孔洞位置块
        self.outline = self.section_view.blocks.new("section_a_outline")  # 添加标高块

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for num in range(len(profile_points)):
            profile_points[num][0] = transform_point_from_xz_to_xy(
                list(profile_points[num][0])
            )  # 起点
            profile_points[num][1] = transform_point_from_xz_to_xy(
                list(profile_points[num][1])
            )  # 终点
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.section_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_profile_polyline(self):
        """
        绘制轮廓线
        :return:
        """
        profile_points = self.stair_section_data.get_stair_solid_profile_cut_drawing()
        layer_name = "G_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_hole_shape(self):
        """
        开始绘制孔洞
        :return:
        """
        hole_points = (
            self.stair_section_data.get_stair_hole_profile_cut_drawing()
        )  # 获取楼梯剖面数据
        layer_name = "G_Hole_Outline"
        self.begin_draw_standard_polyline(hole_points, layer_name)

    def begin_draw_hole_rein_rebar_shape(self):
        """
        开始绘制孔洞加强钢筋形状
        :return:
        """
        hole_rein_rebar_loc = (
            self.stair_section_data.get_stair_hole_rein_rebar_projection()
        )  # 获取孔洞加强钢筋坐标
        layer_name = "G_Rebar_Section"
        self.begin_draw_standard_polyline(hole_rein_rebar_loc, layer_name)

    def begin_draw_top_edge_long_rebar_shape(self):
        """
        获取顶部边缘纵筋的形状
        :return:
        """
        top_edge_long_rebar_loc = (
            self.stair_section_data.get_stair_top_edge_long_rebar_cut_shape()
        )  # 获取绘制数据
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(top_edge_long_rebar_loc)):
            for k in range(len(top_edge_long_rebar_loc[num])):
                top_edge_long_rebar_loc[num][k] = transform_point_from_xz_to_xy(
                    list(top_edge_long_rebar_loc[num][k])
                )  # 终点
                top_edge_long_rebar_loc[num][k] = adjust_drawing_scale(
                    top_edge_long_rebar_loc[num][k], 1 / self.scale
                )
        for num in range(len(top_edge_long_rebar_loc)):
            current_rebar = top_edge_long_rebar_loc[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_top_edge_stir_shape(self):
        """
        开始绘制顶部边缘箍筋形状
        :return:
        """
        top_edge_stir_line_loc = (
            self.stair_section_data.get_top_edge_stir_cut_shape()
        )  # 获取顶部边缘箍筋数据
        line_loc = top_edge_stir_line_loc["line"]
        arc_loc = top_edge_stir_line_loc["arc"]
        layer_name = "G_Rebar"
        # 开始绘制直线段
        # 变换绘制比例
        for num in range(len(line_loc)):
            for k in range(len(line_loc[num])):
                line_loc[num][k] = transform_point_from_xz_to_xy(
                    list(line_loc[num][k])
                )  # 终点
                line_loc[num][k] = adjust_drawing_scale(
                    line_loc[num][k], 1 / self.scale
                )
        for num in range(len(line_loc)):
            current_rebar = line_loc[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )
        # 开始绘制弧线段
        for num in range(len(arc_loc)):
            current_rebar = arc_loc[num]
            center = transform_point_from_xz_to_xy(current_rebar[0])
            center = adjust_drawing_scale(center, 1 / self.scale)
            radius = current_rebar[1] * (1 / self.scale)
            arc_loc[num][0] = center
            arc_loc[num][1] = radius
        for num in range(len(arc_loc)):
            current_rebar = arc_loc[num]
            center = tuple(current_rebar[0])
            radius = current_rebar[1]
            theta_1 = current_rebar[2]
            theta_2 = current_rebar[3]
            self.rebar_shape.add_arc(
                center, radius, theta_1, theta_2, dxfattribs={"layer": layer_name}
            )

    def begin_draw_top_edge_rein_rebar_shape(self):
        """
        开始绘制顶端上部边缘加强筋形状
        :return:
        """
        top_edge_rein_rebar_loc = (
            self.stair_section_data.get_top_edge_rein_rebar_cut_shape()
        )
        rebar_loc = top_edge_rein_rebar_loc["location"]  # 获取钢筋的位置
        rebar_diam = top_edge_rein_rebar_loc["diameter"]  # 获取钢筋的直径
        layer_name = "G_Rebar_Section"
        for num in range(len(rebar_loc)):
            current_point = rebar_loc[num]
            current_point = transform_point_from_xz_to_xy(list(current_point))  # 终点
            rebar_loc[num] = adjust_drawing_scale(current_point, 1 / self.scale)
        # 缩放一次即可
        rebar_diam *= 1 / self.scale
        for num in range(len(rebar_loc)):
            current_point = tuple(rebar_loc[num])
            self.rebar_shape.add_circle(
                current_point, rebar_diam / 2, dxfattribs={"layer": layer_name}
            )

    def begin_draw_bottom_edge_rein_rebar_shape(self):
        """
        开始绘制顶端下部边缘加强筋形状
        :return:
        """
        bottom_edge_rein_rebar_loc = (
            self.stair_section_data.get_bottom_edge_rein_rebar_cut_shape()
        )
        rebar_loc = bottom_edge_rein_rebar_loc["location"]  # 获取钢筋的位置
        rebar_diam = bottom_edge_rein_rebar_loc["diameter"]  # 获取钢筋的直径
        layer_name = "G_Rebar_Section"
        for num in range(len(rebar_loc)):
            current_point = rebar_loc[num]
            current_point = transform_point_from_xz_to_xy(list(current_point))  # 终点
            rebar_loc[num] = adjust_drawing_scale(current_point, 1 / self.scale)
        # 缩放一次即可
        rebar_diam *= 1 / self.scale
        for num in range(len(rebar_loc)):
            current_point = tuple(rebar_loc[num])
            self.rebar_shape.add_circle(
                current_point, rebar_diam / 2, dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
        self, dimension_loc, offset
    ):
        """
        楼梯轮廓标准标注方式
        :return:
        """
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = offset
            start_point = transform_point_from_xz_to_xy(list(segment[0]))
            end_point = transform_point_from_xz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                dim = self.dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_profile_dimension_point(self):
        """
        开始标注混凝土轮廓
        :return:
        """
        dimension_loc = (
            self.section_view_dimension_data.get_stair_solid_profile_dimension_point()
        )  # 楼梯轮廓点
        current_offset = self.dimension_offset * 0.75
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            dimension_loc, current_offset
        )

    def begin_draw_top_edge_long_rebar_dimension_point(self):
        """
        开始顶部边缘纵筋尺寸标注
        :return:
        """
        dimension_loc = (
            self.section_view_dimension_data.get_stair_top_edge_long_rebar_dimension_point()
        )
        current_offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            dimension_loc, current_offset
        )

    def begin_draw_top_edge_stir_dimension_point(self):
        """
        开始绘制顶部边缘箍筋尺寸标注点
        :return:
        """
        dimension_loc = (
            self.section_view_dimension_data.get_stair_top_edge_stir_dimension_point()
        )
        current_offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            dimension_loc, current_offset
        )

    def begin_draw_standard_outline_xz_xy(
        self, dimension_info, line_layer, text_layer, text_align_loc
    ):
        """
        开始绘制标准引出线xz_xy
        :return:
        """
        dimension_line = dimension_info[0]  # 引出线绘制
        dimension_text = dimension_info[1]  # 引出线文字说明
        # 开始绘制直线
        for segment in dimension_line:
            start_point = transform_point_from_xz_to_xy(list(segment[0]))
            end_point = transform_point_from_xz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        for segment in dimension_text:
            point_loc = segment[0]  # 文本防止点
            point_loc = transform_point_from_xz_to_xy(list(point_loc))  # 文本位置
            point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
            text = segment[1]  # 文本内容
            self.outline.add_text(
                text,
                height=self.note_text_size,
                rotation=0,
                dxfattribs={"layer": text_layer},
            ).set_placement(point_loc, align=text_align_loc)

    def begin_draw_top_edge_stir_outline(self):
        """
        开始绘制顶部边缘箍筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_top_edge_stir_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_hole_rein_rebar_outline(self):
        """
        开始绘制孔洞加强钢筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_top_hole_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_top_edge_long_rebar_outline(self):
        """
        开始绘制顶部边缘纵筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_top_edge_long_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_top_edge_rein_rebar_outline(self):
        """
        开始绘制顶部边缘加强钢筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_top_edge_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_bottom_edge_rein_rebar_outline(self):
        """
        开始绘制底部边缘加强钢筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_bottom_edge_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def main_run_process(self):
        """
        主要运行过程
        :return:
        """
        self.begin_draw_profile_polyline()  # 开始绘制楼梯轮廓
        self.begin_draw_hole_shape()  # 开始绘制孔洞
        self.begin_draw_hole_rein_rebar_shape()  # 开始绘制孔洞加强筋形状
        self.begin_draw_top_edge_long_rebar_shape()  # 开始绘制顶部边缘纵筋
        self.begin_draw_top_edge_stir_shape()  # 开始绘制顶部边缘箍筋
        self.begin_draw_top_edge_rein_rebar_shape()  # 开始绘制顶端边缘加强筋形状
        self.begin_draw_bottom_edge_rein_rebar_shape()  # 开始绘制底端边缘加强筋形状
        self.begin_draw_stair_profile_dimension_point()  # 开始添加尺寸标注线
        self.begin_draw_top_edge_long_rebar_dimension_point()  # 开始尺寸标注线
        self.begin_draw_top_edge_stir_dimension_point()  # 开始绘制尺寸标注线
        self.begin_draw_top_edge_stir_outline()  # 开始绘制顶部边缘箍筋引出线
        self.begin_draw_hole_rein_rebar_outline()  # 开始绘制顶部孔洞加强筋引出线
        self.begin_draw_top_edge_long_rebar_outline()  # 开始绘制顶部边缘纵筋引出线
        self.begin_draw_top_edge_rein_rebar_outline()  # 开始绘制顶部边缘加强筋引出线
        self.begin_draw_bottom_edge_rein_rebar_outline()  # 开始绘制底部边缘加强筋引出线


class StairRebarSectionBToBView(object):
    """
    楼梯配筋剖面b-b视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_section_data = StairRebarSectionBToBViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.section_view_dimension_data = StairRebarSectionBToBViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.section_view = self.dxf_doc
        self.model_space = self.section_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.section_shape = self.section_view.blocks.new("section_b_view")  # 添加视图块
        self.hole_shape = self.section_view.blocks.new("section_b_hole")  # 添加孔洞形状块
        self.rebar_shape = self.section_view.blocks.new("section_b_rebar")  # 添加钢筋形状块
        self.dimension = self.section_view.blocks.new("section_b_dimension")  # 添加孔洞位置块
        self.outline = self.section_view.blocks.new("section_b_outline")  # 添加标高块

    def begin_draw_standard_polyline(
        self, theta, profile_points, base_point, layer_name
    ):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for num in range(len(profile_points)):
            # 旋转图形
            profile_points[num][0] = rotation_point_from_base_to_theta(
                list(profile_points[num][0]), base_point, theta
            )  # 起点
            profile_points[num][1] = rotation_point_from_base_to_theta(
                list(profile_points[num][1]), base_point, theta
            )  # 终点
            # 变换坐标平面
            profile_points[num][0] = transform_point_from_xz_to_xy(
                list(profile_points[num][0])
            )  # 起点
            profile_points[num][1] = transform_point_from_xz_to_xy(
                list(profile_points[num][1])
            )  # 终点
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.section_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_polyline_shape(self):
        """
        开始绘制楼梯多段线形状
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        profile_points = self.stair_section_data.get_stair_solid_profile_cut_drawing()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        layer_name = "G_Outline"
        self.begin_draw_standard_polyline(theta, profile_points, base_point, layer_name)

    def begin_draw_standard_rebar_shape_xz_xy(
        self, theta, top_edge_rein_rebar_loc, base_point, layer_name
    ):
        """
        开始绘制基本钢筋形状xz_xy
        :return:
        """
        for num in range(len(top_edge_rein_rebar_loc)):
            for k in range(len(top_edge_rein_rebar_loc[num])):
                top_edge_rein_rebar_loc[num][k] = rotation_point_from_base_to_theta(
                    list(top_edge_rein_rebar_loc[num][k]), base_point, theta
                )  # 起点
                top_edge_rein_rebar_loc[num][k] = transform_point_from_xz_to_xy(
                    list(top_edge_rein_rebar_loc[num][k])
                )  # 终点
                top_edge_rein_rebar_loc[num][k] = adjust_drawing_scale(
                    top_edge_rein_rebar_loc[num][k], 1 / self.scale
                )
        for num in range(len(top_edge_rein_rebar_loc)):
            current_rebar = top_edge_rein_rebar_loc[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_top_edge_rein_rebar_shape(self):
        """
        开始绘制顶端上部边缘加强筋形状
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        top_edge_rein_rebar_loc = (
            self.stair_section_data.get_stair_solid_top_edge_rein_rebar_profile_cut_drawing()
        )
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        layer_name = "G_Rebar_Section"
        self.begin_draw_standard_rebar_shape_xz_xy(
            theta, top_edge_rein_rebar_loc, base_point, layer_name
        )

    def begin_draw_bottom_edge_rein_rebar_shape(self):
        """
        开始绘制顶端下部边缘加强筋形状
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        bottom_edge_rein_rebar_loc = (
            self.stair_section_data.get_stair_solid_bottom_edge_rein_rebar_profile_cut_drawing()
        )
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        layer_name = "G_Rebar_Section"
        self.begin_draw_standard_rebar_shape_xz_xy(
            theta, bottom_edge_rein_rebar_loc, base_point, layer_name
        )

    def begin_draw_bottom_long_rebar_shape(self):
        """
        获取下部纵筋数据点
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        bottom_long_rebar_loc = (
            self.stair_section_data.get_stair_solid_bottom_long_rebar_profile_cut_drawing()
        )
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        layer_name = "G_Rebar"
        self.begin_draw_standard_rebar_shape_xz_xy(
            theta, bottom_long_rebar_loc, base_point, layer_name
        )

    def begin_draw_top_long_rebar_shape(self):
        """
        获取上部部纵筋数据点
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        top_long_rebar_loc = (
            self.stair_section_data.get_stair_solid_top_long_rebar_profile_cut_drawing()
        )
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        layer_name = "G_Rebar"
        self.begin_draw_standard_rebar_shape_xz_xy(
            theta, top_long_rebar_loc, base_point, layer_name
        )

    def begin_draw_mid_distribute_rebar_shape(self):
        """
        中部分布筋形状
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        mid_distribute_rebar_loc = (
            self.stair_section_data.get_stair_solid_mid_distribute_rebar_match_point()
        )  # 坐标
        layer_name = "G_Rebar"
        for m in range(len(mid_distribute_rebar_loc)):
            current_rebar_ = mid_distribute_rebar_loc[m]
            for num in range(len(current_rebar_)):
                for k in range(len(current_rebar_[num])):
                    current_rebar_[num][k] = rotation_point_from_base_to_theta(
                        list(current_rebar_[num][k]), base_point, theta
                    )  # 起点
                    current_rebar_[num][k] = transform_point_from_xz_to_xy(
                        list(current_rebar_[num][k])
                    )  # 终点
                    current_rebar_[num][k] = adjust_drawing_scale(
                        current_rebar_[num][k], 1 / self.scale
                    )
            mid_distribute_rebar_loc[m] = current_rebar_
        for rebar in mid_distribute_rebar_loc:
            for num in range(len(rebar)):
                current_rebar = rebar[num]
                for k in range(len(current_rebar) - 1):
                    point_1 = current_rebar[k]  # 钢筋当前段
                    point_2 = current_rebar[k + 1]  # 钢筋当前段
                    seg = [
                        (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                        (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                    ]  # 将三维点转化为平面点
                    self.rebar_shape.add_line(
                        seg[0], seg[1], dxfattribs={"layer": layer_name}
                    )

    def begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
        self, theta, base_point, dimension_loc, offset
    ):
        """
        楼梯轮廓标准标注方式
        :return:
        """
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = offset
            start_point = segment[0]
            end_point = segment[1]
            # 旋转
            start_point = rotation_point_from_base_to_theta(
                list(start_point), base_point, theta
            )  # 起点
            end_point = rotation_point_from_base_to_theta(
                list(end_point), base_point, theta
            )  # 起点
            # 平面变换
            start_point = transform_point_from_xz_to_xy(list(start_point))
            end_point = transform_point_from_xz_to_xy(list(end_point))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                dim = self.dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_profile_first_floor_dimension_point(self):
        """
        开始标注混凝土轮廓第一层标注
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_loc = (
            self.section_view_dimension_data.get_stair_solid_profile_first_floor_dimension_point()
        )  # 楼梯轮廓点
        offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            theta, base_point, dimension_loc, offset
        )

    def begin_draw_stair_profile_second_floor_dimension_point(self):
        """
        开始标注混凝土轮廓第二层标注
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_loc = (
            self.section_view_dimension_data.get_stair_solid_profile_second_floor_dimension_point()
        )  # 楼梯轮廓点
        offset = self.dimension_offset * 0.75
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            theta, base_point, dimension_loc, offset
        )

    def begin_draw_stair_bottom_long_rebar_dimension_point(self):
        """
        开始标注底部纵筋尺寸
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_loc = (
            self.section_view_dimension_data.get_bottom_long_rebar_dimension_point()
        )  # 底部纵筋尺寸标注
        offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            theta, base_point, dimension_loc, offset
        )

    def begin_draw_stair_top_long_rebar_dimension_point(self):
        """
        开始标注底部纵筋尺寸
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_loc = (
            self.section_view_dimension_data.get_top_long_rebar_dimension_point()
        )  # 顶部纵筋尺寸标注
        offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            theta, base_point, dimension_loc, offset
        )

    def begin_draw_standard_outline_xz_xy(
        self, theta, base_point, dimension_info, line_layer, text_layer, text_align_loc
    ):
        """
        开始绘制标准引出线xz_xy
        :params theta:旋转角
        :params base_point:基点
        :params dimension_info:标注信息
        :params line_layer:线型图层
        :params text_layer:文本图层
        :params text_align_loc:文本对齐方式
        :return:
        """
        dimension_line = dimension_info[0]  # 引出线绘制
        dimension_text = dimension_info[1]  # 引出线文字说明
        # 开始绘制直线
        for segment in dimension_line:
            start_point = segment[0]
            end_point = segment[1]
            # 旋转
            start_point = rotation_point_from_base_to_theta(
                list(start_point), base_point, theta
            )  # 起点
            end_point = rotation_point_from_base_to_theta(
                list(end_point), base_point, theta
            )  # 起点
            start_point = transform_point_from_xz_to_xy(list(start_point))
            end_point = transform_point_from_xz_to_xy(list(end_point))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        for segment in dimension_text:
            point_loc = segment[0]  # 文本防止点
            point_loc = transform_point_from_xz_to_xy(list(point_loc))  # 文本位置
            point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
            text = segment[1]  # 文本内容
            self.outline.add_text(
                text,
                height=self.note_text_size,
                rotation=0,
                dxfattribs={"layer": text_layer},
            ).set_placement(point_loc, align=text_align_loc)

    def begin_draw_bottom_long_rebar_outline_shape(self):
        """
        开始绘制底部纵筋引出线
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_info = (
            self.section_view_dimension_data.get_stair_bottom_long_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            theta,
            base_point,
            dimension_info,
            line_layer_name,
            text_layer,
            text_align_loc,
        )

    def begin_draw_top_long_rebar_outline_shape(self):
        """
        开始绘制顶部纵筋引出线
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_info = (
            self.section_view_dimension_data.get_stair_top_long_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_RIGHT
        self.begin_draw_standard_outline_xz_xy(
            theta,
            base_point,
            dimension_info,
            line_layer_name,
            text_layer,
            text_align_loc,
        )

    def begin_draw_mid_distribute_rebar_outline_shape(self):
        """
        开始绘制中部分布筋引出线
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_info = (
            self.section_view_dimension_data.get_stair_solid_mid_distribute_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_RIGHT
        self.begin_draw_standard_outline_xz_xy(
            theta,
            base_point,
            dimension_info,
            line_layer_name,
            text_layer,
            text_align_loc,
        )

    def begin_draw_top_edge_rein_rebar_outline_shape(self):
        """
        开始绘制顶部边缘加强筋引出线
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_info = (
            self.section_view_dimension_data.get_stair_solid_top_edge_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_RIGHT
        self.begin_draw_standard_outline_xz_xy(
            theta,
            base_point,
            dimension_info,
            line_layer_name,
            text_layer,
            text_align_loc,
        )

    def begin_draw_bottom_edge_rein_rebar_outline_shape(self):
        """
        开始绘制底部边缘加强筋引出线
        :return:
        """
        theta = self.stair_section_data.get_view_rotation_angle()
        base_point = self.stair_section_data.get_stair_solid_profile_character_point()
        dimension_info = (
            self.section_view_dimension_data.get_stair_solid_bottom_edge_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_RIGHT
        self.begin_draw_standard_outline_xz_xy(
            theta,
            base_point,
            dimension_info,
            line_layer_name,
            text_layer,
            text_align_loc,
        )

    def main_run_process(self):
        """
        主要运行过程
        :return:
        """
        self.begin_draw_stair_polyline_shape()  # 开始绘制楼梯轮廓
        self.begin_draw_top_edge_rein_rebar_shape()  # 开始绘制顶端边缘加强筋形状
        self.begin_draw_bottom_edge_rein_rebar_shape()  # 开始绘制底端边缘加强筋形状
        self.begin_draw_bottom_long_rebar_shape()  # 开始绘制下部纵筋形状
        self.begin_draw_top_long_rebar_shape()  # 开始绘制上部纵筋形状
        self.begin_draw_mid_distribute_rebar_shape()  # 开始绘制中部分布筋形状
        self.begin_draw_stair_profile_first_floor_dimension_point()  # 第一层楼梯轮廓尺寸标注
        self.begin_draw_stair_profile_second_floor_dimension_point()  # 第二层楼梯轮廓尺寸标注
        self.begin_draw_stair_bottom_long_rebar_dimension_point()  # 底部纵筋尺寸标注
        self.begin_draw_stair_top_long_rebar_dimension_point()  # 顶部纵筋尺寸标注
        self.begin_draw_bottom_long_rebar_outline_shape()  # 开始绘制底部纵筋引出线形状
        self.begin_draw_top_long_rebar_outline_shape()  # 开始绘制顶部纵筋引出线形状
        self.begin_draw_mid_distribute_rebar_outline_shape()  # 开始绘制中部分布筋引出线形状
        self.begin_draw_top_edge_rein_rebar_outline_shape()  # 开始顶部边缘加强筋引出线形状
        self.begin_draw_bottom_edge_rein_rebar_outline_shape()  # 开始底部边缘加强筋引出线


class StairRebarSectionCToCView(object):
    """
    楼梯配筋剖面c-c视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_section_data = StairRebarSectionCToCViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )  # 获取楼梯投影数据
        self.section_view_dimension_data = StairRebarSectionCToCViewDimensionData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        创建dxf文件
        :return:
        """
        self.section_view = self.dxf_doc
        self.model_space = self.section_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        """
        self.section_shape = self.section_view.blocks.new("section_c_view")  # 添加视图块
        self.hole_shape = self.section_view.blocks.new("section_c_hole")  # 添加孔洞形状块
        self.rebar_shape = self.section_view.blocks.new("section_c_rebar")  # 添加钢筋形状块
        self.dimension = self.section_view.blocks.new("section_c_dimension")  # 添加孔洞位置块
        self.outline = self.section_view.blocks.new("section_c_outline")  # 添加标高块

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for num in range(len(profile_points)):
            profile_points[num][0] = transform_point_from_xz_to_xy(
                list(profile_points[num][0])
            )  # 起点
            profile_points[num][1] = transform_point_from_xz_to_xy(
                list(profile_points[num][1])
            )  # 终点
            profile_points[num][0] = adjust_drawing_scale(
                profile_points[num][0], 1 / self.scale
            )
            profile_points[num][1] = adjust_drawing_scale(
                profile_points[num][1], 1 / self.scale
            )
        for segment in profile_points:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.section_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_profile_polyline(self):
        """
        绘制轮廓线
        :return:
        """
        profile_points = self.stair_section_data.get_stair_solid_profile_cut_drawing()
        layer_name = "G_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_hole_shape(self):
        """
        开始绘制孔洞
        :return:
        """
        hole_points = (
            self.stair_section_data.get_stair_hole_profile_cut_drawing()
        )  # 获取楼梯剖面数据
        layer_name = "G_Hole_Outline"
        self.begin_draw_standard_polyline(hole_points, layer_name)

    def begin_draw_hole_rein_rebar_shape(self):
        """
        开始绘制孔洞加强钢筋形状
        :return:
        """
        hole_rein_rebar_loc = (
            self.stair_section_data.get_stair_hole_rein_rebar_projection()
        )  # 获取孔洞加强钢筋坐标
        layer_name = "G_Rebar_Section"
        self.begin_draw_standard_polyline(hole_rein_rebar_loc, layer_name)

    def begin_draw_standard_rebar_shape_xz_xy(self, dimension_loc, layer):
        """
        获取标准钢筋形状绘制方法xz_xy
        :return:
        """
        # 变换绘制比例
        for num in range(len(dimension_loc)):
            for k in range(len(dimension_loc[num])):
                dimension_loc[num][k] = transform_point_from_xz_to_xy(
                    list(dimension_loc[num][k])
                )  # 终点
                dimension_loc[num][k] = adjust_drawing_scale(
                    dimension_loc[num][k], 1 / self.scale
                )
        for num in range(len(dimension_loc)):
            current_rebar = dimension_loc[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(seg[0], seg[1], dxfattribs={"layer": layer})

    def begin_draw_bottom_edge_long_rebar_shape(self):
        """
        获取底部边缘纵筋的形状
        :return:
        """
        bottom_edge_long_rebar_loc = (
            self.stair_section_data.get_stair_bottom_edge_long_rebar_cut_shape()
        )  # 获取绘制数据
        layer_name = "G_Rebar"
        self.begin_draw_standard_rebar_shape_xz_xy(
            bottom_edge_long_rebar_loc, layer_name
        )

    def begin_draw_bottom_edge_stir_shape(self):
        """
        开始绘制底部边缘箍筋形状
        :return:
        """
        bottom_edge_stir_line_loc = (
            self.stair_section_data.get_bottom_edge_stir_cut_shape()
        )  # 获取底部边缘箍筋数据
        line_loc = bottom_edge_stir_line_loc["line"]
        arc_loc = bottom_edge_stir_line_loc["arc"]
        layer_name = "G_Rebar"
        # 开始绘制直线段
        # 变换绘制比例
        for num in range(len(line_loc)):
            for k in range(len(line_loc[num])):
                line_loc[num][k] = transform_point_from_xz_to_xy(
                    list(line_loc[num][k])
                )  # 终点
                line_loc[num][k] = adjust_drawing_scale(
                    line_loc[num][k], 1 / self.scale
                )
        for num in range(len(line_loc)):
            current_rebar = line_loc[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )
        # 开始绘制弧线段
        for num in range(len(arc_loc)):
            current_rebar = arc_loc[num]
            center = transform_point_from_xz_to_xy(current_rebar[0])
            center = adjust_drawing_scale(center, 1 / self.scale)
            radius = current_rebar[1] * (1 / self.scale)
            arc_loc[num][0] = center
            arc_loc[num][1] = radius
        for num in range(len(arc_loc)):
            current_rebar = arc_loc[num]
            center = tuple(current_rebar[0])
            radius = current_rebar[1]
            theta_1 = current_rebar[2]
            theta_2 = current_rebar[3]
            self.rebar_shape.add_arc(
                center, radius, theta_1, theta_2, dxfattribs={"layer": layer_name}
            )

    def begin_draw_top_edge_rein_rebar_shape(self):
        """
        开始绘制底端上部边缘加强筋形状
        :return:
        """
        top_edge_rein_rebar_loc = (
            self.stair_section_data.get_top_edge_rein_rebar_cut_shape()
        )
        rebar_loc = top_edge_rein_rebar_loc["location"]  # 获取钢筋的位置
        rebar_diam = top_edge_rein_rebar_loc["diameter"]  # 获取钢筋的直径
        layer_name = "G_Rebar_Section"
        for num in range(len(rebar_loc)):
            current_point = rebar_loc[num]
            current_point = transform_point_from_xz_to_xy(list(current_point))  # 终点
            rebar_loc[num] = adjust_drawing_scale(current_point, 1 / self.scale)
        # 缩放一次即可
        rebar_diam *= 1 / self.scale
        for num in range(len(rebar_loc)):
            current_point = tuple(rebar_loc[num])
            self.rebar_shape.add_circle(
                current_point, rebar_diam / 2, dxfattribs={"layer": layer_name}
            )

    def begin_draw_bottom_edge_rein_rebar_shape(self):
        """
        开始绘制底端端下部边缘加强筋形状
        :return:
        """
        bottom_edge_rein_rebar_loc = (
            self.stair_section_data.get_bottom_edge_rein_rebar_cut_shape()
        )
        layer_name = "G_Rebar_Section"
        # 变换绘制比例
        for num in range(len(bottom_edge_rein_rebar_loc)):
            for k in range(len(bottom_edge_rein_rebar_loc[num])):
                bottom_edge_rein_rebar_loc[num][k] = transform_point_from_xz_to_xy(
                    list(bottom_edge_rein_rebar_loc[num][k])
                )  # 终点
                bottom_edge_rein_rebar_loc[num][k] = adjust_drawing_scale(
                    bottom_edge_rein_rebar_loc[num][k], 1 / self.scale
                )
        for num in range(len(bottom_edge_rein_rebar_loc)):
            current_rebar = bottom_edge_rein_rebar_loc[num]
            for k in range(len(current_rebar) - 1):
                point_1 = current_rebar[k]  # 钢筋当前段
                point_2 = current_rebar[k + 1]  # 钢筋当前段
                seg = [
                    (abs(point_1[0]), abs(point_1[1]), abs(point_1[2])),
                    (abs(point_2[0]), abs(point_2[1]), abs(point_2[2])),
                ]  # 将三维点转化为平面点
                self.rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_bottom_long_rebar_shape(self):
        """
        开始绘制下部纵筋形状
        :return:
        """
        bottom_long_rebar_loc = (
            self.stair_section_data.get_bottom_long_rebar_cut_shape()
        )  # 获取下部纵筋坐标
        layer_name = "G_Rebar"
        self.begin_draw_standard_rebar_shape_xz_xy(bottom_long_rebar_loc, layer_name)

    def begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
        self, dimension_loc, offset
    ):
        """
        楼梯轮廓标准标注方式
        :return:
        """
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = offset
            start_point = transform_point_from_xz_to_xy(list(segment[0]))
            end_point = transform_point_from_xz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 存在截断误差
                dim = self.dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_profile_dimension_point(self):
        """
        开始标注混凝土轮廓
        :return:
        """
        dimension_loc = (
            self.section_view_dimension_data.get_stair_solid_profile_dimension_point()
        )  # 楼梯轮廓点
        current_offset = self.dimension_offset * 0.75
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            dimension_loc, current_offset
        )

    def begin_draw_bottom_edge_long_rebar_dimension_point(self):
        """
        开始顶部边缘纵筋尺寸标注
        :return:
        """
        dimension_loc = (
            self.section_view_dimension_data.get_stair_bottom_edge_long_rebar_dimension_point()
        )
        current_offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            dimension_loc, current_offset
        )

    def begin_draw_bottom_edge_stir_dimension_point(self):
        """
        开始绘制顶部边缘箍筋尺寸标注点
        :return:
        """
        dimension_loc = (
            self.section_view_dimension_data.get_stair_bottom_edge_stir_dimension_point()
        )
        current_offset = self.dimension_offset * 0.25
        self.begin_draw_stair_standard_profile_dimension_point_xz_to_xy(
            dimension_loc, current_offset
        )

    def begin_draw_standard_outline_xz_xy(
        self, dimension_info, line_layer, text_layer, text_align_loc
    ):
        """
        开始绘制标准引出线xz_xy
        :return:
        """
        dimension_line = dimension_info[0]  # 引出线绘制
        dimension_text = dimension_info[1]  # 引出线文字说明
        # 开始绘制直线
        for segment in dimension_line:
            start_point = transform_point_from_xz_to_xy(list(segment[0]))
            end_point = transform_point_from_xz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        for segment in dimension_text:
            point_loc = segment[0]  # 文本防止点
            point_loc = transform_point_from_xz_to_xy(list(point_loc))  # 文本位置
            point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
            text = segment[1]  # 文本内容
            self.outline.add_text(
                text,
                height=self.note_text_size,
                rotation=0,
                dxfattribs={"layer": text_layer},
            ).set_placement(point_loc, align=text_align_loc)

    def begin_draw_bottom_long_rebar_outline(self):
        """
        开始绘制底部纵筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_bottom_long_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_bottom_edge_stir_outline(self):
        """
        开始绘制底部边缘箍筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_bottom_edge_stir_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_hole_rein_rebar_outline(self):
        """
        开始绘制孔洞加强钢筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_bottom_hole_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_bottom_edge_long_rebar_outline(self):
        """
        开始绘制底部边缘纵筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_bottom_edge_long_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_top_edge_rein_rebar_outline(self):
        """
        开始绘制顶部边缘加强钢筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_top_edge_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def begin_draw_bottom_edge_rein_rebar_outline(self):
        """
        开始绘制底部边缘加强钢筋引出线
        :return:
        """
        dimension_info = (
            self.section_view_dimension_data.get_stair_bottom_edge_rein_rebar_outline_shape()
        )
        line_layer_name = "G_Label"
        text_layer = "G_Note"
        text_align_loc = TextEntityAlignment.MIDDLE_LEFT
        self.begin_draw_standard_outline_xz_xy(
            dimension_info, line_layer_name, text_layer, text_align_loc
        )

    def main_run_process(self):
        """
        主要运行过程
        :return:
        """
        self.begin_draw_profile_polyline()  # 开始绘制楼梯轮廓
        self.begin_draw_hole_shape()  # 开始绘制孔洞
        self.begin_draw_hole_rein_rebar_shape()  # 开始绘制孔洞加强筋形状
        self.begin_draw_bottom_edge_long_rebar_shape()  # 开始绘制顶部边缘纵筋
        self.begin_draw_bottom_edge_stir_shape()  # 开始绘制顶部边缘箍筋
        self.begin_draw_top_edge_rein_rebar_shape()  # 开始绘制顶端边缘加强筋形状
        self.begin_draw_bottom_edge_rein_rebar_shape()  # 开始绘制底端边缘加强筋形状
        self.begin_draw_bottom_long_rebar_shape()  # 开始绘制下部纵筋形状
        self.begin_draw_stair_profile_dimension_point()  # 开始添加尺寸标注线
        self.begin_draw_bottom_edge_long_rebar_dimension_point()  # 开始尺寸标注线
        self.begin_draw_bottom_edge_stir_dimension_point()  # 开始绘制尺寸标注线
        self.begin_draw_bottom_long_rebar_outline()  # 开始绘制下部纵筋引出线
        self.begin_draw_bottom_edge_stir_outline()  # 开始绘制顶部边缘箍筋引出线
        self.begin_draw_hole_rein_rebar_outline()  # 开始绘制顶部孔洞加强筋引出线
        self.begin_draw_bottom_edge_long_rebar_outline()  # 开始绘制顶部边缘纵筋引出线
        self.begin_draw_top_edge_rein_rebar_outline()  # 开始绘制顶部边缘加强筋引出线
        self.begin_draw_bottom_edge_rein_rebar_outline()  # 开始绘制底部边缘加强筋引出线


class StairBottomInstallNodeView(object):
    """
    产生楼梯底端安装节点图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairBottomInstallNodeViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.bottom_install_view = self.dxf_doc
        self.model_space = self.bottom_install_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.bottom_install_shape = self.bottom_install_view.blocks.new(
            "bottom_install_view"
        )  # 添加安装视图块
        self.hole_shape = self.bottom_install_view.blocks.new(
            "bottom_install_hole_view"
        )  # 添加孔洞视图块
        self.connect_embedded = self.bottom_install_view.blocks.new(
            "bottom_install_connect_embedded"
        )  # 添加连接件
        self.line_dimension = self.bottom_install_view.blocks.new(
            "bottom_install_line_dimension"
        )  # 添加线性标注块
        self.break_line = self.bottom_install_view.blocks.new(
            "bottom_install_break_line"
        )  # 添加折断线块
        self.elevation_shape = self.bottom_install_view.blocks.new(
            "bottom_install_elevation_line"
        )  # 添加标高线块
        self.outline = self.bottom_install_view.blocks.new(
            "bottom_install_outLine"
        )  # 引出线块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.shim_hatch = self.model_space.add_hatch(color=7)
        self.glue_hatch = self.model_space.add_hatch()
        self.mid_hatch = self.model_space.add_hatch()
        self.top_hatch = self.model_space.add_hatch()
        self.bottom_slide_hatch = self.model_space.add_hatch()
        self.mid_slide_hatch = self.model_space.add_hatch()
        self.top_slide_hatch = self.model_space.add_hatch()
        self.bottom_fix_hatch = self.model_space.add_hatch()
        self.mid_fix_hatch = self.model_space.add_hatch()
        self.top_fix_hatch = self.model_space.add_hatch()
        self.pe_hatch = self.model_space.add_hatch(color=7)

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.bottom_install_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer}
                )

    def begin_draw_stair_bottom_beam_slab_profile_shape(self):
        """
        开始绘制楼梯底部梁和板轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_bottom_beam_slab_bounding_profile_points()
        )
        layer_name = "G_Special_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_stair_bottom_local_shape_profile_points(self):
        """
        开始绘制楼梯底部局部形状轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_bottom_local_shape_bounding_profile_points()
        )
        layer_name = "G_Special_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_stair_bottom_hole_shape_profile_points(self):
        """
        开始绘制楼梯底部孔洞形状轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_bottom_right_hole_bounding_profile_points()
        )
        layer_name = "G_Hole_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_stair_bottom_connect_embedded_shape_profile_points(self):
        """
        开始绘制楼梯底部连接埋件形状轮廓形状
        :return:
        """
        rebar_profile_points = (
            self.stair_cut_data.get_stair_solid_bottom_anchor_rebar_cut_drawing()
        )  # 底部锚固钢筋数据
        connect_embedded_layer = "G_Rebar"
        # 变换绘制比例
        for segment in rebar_profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in rebar_profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.connect_embedded.add_line(
                    seg[0], seg[1], dxfattribs={"layer": connect_embedded_layer}
                )
        if (
            self.detailed_design.construction_detailed.bottom_hole_type.value == 1
        ):  # 底端连接节点与底部孔洞类型对应，0--固定铰，1--滑动铰
            nut_profile_points = (
                self.stair_cut_data.get_stair_bottom_connect_nut_projection_drawing()
            )  # 底部螺母数据
            shim_profile_points = (
                self.stair_cut_data.get_stair_bottom_connect_shim_projection_drawing()
            )  # 底部垫片数据
            # 螺母--变换绘制比例
            for segment in nut_profile_points:
                for num in range(len(segment)):
                    segment[num] = transform_point_from_xy_to_yx(
                        list(segment[num])
                    )  # 起点
                    segment[num] = adjust_drawing_scale(
                        list(segment[num]), 1 / self.scale
                    )
            for segment in nut_profile_points:
                for num in range(len(segment) - 1):
                    seg = [
                        (
                            abs(segment[num][0]),
                            abs(segment[num][1]),
                            abs(segment[num][2]),
                        ),
                        (
                            abs(segment[num + 1][0]),
                            abs(segment[num + 1][1]),
                            abs(segment[num + 1][2]),
                        ),
                    ]  # 将三维点转化为平面点
                    self.connect_embedded.add_line(
                        seg[0], seg[1], dxfattribs={"layer": connect_embedded_layer}
                    )
            # 垫片--变换绘制比例
            for segment in shim_profile_points:
                for num in range(len(segment)):
                    segment[num] = transform_point_from_xy_to_yx(
                        list(segment[num])
                    )  # 起点
                    segment[num] = adjust_drawing_scale(
                        list(segment[num]), 1 / self.scale
                    )
            for segment in shim_profile_points:
                for num in range(len(segment) - 1):
                    seg = [
                        (
                            abs(segment[num][0]),
                            abs(segment[num][1]),
                            abs(segment[num][2]),
                        ),
                        (
                            abs(segment[num + 1][0]),
                            abs(segment[num + 1][1]),
                            abs(segment[num + 1][2]),
                        ),
                    ]  # 将三维点转化为平面点
                    self.connect_embedded.add_line(
                        seg[0], seg[1], dxfattribs={"layer": connect_embedded_layer}
                    )

    def begin_draw_bottom_shim_hatch_pattern(self):
        """
        获取底部垫片填充图案
        :return: None
        """
        hatch_points = self.stair_cut_data.get_stair_bottom_shim_corner_point()
        # 垫片--变换绘制比例
        for num in range(len(hatch_points)):
            hatch_points[num] = transform_point_from_xy_to_yx(
                list(hatch_points[num])
            )  # 起点
            hatch_points[num] = adjust_drawing_scale(
                list(hatch_points[num]), 1 / self.scale
            )
        # 开始绘制填充轮廓点
        self.shim_hatch.paths.add_polyline_path(hatch_points, is_closed=True)

    def begin_draw_stair_beam_slab_left_boundary_breakline(self):
        """
        开始绘制楼梯平台板和梁左侧边界折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_left_boundary_breakline()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.break_line.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_draw_stair_local_right_boundary_breakline(self):
        """
        开始绘制楼梯底端局部边界折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_right_boundary_breakline()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.break_line.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_draw_stair_elevation_line_shape(self):
        """
        开始绘制标高线
        :return:
        """
        elevation_line_points = (
            self.stair_cut_data.get_stair_bottom_beam_slab_elevation_line_points()
        )  # 获取标高线上的点
        layer_name = "G_Elevation_Dashed"
        # 调整各点的比例
        for num in range(len(elevation_line_points)):
            point = elevation_line_points[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            elevation_line_points[num] = adjust_drawing_scale(
                list(point), 1 / self.scale
            )
        # 遍历线段点
        for num in range(len(elevation_line_points) - 1):
            seg = [
                (
                    (elevation_line_points[num][0]),
                    (elevation_line_points[num][1]),
                    (elevation_line_points[num][2]),
                ),
                (
                    (elevation_line_points[num + 1][0]),
                    (elevation_line_points[num + 1][1]),
                    (elevation_line_points[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.elevation_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_elevation_sign_shape(self):
        """
        开始绘制楼梯标高符号
        :return:
        """
        elevation_info = (
            self.stair_cut_data.get_stair_solid_elevation_special_point()
        )  # 标高信息
        draw_points = elevation_info[0]  # 标高线点
        text_info = elevation_info[1]  # 文字说明
        line_layer_name = "G_Elevation_Line"
        text_layer = "G_Note"
        # 开始绘制符号形状
        for segment in draw_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in draw_points:
            for num in range(len(segment) - 1):
                seg = [
                    ((segment[num][0]), (segment[num][1]), (segment[num][2])),
                    (
                        (segment[num + 1][0]),
                        (segment[num + 1][1]),
                        (segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                self.elevation_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": line_layer_name}
                )
        # 开始设置标高文本
        # 添加文本
        current_point = text_info[0]
        current_point = transform_point_from_yz_to_xy_s(list(current_point))  # 终点
        current_point = adjust_drawing_scale(current_point, 1 / self.scale)
        text = text_info[1]
        self.elevation_shape.add_text(
            text, height=self.note_text_size, dxfattribs={"layer": text_layer}
        ).set_placement(
            current_point, align=TextEntityAlignment.BOTTOM_LEFT
        )  # 防止重叠

    def begin_draw_stair_construction_dimension_points(self):
        """
        获取楼梯尺寸标注点
        :return:
        """
        dimension_loc = (
            self.stair_cut_data.get_stair_construction_method_dimension_points()
        )  # 获取楼梯构造标注点
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.15
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 计算机存储精度
                dim = self.line_dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",  # 默认是1:100进行绘制，输入的数值是绘制的大小，实际大小需要乘以绘图比例。
                )
            else:
                dim = self.line_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",  # 按照1：100进行绘制
                )
            dim.render()

    def begin_draw_stair_bottom_glue_hatch_shape(self):
        """
        开始绘制楼梯底端打胶填充图案形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_bottom_glue_hatch_points()
        profile_points = hatch_info["hatch_profile"]
        outline_points = hatch_info["outline"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_line = "G_Outline"
        text_layer = "G_note"
        for num in range(len(draw_line)):
            point = draw_line[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_line}
            )
        # 开始填充图案
        for num in range(len(profile_points)):
            profile_points[num] = transform_point_from_yz_to_xy_s(
                list(profile_points[num])
            )  # 起点
            profile_points[num] = adjust_drawing_scale(
                list(profile_points[num]), 1 / self.scale
            )
            profile_points[num] = tuple(profile_points[num])
        # 开始绘制填充轮廓点
        self.glue_hatch.paths.add_polyline_path(profile_points, is_closed=1)
        self.glue_hatch.set_pattern_fill("ANSI31", scale=0.5 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_mid_hatch_pattern_shape(self):
        """
        开始绘制楼梯底端中部填充图案形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_bottom_mid_hatch_pattern_points()
        profile_points = hatch_info["hatch_profile"]
        outline_points = hatch_info["outline"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        line_layer = "G_Outline"
        text_layer = "G_Note"
        for num in range(len(draw_line)):
            point = draw_line[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(profile_points)):
            profile_points[num] = transform_point_from_yz_to_xy_s(
                list(profile_points[num])
            )  # 起点
            profile_points[num] = adjust_drawing_scale(
                list(profile_points[num]), 1 / self.scale
            )
            profile_points[num] = tuple(profile_points[num])
        # 开始绘制填充轮廓点
        self.mid_hatch.paths.add_polyline_path(profile_points, is_closed=1)
        self.mid_hatch.set_pattern_fill("HONEY", scale=2 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_pe_rod_hatch_shape(self):
        """
        开始绘制楼梯PE棒填充图案
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_bottom_pe_rod_hatch_points()
        profile_points = hatch_info["hatch_profile"]
        center = profile_points[0]  # 圆心
        diameter = profile_points[1]  # 直径
        outline_points = hatch_info["outline"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        text_layer = "G_Note"
        line_layer = "G_Outline"
        # 开始绘制圆
        center = transform_point_from_yz_to_xy_s(list(center))  # 起点
        center = adjust_drawing_scale(list(center), 1 / self.scale)
        diameter = diameter * (1 / self.scale)
        self.bottom_install_shape.add_circle(
            tuple(center), diameter / 2, dxfattribs={"layer": line_layer}
        )
        # 开始填充图案
        # 开始绘制填充轮廓点
        edge_path = self.pe_hatch.paths.add_edge_path()  # 开始绘制引出线
        edge_path.add_arc(tuple(center), diameter / 2)
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_bottom_slide_hatch_pattern(self):
        """
        开始绘制底部滑动铰填充图案
        :return:
        """
        # 填充图案
        # 1.底部滑动铰填充信息
        bottom_slide_info = self.stair_cut_data.get_stair_bottom_slide_hatch_points()
        bottom_profile_t = bottom_slide_info["hatch_profile"]
        bottom_outline_points = bottom_slide_info["outline"]
        bottom_outline_text = bottom_slide_info["outline_text"]
        draw_line = copy.deepcopy(bottom_profile_t)
        draw_line.append(bottom_profile_t[0])
        text_layer = "G_Note"
        line_layer = "G_Outline"
        for num in range(len(draw_line)):
            point = draw_line[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(bottom_profile_t)):
            bottom_profile_t[num] = transform_point_from_yz_to_xy_s(
                list(bottom_profile_t[num])
            )  # 起点
            bottom_profile_t[num] = adjust_drawing_scale(
                list(bottom_profile_t[num]), 1 / self.scale
            )
            bottom_profile_t[num] = tuple(bottom_profile_t[num])
        # 开始绘制填充轮廓点
        self.top_slide_hatch.paths.add_polyline_path(bottom_profile_t, is_closed=1)
        self.top_slide_hatch.set_pattern_fill("AR-SAND", scale=0.5 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in bottom_outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = bottom_outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = bottom_outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 底部节点接缝1图案填充信息
        bottom_hatch_one_info = (
            self.stair_cut_data.get_stair_bottom_slide_hatch_one_points()
        )
        bottom_profile_t_1 = bottom_hatch_one_info["hatch_profile"]
        bottom_outline_points_1 = bottom_hatch_one_info["outline_1"]
        bottom_outline_points_2 = bottom_hatch_one_info["outline_2"]
        bottom_outline_text_1 = bottom_hatch_one_info["outline_text_1"]
        bottom_outline_text_2 = bottom_hatch_one_info["outline_text_2"]
        draw_line_1 = copy.deepcopy(bottom_profile_t_1)
        draw_line_1.append(bottom_profile_t_1[0])
        for num in range(len(draw_line_1)):
            point = draw_line_1[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_1[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_1) - 1):
            seg = [
                ((draw_line_1[num][0]), (draw_line_1[num][1]), (draw_line_1[num][2])),
                (
                    (draw_line_1[num + 1][0]),
                    (draw_line_1[num + 1][1]),
                    (draw_line_1[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(bottom_profile_t_1)):
            bottom_profile_t_1[num] = transform_point_from_yz_to_xy_s(
                list(bottom_profile_t_1[num])
            )  # 起点
            bottom_profile_t_1[num] = adjust_drawing_scale(
                list(bottom_profile_t_1[num]), 1 / self.scale
            )
            bottom_profile_t_1[num] = tuple(bottom_profile_t_1[num])
        # 开始绘制填充轮廓点
        self.mid_slide_hatch.paths.add_polyline_path(bottom_profile_t_1, is_closed=1)
        self.mid_slide_hatch.set_pattern_fill("AR-SAND", scale=0.5 * 1 / self.scale)
        # 开始绘制直线
        for segment in bottom_outline_points_1:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        for segment in bottom_outline_points_2:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        # 文本1
        point_loc = bottom_outline_text_1[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = bottom_outline_text_1[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 文本2
        point_loc = bottom_outline_text_2[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = bottom_outline_text_2[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 底部滑动铰接缝2信息
        bottom_slide_two_info = (
            self.stair_cut_data.get_stair_bottom_slide_hatch_two_points()
        )
        bottom_profile_t_3 = bottom_slide_two_info["hatch_profile"]
        bottom_outline_points_3 = bottom_slide_two_info["outline"]
        bottom_outline_text_3 = bottom_slide_two_info["outline_text"]
        draw_line_2 = copy.deepcopy(bottom_profile_t_3)
        draw_line_2.append(bottom_profile_t_3[0])
        for num in range(len(draw_line_2)):
            point = draw_line_2[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_2[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_2) - 1):
            seg = [
                ((draw_line_2[num][0]), (draw_line_2[num][1]), (draw_line_2[num][2])),
                (
                    (draw_line_2[num + 1][0]),
                    (draw_line_2[num + 1][1]),
                    (draw_line_2[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(bottom_profile_t_3)):
            bottom_profile_t_3[num] = transform_point_from_yz_to_xy_s(
                list(bottom_profile_t_3[num])
            )  # 起点
            bottom_profile_t_3[num] = adjust_drawing_scale(
                list(bottom_profile_t_3[num]), 1 / self.scale
            )
            bottom_profile_t_3[num] = tuple(bottom_profile_t_3[num])
        # 开始绘制填充轮廓点
        self.bottom_slide_hatch.paths.add_polyline_path(bottom_profile_t_3, is_closed=1)
        self.bottom_slide_hatch.set_pattern_fill("AR-HBONE", scale=0.5 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in bottom_outline_points_3:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = bottom_outline_text_3[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = bottom_outline_text_3[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

        # 垫片和螺母
        # 螺母
        nut_info = self.stair_cut_data.get_stair_bottom_slide_nut_outline_points()
        nut_outline = nut_info["outline_points"]
        nut_text = nut_info["outline_text"]
        for segment in nut_outline:
            # start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            # end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(list(segment[0]), 1 / self.scale)
            end_point = adjust_drawing_scale(list(segment[1]), 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = nut_text[0]  # 文本防止点
        # point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = nut_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_bottom_fix_hatch_pattern(self):
        """
        开始绘制底部固定铰填充图案
        :return:
        """
        # 固定铰支座顶部填充内容
        bottom_fix_top_hatch_info = (
            self.stair_cut_data.get_stair_bottom_fix_node_top_hatch_pattern_points()
        )
        hatch_top_points = bottom_fix_top_hatch_info["hatch_points"]
        outline_top_points = bottom_fix_top_hatch_info["outline_points"]
        outline_top_text = bottom_fix_top_hatch_info["outline_text"]
        draw_line_1 = copy.deepcopy(hatch_top_points)
        draw_line_1.append(hatch_top_points[0])
        text_layer = "G_Note"
        line_layer = "G_Outline"
        for num in range(len(draw_line_1)):
            point = draw_line_1[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_1[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_1) - 1):
            seg = [
                ((draw_line_1[num][0]), (draw_line_1[num][1]), (draw_line_1[num][2])),
                (
                    (draw_line_1[num + 1][0]),
                    (draw_line_1[num + 1][1]),
                    (draw_line_1[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(hatch_top_points)):
            hatch_top_points[num] = transform_point_from_yz_to_xy_s(
                list(hatch_top_points[num])
            )  # 起点
            hatch_top_points[num] = adjust_drawing_scale(
                list(hatch_top_points[num]), 1 / self.scale
            )
            hatch_top_points[num] = tuple(hatch_top_points[num])
        # 开始绘制填充轮廓点
        self.top_fix_hatch.paths.add_polyline_path(hatch_top_points, is_closed=1)
        self.top_fix_hatch.set_pattern_fill("AR-SAND", scale=5 * 1 / self.scale)  # TODO
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_top_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_top_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_top_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 固定铰支座中部填充内容
        bottom_fix_mid_hatch_info = (
            self.stair_cut_data.get_stair_bottom_fix_node_mid_hatch_pattern_points()
        )
        hatch_mid_points = bottom_fix_mid_hatch_info["hatch_points"]
        outline_mid_points = bottom_fix_mid_hatch_info["outline_points"]
        outline_mid_text = bottom_fix_mid_hatch_info["outline_text"]
        draw_line_2 = copy.deepcopy(hatch_mid_points)
        draw_line_2.append(hatch_mid_points[0])
        for num in range(len(draw_line_2)):
            point = draw_line_2[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_2[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_2) - 1):
            seg = [
                ((draw_line_2[num][0]), (draw_line_2[num][1]), (draw_line_2[num][2])),
                (
                    (draw_line_2[num + 1][0]),
                    (draw_line_2[num + 1][1]),
                    (draw_line_2[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(outline_mid_points)):
            outline_mid_points[num] = transform_point_from_yz_to_xy_s(
                list(outline_mid_points[num])
            )  # 起点
            outline_mid_points[num] = adjust_drawing_scale(
                list(outline_mid_points[num]), 1 / self.scale
            )
            outline_mid_points[num] = tuple(outline_mid_points[num])
        # 开始绘制填充轮廓点
        self.mid_fix_hatch.paths.add_polyline_path(outline_mid_points, is_closed=1)
        self.mid_fix_hatch.set_pattern_fill("ANSI32", scale=5 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_mid_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_mid_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_mid_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

        # 底端固定铰支座底部填充内容
        bottom_slide_hatch_info = (
            self.stair_cut_data.get_stair_bottom_fix_hatch_points()
        )
        bottom_profile_t_1 = bottom_slide_hatch_info["hatch_profile"]
        bottom_outline_points_1 = bottom_slide_hatch_info["outline_1"]
        bottom_outline_points_2 = bottom_slide_hatch_info["outline_2"]
        bottom_outline_text_1 = bottom_slide_hatch_info["outline_text_1"]
        bottom_outline_text_2 = bottom_slide_hatch_info["outline_text_2"]
        draw_line_1 = copy.deepcopy(bottom_profile_t_1)
        draw_line_1.append(bottom_profile_t_1[0])
        for num in range(len(draw_line_1)):
            point = draw_line_1[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_1[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_1) - 1):
            seg = [
                ((draw_line_1[num][0]), (draw_line_1[num][1]), (draw_line_1[num][2])),
                (
                    (draw_line_1[num + 1][0]),
                    (draw_line_1[num + 1][1]),
                    (draw_line_1[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(bottom_profile_t_1)):
            bottom_profile_t_1[num] = transform_point_from_yz_to_xy_s(
                list(bottom_profile_t_1[num])
            )  # 起点
            bottom_profile_t_1[num] = adjust_drawing_scale(
                list(bottom_profile_t_1[num]), 1 / self.scale
            )
            bottom_profile_t_1[num] = tuple(bottom_profile_t_1[num])
        # 开始绘制填充轮廓点
        self.bottom_fix_hatch.paths.add_polyline_path(bottom_profile_t_1, is_closed=1)
        self.bottom_fix_hatch.set_pattern_fill("AR-SAND", scale=5 * 1 / self.scale)
        # 开始绘制直线
        for segment in bottom_outline_points_1:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        for segment in bottom_outline_points_2:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        # 文本1
        point_loc = bottom_outline_text_1[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = bottom_outline_text_1[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 文本2
        point_loc = bottom_outline_text_2[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = bottom_outline_text_2[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_beam_slab_outline_shape(self):
        """
        开始绘制平台梁和平台板引出线形状图
        :return:
        """
        outline_info = self.stair_cut_data.get_stair_bottom_beam_outline_points()
        outline_points = outline_info["outline_points"]
        outline_text = outline_info["outline_text"]
        text_layer = "G_Note"
        line_layer = "G_Outline"
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        # 文本1
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_judge_node_type_to_draw(self):
        """
        通过判断节点类型进而绘图
        :return:
        """
        if (
            self.detailed_design.construction_detailed.bottom_hole_type.value != 2
        ):  # 0--固定铰，1--滑动铰
            self.begin_draw_bottom_slide_hatch_pattern()
        else:
            self.begin_draw_bottom_fix_hatch_pattern()

    def main_run_process(self):
        """
        开始运行过程
        :return:
        """
        self.begin_draw_stair_bottom_beam_slab_profile_shape()
        self.begin_draw_stair_bottom_local_shape_profile_points()
        self.begin_draw_stair_bottom_hole_shape_profile_points()
        self.begin_draw_stair_bottom_connect_embedded_shape_profile_points()
        # self.begin_draw_bottom_shim_hatch_pattern()  # 开始绘制底部垫片填充图案
        self.begin_draw_stair_beam_slab_left_boundary_breakline()  # 开始绘制平台梁和板左侧折断线
        self.begin_draw_stair_local_right_boundary_breakline()  # 开始绘制楼梯底部右侧局部折断线
        self.begin_draw_stair_elevation_line_shape()  # 开始绘制标高线
        self.begin_draw_stair_elevation_sign_shape()  # 开始绘制标高符号和文本内容
        self.begin_draw_stair_construction_dimension_points()  # 开始尺寸标注
        # self.begin_draw_stair_bottom_glue_hatch_shape()
        # self.begin_draw_stair_mid_hatch_pattern_shape()  # 开始绘制中部
        # self.begin_draw_stair_pe_rod_hatch_shape()  # 开始填充PE棒
        # self.begin_judge_node_type_to_draw()
        self.begin_draw_beam_slab_outline_shape()  # 开始绘制梯梁填充图


class StairTopInstallNodeView(object):
    """
    产生楼梯顶端安装节点图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairTopInstallNodeViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.top_install_view = self.dxf_doc
        self.model_space = self.top_install_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        :return:
        """
        self.top_install_shape = self.top_install_view.blocks.new(
            "top_install_view"
        )  # 添加安装视图块
        self.hole_shape = self.top_install_view.blocks.new(
            "top_install_hole_view"
        )  # 添加孔洞视图块
        self.connect_embedded = self.top_install_view.blocks.new(
            "top_install_connect_embedded"
        )  # 添加连接件
        self.line_dimension = self.top_install_view.blocks.new(
            "top_install_line_dimension"
        )  # 添加线性标注块
        self.break_line = self.top_install_view.blocks.new(
            "top_install_break_line"
        )  # 添加折断线块
        self.elevation_shape = self.top_install_view.blocks.new(
            "top_install_elevation_line"
        )  # 添加标高线块
        self.outline = self.top_install_view.blocks.new("top_install_outLine")  # 引出线块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.shim_hatch = self.model_space.add_hatch(color=7)
        self.glue_hatch = self.model_space.add_hatch()
        self.mid_hatch = self.model_space.add_hatch()
        self.top_hatch = self.model_space.add_hatch()
        self.bottom_slide_hatch = self.model_space.add_hatch()
        self.mid_slide_hatch = self.model_space.add_hatch()
        self.top_slide_hatch = self.model_space.add_hatch()
        self.bottom_fix_hatch = self.model_space.add_hatch()
        self.mid_fix_hatch = self.model_space.add_hatch()
        self.top_fix_hatch = self.model_space.add_hatch()
        self.pe_hatch = self.model_space.add_hatch(color=7)

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.top_install_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer}
                )

    def begin_draw_stair_top_beam_slab_profile_shape(self):
        """
        开始绘制楼梯顶部梁和板轮廓形状
        :return:
        """
        profile_points = self.stair_cut_data.get_top_beam_slab_bounding_profile_points()
        layer_name = "G_Special_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_stair_top_local_shape_profile_points(self):
        """
        开始绘制楼梯顶部局部形状轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_top_local_shape_bounding_profile_points()
        )
        layer_name = "G_Special_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_stair_top_hole_shape_profile_points(self):
        """
        开始绘制楼梯顶部孔洞形状轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_top_right_hole_bounding_profile_points()
        )
        layer_name = "G_Hole_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_stair_top_connect_embedded_shape_profile_points(self):
        """
        开始绘制楼梯顶部连接埋件形状轮廓形状
        :return:
        """
        rebar_profile_points = (
            self.stair_cut_data.get_stair_solid_top_anchor_rebar_cut_drawing()
        )  # 顶部锚固钢筋数据
        connect_embedded_layer = "G_Rebar"
        # 变换绘制比例
        for segment in rebar_profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in rebar_profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.connect_embedded.add_line(
                    seg[0], seg[1], dxfattribs={"layer": connect_embedded_layer}
                )
        if (
            self.detailed_design.construction_detailed.top_hole_type.value == 1
        ):  # 顶端连接节点,节点类型与孔洞类型对应,0--固定铰，1--滑动铰
            nut_profile_points = (
                self.stair_cut_data.get_stair_top_connect_nut_projection_drawing()
            )  # 顶部螺母数据
            shim_profile_points = (
                self.stair_cut_data.get_stair_top_connect_shim_projection_drawing()
            )  # 顶部垫片数据
            # 螺母--变换绘制比例
            for segment in nut_profile_points:
                for num in range(len(segment)):
                    segment[num] = transform_point_from_xy_to_yx(
                        list(segment[num])
                    )  # 起点
                    segment[num] = adjust_drawing_scale(
                        list(segment[num]), 1 / self.scale
                    )
            for segment in nut_profile_points:
                for num in range(len(segment) - 1):
                    seg = [
                        (
                            abs(segment[num][0]),
                            abs(segment[num][1]),
                            abs(segment[num][2]),
                        ),
                        (
                            abs(segment[num + 1][0]),
                            abs(segment[num + 1][1]),
                            abs(segment[num + 1][2]),
                        ),
                    ]  # 将三维点转化为平面点
                    self.connect_embedded.add_line(
                        seg[0], seg[1], dxfattribs={"layer": connect_embedded_layer}
                    )
            # 垫片--变换绘制比例
            for segment in shim_profile_points:
                for num in range(len(segment)):
                    segment[num] = transform_point_from_xy_to_yx(
                        list(segment[num])
                    )  # 起点
                    segment[num] = adjust_drawing_scale(
                        list(segment[num]), 1 / self.scale
                    )
            for segment in shim_profile_points:
                for num in range(len(segment) - 1):
                    seg = [
                        (
                            abs(segment[num][0]),
                            abs(segment[num][1]),
                            abs(segment[num][2]),
                        ),
                        (
                            abs(segment[num + 1][0]),
                            abs(segment[num + 1][1]),
                            abs(segment[num + 1][2]),
                        ),
                    ]  # 将三维点转化为平面点
                    self.connect_embedded.add_line(
                        seg[0], seg[1], dxfattribs={"layer": connect_embedded_layer}
                    )

    def begin_draw_top_shim_hatch_pattern(self):
        """
        获取底部垫片填充图案
        :return: None
        """
        if (
            self.detailed_design.construction_detailed.top_hole_type.value == 1
        ):  # 顶端连接节点,节点类型与孔洞类型对应,0--固定铰，1--滑动铰
            hatch_points = self.stair_cut_data.get_stair_top_shim_corner_point()
            # 垫片--变换绘制比例
            for num in range(len(hatch_points)):
                hatch_points[num] = transform_point_from_xy_to_yx(
                    list(hatch_points[num])
                )  # 起点
                hatch_points[num] = adjust_drawing_scale(
                    list(hatch_points[num]), 1 / self.scale
                )
            # 开始绘制填充轮廓点
            self.shim_hatch.paths.add_polyline_path(hatch_points, is_closed=True)

    def begin_draw_stair_beam_slab_left_boundary_breakline(self):
        """
        开始绘制楼梯平台板和梁左侧边界折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_left_boundary_breakline()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.break_line.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_draw_stair_local_right_boundary_breakline(self):
        """
        开始绘制楼梯底端局部边界折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_right_boundary_breakline()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.break_line.add_line(seg[0], seg[1], dxfattribs={"layer": layer_name})

    def begin_draw_stair_elevation_line_shape(self):
        """
        开始绘制标高线
        :return:
        """
        elevation_line_points = (
            self.stair_cut_data.get_stair_top_beam_slab_elevation_line_points()
        )  # 获取标高线上的点
        layer_name = "G_Elevation_Dashed"
        # 调整各点的比例
        for num in range(len(elevation_line_points)):
            point = elevation_line_points[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            elevation_line_points[num] = adjust_drawing_scale(
                list(point), 1 / self.scale
            )
        # 遍历线段点
        for num in range(len(elevation_line_points) - 1):
            seg = [
                (
                    (elevation_line_points[num][0]),
                    (elevation_line_points[num][1]),
                    (elevation_line_points[num][2]),
                ),
                (
                    (elevation_line_points[num + 1][0]),
                    (elevation_line_points[num + 1][1]),
                    (elevation_line_points[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.elevation_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_elevation_sign_shape(self):
        """
        开始绘制楼梯标高符号
        :return:
        """
        elevation_info = (
            self.stair_cut_data.get_stair_solid_elevation_special_point()
        )  # 标高信息
        draw_points = elevation_info[0]  # 标高线点
        text_info = elevation_info[1]  # 文字说明
        line_layer_name = "G_Elevation_Line"
        text_layer = "G_Note"
        # 开始绘制符号形状
        for segment in draw_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in draw_points:
            for num in range(len(segment) - 1):
                seg = [
                    ((segment[num][0]), (segment[num][1]), (segment[num][2])),
                    (
                        (segment[num + 1][0]),
                        (segment[num + 1][1]),
                        (segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                self.elevation_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": line_layer_name}
                )
        # 开始设置标高文本
        # 添加文本
        current_point = text_info[0]
        current_point = transform_point_from_yz_to_xy_s(list(current_point))  # 终点
        current_point = adjust_drawing_scale(current_point, 1 / self.scale)
        text = text_info[1]
        self.elevation_shape.add_text(
            text, height=self.note_text_size, dxfattribs={"layer": text_layer}
        ).set_placement(
            current_point, align=TextEntityAlignment.BOTTOM_LEFT
        )  # 防止重叠

    def begin_draw_stair_construction_dimension_points(self):
        """
        获取楼梯尺寸标注点
        :return:
        """
        dimension_loc = (
            self.stair_cut_data.get_stair_construction_method_dimension_points()
        )  # 获取楼梯构造标注点
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.15
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 计算机存储精度
                dim = self.line_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.line_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_top_glue_hatch_shape(self):
        """
        开始绘制楼梯顶端打胶填充图案形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_top_glue_hatch_points()
        profile_points = hatch_info["hatch_profile"]
        outline_points = hatch_info["outline"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_line = "G_Outline"
        text_layer = "G_note"
        for num in range(len(draw_line)):
            point = draw_line[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_line}
            )
        # 开始填充图案
        for num in range(len(profile_points)):
            profile_points[num] = transform_point_from_yz_to_xy_s(
                list(profile_points[num])
            )  # 起点
            profile_points[num] = adjust_drawing_scale(
                list(profile_points[num]), 1 / self.scale
            )
            profile_points[num] = tuple(profile_points[num])
        # 开始绘制填充轮廓点
        self.glue_hatch.paths.add_polyline_path(profile_points, is_closed=1)
        self.glue_hatch.set_pattern_fill("ANSI31", scale=0.5 * 1 / self.scale)  # TODO
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_mid_hatch_pattern_shape(self):
        """
        开始绘制楼梯顶端中部填充图案形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_top_mid_hatch_pattern_points()
        profile_points = hatch_info["hatch_profile"]
        outline_points = hatch_info["outline"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        line_layer = "G_Outline"
        text_layer = "G_Note"
        for num in range(len(draw_line)):
            point = draw_line[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(profile_points)):
            profile_points[num] = transform_point_from_yz_to_xy_s(
                list(profile_points[num])
            )  # 起点
            profile_points[num] = adjust_drawing_scale(
                list(profile_points[num]), 1 / self.scale
            )
            profile_points[num] = tuple(profile_points[num])
        # 开始绘制填充轮廓点
        self.mid_hatch.paths.add_polyline_path(profile_points, is_closed=1)
        self.mid_hatch.set_pattern_fill("HONEY", scale=2 * 1 / self.scale)  # TODO
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_pe_rod_hatch_shape(self):
        """
        开始绘制楼梯PE棒填充图案
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_top_pe_rod_hatch_points()
        profile_points = hatch_info["hatch_profile"]
        center = profile_points[0]  # 圆心
        diameter = profile_points[1]  # 直径
        outline_points = hatch_info["outline"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        text_layer = "G_Note"
        line_layer = "G_Outline"
        # 开始绘制圆
        center = transform_point_from_yz_to_xy_s(list(center))  # 起点
        center = adjust_drawing_scale(list(center), 1 / self.scale)
        diameter = diameter * (1 / self.scale)
        self.top_install_shape.add_circle(
            tuple(center), diameter / 2, dxfattribs={"layer": line_layer}
        )
        # 开始填充图案
        # 开始绘制填充轮廓点
        edge_path = self.pe_hatch.paths.add_edge_path()  # 开始绘制引出线
        edge_path.add_arc(tuple(center), diameter / 2)
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_top_slide_hatch_pattern(self):
        """
        开始绘制顶部滑动铰填充图案
        :return:
        """
        # 填充图案
        # 1.顶部滑动铰填充信息
        top_slide_info = self.stair_cut_data.get_stair_top_slide_hatch_points()
        top_profile_t = top_slide_info["hatch_profile"]
        top_outline_points = top_slide_info["outline"]
        top_outline_text = top_slide_info["outline_text"]
        draw_line = copy.deepcopy(top_profile_t)
        draw_line.append(top_profile_t[0])
        text_layer = "G_Note"
        line_layer = "G_Outline"
        for num in range(len(draw_line)):
            point = draw_line[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(top_profile_t)):
            top_profile_t[num] = transform_point_from_yz_to_xy_s(
                list(top_profile_t[num])
            )  # 起点
            top_profile_t[num] = adjust_drawing_scale(
                list(top_profile_t[num]), 1 / self.scale
            )
            top_profile_t[num] = tuple(top_profile_t[num])
        # 开始绘制填充轮廓点
        self.top_slide_hatch.paths.add_polyline_path(top_profile_t, is_closed=1)
        self.top_slide_hatch.set_pattern_fill("AR-SAND", scale=0.5 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in top_outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = top_outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = top_outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 顶部节点接缝1图案填充信息
        top_hatch_one_info = self.stair_cut_data.get_stair_top_slide_hatch_one_points()
        top_profile_t_1 = top_hatch_one_info["hatch_profile"]
        top_outline_points_1 = top_hatch_one_info["outline_1"]
        top_outline_points_2 = top_hatch_one_info["outline_2"]
        top_outline_text_1 = top_hatch_one_info["outline_text_1"]
        top_outline_text_2 = top_hatch_one_info["outline_text_2"]
        draw_line_1 = copy.deepcopy(top_profile_t_1)
        draw_line_1.append(top_profile_t_1[0])
        for num in range(len(draw_line_1)):
            point = draw_line_1[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_1[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_1) - 1):
            seg = [
                ((draw_line_1[num][0]), (draw_line_1[num][1]), (draw_line_1[num][2])),
                (
                    (draw_line_1[num + 1][0]),
                    (draw_line_1[num + 1][1]),
                    (draw_line_1[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(top_profile_t_1)):
            top_profile_t_1[num] = transform_point_from_yz_to_xy_s(
                list(top_profile_t_1[num])
            )  # 起点
            top_profile_t_1[num] = adjust_drawing_scale(
                list(top_profile_t_1[num]), 1 / self.scale
            )
            top_profile_t_1[num] = tuple(top_profile_t_1[num])
        # 开始绘制填充轮廓点
        self.mid_slide_hatch.paths.add_polyline_path(top_profile_t_1, is_closed=1)
        self.mid_slide_hatch.set_pattern_fill(
            "AR-SAND", scale=0.5 * 1 / self.scale
        )  # TODO
        # 开始绘制直线
        for segment in top_outline_points_1:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        for segment in top_outline_points_2:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        # 文本1
        point_loc = top_outline_text_1[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = top_outline_text_1[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 文本2
        point_loc = top_outline_text_2[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = top_outline_text_2[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 顶部滑动铰接缝2信息
        top_slide_two_info = self.stair_cut_data.get_stair_top_slide_hatch_two_points()
        top_profile_t_3 = top_slide_two_info["hatch_profile"]
        top_outline_points_3 = top_slide_two_info["outline"]
        top_outline_text_3 = top_slide_two_info["outline_text"]
        draw_line_2 = copy.deepcopy(top_profile_t_3)
        draw_line_2.append(top_profile_t_3[0])
        for num in range(len(draw_line_2)):
            point = draw_line_2[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_2[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_2) - 1):
            seg = [
                ((draw_line_2[num][0]), (draw_line_2[num][1]), (draw_line_2[num][2])),
                (
                    (draw_line_2[num + 1][0]),
                    (draw_line_2[num + 1][1]),
                    (draw_line_2[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(top_profile_t_3)):
            top_profile_t_3[num] = transform_point_from_yz_to_xy_s(
                list(top_profile_t_3[num])
            )  # 起点
            top_profile_t_3[num] = adjust_drawing_scale(
                list(top_profile_t_3[num]), 1 / self.scale
            )
            top_profile_t_3[num] = tuple(top_profile_t_3[num])
        # 开始绘制填充轮廓点
        self.bottom_slide_hatch.paths.add_polyline_path(top_profile_t_3, is_closed=1)
        self.bottom_slide_hatch.set_pattern_fill(
            "AR-HBONE", scale=0.5 * 1 / self.scale
        )  # TODO
        # 开始绘制引出线
        # 开始绘制直线
        for segment in top_outline_points_3:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = top_outline_text_3[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = top_outline_text_3[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

        # 垫片和螺母
        # 螺母
        nut_info = self.stair_cut_data.get_stair_top_slide_nut_outline_points()
        nut_outline = nut_info["outline_points"]
        nut_text = nut_info["outline_text"]
        for segment in nut_outline:
            # start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            # end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(list(segment[0]), 1 / self.scale)
            end_point = adjust_drawing_scale(list(segment[1]), 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = nut_text[0]  # 文本防止点
        # point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = nut_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_top_fix_hatch_pattern(self):
        """
        开始绘制顶部固定铰填充图案
        :return:
        """
        # 固定铰支座顶部填充内容
        top_fix_top_hatch_info = (
            self.stair_cut_data.get_stair_top_fix_node_top_hatch_pattern_points()
        )
        hatch_top_points = top_fix_top_hatch_info["hatch_points"]
        outline_top_points = top_fix_top_hatch_info["outline_points"]
        outline_top_text = top_fix_top_hatch_info["outline_text"]
        draw_line_1 = copy.deepcopy(hatch_top_points)
        draw_line_1.append(hatch_top_points[0])
        text_layer = "G_Note"
        line_layer = "G_Outline"
        for num in range(len(draw_line_1)):
            point = draw_line_1[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_1[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_1) - 1):
            seg = [
                ((draw_line_1[num][0]), (draw_line_1[num][1]), (draw_line_1[num][2])),
                (
                    (draw_line_1[num + 1][0]),
                    (draw_line_1[num + 1][1]),
                    (draw_line_1[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(hatch_top_points)):
            hatch_top_points[num] = transform_point_from_yz_to_xy_s(
                list(hatch_top_points[num])
            )  # 起点
            hatch_top_points[num] = adjust_drawing_scale(
                list(hatch_top_points[num]), 1 / self.scale
            )
            hatch_top_points[num] = tuple(hatch_top_points[num])
        # 开始绘制填充轮廓点
        self.top_fix_hatch.paths.add_polyline_path(hatch_top_points, is_closed=1)
        self.top_fix_hatch.set_pattern_fill(
            "AR-SAND", scale=0.1 * 1 / self.scale
        )  # TODO
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_top_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_top_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_top_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 固定铰支座中部填充内容
        top_fix_mid_hatch_info = (
            self.stair_cut_data.get_stair_top_fix_node_mid_hatch_pattern_points()
        )
        hatch_mid_points = top_fix_mid_hatch_info["hatch_points"]
        outline_mid_points = top_fix_mid_hatch_info["outline_points"]
        outline_mid_text = top_fix_mid_hatch_info["outline_text"]
        draw_line_2 = copy.deepcopy(hatch_mid_points)
        draw_line_2.append(hatch_mid_points[0])
        for num in range(len(draw_line_2)):
            point = draw_line_2[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_2[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_2) - 1):
            seg = [
                ((draw_line_2[num][0]), (draw_line_2[num][1]), (draw_line_2[num][2])),
                (
                    (draw_line_2[num + 1][0]),
                    (draw_line_2[num + 1][1]),
                    (draw_line_2[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for m in range(len(hatch_mid_points)):
            current_p = hatch_mid_points[m]
            current_p = transform_point_from_yz_to_xy_s(list(current_p))  # 起点
            current_p = adjust_drawing_scale(list(current_p), 1 / self.scale)
            hatch_mid_points[m] = tuple(current_p)
        # 开始绘制填充轮廓点
        self.mid_fix_hatch.paths.add_polyline_path(hatch_mid_points, is_closed=1)
        self.mid_fix_hatch.set_pattern_fill("ANSI32", scale=1 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_mid_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        point_loc = outline_mid_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_mid_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

        # 顶端固定铰支座底部填充内容
        top_slide_hatch_info = self.stair_cut_data.get_stair_top_fix_hatch_points()
        top_profile_t_1 = top_slide_hatch_info["hatch_profile"]
        top_outline_points_1 = top_slide_hatch_info["outline_1"]
        top_outline_points_2 = top_slide_hatch_info["outline_2"]
        top_outline_text_1 = top_slide_hatch_info["outline_text_1"]
        top_outline_text_2 = top_slide_hatch_info["outline_text_2"]
        draw_line_1 = copy.deepcopy(top_profile_t_1)
        draw_line_1.append(top_profile_t_1[0])
        for num in range(len(draw_line_1)):
            point = draw_line_1[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_line_1[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line_1) - 1):
            seg = [
                ((draw_line_1[num][0]), (draw_line_1[num][1]), (draw_line_1[num][2])),
                (
                    (draw_line_1[num + 1][0]),
                    (draw_line_1[num + 1][1]),
                    (draw_line_1[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_install_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": line_layer}
            )
        # 开始填充图案
        for num in range(len(top_profile_t_1)):
            top_profile_t_1[num] = transform_point_from_yz_to_xy_s(
                list(top_profile_t_1[num])
            )  # 起点
            top_profile_t_1[num] = adjust_drawing_scale(
                list(top_profile_t_1[num]), 1 / self.scale
            )
            top_profile_t_1[num] = tuple(top_profile_t_1[num])
        # 开始绘制填充轮廓点
        self.bottom_fix_hatch.paths.add_polyline_path(top_profile_t_1, is_closed=1)
        self.bottom_fix_hatch.set_pattern_fill(
            "AR-SAND", scale=0.1 * 1 / self.scale
        )  # TODO
        # 开始绘制直线
        for segment in top_outline_points_1:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        for segment in top_outline_points_2:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        # 文本1
        point_loc = top_outline_text_1[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = top_outline_text_1[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)
        # 文本2
        point_loc = top_outline_text_2[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = top_outline_text_2[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_beam_slab_outline_shape(self):
        """
        开始绘制平台梁和平台板引出线形状图
        :return:
        """
        outline_info = self.stair_cut_data.get_stair_top_beam_outline_points()
        outline_points = outline_info["outline_points"]
        outline_text = outline_info["outline_text"]
        text_layer = "G_Note"
        line_layer = "G_Outline"
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": line_layer}
            )
        # 开始增加文本
        # 文本1
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_judge_node_type_to_draw(self):
        """
        通过判断节点类型进而绘图
        :return:
        """
        if (
            self.detailed_design.construction_detailed.top_hole_type.value == 1
        ):  # 顶部孔洞类型与节点类型对应，0--固定铰，1---滑动铰
            self.begin_draw_top_slide_hatch_pattern()
        else:
            self.begin_draw_top_fix_hatch_pattern()

    def main_run_process(self):
        """
        开始运行过程
        :return:
        """
        self.begin_draw_stair_top_beam_slab_profile_shape()
        self.begin_draw_stair_top_local_shape_profile_points()
        self.begin_draw_stair_top_hole_shape_profile_points()
        self.begin_draw_stair_top_connect_embedded_shape_profile_points()
        self.begin_draw_stair_beam_slab_left_boundary_breakline()  # 开始绘制平台梁和板左侧折断线
        self.begin_draw_stair_local_right_boundary_breakline()  # 开始绘制楼梯底部右侧局部折断线
        self.begin_draw_stair_elevation_line_shape()  # 开始绘制标高线
        self.begin_draw_stair_elevation_sign_shape()  # 开始绘制标高符号和文本内容
        self.begin_draw_stair_construction_dimension_points()  # 开始尺寸标注
        # self.begin_draw_stair_top_glue_hatch_shape()
        # self.begin_draw_stair_mid_hatch_pattern_shape()  # 开始绘制中部
        # self.begin_draw_stair_pe_rod_hatch_shape()  # 开始填充PE棒
        self.begin_draw_beam_slab_outline_shape()  # 开始绘制梯梁填充图
        # self.begin_draw_top_shim_hatch_pattern()  # 开始绘制顶部垫片填充图案
        # self.begin_judge_node_type_to_draw()


class StairBottomHoleReinRebarView(object):
    """
    楼梯底部孔洞加强筋视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairBottomHoleReinRebarViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )

        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        创建dxf文件
        :return:
        """
        self.bottom_hole_rein_view = self.dxf_doc
        self.model_space = self.bottom_hole_rein_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.bottom_hole_rein_shape = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_view"
        )  # 添加轮廓视图块
        self.bottom_hole_left_dimension = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_left_dimension"
        )  # 添加左侧线性标注块
        self.bottom_hole_top_dimension = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_top_dimension"
        )  # 添加俯视线性标注块
        self.bottom_hole_break_line = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_break_line"
        )  # 添加折断线块
        self.bottom_hole_outline = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_outline"
        )  # 引出线块
        self.bottom_hole_cut_outline = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_cut_outline"
        )  # 引出线块
        self.bottom_hole_rein_rebar = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_rein_rebar"
        )  # 孔洞加强钢筋块
        self.bottom_cut_hole_rein_rebar_shape = self.bottom_hole_rein_view.blocks.new(
            "bottom_hole_cut_rein_rebar"
        )  # 孔洞顶部加强钢筋块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.hole_hatch = self.model_space.add_hatch(color=4)

    def begin_draw_standard_polyline(self, profile_points, layer):
        """
        开始绘制标准多线段
        :return:
        """
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.bottom_hole_rein_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer}
                )

    def begin_draw_stair_bottom_local_profile_shape(self):
        """
        开始绘制楼梯底端局部轮廓点
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_bottom_local_bounding_profile_points()
        )
        layer_name = "G_Special_Outline"
        self.begin_draw_standard_polyline(profile_points, layer_name)

    def begin_draw_bottom_breakline_shape(self):
        """
        开始绘制底部折断线形状
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_bottom_left_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.bottom_hole_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_left_hole_rein_rebar(self):
        """
        绘制孔洞加强筋投影图
        :return:None
        """
        hole_rein_rebar_loc = (
            self.stair_cut_data.get_stair_hole_rein_rebar_left_projection()
        )  # 获取孔洞加强筋数据
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(hole_rein_rebar_loc)):
            for k in range(len(hole_rein_rebar_loc[num])):
                hole_rein_rebar_loc[num][k][0] = transform_point_from_yz_to_xy_s(
                    list(hole_rein_rebar_loc[num][k][0])
                )  # 起点
                hole_rein_rebar_loc[num][k][1] = transform_point_from_yz_to_xy_s(
                    list(hole_rein_rebar_loc[num][k][1])
                )  # 终点
                hole_rein_rebar_loc[num][k][0] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][0], 1 / self.scale
                )
                hole_rein_rebar_loc[num][k][1] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][1], 1 / self.scale
                )
        for rebar in hole_rein_rebar_loc:
            for k in range(len(rebar)):
                segment = rebar[k]  # 钢筋当前段
                seg = [
                    (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                    (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
                ]  # 将三维点转化为平面点
                self.bottom_hole_rein_rebar.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_bottom_hole_shape(self):
        """
        开始绘制楼梯底部孔洞形状
        :return:
        """
        hole_data = (
            self.stair_cut_data.get_stair_bottom_left_hole_bounding_profile_points()
        )  # 获取孔洞数据
        layer_name = "G_Hole_Outline"
        # 变换绘制比例
        for num in range(len(hole_data)):
            hole_data[num][0] = transform_point_from_yz_to_xy_s(
                list(hole_data[num][0])
            )  # 起点
            hole_data[num][1] = transform_point_from_yz_to_xy_s(
                list(hole_data[num][1])
            )  # 终点
            hole_data[num][0] = adjust_drawing_scale(hole_data[num][0], 1 / self.scale)
            hole_data[num][1] = adjust_drawing_scale(hole_data[num][1], 1 / self.scale)
        for segment in hole_data:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.bottom_hole_rein_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_bottom_hole_dimension_points(self):
        """
        获取楼梯底部孔洞标注点
        :return:
        """
        if (
            self.detailed_design.construction_detailed.bottom_hole_type.value == 1
        ):  # 0--固定铰，1---滑动铰
            dimension_info = (
                self.stair_cut_data.get_stair_slide_hole_dimension_points()
            )  # 获取楼梯滑动孔洞标注点
            special_dimension = dimension_info["special_dimension"]
            special_dimension_points = special_dimension[0]
            special_dimension_text = special_dimension[1]
            basic_dimension = dimension_info["basic_dimension"]
            # 特殊文本标注
            for num in range(len(special_dimension_points)):
                # 调整该层标注相对实体偏移量
                segment = special_dimension_points[num]
                text = special_dimension_text[num]
                current_offset = self.dimension_offset * 0.25
                start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
                end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                    dim = self.bottom_hole_left_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        text=text,
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.bottom_hole_left_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        text=text,
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()
            # 常规线性标注
            for segment in basic_dimension:
                # 调整该层标注相对实体偏移量
                current_offset = self.dimension_offset * 0.25
                # 调整点的比例
                start_point = transform_point_from_yz_to_xy_s(list(segment[0]))  # 起点
                end_point = transform_point_from_yz_to_xy_s(list(segment[1]))  # 终点
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                    dim = self.bottom_hole_left_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        dimstyle="G_DS_20",  # 默认是1:100进行绘制，输入的数值是绘制的大小，实际大小需要乘以绘图比例。
                    )
                else:
                    dim = self.bottom_hole_left_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()
        else:
            dimension_info = self.stair_cut_data.get_stair_fix_hole_dimension_points()
            dimension_points = dimension_info[0]
            dimension_text = dimension_info[1]
            for num in range(len(dimension_points)):
                # 调整该层标注相对实体偏移量
                segment = dimension_points[num]
                text = dimension_text[num]
                current_offset = self.dimension_offset * 0.25
                start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
                end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                    dim = self.bottom_hole_left_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        text=text,
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.bottom_hole_left_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        text=text,
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()

    def begin_draw_stair_dimension_profile_points(self):
        """
        开始绘制楼梯轮廓标注点
        :return:
        """
        dimension_info = self.stair_cut_data.get_stair_profile_dimension_points()
        first_floor_dimension = dimension_info["first_floor"]
        second_floor_dimension = dimension_info["second_floor"]
        # 第一层标注点
        for segment in first_floor_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))  # 起点
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.bottom_hole_left_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.bottom_hole_left_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 第二层标注点
        for segment in second_floor_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))  # 起点
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.bottom_hole_left_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.bottom_hole_left_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_outline_shape(self):
        """
        开始绘制楼梯引出线形状
        :return:
        """
        outline_info = self.stair_cut_data.get_stair_hole_rein_rebar_outline_points()
        outline_points = outline_info["outline_points"]
        outline_text = outline_info["outline_text"]
        layer_line = "G_Label"
        text_layer = "G_note"
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.bottom_hole_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.bottom_hole_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def get_stair_cut_rein_rebar_place_point(self):
        """
        获取楼梯剖切钢筋块放置点
        :return:
        """
        bounding_box_loc = (
            self.stair_cut_data.get_stair_bottom_local_profile_bounding_box()
        )
        max_z = 0
        for point in bounding_box_loc:
            if max_z < point[2]:
                max_z = point[2]
        block_placement_point = [bounding_box_loc[0][0], 0, max_z + self.tabu_h]
        return block_placement_point

    def begin_draw_stair_cut_rein_rebar_profile_shape(self):
        """
        开始绘制楼梯加强筋俯视轮廓形状图
        :return:
        """
        profile_points = self.stair_cut_data.get_stair_hole_rein_rebar_bounding_points()
        layer_line = "G_Rebar"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_xy_to_yx(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.bottom_cut_hole_rein_rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_line}
                )

    def begin_draw_stair_cut_rein_rebar_standard_hatch_shape(
        self, draw_circle, draw_spline, outline_points, outline_text
    ):
        """
        开始绘制标准填充形状
        :param draw_circle:
        :param draw_spline:
        :param outline_points:
        :param outline_text:
        :return:
        """
        # 获取基本数据
        circle_loc_points = draw_circle[0]
        circle_diameter = draw_circle[1]
        spline_points = [draw_spline[0]]
        arc_points = draw_spline[1]
        layer_line = "G_Hole_Outline_Continuous"
        layer_outline = "G_Label"
        text_layer = "G_Note"
        # 开始绘制形状
        for num in range(len(circle_loc_points)):
            center = circle_loc_points[num]
            diameter = circle_diameter[num]
            center = transform_point_from_xy_to_yx(list(center))
            center = adjust_drawing_scale(list(center), 1 / self.scale)
            diameter = diameter * 1 / self.scale
            self.bottom_cut_hole_rein_rebar_shape.add_circle(
                tuple(center), diameter / 2, dxfattribs={"layer": layer_line}
            )
        # 开始填充--添加圆弧形状
        hole_hatch = self.hole_hatch.paths.add_edge_path()
        arc_points[0] = transform_point_from_xy_to_yx(list(arc_points[0]))
        arc_points[0] = adjust_drawing_scale(list(arc_points[0]), 1 / self.scale)
        arc_points[1] = arc_points[1] * 1 / self.scale
        hole_hatch.add_arc(
            tuple(arc_points[0]), arc_points[1] / 2, arc_points[2], arc_points[3]
        )
        for segment in spline_points:
            segment[0] = transform_point_from_xy_to_yx(list(segment[0]))
            segment[0] = adjust_drawing_scale(list(segment[0]), 1 / self.scale)
            segment[1] = transform_point_from_xy_to_yx(list(segment[1]))
            segment[1] = adjust_drawing_scale(list(segment[1]), 1 / self.scale)
            segment[2] = transform_point_from_xy_to_yx(list(segment[2]))
            segment[2] = adjust_drawing_scale(list(segment[2]), 1 / self.scale)
            seg = [
                tuple(segment[0]),
                tuple(segment[1]),
                tuple(segment[2]),
            ]  # 将三维点转化为平面点
            self.bottom_cut_hole_rein_rebar_shape.add_spline(
                seg, dxfattribs={"layer": layer_line}
            )  # 圆弧线
            hole_hatch.add_spline(control_points=seg)
        # 开始绘制引出线
        for segment in outline_points:
            start_point = transform_point_from_xy_to_yx(list(segment[0]))
            end_point = transform_point_from_xy_to_yx(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.bottom_hole_cut_outline.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": layer_outline},
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_xy_to_yx(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.bottom_hole_cut_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_cut_rein_rebar_hatch_shape(self):
        """
        开始绘制楼梯剖切加强筋填充形状
        :return:
        """
        if (
            self.detailed_design.construction_detailed.bottom_hole_type.value == 1
        ):  # 滑动铰支座---1,固定铰--0
            hatch_info = (
                self.stair_cut_data.get_stair_slide_rein_rebar_top_hatch_points()
            )  # 开始楼梯滑动剖切加强筋填充点
        else:  # 固定支座
            hatch_info = (
                self.stair_cut_data.get_stair_fix_rein_rebar_top_hatch_points()
            )  # 开始楼梯滑动剖切加强筋填充点
        draw_circle = hatch_info["draw_circle"]
        draw_spline = hatch_info["draw_spline"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        self.begin_draw_stair_cut_rein_rebar_standard_hatch_shape(
            draw_circle, draw_spline, outline_points, outline_text
        )

    def begin_draw_stair_cut_rein_rebar_dimension_shape(self):
        """
        开始绘制楼梯剖切加强筋标注形状
        :return:
        """
        dimension_points = (
            self.stair_cut_data.get_stair_hole_rein_rebar_top_dimension_points()
        )
        # 第一层标注点
        for segment in dimension_points:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = transform_point_from_xy_to_yx(list(segment[0]))  # 起点
            end_point = transform_point_from_xy_to_yx(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.bottom_hole_top_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.bottom_hole_top_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def main_run_process(self):
        """
        开始运行过程
        :return:
        """
        self.begin_draw_stair_bottom_local_profile_shape()  # 开始绘制底端局部轮廓形状
        self.begin_draw_bottom_breakline_shape()  # 开始绘制底端折断线
        self.begin_draw_left_hole_rein_rebar()  # 开始绘制孔洞加强钢筋
        self.begin_draw_stair_bottom_hole_shape()  # 开始绘制楼梯底部孔洞形状
        self.begin_draw_stair_bottom_hole_dimension_points()  # 开始绘制孔洞标注点
        self.begin_draw_stair_dimension_profile_points()  # 开始绘制楼梯轮廓点
        self.begin_draw_stair_cut_rein_rebar_profile_shape()  # 开始绘制楼梯剖切加强筋轮廓形状
        self.begin_draw_stair_outline_shape()  # 绘制引出线形状
        # # 调整块放置点
        # self.begin_draw_stair_cut_rein_rebar_hatch_shape()  # 开始绘制剖切加强钢筋填充形状  TODO有问题
        self.begin_draw_stair_cut_rein_rebar_dimension_shape()


class StairTopHoleReinRebarView(object):
    """
    楼梯顶部孔洞加强筋视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairTopHoleReinRebarViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.top_hole_rein_view = self.dxf_doc
        self.model_space = self.top_hole_rein_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        """
        self.top_hole_rein_shape = self.top_hole_rein_view.blocks.new(
            "top_hole_view"
        )  # 添加轮廓视图块
        self.top_hole_left_dimension = self.top_hole_rein_view.blocks.new(
            "top_hole_left_dimension"
        )  # 添加左侧线性标注块
        self.top_hole_top_dimension = self.top_hole_rein_view.blocks.new(
            "top_hole_top_dimension"
        )  # 添加俯视线性标注块
        self.top_hole_break_line = self.top_hole_rein_view.blocks.new(
            "top_hole_break_line"
        )  # 添加折断线块
        self.top_hole_outline = self.top_hole_rein_view.blocks.new(
            "top_hole_outline"
        )  # 引出线块
        self.top_hole_cut_outline = self.top_hole_rein_view.blocks.new(
            "top_hole_cut_outline"
        )  # 引出线块
        self.top_hole_rein_rebar = self.top_hole_rein_view.blocks.new(
            "top_hole_rein_rebar"
        )  # 孔洞加强钢筋块
        self.top_cut_hole_rein_rebar_shape = self.top_hole_rein_view.blocks.new(
            "top_hole_cut_rein_rebar"
        )  # 孔洞顶部加强钢筋块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.hole_hatch = self.model_space.add_hatch(color=4)

    def begin_draw_stair_top_local_profile_shape(self):
        """
        开始绘制楼梯顶端局部轮廓点
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_top_local_bounding_profile_points()
        )
        layer_name = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.top_hole_rein_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_top_breakline_shape(self):
        """
        开始绘制顶部折断线形状
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_top_right_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = transform_point_from_yz_to_xy_s(list(draw_lines[num]))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.top_hole_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_left_hole_rein_rebar(self):
        """
        绘制孔洞加强筋投影图
        :return:None
        """
        hole_rein_rebar_loc = (
            self.stair_cut_data.get_stair_hole_rein_rebar_left_projection()
        )  # 获取孔洞加强筋数据
        layer_name = "G_Rebar"
        # 变换绘制比例
        for num in range(len(hole_rein_rebar_loc)):
            for k in range(len(hole_rein_rebar_loc[num])):
                hole_rein_rebar_loc[num][k][0] = transform_point_from_yz_to_xy_s(
                    list(hole_rein_rebar_loc[num][k][0])
                )  # 起点
                hole_rein_rebar_loc[num][k][1] = transform_point_from_yz_to_xy_s(
                    list(hole_rein_rebar_loc[num][k][1])
                )  # 终点
                hole_rein_rebar_loc[num][k][0] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][0], 1 / self.scale
                )
                hole_rein_rebar_loc[num][k][1] = adjust_drawing_scale(
                    hole_rein_rebar_loc[num][k][1], 1 / self.scale
                )
        for rebar in hole_rein_rebar_loc:
            for k in range(len(rebar)):
                segment = rebar[k]  # 钢筋当前段
                seg = [
                    (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                    (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
                ]  # 将三维点转化为平面点
                self.top_hole_rein_rebar.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_top_hole_shape(self):
        """
        开始绘制楼梯顶部孔洞形状
        :return:
        """
        hole_data = (
            self.stair_cut_data.get_stair_top_left_hole_bounding_profile_points()
        )  # 获取孔洞数据
        layer_name = "G_Hole_Outline"
        # 变换绘制比例
        for num in range(len(hole_data)):
            hole_data[num][0] = transform_point_from_yz_to_xy_s(
                list(hole_data[num][0])
            )  # 起点
            hole_data[num][1] = transform_point_from_yz_to_xy_s(
                list(hole_data[num][1])
            )  # 终点
            hole_data[num][0] = adjust_drawing_scale(hole_data[num][0], 1 / self.scale)
            hole_data[num][1] = adjust_drawing_scale(hole_data[num][1], 1 / self.scale)
        for segment in hole_data:
            seg = [
                (abs(segment[0][0]), abs(segment[0][1]), abs(segment[0][2])),
                (abs(segment[1][0]), abs(segment[1][1]), abs(segment[1][2])),
            ]  # 将三维点转化为平面点
            self.top_hole_rein_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_top_hole_dimension_points(self):
        """
        获取楼梯顶部孔洞标注点
        :return:
        """
        if (
            self.detailed_design.construction_detailed.top_hole_type.value == 1
        ):  # 1--滑动铰
            dimension_info = (
                self.stair_cut_data.get_stair_slide_hole_dimension_points()
            )  # 获取楼梯滑动孔洞标注点
            special_dimension = dimension_info["special_dimension"]
            special_dimension_points = special_dimension[0]
            special_dimension_text = special_dimension[1]
            basic_dimension = dimension_info["basic_dimension"]
            # 特殊文本标注
            for num in range(len(special_dimension_points)):
                # 调整该层标注相对实体偏移量
                segment = special_dimension_points[num]
                text = special_dimension_text[num]
                current_offset = self.dimension_offset * 0.25
                start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
                end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                    dim = self.top_hole_left_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        text=text,
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.top_hole_left_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        text=text,
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()
            # 常规线性标注
            for segment in basic_dimension:
                # 调整该层标注相对实体偏移量
                current_offset = self.dimension_offset * 0.25
                # 调整点的比例
                start_point = transform_point_from_yz_to_xy_s(list(segment[0]))  # 起点
                end_point = transform_point_from_yz_to_xy_s(list(segment[1]))  # 终点
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                    dim = self.top_hole_left_dimension.add_linear_dim(
                        base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                    )
                else:
                    dim = self.top_hole_left_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()
        else:
            dimension_info = self.stair_cut_data.get_stair_fix_hole_dimension_points()
            dimension_points = dimension_info[0]
            dimension_text = dimension_info[1]
            for num in range(len(dimension_points)):
                # 调整该层标注相对实体偏移量
                segment = dimension_points[num]
                text = dimension_text[num]
                current_offset = self.dimension_offset * 0.25
                start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
                end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
                # 调整点的比例
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                # 对尺寸数字反向内容进行特殊操作
                direction = calculate_normal_vector(start_point, end_point, 0)
                base_p_ = np.array(direction) * current_offset + np.array(
                    list(end_point)
                )
                base_p = tuple(base_p_)  # 对齐点
                if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                    dim = self.top_hole_left_dimension.add_linear_dim(
                        base=base_p,
                        p1=start_point,
                        p2=end_point,
                        text=text,
                        dimstyle="G_DS_20",
                    )
                else:
                    dim = self.top_hole_left_dimension.add_aligned_dim(
                        p1=end_point,
                        p2=start_point,
                        text=text,
                        distance=current_offset,
                        dimstyle="G_DS_20",
                    )
                dim.render()

    def begin_draw_stair_dimension_profile_points(self):
        """
        开始绘制楼梯轮廓标注点
        :return:
        """
        dimension_info = self.stair_cut_data.get_stair_profile_dimension_points()
        first_floor_dimension = dimension_info["first_floor"]
        second_floor_dimension = dimension_info["second_floor"]
        # 第一层标注点
        for segment in first_floor_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))  # 起点
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.top_hole_left_dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",  # 默认是1:100进行绘制，输入的数值是绘制的大小，实际大小需要乘以绘图比例。
                )
            else:
                dim = self.top_hole_left_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",  # 按照1：100进行绘制
                )
            dim.render()
        # 第二层标注点
        for segment in second_floor_dimension:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.5
            # 调整点的比例
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))  # 起点
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.top_hole_left_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.top_hole_left_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_outline_shape(self):
        """
        开始绘制楼梯引出线形状
        :return:
        """
        outline_info = self.stair_cut_data.get_stair_hole_rein_rebar_outline_points()
        outline_points = outline_info["outline_points"]
        outline_text = outline_info["outline_text"]
        layer_line = "G_Label"
        text_layer = "G_note"
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.top_hole_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": "Outline"}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.top_hole_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def get_stair_cut_rein_rebar_place_point(self):
        """
        获取楼梯剖切钢筋块放置点
        :return:
        """
        bounding_box_loc = (
            self.stair_cut_data.get_stair_top_local_profile_bounding_box()
        )
        max_z = 0
        min_z = self.h2 + 2 * self.h
        for point in bounding_box_loc:
            if max_z < point[2]:
                max_z = point[2]
            if min_z > point[2]:
                min_z = point[2]
        block_placement_point = [0, self.h2 + self.h + self.tabu_h / 2, 0]
        return block_placement_point

    def begin_draw_stair_cut_rein_rebar_profile_shape(self):
        """
        开始绘制楼梯加强筋俯视轮廓形状图
        :return:
        """
        profile_points = self.stair_cut_data.get_stair_hole_rein_rebar_bounding_points()
        layer_line = "G_Rebar"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_xy_to_yx(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.top_cut_hole_rein_rebar_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_line}
                )

    def begin_draw_stair_cut_rein_rebar_standard_hatch_shape(
        self, draw_circle, draw_spline, outline_points, outline_text
    ):
        """
        开始绘制标准填充形状
        :param draw_circle:
        :param draw_spline:
        :param outline_points:
        :param outline_text:
        :return:
        """
        # 获取基本数据
        circle_loc_points = draw_circle[0]
        circle_diameter = draw_circle[1]
        spline_points = [draw_spline[0]]
        arc_points = draw_spline[1]
        layer_line = "G_Hole_Outline_Continuous"
        layer_outline = "G_Label"
        text_layer = "G_Note"
        # 开始绘制形状
        for num in range(len(circle_loc_points)):
            center = circle_loc_points[num]
            diameter = circle_diameter[num]
            center = transform_point_from_xy_to_yx(list(center))
            center = adjust_drawing_scale(list(center), 1 / self.scale)
            diameter = diameter * 1 / self.scale
            self.top_cut_hole_rein_rebar_shape.add_circle(
                tuple(center), diameter / 2, dxfattribs={"layer": layer_line}
            )
        # 开始填充--添加圆弧形状
        hole_hatch = self.hole_hatch.paths.add_edge_path()
        arc_points[0] = transform_point_from_xy_to_yx(list(arc_points[0]))
        arc_points[0] = adjust_drawing_scale(list(arc_points[0]), 1 / self.scale)
        arc_points[1] = arc_points[1] * 1 / self.scale
        hole_hatch.add_arc(
            tuple(arc_points[0]), arc_points[1] / 2, arc_points[2], arc_points[3]
        )
        for segment in spline_points:
            segment[0] = transform_point_from_xy_to_yx(list(segment[0]))
            segment[0] = adjust_drawing_scale(list(segment[0]), 1 / self.scale)
            segment[1] = transform_point_from_xy_to_yx(list(segment[1]))
            segment[1] = adjust_drawing_scale(list(segment[1]), 1 / self.scale)
            segment[2] = transform_point_from_xy_to_yx(list(segment[2]))
            segment[2] = adjust_drawing_scale(list(segment[2]), 1 / self.scale)
            seg = [
                tuple(segment[0]),
                tuple(segment[1]),
                tuple(segment[2]),
            ]  # 将三维点转化为平面点
            self.top_cut_hole_rein_rebar_shape.add_spline(
                seg, dxfattribs={"layer": layer_line}
            )  # 圆弧线
            hole_hatch.add_spline(control_points=seg)
        # 开始绘制引出线
        for segment in outline_points:
            start_point = transform_point_from_xy_to_yx(list(segment[0]))
            end_point = transform_point_from_xy_to_yx(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.top_hole_cut_outline.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": layer_outline},
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_xy_to_yx(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.top_hole_cut_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_cut_rein_rebar_hatch_shape(self):
        """
        开始绘制楼梯剖切加强筋填充形状
        :return:
        """
        if (
            self.detailed_design.construction_detailed.top_hole_type.value == 1
        ):  # 滑动铰支座--1
            hatch_info = (
                self.stair_cut_data.get_stair_slide_rein_rebar_top_hatch_points()
            )  # 开始楼梯滑动剖切加强筋填充点
        else:  # 固定支座
            hatch_info = (
                self.stair_cut_data.get_stair_fix_rein_rebar_top_hatch_points()
            )  # 开始楼梯滑动剖切加强筋填充点
        draw_circle = hatch_info["draw_circle"]
        draw_spline = hatch_info["draw_spline"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        self.begin_draw_stair_cut_rein_rebar_standard_hatch_shape(
            draw_circle, draw_spline, outline_points, outline_text
        )

    def begin_draw_stair_cut_rein_rebar_dimension_shape(self):
        """
        开始绘制楼梯剖切加强筋标注形状
        :return:
        """
        dimension_points = (
            self.stair_cut_data.get_stair_hole_rein_rebar_top_dimension_points()
        )
        # 第一层标注点
        for segment in dimension_points:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = transform_point_from_xy_to_yx(list(segment[0]))  # 起点
            end_point = transform_point_from_xy_to_yx(list(segment[1]))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.top_hole_top_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.top_hole_top_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def main_run_process(self):
        """
        开始运行过程
        :return:
        """
        self.begin_draw_stair_top_local_profile_shape()  # 开始绘制顶端局部轮廓形状
        self.begin_draw_top_breakline_shape()  # 开始绘制顶端折断线
        self.begin_draw_left_hole_rein_rebar()  # 开始绘制孔洞加强钢筋
        self.begin_draw_stair_top_hole_shape()  # 开始绘制楼梯顶部孔洞形状
        self.begin_draw_stair_top_hole_dimension_points()  # 开始绘制孔洞标注点
        self.begin_draw_stair_dimension_profile_points()  # 开始绘制楼梯轮廓点
        self.begin_draw_stair_cut_rein_rebar_profile_shape()  # 开始绘制楼梯剖切加强筋轮廓形状
        self.begin_draw_stair_outline_shape()  # 绘制引出线形状
        # self.begin_draw_stair_cut_rein_rebar_hatch_shape()  # 开始绘制剖切加强钢筋填充形状
        self.begin_draw_stair_cut_rein_rebar_dimension_shape()


class StairStepSlotLeftView(object):
    """
    产生楼梯防滑条侧视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairStepSlotLeftViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.left_view = self.dxf_doc
        self.model_space = self.left_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.step_slot_left_shape = self.left_view.blocks.new(
            "step_slot_left_view"
        )  # 添加视图块
        self.step_slot_left_radius_dimension = self.left_view.blocks.new(
            "step_slot_radius_dimension"
        )  # 添加圆半径标注块
        self.step_slot_left_line_dimension = self.left_view.blocks.new(
            "step_slot_line_dimension"
        )  # 添加线性标注块
        self.step_slot_left_break_line = self.left_view.blocks.new(
            "step_slot_break_line"
        )  # 添加折断线块

    def begin_draw_stair_step_slot_profile_shape(self):
        """
        开始绘制楼梯防滑槽轮廓形状
        :return:
        """
        profile_points = self.stair_cut_data.choose_inner_point_of_bounding_box()
        layer_name = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (abs(segment[num][0]), abs(segment[num][1]), abs(segment[num][2])),
                    (
                        abs(segment[num + 1][0]),
                        abs(segment[num + 1][1]),
                        abs(segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                self.step_slot_left_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_step_slot_arc_dimension_shape(self):
        """
        开始进行楼梯防滑槽圆弧标注形状
        :return:
        """
        (
            radius,
            center_loc,
        ) = self.stair_cut_data.get_stair_step_slot_top_arc_dimension_point_data()
        center_loc = transform_point_from_yz_to_xy_s(list(center_loc))  # 起点
        center_loc = adjust_drawing_scale(list(center_loc), 1 / self.scale)
        radius = radius * (1 / self.scale)
        dim = self.step_slot_left_radius_dimension.add_radius_dim(
            center=(center_loc[0], center_loc[1], center_loc[2]),
            radius=radius,
            angle=135,
            dimstyle="G_DS_20",
        )
        dim.render()

    def begin_draw_stair_step_slot_left_boundary_breakline(self):
        """
        开始绘制防滑槽左侧边界折断线
        :return:
        """
        layer_name = "G_Break_Line"
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        left_boundary_info = (
            self.stair_cut_data.get_stair_solid_left_transverse_boundary_data()
        )
        left_loc = left_boundary_info[0]
        limit_z = (left_loc[0][2] + left_loc[1][2]) / 2
        top_loc = [0, 0, 0]
        bottom_loc = [0, 0, 0]
        for point in left_loc:
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
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                (
                    abs(draw_lines[num][0]),
                    abs(draw_lines[num][1]),
                    abs(draw_lines[num][2]),
                ),
                (
                    abs(draw_lines[num + 1][0]),
                    abs(draw_lines[num + 1][1]),
                    abs(draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.step_slot_left_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_step_slot_right_boundary_breakline(self):
        """
        开始绘制防滑槽右侧边界折断线
        :return:
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        right_boundary_info = (
            self.stair_cut_data.get_stair_solid_right_transverse_boundary_data()
        )
        right_loc = right_boundary_info[0]
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
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        layer_name = "G_Break_Line"
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                (
                    abs(draw_lines[num][0]),
                    abs(draw_lines[num][1]),
                    abs(draw_lines[num][2]),
                ),
                (
                    abs(draw_lines[num + 1][0]),
                    abs(draw_lines[num + 1][1]),
                    abs(draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.step_slot_left_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_step_slot_line_dimension_shape(self):
        """
        开始进行尺寸标注
        :return:
        """
        dimension_loc = (
            self.stair_cut_data.get_stair_step_slot_top_dimension_point_data()
        )  # 标注数据
        # 第一层标注对象
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.15
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 计算机存储精度
                dim = self.step_slot_left_line_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.step_slot_left_line_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def main_run_process(self):
        """
        主要运行过程
        :return:
        """
        self.begin_draw_stair_step_slot_profile_shape()
        self.begin_draw_stair_step_slot_arc_dimension_shape()
        self.begin_draw_stair_step_slot_left_boundary_breakline()
        self.begin_draw_stair_step_slot_right_boundary_breakline()
        self.begin_draw_stair_step_slot_line_dimension_shape()


class StairStepSlotTopView(object):
    """
    产生楼梯防滑条俯视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairStepSlotTopViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.step_slot_top_view = self.dxf_doc
        self.model_space = self.step_slot_top_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        """
        self.step_slot_top_shape = self.step_slot_top_view.blocks.new(
            "step_slot_top_view"
        )  # 添加俯视图块
        self.step_slot_top_line_dimension = self.step_slot_top_view.blocks.new(
            "step_slot_top_line_dimension"
        )  # 添加线性标注块
        self.step_slot_top_break_line = self.step_slot_top_view.blocks.new(
            "step_slot_top_break_line"
        )  # 添加折断线块

    def begin_draw_step_slot_profile_line_shape(self):
        """
        开始绘制防滑槽轮廓数据
        """
        profile_points = (
            self.stair_cut_data.choose_step_slot_projection_inner_point_of_bounding_box()
        )
        layer_name = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                # 旋转坐标点
                segment[num] = rotation_3d(
                    np.asarray(segment[num]), np.asarray([0, 0, 1]), self.rotate_angle
                )
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (abs(segment[num][0]), abs(segment[num][1]), abs(segment[num][2])),
                    (
                        abs(segment[num + 1][0]),
                        abs(segment[num + 1][1]),
                        abs(segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                self.step_slot_top_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_profile_line_shape(self):
        """
        开始绘制楼梯轮廓线形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.choose_stair_projection_inner_point_of_bounding_box()
        )
        layer_name = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = rotation_3d(
                    np.asarray(segment[num]), np.asarray([0, 0, 1]), self.rotate_angle
                )
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (abs(segment[num][0]), abs(segment[num][1]), abs(segment[num][2])),
                    (
                        abs(segment[num + 1][0]),
                        abs(segment[num + 1][1]),
                        abs(segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                self.step_slot_top_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_step_slot_special_line_shape(self):
        """
        开始绘制楼梯防滑槽特殊线形状
        :return:
        """
        special_data = self.stair_cut_data.get_stair_step_slot_special_line_shape_data()
        layer_name = "G_Step_Slot"
        # 变换绘制比例
        for segment in special_data:
            for num in range(len(segment)):
                # 旋转坐标点
                segment[num] = rotation_3d(
                    np.asarray(segment[num]), np.asarray([0, 0, 1]), self.rotate_angle
                )
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in special_data:
            for num in range(len(segment) - 1):
                seg = [
                    (abs(segment[num][0]), abs(segment[num][1]), abs(segment[num][2])),
                    (
                        abs(segment[num + 1][0]),
                        abs(segment[num + 1][1]),
                        abs(segment[num + 1][2]),
                    ),
                ]  # 将三维点转化为平面点
                self.step_slot_top_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_top_profile_breakline_shape(self):
        """
        开始绘制楼梯顶部轮廓折断线形状
        :return:
        """
        extend_l = 50  # 两边延伸距离
        l_space = 40  # 间断
        offset = 30  # 偏移长度
        theta = math.pi / 9  # 弧度值
        stair_profile = (
            self.stair_cut_data.choose_stair_projection_inner_point_of_bounding_box()
        )
        # 获取边界线上的点
        boundary_line = []
        for seg in stair_profile:
            point_1 = list(seg[0])
            point_2 = list(seg[1])
            direction = np.array(np.array(point_2) - np.array(point_1))
            if direction[0] != 0 and direction[1] == 0:
                if point_1[0] > point_2[0]:
                    boundary_line.append(point_1)
                else:
                    boundary_line.append(point_2)
        # 调整边界线上点的顺序
        if boundary_line[0][1] < boundary_line[1][1]:
            current_p = boundary_line[0]
            boundary_line[0] = boundary_line[1]
            boundary_line[1] = current_p
        # 折断线上点系列
        point_0 = copy.deepcopy(boundary_line[0])
        point_0[1] += extend_l
        point_1 = copy.deepcopy(boundary_line[0])
        point_2 = [0, 0, 0]
        point_2[0] = (boundary_line[0][0] + boundary_line[1][0]) / 2
        point_2[1] = (boundary_line[0][1] + boundary_line[1][1]) / 2 + l_space / 2
        point_2[2] = (boundary_line[0][2] + boundary_line[1][2]) / 2
        point_3 = copy.deepcopy(point_2)
        point_3[0] += offset * math.cos(theta)
        point_3[1] += -offset * math.sin(theta)
        point_5 = copy.deepcopy(point_2)
        point_5[1] += -l_space
        point_4 = copy.deepcopy(point_5)
        point_4[0] += -offset * math.cos(theta)
        point_4[1] += offset * math.sin(theta)
        point_6 = copy.deepcopy(boundary_line[1])
        point_7 = copy.deepcopy(point_6)
        point_7[1] += -extend_l
        draw_lines = [
            point_0,
            point_1,
            point_2,
            point_3,
            point_4,
            point_5,
            point_6,
            point_7,
        ]
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            # 旋转坐标点
            point = rotation_3d(
                np.asarray(point), np.asarray([0, 0, 1]), self.rotate_angle
            )
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        layer_name = "G_Break_Line"
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                (
                    abs(draw_lines[num][0]),
                    abs(draw_lines[num][1]),
                    abs(draw_lines[num][2]),
                ),
                (
                    abs(draw_lines[num + 1][0]),
                    abs(draw_lines[num + 1][1]),
                    abs(draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.step_slot_top_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_dimension_step_slot_location(self):
        """
        获取防滑槽的标注位置
        :return:
        """
        step_slot_loc = (
            self.stair_cut_data.get_stair_step_slot_dimension_point_data()
        )  # 开始标注防滑槽
        for num in range(len(step_slot_loc)):
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.1
            segment = copy.deepcopy(step_slot_loc[num])
            # 调整点的比例
            start_point = adjust_drawing_scale(segment[0], 1 / self.scale)
            end_point = adjust_drawing_scale(segment[1], 1 / self.scale)
            # 旋转坐标点
            start_point = tuple(
                rotation_3d(
                    np.asarray(list(start_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            end_point = tuple(
                rotation_3d(
                    np.asarray(list(end_point)),
                    np.asarray([0, 0, 1]),
                    self.rotate_angle,
                )
            )
            # 转换坐标值
            start_point = (
                abs(start_point[0]),
                abs(start_point[1]),
                abs(start_point[2]),
            )
            end_point = (abs(end_point[0]), abs(end_point[1]), abs(end_point[2]))
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * (current_offset) + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.0001:  # 存在截断误差
                dim = self.step_slot_top_line_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.step_slot_top_line_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def main_run_process(self):
        """
        主要运行过程
        :return:
        """
        self.begin_draw_stair_profile_line_shape()  # 开始绘制楼梯轮廓图线
        self.begin_draw_step_slot_profile_line_shape()  # 开始绘制楼梯防滑槽轮廓图线
        self.begin_draw_stair_step_slot_special_line_shape()  # 开始绘制楼梯防滑槽特殊线
        self.begin_draw_stair_top_profile_breakline_shape()  # 开始绘制楼梯踏步折断线
        self.begin_dimension_step_slot_location()  # 开始标注防滑槽位置


class StairDoubleSideWallJointLongView(object):
    """
    产生楼梯两侧接缝纵向视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairDoubleSideWallJointLongViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.wall_long_joint_view = self.dxf_doc
        self.model_space = self.wall_long_joint_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有块
        """
        self.wall_long_joint_shape = self.wall_long_joint_view.blocks.new(
            "wall_long_joint_view"
        )  # 添加轮廓视图块
        self.wall_long_joint_line_dimension = self.wall_long_joint_view.blocks.new(
            "wall_long_joint_line_dimension"
        )  # 添加线性标注块
        self.wall_long_joint_break_line = self.wall_long_joint_view.blocks.new(
            "wall_long_joint_break_line"
        )  # 添加折断线块
        self.wall_long_joint_outline = self.wall_long_joint_view.blocks.new(
            "wall_long_joint_outline"
        )  # 引出线块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.top_edge_hatch = self.model_space.add_hatch()
        self.top_mid_hatch = self.model_space.add_hatch(color=5)
        self.mid_hatch = self.model_space.add_hatch()
        self.bottom_mid_hatch = self.model_space.add_hatch(color=5)
        self.bottom_edge_hatch = self.model_space.add_hatch()

    def begin_draw_stair_bounding_profile_shape(self):
        """
        获取楼梯轮廓形状
        :return:
        """
        profile_points = self.stair_cut_data.get_stair_bounding_profile_points()
        layer_name = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_yz_to_xy_s(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.wall_long_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_name}
                )

    def begin_draw_stair_left_breakline_shape(self):
        """
        获取楼梯左侧折断线形状
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_left_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.wall_long_joint_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_right_breakline_shape(self):
        """
        获取楼梯右侧折断线形状
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_right_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_yz_to_xy_s(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.wall_long_joint_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_stair_top_edge_hatch_shape(self):
        """
        获取楼梯顶部边缘填充形状
        :return:
        """
        top_edge_hatch_info = (
            self.stair_cut_data.get_stair_top_edge_hatch_profile_points()
        )  # 获取楼梯顶部边缘填充点
        profile_points = top_edge_hatch_info["profile_points"]
        outline_points = top_edge_hatch_info["outline_points"]
        outline_text = top_edge_hatch_info["outline_text"]
        layer_line = "G_Label"
        text_layer = "G_Note"
        # 开始绘制填充轮廓点
        top_edge_path = self.top_edge_hatch.paths.add_edge_path()
        for seg in profile_points:
            for num in range(len(seg) - 1):
                start_point = seg[num]
                end_point = seg[num + 1]
                start_point = transform_point_from_yz_to_xy_s(list(start_point))
                end_point = transform_point_from_yz_to_xy_s(list(end_point))
                start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
                end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
                self.wall_long_joint_shape.add_line(
                    tuple(start_point), tuple(end_point)
                )
                top_edge_path.add_line(tuple(start_point), tuple(end_point))
        self.top_edge_hatch.set_pattern_fill("ANSI37", scale=1 * 1 / self.scale)  # TODO
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_long_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_long_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_top_mid_hatch_shape(self):
        """
        获取楼梯顶部中间填充形状
        :return:
        """
        top_mid_hatch_points = (
            self.stair_cut_data.get_stair_top_mid_hatch_profile_points()
        )
        profile_points = top_mid_hatch_points["profile_points"]
        outline_points = top_mid_hatch_points["outline_points"]
        outline_text = top_mid_hatch_points["outline_text"]
        draw_lines = copy.deepcopy(profile_points)
        draw_lines.append(profile_points[0])
        layer_line = "G_Label"
        text_layer = "G_Note"
        for num in range(len(draw_lines) - 1):
            start_point = draw_lines[num]
            end_point = draw_lines[num + 1]
            start_point = transform_point_from_yz_to_xy_s(list(start_point))
            end_point = transform_point_from_yz_to_xy_s(list(end_point))
            start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
            end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
            self.wall_long_joint_shape.add_line(tuple(start_point), tuple(end_point))
        # 开始绘制填充轮廓点
        for num in range(len(profile_points)):
            current_point = profile_points[num]
            current_point = transform_point_from_yz_to_xy_s(list(current_point))
            current_point = adjust_drawing_scale(list(current_point), 1 / self.scale)
            profile_points[num] = current_point
        self.top_mid_hatch.paths.add_polyline_path(profile_points)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_long_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_long_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_mid_hatch_shape(self):
        """
        开始绘制楼梯中部填充形状
        :return:
        """
        mid_hatch_points = self.stair_cut_data.get_stair_mid_hatch_profile_points()
        profile_points = mid_hatch_points["profile_points"]
        outline_points = mid_hatch_points["outline_points"]
        outline_text = mid_hatch_points["outline_text"]
        draw_lines = copy.deepcopy(profile_points)
        draw_lines.append(profile_points[0])
        layer_line = "G_Label"
        text_layer = "G_Note"
        for num in range(len(draw_lines) - 1):
            start_point = draw_lines[num]
            end_point = draw_lines[num + 1]
            start_point = transform_point_from_yz_to_xy_s(list(start_point))
            end_point = transform_point_from_yz_to_xy_s(list(end_point))
            start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
            end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
            self.wall_long_joint_shape.add_line(tuple(start_point), tuple(end_point))
        # 开始绘制填充轮廓点
        for num in range(len(profile_points)):
            current_point = profile_points[num]
            current_point = transform_point_from_yz_to_xy_s(list(current_point))
            current_point = adjust_drawing_scale(list(current_point), 1 / self.scale)
            profile_points[num] = current_point
        self.mid_hatch.paths.add_polyline_path(profile_points)
        self.mid_hatch.set_pattern_fill("ANGLE", scale=1 * 1 / self.scale)
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            start_point = transform_point_from_yz_to_xy_s(list(segment[0]))
            end_point = transform_point_from_yz_to_xy_s(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_long_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = transform_point_from_yz_to_xy_s(list(point_loc))  # 文本位置
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_long_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_bottom_mid_hatch_shape(self):
        """
        开始绘制底端中间填充形状
        :return:
        """
        profile_points = self.stair_cut_data.get_stair_bottom_mid_hatch_profile_points()
        draw_lines = copy.deepcopy(profile_points)
        draw_lines.append(profile_points[0])
        for num in range(len(draw_lines) - 1):
            start_point = draw_lines[num]
            end_point = draw_lines[num + 1]
            start_point = transform_point_from_yz_to_xy_s(list(start_point))
            end_point = transform_point_from_yz_to_xy_s(list(end_point))
            start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
            end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
            self.wall_long_joint_shape.add_line(tuple(start_point), tuple(end_point))
        # 开始绘制填充轮廓点
        for num in range(len(profile_points)):
            current_point = profile_points[num]
            current_point = transform_point_from_yz_to_xy_s(list(current_point))
            current_point = adjust_drawing_scale(list(current_point), 1 / self.scale)
            profile_points[num] = current_point
        self.bottom_mid_hatch.paths.add_polyline_path(profile_points)

    def begin_draw_stair_bottom_edge_hatch_shape(self):
        """
        开始绘制底端边缘填充形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_bottom_edge_hatch_profile_points()
        )
        draw_lines = copy.deepcopy(profile_points)
        draw_lines.append(profile_points[0])
        for num in range(len(draw_lines) - 1):
            start_point = draw_lines[num]
            end_point = draw_lines[num + 1]
            start_point = transform_point_from_yz_to_xy_s(list(start_point))
            end_point = transform_point_from_yz_to_xy_s(list(end_point))
            start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
            end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
            self.wall_long_joint_shape.add_line(tuple(start_point), tuple(end_point))
        # 开始绘制填充轮廓点
        for num in range(len(profile_points)):
            current_point = profile_points[num]
            current_point = transform_point_from_yz_to_xy_s(list(current_point))
            current_point = adjust_drawing_scale(list(current_point), 1 / self.scale)
            profile_points[num] = current_point
        self.bottom_edge_hatch.paths.add_polyline_path(profile_points)
        self.bottom_edge_hatch.set_pattern_fill("ANSI37", scale=0.5 * 1 / self.scale)

    def main_run_process(self):
        """
        开始运行过程
        :return:
        """
        self.begin_draw_stair_bounding_profile_shape()
        self.begin_draw_stair_left_breakline_shape()
        self.begin_draw_stair_right_breakline_shape()
        # self.begin_draw_stair_top_edge_hatch_shape()  # 获取楼梯顶部边缘填充形状
        # self.begin_draw_stair_top_mid_hatch_shape()  # 获取楼梯顶部中间填充形状
        # self.begin_draw_stair_mid_hatch_shape()  # 开始绘制楼梯中部填充形状
        # self.begin_draw_stair_bottom_mid_hatch_shape()  # 开始绘制楼梯底端中部填充形状
        # self.begin_draw_stair_bottom_edge_hatch_shape()  # 开始绘制楼梯底端边缘填充形状


class StairDoubleSideWallJointTransverseView(object):
    """
    阐述楼梯两侧墙接缝横向视图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairDoubleSideWallJointTransverseViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.wall_tran_joint_view = self.dxf_doc
        self.model_space = self.wall_tran_joint_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.wall_tran_joint_shape = self.wall_tran_joint_view.blocks.new(
            "wall_tran_joint_view"
        )  # 添加轮廓视图块
        self.wall_tran_joint_line_dimension = self.wall_tran_joint_view.blocks.new(
            "wall_tran_joint_line_dimension"
        )  # 添加线性标注块
        self.wall_tran_joint_break_line = self.wall_tran_joint_view.blocks.new(
            "wall_tran_joint_break_line"
        )  # 添加折断线块
        self.wall_tran_joint_outline = self.wall_tran_joint_view.blocks.new(
            "wall_tran_joint_outline"
        )  # 引出线块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.tabu_hatch = self.model_space.add_hatch()
        self.top_edge_hatch = self.model_space.add_hatch()
        self.top_mid_hatch = self.model_space.add_hatch(color=5)
        self.mid_hatch = self.model_space.add_hatch()
        self.bottom_mid_hatch = self.model_space.add_hatch(color=5)
        self.bottom_edge_hatch = self.model_space.add_hatch()

    def begin_draw_stair_wall_profile_shape(self):
        """
        获取楼梯墙体轮廓形状
        :return:
        """
        wall_info = self.stair_cut_data.get_stair_wall_profile_points()
        profile_points = wall_info["profile_points"]
        outline_points = wall_info["outline_points"]
        outline_text = wall_info["outline_text"]
        layer_outline_name = "G_Special_Outline"
        layer_label_name = "G_Label"
        layer_text = "G_Note"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline_name}
                )
        # 开始绘制引出线
        # 开始绘制直线
        for segment in outline_points:
            # 调整点的比例
            start_point = adjust_drawing_scale(list(segment[0]), 1 / self.scale)
            end_point = adjust_drawing_scale(list(segment[1]), 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": layer_label_name},
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": layer_text},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_tabu_profile_shape(self):
        """
        开始绘制踏步轮廓形状
        :return:
        """
        profile_points = self.stair_cut_data.get_stair_tabu_profile_points()
        layer_outline_name = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline_name}
                )

    def begin_draw_bottom_breakline_shape(self):
        """
        开始绘制底部折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_bottom_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.wall_tran_joint_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_top_breakline_shape(self):
        """
        开始绘制顶部折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_top_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.wall_tran_joint_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_right_breakline_shape(self):
        """
        开始绘制顶部折断线
        :return:
        """
        draw_lines = self.stair_cut_data.get_stair_right_breakline_points()
        layer_name = "G_Break_Line"
        # 调整各点的比例
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.wall_tran_joint_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_name}
            )

    def begin_draw_dimension_shape(self):
        """
        开始尺寸标注
        :return:
        """
        dimension_loc = self.stair_cut_data.get_stair_dimension_points()  # 获取楼梯构造标注点
        for segment in dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.15
            # 调整点的比例
            start_point = adjust_drawing_scale(list(segment[0]), 1 / self.scale)
            end_point = adjust_drawing_scale(list(segment[1]), 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > 0.00001:  # 计算机存储精度
                dim = self.wall_tran_joint_line_dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    dimstyle="G_DS_20",  # 默认是1:100进行绘制，输入的数值是绘制的大小，实际大小需要乘以绘图比例。
                )
            else:
                dim = self.wall_tran_joint_line_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",  # 按照1：100进行绘制
                )
            dim.render()

    def begin_draw_stair_tabu_hatch_shape(self):
        """
        开始绘制踏步填充形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_tabu_hatch_profile_points()
        profile_points = hatch_info["profile_points"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_outline = "G_Outline"
        layer_line = "G_Label"
        text_layer = "G_Note"
        for num in range(len(draw_line)):
            point = draw_line[num]
            draw_line[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_line) - 1):
            seg = [
                ((draw_line[num][0]), (draw_line[num][1]), (draw_line[num][2])),
                (
                    (draw_line[num + 1][0]),
                    (draw_line[num + 1][1]),
                    (draw_line[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.wall_tran_joint_shape.add_line(
                seg[0], seg[1], dxfattribs={"layer": layer_outline}
            )
        # 开始填充图案
        for num in range(len(profile_points)):
            profile_points[num] = adjust_drawing_scale(
                list(profile_points[num]), 1 / self.scale
            )
            profile_points[num] = tuple(profile_points[num])
        # 开始绘制填充轮廓点
        self.tabu_hatch.paths.add_polyline_path(profile_points, is_closed=1)
        self.tabu_hatch.set_pattern_fill("ANSI34", scale=5 * 1 / self.scale)
        # 开始绘制引出线
        for segment in outline_points:
            # 调整点的比例
            start_point = copy.deepcopy(segment[0])
            end_point = copy.deepcopy(segment[1])
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_top_edge_hatch_shape(self):
        """
        开始绘制楼梯顶部边缘形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_joint_top_edge_hatch_points()
        profile_points = hatch_info["profile_points"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_outline = "G_Outline"
        layer_line = "G_Label"
        text_layer = "G_Note"
        for m in range(len(draw_line)):
            for n in range(len(draw_line[m])):
                point = draw_line[m][n]
                draw_line[m][n] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        top_edge_hatch = self.top_edge_hatch.paths.add_edge_path()
        for segment in draw_line:
            if len(segment) == 2:
                seg = [tuple(segment[0]), tuple(segment[1])]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )
                top_edge_hatch.add_line(seg[0], seg[1])
            else:
                seg = [
                    tuple(segment[0]),
                    tuple(segment[1]),
                    tuple(segment[2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_spline(
                    seg, dxfattribs={"layer": layer_outline}
                )
                top_edge_hatch.add_spline(control_points=seg)
        # self.top_edge_hatch.set_pattern_fill("ANSI37", scale=1 * 1 / self.scale)
        # 开始绘制引出线
        for segment in outline_points:
            # 调整点的比例
            start_point = copy.deepcopy(segment[0])
            end_point = copy.deepcopy(segment[1])
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_top_mid_hatch_shape(self):
        """
        开始绘制楼梯顶部中间形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_joint_top_mid_hatch_points()
        profile_points = hatch_info["profile_points"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_outline = "G_Outline"
        layer_line = "G_Label"
        text_layer = "G_Note"
        for m in range(len(draw_line)):
            for n in range(len(draw_line[m])):
                point = draw_line[m][n]
                draw_line[m][n] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        top_mid_hatch = self.top_mid_hatch.paths.add_edge_path()
        for segment in draw_line:
            if len(segment) == 2:
                seg = [tuple(segment[0]), tuple(segment[1])]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )
                top_mid_hatch.add_line(seg[0], seg[1])
            else:
                seg = [
                    tuple(segment[0]),
                    tuple(segment[1]),
                    tuple(segment[2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_spline(
                    seg, dxfattribs={"layer": layer_outline}
                )
                top_mid_hatch.add_spline(control_points=seg)
        # self.top_mid_hatch.set_pattern_fill("ANSI37", scale=1 * 1 / self.scale)  # TODO  填充不上
        # 开始绘制引出线
        for segment in outline_points:
            # 调整点的比例
            start_point = copy.deepcopy(segment[0])
            end_point = copy.deepcopy(segment[1])
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_mid_hatch_shape(self):
        """
        开始绘制楼梯顶部中间形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_mid_joint_hatch_points()
        profile_points = hatch_info["profile_points"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_outline = "G_Outline"
        layer_line = "G_Label"
        text_layer = "G_Note"
        for m in range(len(draw_line)):
            for n in range(len(draw_line[m])):
                point = draw_line[m][n]
                draw_line[m][n] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        mid_hatch = self.mid_hatch.paths.add_edge_path()
        for segment in draw_line:
            if len(segment) == 2:
                seg = [tuple(segment[0]), tuple(segment[1])]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )
                mid_hatch.add_line(seg[0], seg[1])
            else:
                seg = [
                    tuple(segment[0]),
                    tuple(segment[1]),
                    tuple(segment[2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_spline(
                    seg, dxfattribs={"layer": layer_outline}
                )
                mid_hatch.add_spline(control_points=seg)
        # self.mid_hatch.set_pattern_fill("ANSI37", scale=1 * 1 / self.scale)  # TODO  填充不上
        # 开始绘制引出线
        for segment in outline_points:
            # 调整点的比例
            start_point = copy.deepcopy(segment[0])
            end_point = copy.deepcopy(segment[1])
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_bottom_mid_hatch_shape(self):
        """
        开始绘制楼梯顶部中间形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_joint_bottom_mid_hatch_points()
        profile_points = hatch_info["profile_points"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_outline = "G_Outline"
        layer_line = "G_Label"
        text_layer = "G_Note"
        for m in range(len(draw_line)):
            for n in range(len(draw_line[m])):
                point = draw_line[m][n]
                draw_line[m][n] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        bottom_mid_hatch = self.bottom_mid_hatch.paths.add_edge_path()
        for segment in draw_line:
            if len(segment) == 2:
                seg = [tuple(segment[0]), tuple(segment[1])]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )
                bottom_mid_hatch.add_line(seg[0], seg[1])
            else:
                seg = [
                    tuple(segment[0]),
                    tuple(segment[1]),
                    tuple(segment[2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_spline(
                    seg, dxfattribs={"layer": layer_outline}
                )
                bottom_mid_hatch.add_spline(control_points=seg)
        # self.bottom_mid_hatch.set_pattern_fill("ANSI37", scale=1 * 1 / self.scale)  # TODO  填充不上
        # 开始绘制引出线
        for segment in outline_points:
            # 调整点的比例
            start_point = copy.deepcopy(segment[0])
            end_point = copy.deepcopy(segment[1])
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_bottom_edge_hatch_shape(self):
        """
        开始绘制楼梯顶部中间形状
        :return:
        """
        hatch_info = self.stair_cut_data.get_stair_joint_bottom_edge_hatch_points()
        profile_points = hatch_info["profile_points"]
        outline_points = hatch_info["outline_points"]
        outline_text = hatch_info["outline_text"]
        draw_line = copy.deepcopy(profile_points)
        draw_line.append(profile_points[0])
        layer_outline = "G_Outline"
        layer_line = "G_Label"
        text_layer = "G_Note"
        for m in range(len(draw_line)):
            for n in range(len(draw_line[m])):
                point = draw_line[m][n]
                draw_line[m][n] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        bottom_edge_hatch = self.bottom_edge_hatch.paths.add_edge_path()
        for segment in draw_line:
            if len(segment) == 2:
                seg = [tuple(segment[0]), tuple(segment[1])]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )
                bottom_edge_hatch.add_line(seg[0], seg[1])
            else:
                seg = [
                    tuple(segment[0]),
                    tuple(segment[1]),
                    tuple(segment[2]),
                ]  # 将三维点转化为平面点
                self.wall_tran_joint_shape.add_spline(
                    seg, dxfattribs={"layer": layer_outline}
                )
                bottom_edge_hatch.add_spline(control_points=seg)
        # self.bottom_edge_hatch.set_pattern_fill("ANSI37", scale=1 * 1 / self.scale)  # TODO  填充不上
        # 开始绘制引出线
        for segment in outline_points:
            # 调整点的比例
            start_point = copy.deepcopy(segment[0])
            end_point = copy.deepcopy(segment[1])
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.wall_tran_joint_outline.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": layer_line}
            )
        # 开始增加文本
        point_loc = outline_text[0]  # 文本防止点
        point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
        text = outline_text[1]  # 文本内容
        self.wall_tran_joint_outline.add_text(
            text,
            height=self.note_text_size,
            rotation=0,
            dxfattribs={"layer": text_layer},
        ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_RIGHT)

    def main_run_process(self):
        """
        开始运行过程
        :return:
        """
        self.begin_draw_stair_wall_profile_shape()  # 开始绘制墙体
        self.begin_draw_tabu_profile_shape()  # 开始绘制踏步形状
        self.begin_draw_bottom_breakline_shape()  # 开始绘制底部折断线形状
        self.begin_draw_top_breakline_shape()  # 开始绘制顶部折断线形状
        self.begin_draw_right_breakline_shape()  # 开始绘制右侧折断线形状
        self.begin_draw_dimension_shape()  # 开始尺寸标注
        # self.begin_draw_stair_tabu_hatch_shape()  # 开始绘制楼梯踏步填充形状
        # self.begin_draw_stair_top_edge_hatch_shape()  # 开始绘制楼梯顶部边缘填充图案
        # self.begin_draw_stair_top_mid_hatch_shape()  # 开始绘制楼梯顶部中间填充图案
        # self.begin_draw_stair_mid_hatch_shape()  # 开始绘制楼梯中部图案
        # self.begin_draw_stair_bottom_mid_hatch_shape()  # 开始绘制楼梯底部中间图案
        # self.begin_draw_stair_bottom_edge_hatch_shape()  # 开始绘制楼梯底部边缘图案


class StairRailEmbeddedDetailView(object):
    """
    楼梯栏杆预埋深化图
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        dxf_doc: Drawing,
        structure_design: StructuralDesign,
        detailed_design: DetailedDesign,
        structure_design_result: StructuralDesignResult,
        detailed_design_result: DetailedDesignResult,
        rebar_for_bim: RebarforBIM,
    ):
        self.dxf_doc = dxf_doc
        self.structure_design = structure_design
        self.structure_design_result = structure_design_result
        self.detailed_design = detailed_design
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.stair_cut_data = StairRailEmbeddedDetailViewData(
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.rotate_angle = math.pi / 2  # 旋转角度
        self.load_dxf_file()  # 加载dxf文件
        self.create_total_block()  # 创建所有图块
        self.create_all_hatch_pattern()  # 创建所有填充图案
        self.note_text_size = 40  # 文字大小
        self.scale = 1  # 实际尺寸与绘图尺寸的比例
        self.dimension_offset = 300 / self.scale  # 单层标注偏移
        self.precision = 0.000001  # 判断精度
        self.generate_basic_data()

    def generate_basic_data(self):
        """
        产生基本类模板基础数据
        :return:
        """
        # 产生基本数据
        self.n = self.structure_design.geometric.steps_number
        self.tabu_h = self.structure_design_result.steps_h
        self.tabu_b = self.structure_design_result.steps_b
        self.b0 = self.detailed_design.geometric_detailed.width
        self.b1 = self.detailed_design.geometric_detailed.top_b  # 上部挑耳
        self.b2 = self.detailed_design.geometric_detailed.bottom_b  # 下部挑耳
        self.h1 = self.detailed_design.geometric_detailed.top_thickness
        self.h2 = self.detailed_design.geometric_detailed.bottom_thickness
        self.lb_d = self.detailed_design.geometric_detailed.bottom_top_length
        self.lt_d = self.detailed_design.geometric_detailed.top_top_length
        self.l1t = self.detailed_design_result.top_bottom_length
        self.l1b = self.detailed_design_result.bottom_bottom_length
        self.ln = self.structure_design.geometric.clear_span
        self.h = self.structure_design.geometric.height
        self.l_total = self.detailed_design_result.l_total  # 纵向总长
        self.h_total = self.detailed_design_result.h_total  # 楼梯总高度
        self.tan = self.detailed_design_result.tan
        self.cos = self.detailed_design_result.cos
        self.sin = self.detailed_design_result.sin
        self.cover = (
            self.structure_design.construction.concrete_cover_thickness
        )  # 保护层厚度

    def load_dxf_file(self):
        """
        加载dxf文件
        :return:
        """
        self.rail_embedded_detail_view = self.dxf_doc
        self.model_space = self.rail_embedded_detail_view.modelspace()  # 创建模型空间

    def create_total_block(self):
        """
        创建所有图块
        """
        self.left_rail_detail_shape = self.rail_embedded_detail_view.blocks.new(
            "left_rail_detail_view"
        )  # 添加轮廓视图块
        self.mid_rail_detail_shape = self.rail_embedded_detail_view.blocks.new(
            "mid_rail_detail_view"
        )  # 添加轮廓视图块
        self.right_rail_detail_shape = self.rail_embedded_detail_view.blocks.new(
            "right_rail_detail_view"
        )  # 添加轮廓视图块
        self.left_rail_dimension = self.rail_embedded_detail_view.blocks.new(
            "left_rail_dimension"
        )  # 添加左侧线性标注块
        self.mid_rail_dimension = self.rail_embedded_detail_view.blocks.new(
            "mid_rail_dimension"
        )  # 添加中部线性标注块
        self.right_rail_dimension = self.rail_embedded_detail_view.blocks.new(
            "right_rail_dimension"
        )  # 添加右侧线性标注块
        self.right_rail_break_line = self.rail_embedded_detail_view.blocks.new(
            "right_rail_break_line"
        )  # 添加折断线块
        self.left_rail_outline = self.rail_embedded_detail_view.blocks.new(
            "left_rail_outline"
        )  # 引出线块
        self.mid_rail_outline = self.rail_embedded_detail_view.blocks.new(
            "mid_rail_outline"
        )  # 引出线块
        self.right_rail_outline = self.rail_embedded_detail_view.blocks.new(
            "right_rail_outline"
        )  # 引出线块
        self.left_rail_rebar = self.rail_embedded_detail_view.blocks.new(
            "left_rail_rebar"
        )  # 左侧钢筋块
        self.mid_rail_rebar = self.rail_embedded_detail_view.blocks.new(
            "mid_rail_rebar"
        )  # 中部钢筋块
        self.right_rail_rebar = self.rail_embedded_detail_view.blocks.new(
            "right_rail_rebar"
        )  # 右侧钢筋块

    def create_all_hatch_pattern(self):
        """
        创建所有的颜色或图案填充
        :return:
        """
        self.mid_hatch = self.model_space.add_hatch(color=7)
        self.right_hatch = self.model_space.add_hatch(color=7)

    def begin_draw_stair_rail_left_profile_shape(self):
        """
        开始绘制楼梯左侧轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_rail_embedded_weld_horizon_bounding_profile_points()
        )
        layer_outline = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                # segment[num] = transform_point_from_xy_to_yx(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.left_rail_detail_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )

    def begin_draw_stair_rail_left_rebar_horizon_profile_shape(self):
        """
        绘制楼梯栏杆左侧图钢筋轮廓形状
        :return:
        """
        rebar_draw_info = (
            self.stair_cut_data.get_stair_rail_embedded_U_rebar_horizon_profile_points()
        )
        draw_circles = rebar_draw_info["draw_circles"]
        circle_center = draw_circles[0]
        circle_diameter = draw_circles[1]
        draw_lines = rebar_draw_info["draw_lines"]
        rebar_layer = "G_Rebar"
        # 绘制圆
        for num in range(len(circle_center)):
            center = circle_center[num]
            diameter = circle_diameter[num]
            # center = transform_point_from_xy_to_yx(list(center))  # 起点
            center = adjust_drawing_scale(list(center), 1 / self.scale)
            diameter = diameter * (1 / self.scale)
            self.left_rail_rebar.add_circle(
                tuple(center), diameter / 2, dxfattribs={"layer": rebar_layer}
            )
        # 绘制线段
        for seg in draw_lines:
            start_point = seg[0]
            end_point = seg[1]
            start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
            end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
            self.left_rail_rebar.add_line(
                tuple(start_point), tuple(end_point), dxfattribs={"layer": rebar_layer}
            )

    def begin_draw_stair_rail_left_dimension_shape(self):
        """
        开始对左侧视图进行尺寸标注
        :return:
        """
        dimension_points = (
            self.stair_cut_data.get_stair_rail_embedded_left_view_dimension_points()
        )
        # 第一层标注点
        for segment in dimension_points:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = list(segment[0])  # 起点
            end_point = list(segment[1])  # 终点
            # start_point = transform_point_from_xy_to_yx(list(start_point))  # 起点
            # end_point = transform_point_from_xy_to_yx(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.left_rail_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.left_rail_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_rail_left_view_outline_shape(self):
        """
        开始绘制楼梯栏杆左侧引出线形状
        :return:
        """
        outline_info = (
            self.stair_cut_data.get_stair_rail_embedded_left_view_outline_points()
        )
        draw_lines = outline_info["draw_lines"]
        special_text = outline_info["special_text"]
        text_loc = special_text[0]
        text_info = special_text[1]
        cutline_layer = "G_Cut_Line"
        text_layer = "G_Note"
        for seg in draw_lines:
            start_point = seg[0]
            end_point = seg[1]
            # start_point = transform_point_from_xy_to_yx(list(start_point))  # 起点
            # end_point = transform_point_from_xy_to_yx(list(end_point))  # 终点
            start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
            end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
            self.left_rail_outline.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": cutline_layer},
            )
        # 设置剖切线
        for num in range(len(text_loc)):
            point = text_loc[num]
            text = text_info[num]
            # point = transform_point_from_xy_to_yx(list(point))  # 终点
            point = adjust_drawing_scale(list(point), 1 / self.scale)
            self.left_rail_outline.add_text(
                text,
                height=self.note_text_size,
                rotation=0,
                dxfattribs={"layer": text_layer},
            ).set_placement(tuple(point), align=TextEntityAlignment.BOTTOM_RIGHT)

    def begin_draw_stair_rail_mid_weld_transverse_profile_shape(self):
        """
        开始绘制楼梯中部焊板横向轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_rail_embedded_weld_transverse_bounding_profile_points()
        )
        layer_outline = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_xz_to_xy(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.mid_rail_detail_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": layer_outline}
                )

    def begin_draw_stair_rail_mid_rebar_transverse_profile_shape(self):
        """
        绘制楼梯栏杆中部图钢筋轮廓形状
        :return:
        """
        rebar_draw_info = (
            self.stair_cut_data.get_stair_rail_embedded_U_rebar_transverse_bounding_profile_points()
        )
        rebar_layer = "G_Rebar"
        # 绘制线段
        for seg in rebar_draw_info:
            for num in range(len(seg) - 1):
                start_point = seg[num]
                end_point = seg[num + 1]
                start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
                end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
                start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
                end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
                self.mid_rail_rebar.add_line(
                    tuple(start_point),
                    tuple(end_point),
                    dxfattribs={"layer": rebar_layer},
                )

    def begin_draw_stair_rail_mid_view_dimension_shape(self):
        """
        开始绘制楼梯栏杆埋件中部视图尺寸标注
        :return:
        """
        dimension_points = (
            self.stair_cut_data.get_stair_rail_embedded_mid_view_dimension_points()
        )
        # 第一层标注点
        for segment in dimension_points:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = list(segment[0])  # 起点
            end_point = list(segment[1])  # 终点
            start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
            end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.mid_rail_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.mid_rail_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def begin_draw_stair_rail_embedded_mid_view_hatch_shape(self):
        """
        开始绘制栏杆埋件中部视图填充形状
        :return:
        """
        hatch_info = (
            self.stair_cut_data.get_stair_rail_embedded_mid_view_hatch_outline_points()
        )
        left_hatch = hatch_info["left_hatch"]
        right_hatch = hatch_info["right_hatch"]
        outline_weld = hatch_info["outline_weld"]
        outline_rebar = hatch_info["outline_rebar"]
        total_outline = [outline_weld, outline_rebar]
        outline_text_w = hatch_info["outline_text_w"]
        outline_text_r = hatch_info["outline_text_r"]
        layer_profile = "G_Embedded_Subline"
        layer_outline = "G_Label"
        text_layer = "G_Note"
        total_outline_text = [outline_text_w, outline_text_r]
        # 开始绘制填充形状
        # 绘制轮廓线
        draw_line_left = copy.deepcopy(left_hatch)
        draw_line_left.append(left_hatch[0])
        for num in range(len(draw_line_left) - 1):
            start_point = draw_line_left[num]
            end_point = draw_line_left[num + 1]
            start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
            end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.mid_rail_detail_shape.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": layer_profile},
            )
        # 开始填充
        for num in range(len(left_hatch)):
            point = left_hatch[num]
            point = transform_point_from_xz_to_xy(list(point))  # 起点
            point = adjust_drawing_scale(point, 1 / self.scale)
            left_hatch[num] = point
        self.mid_hatch.paths.add_polyline_path(left_hatch, is_closed=1)
        # 开始绘制右侧填充轮廓和图形
        draw_line_right = copy.deepcopy(right_hatch)
        draw_line_right.append(right_hatch[0])
        for num in range(len(draw_line_right) - 1):
            start_point = draw_line_right[num]
            end_point = draw_line_right[num + 1]
            start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
            end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.mid_rail_detail_shape.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": layer_profile},
            )
        # 开始填充
        for num in range(len(right_hatch)):
            point = right_hatch[num]
            point = transform_point_from_xz_to_xy(list(point))  # 起点
            point = adjust_drawing_scale(point, 1 / self.scale)
            right_hatch[num] = point
        self.mid_hatch.paths.add_polyline_path(right_hatch, is_closed=1)
        # 开始绘制引出线
        for single in total_outline:
            for segment in single:
                # 调整点的比例
                start_point = copy.deepcopy(segment[0])
                end_point = copy.deepcopy(segment[1])
                start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
                end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
                start_point = adjust_drawing_scale(start_point, 1 / self.scale)
                end_point = adjust_drawing_scale(end_point, 1 / self.scale)
                self.mid_rail_outline.add_line(
                    tuple(start_point),
                    tuple(end_point),
                    dxfattribs={"layer": layer_outline},
                )
        # 开始增加文本
        for single_text in total_outline_text:
            point_loc = single_text[0]  # 文本防止点
            point_loc = transform_point_from_xz_to_xy(list(point_loc))  # 起点
            point_loc = adjust_drawing_scale(point_loc, 1 / self.scale)
            text = single_text[1]  # 文本内容
            self.mid_rail_outline.add_text(
                text,
                height=self.note_text_size,
                rotation=0,
                dxfattribs={"layer": text_layer},
            ).set_placement(point_loc, align=TextEntityAlignment.BOTTOM_LEFT)

    def begin_draw_stair_rail_embedded_right_view_profile_shape(self):
        """
        开始绘制栏杆埋件右侧视图轮廓线
        :return:
        """
        # 实体轮廓线
        profile_points = (
            self.stair_cut_data.get_stair_rail_embedded_rabbet_transverse_bounding_profile_points()
        )
        profile_layer = "G_Special_Outline"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_xz_to_xy(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.right_rail_detail_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": profile_layer}
                )

    def begin_draw_stair_rail_right_weld_transverse_profile_shape(self):
        """
        开始绘制楼梯中部焊板横向轮廓形状
        :return:
        """
        profile_points = (
            self.stair_cut_data.get_stair_rail_embedded_weld_transverse_bounding_profile_points()
        )
        weld_layer = "G_Rebar"
        # 变换绘制比例
        for segment in profile_points:
            for num in range(len(segment)):
                segment[num] = transform_point_from_xz_to_xy(list(segment[num]))  # 起点
                segment[num] = adjust_drawing_scale(list(segment[num]), 1 / self.scale)
        for segment in profile_points:
            for num in range(len(segment) - 1):
                seg = [
                    (segment[num][0], segment[num][1], segment[num][2]),
                    (segment[num + 1][0], segment[num + 1][1], segment[num + 1][2]),
                ]  # 将三维点转化为平面点
                self.right_rail_detail_shape.add_line(
                    seg[0], seg[1], dxfattribs={"layer": weld_layer}
                )

    def begin_draw_stair_rail_embedded_rebar_right_view_profile_shape(self):
        """
        开始绘制楼梯栏杆埋件钢筋右侧视图轮廓线
        :return:
        """
        rebar_draw_info = (
            self.stair_cut_data.get_stair_rail_embedded_U_rebar_transverse_bounding_profile_points()
        )
        rebar_layer = "G_Rebar"
        # 绘制线段
        for seg in rebar_draw_info:
            for num in range(len(seg) - 1):
                start_point = seg[num]
                end_point = seg[num + 1]
                start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
                end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
                start_point = adjust_drawing_scale(list(start_point), 1 / self.scale)
                end_point = adjust_drawing_scale(list(end_point), 1 / self.scale)
                self.right_rail_rebar.add_line(
                    tuple(start_point),
                    tuple(end_point),
                    dxfattribs={"layer": rebar_layer},
                )

    def begin_draw_stair_rail_embedded_right_view_hatch_shape(self):
        """
        开始绘制栏杆埋件中部视图填充形状
        :return:
        """
        hatch_info = (
            self.stair_cut_data.get_stair_rail_embedded_mid_view_hatch_outline_points()
        )
        left_hatch = hatch_info["left_hatch"]
        right_hatch = hatch_info["right_hatch"]
        profile_layer = "G_Embedded_Subline"
        outline_layer = "G_Label"
        # 开始绘制填充形状
        # 绘制轮廓线
        draw_line_left = copy.deepcopy(left_hatch)
        draw_line_left.append(left_hatch[0])
        for num in range(len(draw_line_left) - 1):
            start_point = draw_line_left[num]
            end_point = draw_line_left[num + 1]
            start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
            end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.right_rail_detail_shape.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": profile_layer},
            )
        # 开始填充
        for num in range(len(left_hatch)):
            point = left_hatch[num]
            point = transform_point_from_xz_to_xy(list(point))  # 起点
            point = adjust_drawing_scale(point, 1 / self.scale)
            left_hatch[num] = point
        self.right_hatch.paths.add_polyline_path(left_hatch, is_closed=1)
        # 开始绘制右侧填充轮廓和图形
        draw_line_right = copy.deepcopy(right_hatch)
        draw_line_right.append(right_hatch[0])
        for num in range(len(draw_line_right) - 1):
            start_point = draw_line_right[num]
            end_point = draw_line_right[num + 1]
            start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
            end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            self.right_rail_detail_shape.add_line(
                tuple(start_point),
                tuple(end_point),
                dxfattribs={"layer": profile_layer},
            )
        # 开始填充
        for num in range(len(right_hatch)):
            point = right_hatch[num]
            point = transform_point_from_xz_to_xy(list(point))  # 起点
            point = adjust_drawing_scale(point, 1 / self.scale)
            right_hatch[num] = point
        self.right_hatch.paths.add_polyline_path(right_hatch, is_closed=1)

    def begin_draw_stair_rail_embedded_breakline_shape(self):
        """
        开始绘制折断线形状
        :return:
        """
        draw_lines = (
            self.stair_cut_data.get_stair_rail_embedded_rabbet_transverse_breakline_points()
        )
        break_line_layer = "G_Break_Line"
        for num in range(len(draw_lines)):
            point = draw_lines[num]
            point = transform_point_from_xz_to_xy(list(point))  # 起点
            draw_lines[num] = adjust_drawing_scale(list(point), 1 / self.scale)
        # 遍历线段点
        for num in range(len(draw_lines) - 1):
            seg = [
                ((draw_lines[num][0]), (draw_lines[num][1]), (draw_lines[num][2])),
                (
                    (draw_lines[num + 1][0]),
                    (draw_lines[num + 1][1]),
                    (draw_lines[num + 1][2]),
                ),
            ]  # 将三维点转化为平面点
            self.right_rail_break_line.add_line(
                seg[0], seg[1], dxfattribs={"layer": break_line_layer}
            )

    def begin_draw_stair_rail_embedded_right_view_dimension_shape(self):
        """
        获取楼梯栏杆埋件右侧视图标注点
        :return:
        """
        dimension_info = (
            self.stair_cut_data.get_stair_rail_embedded_right_view_dimension_points()
        )
        basic_dimension_loc = dimension_info["basic_dimension_loc"]
        special_dimension_loc = dimension_info["special_dimension_loc"]
        # 第一层标注点
        for segment in basic_dimension_loc:
            # 调整该层标注相对实体偏移量
            current_offset = self.dimension_offset * 0.25
            # 调整点的比例
            start_point = list(segment[0])  # 起点
            end_point = list(segment[1])  # 终点
            start_point = transform_point_from_xz_to_xy(list(start_point))  # 起点
            end_point = transform_point_from_xz_to_xy(list(end_point))  # 终点
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.right_rail_dimension.add_linear_dim(
                    base=base_p, p1=start_point, p2=end_point, dimstyle="G_DS_20"
                )
            else:
                dim = self.right_rail_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()
        # 特殊文本标注
        special_loc = [special_dimension_loc[0]]
        special_text = special_dimension_loc[1]
        for num in range(len(special_loc)):
            # 调整该层标注相对实体偏移量
            segment = special_loc[num]
            text = special_text[num]
            current_offset = self.dimension_offset * 0.25
            start_point = transform_point_from_xz_to_xy(list(segment[0]))
            end_point = transform_point_from_xz_to_xy(list(segment[1]))
            # 调整点的比例
            start_point = adjust_drawing_scale(start_point, 1 / self.scale)
            end_point = adjust_drawing_scale(end_point, 1 / self.scale)
            # 对尺寸数字反向内容进行特殊操作
            direction = calculate_normal_vector(start_point, end_point, 0)
            base_p_ = np.array(direction) * current_offset + np.array(list(end_point))
            base_p = tuple(base_p_)  # 对齐点
            if direction[1] < 0 and abs(direction[1]) > self.precision:  # 存在截断误差
                dim = self.right_rail_dimension.add_linear_dim(
                    base=base_p,
                    p1=start_point,
                    p2=end_point,
                    text=text,
                    dimstyle="G_DS_20",
                )
            else:
                dim = self.right_rail_dimension.add_aligned_dim(
                    p1=end_point,
                    p2=start_point,
                    text=text,
                    distance=current_offset,
                    dimstyle="G_DS_20",
                )
            dim.render()

    def main_run_process(self):
        """
        主要运行过程
        :return:
        """
        self.begin_draw_stair_rail_left_profile_shape()  # 开始绘制左侧图栏杆埋件
        self.begin_draw_stair_rail_left_rebar_horizon_profile_shape()  # 开始绘制左侧图钢筋
        self.begin_draw_stair_rail_left_dimension_shape()  # 开始绘制楼梯栏杆左侧图尺寸标注
        self.begin_draw_stair_rail_left_view_outline_shape()  # 开始绘制楼梯栏杆左侧图引出线形状
        self.begin_draw_stair_rail_mid_weld_transverse_profile_shape()  # 开始绘制楼梯栏杆中部轮廓形状
        self.begin_draw_stair_rail_mid_rebar_transverse_profile_shape()  # 开始绘制中间图U型钢筋剖切形状
        self.begin_draw_stair_rail_mid_view_dimension_shape()  # 开始绘制中部视图尺寸标注
        self.begin_draw_stair_rail_embedded_mid_view_hatch_shape()  # 开始绘制楼梯栏杆埋件中部视图填充形状
        # 右侧视图
        self.begin_draw_stair_rail_embedded_right_view_profile_shape()  # 开始绘制楼梯栏杆埋件实体轮廓
        self.begin_draw_stair_rail_right_weld_transverse_profile_shape()  # 开始绘制右侧视图焊板形状
        self.begin_draw_stair_rail_embedded_rebar_right_view_profile_shape()  # 开始绘制楼梯栏杆埋件钢筋轮廓
        self.begin_draw_stair_rail_embedded_right_view_hatch_shape()  # 开始绘制右侧图填充图案
        self.begin_draw_stair_rail_embedded_breakline_shape()  # 开始绘制栏杆埋件折断线形状
        self.begin_draw_stair_rail_embedded_right_view_dimension_shape()  # 开始绘制楼梯栏杆右侧图标注点
