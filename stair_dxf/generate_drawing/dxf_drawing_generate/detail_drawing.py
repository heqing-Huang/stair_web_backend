"""
产生所有深化设计图纸
"""
import numpy as np

import ezdxf
from ezdxf.document import Drawing
import math

from stair_detailed.models import DetailedDesignResult
from stair_rebar_layout.models import RebarforBIM
from stair_structure.model import (
    StructuralDesign,
    StructuralDesignResult,
    ConcreteParameter,
    RebarParameter,
)

from stair_dxf.generate_drawing.dxf_drawing_generate.stair_total_detail_drawing import (
    StairTopView,
    StairBottomView,
    StairLeftView,
    StairReinforcementView,
    StairSectionOneView,
    StairSectionTwoView,
    StairRebarSectionAToAView,
    StairRebarSectionBToBView,
    StairRebarSectionCToCView,
    StairBottomInstallNodeView,
    StairTopInstallNodeView,
    StairBottomHoleReinRebarView,
    StairTopHoleReinRebarView,
    StairStepSlotLeftView,
    StairStepSlotTopView,
    StairDoubleSideWallJointLongView,
    StairDoubleSideWallJointTransverseView,
    StairRailEmbeddedDetailView,
)


class StairDetailView(object):
    """
    楼梯深化设计图纸
    color：1---red、2---yellow、3---green、4----cyan、5----blue、6----purple、7----white
    """

    def __init__(
        self,
        structure_design,
        structure_design_result,
        detailed_design_result,
        rebar_for_bim,
        dxf_doc: Drawing,
    ):
        self.structure_design = structure_design
        self.detailed_design = detailed_design_result.detailed_design
        self.structure_design_result = structure_design_result
        self.detailed_design_result = detailed_design_result
        self.rebar_for_bim = rebar_for_bim
        self.dxf_doc = dxf_doc
        self.load_dxf_file()  # 加载dxf文件
        self.generate_basic_data()  # 产生基本数据
        self.create_total_dxf_drawing()  # 产生所有dxf图形

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
        self.model_space = self.dxf_doc.modelspace()  # 创建模型空间
        self.rectangle_shape = self.dxf_doc.blocks.new("rectangle_View")  # 添加视图块

    def create_total_dxf_drawing(self):
        """
        产生所有dxf图形
        :return:
        """
        # 创建楼梯俯视图
        self.stair_top_view = StairTopView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_bottom_view = StairBottomView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_left_view = StairLeftView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_reinforce_view = StairReinforcementView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_section_one_view = StairSectionOneView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_section_two_view = StairSectionTwoView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_section_a_view = StairRebarSectionAToAView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_section_b_view = StairRebarSectionBToBView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_section_c_view = StairRebarSectionCToCView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_bottom_install_view = StairBottomInstallNodeView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_top_install_view = StairTopInstallNodeView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_bottom_hole_rein_view = StairBottomHoleReinRebarView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_top_hole_rein_view = StairTopHoleReinRebarView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_step_slot_left_view = StairStepSlotLeftView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_step_slot_top_view = StairStepSlotTopView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_double_wall_long_joint_view = StairDoubleSideWallJointLongView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_double_wall_tran_joint_view = StairDoubleSideWallJointTransverseView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )
        self.stair_rail_embedded_detail_view = StairRailEmbeddedDetailView(
            self.dxf_doc,
            self.structure_design,
            self.detailed_design,
            self.structure_design_result,
            self.detailed_design_result,
            self.rebar_for_bim,
        )

    def place_stair_top_view_block_sets(self):
        """
        放置楼梯平面图块集合
        """
        # 楼梯俯视图坐标点
        top_view_loc = (7778, 9629, 0)
        self.model_space.add_blockref(
            "rectangle_View",
            top_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转90度
        # 楼梯俯视图块布局
        self.model_space.add_blockref(
            "top_view_dimension",
            top_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度---在前述坐标中已经旋转过
        self.model_space.add_blockref(
            "top_view_hole_dim",
            top_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_view",
            top_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 90,
            },
        )  # 逆时针旋转90度
        self.model_space.add_blockref(
            "top_view_rein_rebar",
            top_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 90,
            },
        )  # 逆时针旋转90度
        self.model_space.add_blockref("top_view_text_dim", top_view_loc)

    def place_stair_bottom_view_block_sets(self):
        """
        放置仰视图块集合
        """
        # 楼梯仰视图
        bottom_view_loc = (7778, 7406, 0)
        self.model_space.add_blockref(
            "bottom_view",
            bottom_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 90,
            },
        )  # 逆时针旋转90度
        self.model_space.add_blockref(
            "bottom_view_hole_dim",
            bottom_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转90度
        self.model_space.add_blockref(
            "bottom_view_rein_rebar",
            bottom_view_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 90,
            },
        )  # 逆时针旋转90度
        self.model_space.add_blockref("bottom_view_text_dim", bottom_view_loc)

    def place_stair_left_view_block_sets(self):
        """
        放置楼梯侧视图块集合
        """
        # 楼梯楼梯侧视图
        left_view_loc = (7778, 3298, 0)
        self.model_space.add_blockref(
            "left_view",
            left_view_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度
        self.model_space.add_blockref(
            "left_view_hole_profile",
            left_view_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度
        self.model_space.add_blockref(
            "left_view_dimension",
            left_view_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度
        self.model_space.add_blockref(
            "left_view_rein_rebar",
            left_view_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度

    def place_stair_reinforce_block_sets(self):
        """
        放置楼梯配筋块集合
        """
        # 楼梯配筋图
        stair_reinforce_loc = (7778, 470, 0)
        self.model_space.add_blockref(
            "stair_reinforce_view",
            stair_reinforce_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度----添加楼梯轮廓块
        self.model_space.add_blockref(
            "stair_reinforce_embedded",
            stair_reinforce_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度---添加预埋件块
        self.model_space.add_blockref(
            "stair_reinforce_rebar",
            stair_reinforce_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转90度---添加钢筋系列块
        self.model_space.add_blockref(
            "stair_reinforce_dimension",
            stair_reinforce_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 标注逆时针旋转90度---添加标注块
        self.model_space.add_blockref(
            "stair_reinforce_cut_signal",
            stair_reinforce_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 标注逆时针旋转90度----添加剖切符块
        self.model_space.add_blockref(
            "stair_reinforce_outline",
            stair_reinforce_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 90},  # 缩放比例  # 缩放比例
        )  # 标注逆时针旋转90度---添加引出线块

    def place_stair_section_one_to_one_block_sets(self):
        """
        放置楼梯1-1剖面块集合
        """
        # 楼梯1-1剖面图
        minus_y = self.h_total - self.h1
        stair_section_one_one_loc = (9331, 10747 - minus_y, 0)
        self.model_space.add_blockref(
            "section_one_view",
            stair_section_one_one_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_one_dimension",
            stair_section_one_one_loc,
            dxfattribs={
                "xscale": 1,  # 缩放比例---模型空间尺寸与图纸空间的比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_one_one_signal",
            stair_section_one_one_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度

    def place_stair_section_two_to_two_block_sets(self):
        """
        放置楼梯2-2剖面块集合
        """
        # 楼梯2-2剖面
        stair_section_two_two_loc = (9331, 9408, 0)
        self.model_space.add_blockref(
            "section_two_view",
            stair_section_two_two_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_two_dimension",
            stair_section_two_two_loc,
            dxfattribs={
                "xscale": 1,  # 缩放比例---模型空间尺寸与图纸空间的比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_two_two_signal",
            stair_section_two_two_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度

    def place_stair_section_a_to_a_block_sets(self):
        """
        放置楼梯a-a剖面图块集合
        """
        # 楼梯a-a剖面图
        minus_y = self.h_total - self.h1
        stair_section_a_loc = (9120, 1260 - minus_y, 0)
        self.model_space.add_blockref(
            "section_a_view",
            stair_section_a_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_a_hole",
            stair_section_a_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_a_rebar",
            stair_section_a_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_a_dimension",
            stair_section_a_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_a_outline",
            stair_section_a_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度

    def place_stair_section_b_to_b_block_sets(self):
        """
        放置楼梯b-b剖面图
        """
        # 楼梯b-b剖面图
        curr_h = self.h2 + (self.h / 2)
        stair_section_b_loc = (10965, 1377 - curr_h, 0)
        self.model_space.add_blockref(
            "section_b_view",
            stair_section_b_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_b_hole",
            stair_section_b_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_b_rebar",
            stair_section_b_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_b_dimension",
            stair_section_b_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_b_outline",
            stair_section_b_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_section_c_to_c_block_sets(self):
        """
        放置楼梯c-c剖面图块集合
        """
        # 楼梯c-c剖面图
        stair_section_c_loc = (12840, 1258.5, 0)
        self.model_space.add_blockref(
            "section_c_view",
            stair_section_c_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_c_hole",
            stair_section_c_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_c_rebar",
            stair_section_c_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_c_dimension",
            stair_section_c_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "section_c_outline",
            stair_section_c_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_bottom_hole_rein_rebar_block_sets(self):
        """
        放置楼梯底部孔洞加强筋块集合
        """
        # 底部销键加强筋做法
        stair_bottom_reinforce_left_loc = (11640.6, 6561.3, 0)
        add_y = 333
        stair_bottom_reinforce_top_loc = (11640.6, 6561.3 + add_y, 0)
        # 孔洞加强钢筋侧视图
        self.model_space.add_blockref(
            "bottom_hole_view",
            stair_bottom_reinforce_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_hole_left_dimension",
            stair_bottom_reinforce_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_hole_break_line",
            stair_bottom_reinforce_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_hole_outline",
            stair_bottom_reinforce_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_hole_rein_rebar",
            stair_bottom_reinforce_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        # 底部孔洞加强筋平面图
        self.model_space.add_blockref(
            "bottom_hole_cut_outline",
            stair_bottom_reinforce_top_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_hole_cut_rein_rebar",
            stair_bottom_reinforce_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_hole_top_dimension",
            stair_bottom_reinforce_top_loc,
            dxfattribs={"xscale": 1, "yscale": 1, "rotation": 0},  # 缩放比例  # 缩放比例
        )  # 逆时针旋转0度

    def place_stair_top_hole_rein_rebar_block_sets(self):
        """
        放置楼梯顶部孔洞加强筋块集合
        """
        # 顶部销键加强筋做法
        add_y = 333
        minus_y = self.h_total - self.h1
        minus_x = self.l_total - self.lt_d
        stair_top_reinforce_loc = (13028 - minus_x, 6641 - minus_y, 0)
        stair_top_reinforce_top_loc = (13028 - minus_x, 6641 + add_y, 0)
        self.model_space.add_blockref(
            "top_hole_view",
            stair_top_reinforce_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_hole_left_dimension",
            stair_top_reinforce_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_hole_break_line",
            stair_top_reinforce_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_hole_outline",
            stair_top_reinforce_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_hole_rein_rebar",
            stair_top_reinforce_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        # 顶部加强筋平面视图
        self.model_space.add_blockref(
            "top_hole_cut_outline",
            stair_top_reinforce_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_hole_cut_rein_rebar",
            stair_top_reinforce_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_hole_top_dimension",
            stair_top_reinforce_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_bottom_install_node_block_sets(self):
        """
        放置楼梯底部安装节点块集合
        """
        # 底部安装节点大样
        stair_bottom_install_node_loc = (11510, 7961, 0)
        self.model_space.add_blockref(
            "bottom_install_view",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_install_hole_view",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_install_line_dimension",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_install_connect_embedded",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_install_break_line",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_install_elevation_line",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "bottom_install_outLine",
            stair_bottom_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_top_install_node_block_sets(self):
        """
        放置楼梯顶部安装节点块集合
        """
        # 顶部安装节点大样
        minus_y = self.h_total
        minus_x = self.l_total - self.lt_d
        stair_top_install_node_loc = (
            13000 - minus_x,
            8276.9 - minus_y,
            0,
        )  # (13296.8,8276.9-minus_y,0)
        self.model_space.add_blockref(
            "top_install_view",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_install_hole_view",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_install_line_dimension",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_install_connect_embedded",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_install_break_line",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_install_elevation_line",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "top_install_outLine",
            stair_top_install_node_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_step_slot_left_block_sets(self):
        """
        放置楼梯左侧防滑槽块集合
        """
        # 防滑槽左侧图
        step_slot_left_loc = (8500, 8200, 0)
        self.model_space.add_blockref(
            "step_slot_left_view",
            step_slot_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "step_slot_radius_dimension",
            step_slot_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "step_slot_break_line",
            step_slot_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "step_slot_line_dimension",
            step_slot_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_step_top_block_sets(self):
        """
        放置楼梯防滑槽俯视图块集合
        """
        # 防滑槽平面图
        step_slot_top_loc = (9145, 8000, 0)
        self.model_space.add_blockref(
            "step_slot_top_view",
            step_slot_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "step_slot_top_break_line",
            step_slot_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "step_slot_top_line_dimension",
            step_slot_top_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_double_wall_long_joint_block_sets(self):
        """
        放置楼梯双层墙纵向节点块集合
        """
        # 纵向塞缝大样
        long_double_wall_loc = (9524, 6375, 0)
        self.model_space.add_blockref(
            "wall_long_joint_view",
            long_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "wall_long_joint_line_dimension",
            long_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "wall_long_joint_break_line",
            long_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "wall_long_joint_outline",
            long_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_double_wall_tran_joint_block_sets(self):
        """
        放置楼梯双侧横向节点块集合
        """
        # 横向塞缝大样
        tran_double_wall_loc = (8811.6, 6446.3, 0)
        self.model_space.add_blockref(
            "wall_tran_joint_view",
            tran_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "wall_tran_joint_line_dimension",
            tran_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "wall_tran_joint_break_line",
            tran_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "wall_tran_joint_outline",
            tran_double_wall_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def place_stair_rail_embedded_block_sets(self):
        """
        放置楼梯栏杆预埋件块集合
        """
        # 栏杆预埋件位置
        rail_embedded_left_loc = (9247, 2800, 0)
        rail_embedded_mid_loc = (9747, 3076, 0)
        rail_embedded_right_loc = (10247, 3076, 0)
        self.model_space.add_blockref(
            "left_rail_detail_view",
            rail_embedded_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "left_rail_rebar",
            rail_embedded_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "left_rail_dimension",
            rail_embedded_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "left_rail_outline",
            rail_embedded_left_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "mid_rail_detail_view",
            rail_embedded_mid_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "mid_rail_rebar",
            rail_embedded_mid_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "mid_rail_dimension",
            rail_embedded_mid_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "mid_rail_outline",
            rail_embedded_mid_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "right_rail_detail_view",
            rail_embedded_right_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "right_rail_rebar",
            rail_embedded_right_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "right_rail_dimension",
            rail_embedded_right_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "right_rail_break_line",
            rail_embedded_right_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度
        self.model_space.add_blockref(
            "right_rail_dimension",
            rail_embedded_right_loc,
            dxfattribs={  # 插入块，插入块参考点---块中所有实体都发生局部坐标到全局坐标的转变，块属性
                "xscale": 1,  # 缩放比例
                "yscale": 1,  # 缩放比例
                "rotation": 0,
            },
        )  # 逆时针旋转0度

    def main_run_process(self):
        """
        主程序
        :return:
        """
        self.stair_top_view.main_run_process()
        self.place_stair_top_view_block_sets()
        self.stair_bottom_view.main_run_process()
        self.place_stair_bottom_view_block_sets()
        self.stair_left_view.main_run_process()
        self.place_stair_left_view_block_sets()

        self.stair_reinforce_view.main_run_process()
        self.place_stair_reinforce_block_sets()

        self.stair_section_one_view.main_run_process()
        self.place_stair_section_one_to_one_block_sets()
        self.stair_section_two_view.main_run_process()
        self.place_stair_section_two_to_two_block_sets()

        self.stair_section_a_view.main_run_process()
        self.place_stair_section_a_to_a_block_sets()
        self.stair_section_b_view.main_run_process()
        self.place_stair_section_b_to_b_block_sets()
        self.stair_section_c_view.main_run_process()
        self.place_stair_section_c_to_c_block_sets()

        self.stair_bottom_install_view.main_run_process()
        self.place_stair_bottom_install_node_block_sets()
        self.stair_top_install_view.main_run_process()
        self.place_stair_top_install_node_block_sets()

        self.stair_bottom_hole_rein_view.main_run_process()
        self.place_stair_bottom_hole_rein_rebar_block_sets()
        self.stair_top_hole_rein_view.main_run_process()
        self.place_stair_top_hole_rein_rebar_block_sets()

        self.stair_step_slot_left_view.main_run_process()
        self.place_stair_step_slot_left_block_sets()
        self.stair_step_slot_top_view.main_run_process()
        self.place_stair_step_top_block_sets()

        self.stair_double_wall_long_joint_view.main_run_process()
        self.place_stair_double_wall_long_joint_block_sets()

        self.stair_double_wall_tran_joint_view.main_run_process()
        self.place_stair_double_wall_tran_joint_block_sets()

        if self.detailed_design.inserts_detailed.rail_design_mode.value != 2:  # 无栏杆预埋件
            self.stair_rail_embedded_detail_view.main_run_process()
            self.place_stair_rail_embedded_block_sets()

        return self.dxf_doc
