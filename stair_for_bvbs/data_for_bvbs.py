"""
# File       : Rebar_layout.py
# Time       ：2022/9/21 9:25
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""
from stair_detailed.models import DetailedDesign, DetailedDesignResult
from stair_rebar_layout.models import RebarforBIM
from stair_structure.model import StructuralDesign, StructuralDesignResult

from stair_for_bvbs.models import RebarforBVBS
from stair_for_bvbs.rebar_data import RebarData


def data_for_bvbs(
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
    rebar_for_BVBS = RebarforBVBS()
    detailed_design = detailed_design_result.detailed_design
    # 钢筋数据
    # 孔洞加强钢筋
    rebar_data = RebarData(
        structure_design=structure_design,
        structure_design_result=structure_design_result,
        detailed_design=detailed_design,
        detailed_design_result=detailed_design_result,
    )
    # 孔洞加强钢筋
    hole_rebar_BVBS = rebar_data.get_hole_rebar()
    rebar_for_BVBS.hole_rebar = hole_rebar_BVBS

    # 吊装加强钢筋纵筋
    lifting_longitudinal_rebar_BVBS = rebar_data.get_lifting_longitudinal_rebar()
    rebar_for_BVBS.lifting_longitudinal_rebar = lifting_longitudinal_rebar_BVBS

    # 吊装加强钢筋点筋
    lifting_point_rebar_BVBS = rebar_data.get_lifting_point_rebar()
    rebar_for_BVBS.lifting_point_rebar = lifting_point_rebar_BVBS

    # 下部加强纵筋边筋
    bottom_edge_rein_rebar_BVBS = rebar_data.get_bottom_edge_reinforce_rebar()
    rebar_for_BVBS.bottom_edge_rein_rebar = bottom_edge_rein_rebar_BVBS

    # 上部加强纵筋边筋
    top_edge_rein_rebar_BVBS = rebar_data.get_top_edge_reinforce_rebar()
    rebar_for_BVBS.top_edge_rein_rebar = top_edge_rein_rebar_BVBS

    # 底部钢筋
    bottom_rebar_BVBS = rebar_data.get_bottom_rebar()
    rebar_for_BVBS.bottom_rebar = bottom_rebar_BVBS

    # 顶部钢筋
    top_rebar_BVBS = rebar_data.get_top_rebar()
    rebar_for_BVBS.top_rebar = top_rebar_BVBS

    # 下部加强箍筋
    bottom_edge_stirrup_rebar_BVBS = rebar_data.get_bottom_edge_stirrup_rebar()
    rebar_for_BVBS.bottom_edge_stirrup_rebar = bottom_edge_stirrup_rebar_BVBS

    # 上部加强箍筋
    top_edge_stirrup_rebar_BVBS = rebar_data.get_top_edge_stirrup_rebar()
    rebar_for_BVBS.top_edge_stirrup_rebar = top_edge_stirrup_rebar_BVBS

    # 下部加强筋（分布）
    bottom_rein_rebar_BVBS = rebar_data.get_bottom_rein_rebar()
    rebar_for_BVBS.bottom_rein_rebar = bottom_rein_rebar_BVBS

    # 上部加强筋 （分布）
    top_rein_rebar_BVBS = rebar_data.get_top_rein_rebar()
    rebar_for_BVBS.top_rein_rebar = top_rein_rebar_BVBS

    # 分布筋
    mid_rebar_BVBS = rebar_data.get_mid_rebar()
    rebar_for_BVBS.mid_rebar = mid_rebar_BVBS

    return rebar_for_BVBS
