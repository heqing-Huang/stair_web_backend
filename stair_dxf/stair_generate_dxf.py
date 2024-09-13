"""
预制楼梯生成dxf文件
"""

import os
from io import BytesIO, StringIO
from typing import TextIO

from stair_detailed.models import DetailedDesign, DetailedDesignResult
from stair_rebar_layout.models import RebarforBIM
from stair_structure.model import StructuralDesign, StructuralDesignResult

import ezdxf

from stair_dxf.generate_drawing.dxf_drawing_generate.detail_drawing import (
    StairDetailView,
)

# 定义包内模板路径
_TEMPLATES_PATH_ = os.path.join(os.path.dirname(__file__), "templates")

# 定义dxf 的模板文件
_DEFAULT_DXF_TEMPLATE = os.path.join(
    _TEMPLATES_PATH_,
    "template.dxf",
)


def stair_generate_dxf(
    structure_design: StructuralDesign,
    structure_design_result: StructuralDesignResult,
    detailed_design: DetailedDesign,
    detailed_design_result: DetailedDesignResult,
    rebar_data: RebarforBIM,
    file: TextIO = None,
):
    """
    楼梯生成dxf文件
    :param structure_design:
    :param structure_design_result:
    :param detailed_design:
    :param detailed_design_result:
    :param rebar_data:
    :param file: 打开的文件,非二进制类型.亦或者None.If None,将选择默认的模板
    :return:
    """
    if file is None:
        dxf_doc = ezdxf.readfile(_DEFAULT_DXF_TEMPLATE)
    else:
        dxf_doc = ezdxf.read(file)
    detailed_drawing = StairDetailView(
        structure_design=structure_design,
        structure_design_result=structure_design_result,
        detailed_design_result=detailed_design_result,
        rebar_for_bim=rebar_data,
        dxf_doc=dxf_doc,
    )
    dxf_file = detailed_drawing.main_run_process()
    dxf_content = StringIO()
    dxf_file.write(dxf_content, fmt="asc")
    dxf_content.seek(0)
    return dxf_content
