"""
Date&Time           2022/8/4 12:22
Author              HaoLan

"""
import os
import logging
import warnings
from dataclasses import asdict
from typing import IO, Optional, Tuple
from traceback import format_exc

from stair_rebar_layout.models import RebarforBIM
from stair_rebar_layout.Rebar_layout import rebar_layout

from stair_detailed.detailed_design import detailed_design, to_word_detailed
from stair_detailed.models import (
    DetailedDesignResult,
    DetailedCalculationBook,
    DetailedDesign,
)

from stair_structure.model import StructuralDesignResult, StructuralDesign
from stair_structure.structure_calculation import (
    CalculationBook,
    to_word,
    structure_cal,
)

from stair_for_bvbs.data_for_bvbs import data_for_bvbs
from stair_rebar_bvbs.create_bvbs import create_bvbs
from stair_rebar_bvbs.create_JSON import create_json, make_zip_content_export

from stair_ifc.create_ifc import stair_IFC_creation

from stair_dxf.stair_generate_dxf import stair_generate_dxf

from . import exchange
from . import models
from . import layers
from .models import (
    DetailData,
    ModelConstructionResult,
    ModelDetailedResult,
    ModelConstructionData,
)
from .exchange import detailed
from .exchange.detailed import (
    ConstructionResultSerializer,
    SerializerFromModelDetailedResultToCalculationBookDetailed,
    detail_result_2_model_result,
    MDResultToDCResult,
)

_logger = logging.getLogger(__name__)
Construction = StructuralDesign
StructureResult = StructuralDesignResult


def api_to_word(result: ModelConstructionResult) -> IO[bytes]:
    construction = result.construction
    structure_parameter = Construction(
        **dict(exchange.structure.StructureDataSerializer(instance=construction).data)
    )
    structure_result = StructureResult(**ConstructionResultSerializer(result).data)
    calculation_book = CalculationBook(
        structure_design=structure_parameter, structure_design_result=structure_result
    )
    app_path = os.path.dirname(__file__)
    template_docx = os.path.join(app_path, "templates", "design", "template.docx")
    with open(template_docx, "rb") as f:
        byte_io, _ = to_word(calculation_book, f)

    return byte_io


def call_design_book(result: ModelDetailedResult) -> IO[bytes]:
    """
    封装对第三方库中,深化设计书的生成调用
    Args:
        result:

    Returns:

    """
    dict_data = SerializerFromModelDetailedResultToCalculationBookDetailed(
        instance=result
    ).data
    dataclass_book_detail = DetailedCalculationBook(**dict_data)
    app_path = os.path.dirname(__file__)
    template_docx = os.path.join(
        app_path, "templates", "design", "template", "detailed.docx"
    )
    with open(template_docx, "rb") as f:
        byte_io, _ = to_word_detailed(dataclass_book_detail, f)
    return byte_io


def call_detailed_design(detail_row: DetailData) -> DetailedDesignResult:
    """
    提取数据,调用深化设计
    Args:
        detail_row:

    Returns:

    """
    detail_serializer = detailed.DetailData2ConstructionDetailedSerializer(
        instance=detail_row
    )
    detailed_parameter = DetailedDesign(**dict(detail_serializer.data))

    structure_parameter = Construction(
        **dict(
            exchange.structure.StructureDataSerializer(instance=detail_row.stair).data
        )
    )
    structure_result = ModelConstructionResult.objects.filter(
        construction=detail_row.stair
    ).first()
    # 适应下层深化设计参数结构变动,'concrete_parameter' and 'rebar_parameter'
    structure_result_dict = ConstructionResultSerializer(instance=structure_result).data

    structure_result_parameter = StructureResult(**structure_result_dict)

    result = detailed_design(
        detailed_parameter, structure_parameter, structure_result_parameter
    )
    return result


def call_detailed_design_by_model(
    detail_row: DetailData,
) -> Optional[ModelDetailedResult]:
    """
    传入表模型,返回表模型,封装深化设计调用过程
    Args:
        detail_row:

    Returns:

    """
    detailed_design_result: DetailedDesignResult = call_detailed_design(detail_row)
    result: ModelDetailedResult = detail_result_2_model_result(
        detailed_design_result, detail_row
    )
    return result


def call_structural_calculation(
    structural: ModelConstructionData,
) -> Optional[ModelConstructionResult]:
    """

    Args:
        structural:

    Returns:

    """
    parameters = Construction(
        **dict(exchange.structure.StructureDataSerializer(instance=structural).data)
    )
    try:
        result: StructureResult = structure_cal(parameters)
    except Exception as e:
        _logger.error(f"结构计算发生错误:{e},parameters:{parameters}")
        _logger.error(format_exc())
        query_set = ModelConstructionResult.objects.filter(construction=structural)
        if query_set.exists():
            instance = query_set.last()
            instance.success = False
            instance.message = str(e)
        else:
            model_result = ModelConstructionResult.objects.create(
                success=False, message=str(e), construction=structural
            )
            return model_result
    else:
        if result is not None and isinstance(result, StructureResult):
            result_js = asdict(
                result, dict_factory=exchange.tools.custom_asdict_contain_enum
            )
            query_sets = ModelConstructionResult.objects.filter(construction=structural)
            if query_sets.exists():
                query_set = query_sets.last()
            else:
                query_set = None

            serializer = exchange.structure.ResultBothWaySerializer(
                data=result_js, instance=query_set
            )
            if serializer.is_valid():
                instance = serializer.save(success=True, construction=structural)
            else:
                raise serializer.errors
            return instance
            # model_result = exchange.structure.dataclass_to_parameter_row(
            #     result,
            #     structural
            # )
            # return model_result
        else:
            raise Exception("stair_structure 中结构计算模块异常")


def call_rebar_layout(model_detail_result: ModelDetailedResult) -> RebarforBIM:
    """
    完成对后台逻辑对钢筋排布的调用

    参阅:docs/流程图-参考/调用钢筋排布.md

    Args:
        model_detail_result:

    Returns:

    """
    # 结构参数处理
    dc_structure_obj = Construction(
        **dict(
            exchange.structure.StructureDataSerializer(
                instance=model_detail_result.stair
            ).data
        )
    )
    # 结构设计结果
    model_result_s = ModelConstructionResult.objects.filter(
        construction=model_detail_result.stair
    )
    if not model_result_s.exists():
        raise Exception("未完成结构计算,不应该出现该情况,请修复")
    if model_result_s.count() > 1:
        warnings.warn("结构计算结果不唯一,请检查前后逻辑并修复")

    model_result = model_result_s.first()
    dc_structure_result_obj = StructureResult(
        **ConstructionResultSerializer(model_result).data
    )
    # 深化设计结果
    dc_detailed_result_obj = DetailedDesignResult(
        **dict(MDResultToDCResult(model_detail_result).data)
    )

    data_detail = DetailedDesign(
        **dict(
            detailed.DetailData2ConstructionDetailedSerializer(
                instance=DetailData.objects.get(stair=model_detail_result.stair)
            ).data
        )
    )

    dc_rebar_bim_obj = rebar_layout(
        structure_design=dc_structure_obj,
        structure_design_result=dc_structure_result_obj,
        detailed_design=data_detail,
        detailed_design_result=dc_detailed_result_obj,
    )
    return dc_rebar_bim_obj


class BeforeFinalCall:
    def __init__(self, rebar_result: models.RebarLayoutModel):
        structure_kwargs = dict(
            exchange.structure.StructureDataSerializer(instance=rebar_result.stair).data
        )
        self.structure_design = StructuralDesign(**structure_kwargs)
        md_structure_result = models.ModelConstructionResult.objects.get(
            construction=rebar_result.stair
        )
        self.structure_result = StructuralDesignResult(
            **dict(
                exchange.structure.ResultBothWaySerializer(
                    instance=md_structure_result
                ).data
            )
        )
        md_detail_deign = models.DetailData.objects.get(stair=rebar_result.stair)
        self.detail_design = DetailedDesign(
            **dict(
                exchange.detailed.DetailData2ConstructionDetailedSerializer(
                    instance=md_detail_deign
                ).data
            )
        )
        md_detail_result = models.ModelDetailedResult.objects.get(
            stair=rebar_result.stair
        )
        detail_result_kwargs = exchange.detailed.MDResultToDCResult(
            instance=md_detail_result
        ).data
        self.detail_result = DetailedDesignResult(**dict(detail_result_kwargs))


def call_ifc_create(
    exchanged: BeforeFinalCall, rebar_result: models.RebarLayoutModel
) -> str:
    """
    orm 数据到IFC content str 的调用生成
    Args:
        exchanged: 数据层转换结果
        rebar_result: 对IFC 调用需要钢筋数据,这是最后生成的

    Returns:

    """
    rebar_for_bim = RebarforBIM(**rebar_result.content)
    ifc_content = stair_IFC_creation(
        exchanged.structure_design,
        exchanged.structure_result,
        exchanged.detail_design,
        exchanged.detail_result,
        rebar_for_bim,
    )
    return ifc_content.ifcfile.to_string()


def make_call_bvbs(
    exchanged: BeforeFinalCall, rebar_result: models.RebarLayoutModel
) -> Tuple[str, bytes]:
    """
    计算bvbs数据
    Args:
        exchanged:
        rebar_result:

    Returns:

    """
    rebar_for_bvbs = data_for_bvbs(
        exchanged.structure_design,
        exchanged.structure_result,
        exchanged.detail_design,
        exchanged.detail_result,
    )
    rebar_data_ascii_strings = create_bvbs(rebar_for_bvbs)
    file_name, write_rebar, read_rebar = create_json(rebar_for_bvbs)
    zip_content = make_zip_content_export(file_name, write_rebar, read_rebar)
    return rebar_data_ascii_strings, zip_content


def make_call_dxf(exchanged: BeforeFinalCall, rebar_data: RebarforBIM) -> bytes:
    """
    调用dxf 文件生成函数
    Args:
        rebar_data:
        exchanged:

    Returns:

    """
    try:
        bytes_content = stair_generate_dxf(
            exchanged.structure_design,
            exchanged.structure_result,
            exchanged.detail_design,
            exchanged.detail_result,
            rebar_data,
        )
    except Exception as e:
        # TODO(bigpangl@163.com):待内部相对稳定后,删除外层,为了调试提供数据所做的输出
        _logger.info(f"{asdict(exchanged.structure_design)=}")
        _logger.info(f"{asdict(exchanged.structure_result)=}")
        _logger.info(f"{asdict(exchanged.detail_design)=}")
        _logger.info(f"{asdict(exchanged.detail_result)=}")
        _logger.info(f"{asdict(rebar_data)=}")
        raise e
    else:
        bytes_content.seek(0)
        return bytes_content.read()
