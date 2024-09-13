import json
import logging
import uuid
import importlib
import warnings
from dataclasses import asdict

from celery import shared_task

from django.contrib.auth.models import User
from django.core.files.base import ContentFile
from django.contrib.admin.options import get_content_type_for_model
from django.contrib.admin.models import LogEntry, CHANGE, ADDITION, settings

from stair_rebar_layout.models import RebarforBIM

from . import models as db_models
from .models import ModelDetailedResult, RebarLayoutModel
from . import tools
from .tools import call_rebar_layout

logger = logging.getLogger(__name__)


@shared_task(bind=True, acks_late=True)
def test(self):
    print("你好")


def rpc_rebar_layout(detailed_result_row_id: int) -> None:
    """
    传入深化设计的row ID,然后查询结果后,调用钢筋排布的计算
    Args:
        detailed_result_row_id:

    Returns:

    """
    warnings.warn("遗弃该函数,后台调用逻辑未规划,目前封装思路无法满足使用", stacklevel=2)
    detailed_result_row = ModelDetailedResult.objects.get(id=detailed_result_row_id)
    rebar_bim: RebarforBIM = call_rebar_layout(detailed_result_row)
    rebar_obj = asdict(rebar_bim)
    RebarLayoutModel.objects.update_or_create(
        defaults={"content": rebar_obj}, stair=detailed_result_row.stair
    )


@shared_task(bind=True, acks_late=True)
def total_back_handle(_, detailed_result_id: int) -> None:
    """
    组装内部的所有逻辑（深化设计结束之后）,包括钢筋排布，文件生成
    Args:
        _:
        detailed_result_id:

    Returns:

    """

    time_uuid = str(uuid.uuid4())

    # 钢筋排布
    detail_result_row = ModelDetailedResult.objects.get(id=detailed_result_id)
    rebar_bim: RebarforBIM = call_rebar_layout(detail_result_row)
    rebar_row, _ = RebarLayoutModel.objects.update_or_create(
        defaults={"content": asdict(rebar_bim)}, stair=detail_result_row.stair
    )
    # 预处理的数据
    logger.debug(rebar_row)
    exchanged = tools.BeforeFinalCall(rebar_row)
    ifc_content = tools.call_ifc_create(exchanged, rebar_row)

    export_manager, created = db_models.FileExport.objects.update_or_create(
        stair=detail_result_row.stair
    )

    export_manager.ifc.save(
        name=f"stair_{detail_result_row.stair.id}_{time_uuid}.ifc",
        content=ContentFile(ifc_content),
    )

    # BVBS 生成
    bvbs, zip_json = tools.make_call_bvbs(exchanged, rebar_row)
    export_manager.bvbs.save(
        name=f"stair_{detail_result_row.stair.id}_{time_uuid}.bvbs",
        content=ContentFile(bvbs),
    )
    export_manager.zip_json.save(
        name=f"stair_{detail_result_row.stair.id}_{time_uuid}.zip",
        content=ContentFile(zip_json),
    )

    # DXF 文件生成调用  save dxf file
    try:
        dxf_bytes_content = tools.make_call_dxf(exchanged, rebar_bim)
    except FileNotFoundError as e:
        logger.debug(f"模板文件缺失,不对其进行处理:{e}")
    else:
        # 调用dxf 生成部分
        if dxf_bytes_content:
            export_manager.dxf.save(
                name=f"stair_{detail_result_row.stair.id}_{time_uuid}.dxf",
                content=ContentFile(dxf_bytes_content),
            )
        else:
            logger.warning("dxf bytes content 生成异常")
    export_manager.save()
